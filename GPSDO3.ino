// Jim's take on GPSDO, based on code by Lars Walenius 
// Uses 1 PPS interrupt from GPS shield on pin D8
// Reads TIC value on A0
//    10/21/14 change lockPPSlimit 100 to 250, improve sample for plotXY
//    10/26/14 change histogram to watch 5-min average TIC value 
//    10/28/14 change dacValue to float, doesn't help offset (about 458 at TC=2048)
//    10/29/14 change TIC_ValueFiltered and TIC_ValueOld to float
//    11/01/14 enabled Input Capture w/ comparison to timer value from ISR
//              removed overflow pending stuff from pps ISR
//    11/02/14 switched to using ICR
//    11/03/14 modified to save and show TIC Avg from 24 hrs ago
//    11/10/14 cleaned up comments
//    12/13/14 modified init of TIC_ValueFiltered for smoother startup

#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

const char headline[] = "Arduino GPSDO by Lars Walenius - JH version";
const char verNo[] = "Rev 12/13/2014";

// filter constants and variables - note these must be signed to make division work
//  for negative numerators!
//  timeConst of 2048 seems good for my OCXO
const long timeConst = 512;
// (was 6500) loop gain - VCO DAC counts to make TIC move 1 count/sec
const long gain = 147;       
//Damping factor - 100 = 1.0 higher is less overshoot but slower response
const int damping = 200;     
const int filterDiv = 4;
const long filterConst = timeConst/filterDiv;
const long filterConst2 = ((filterConst*damping)/100)*timeConst;
const int TIC_Offset = 500;           //nominal ADC value - about half scale

volatile int TIC_Value = TIC_Offset;  // value read from ADC
float TIC_ValueFiltered;              //was long
float TIC_ValueOld;                   //was long
long TIC_ValueSmoothed = TIC_Offset * 16;  // used for lock detection and startup


float dacValue;                 //was long
long dacValueOut = 2048;        //as sent to DAC - initial value is set from previous runs

// timekeeping variables
unsigned long time;                       //tenths of second since start
unsigned long timeOld;                    //time at last pps
int missedPPS = 0;                        //counter for missed pps interrupts
volatile unsigned int timer1CounterValue; // 200 ns counts since last overflow
// number of overflows - incr every 13.1 ms, overflows after 858.99 sec
volatile unsigned int overflowCount = 0;  
// total time in 200 ns counts, overflows after 858.99 sec
volatile unsigned long timerCounts;       
unsigned long timerCountsOld;             // counts at last pps interrupt
// jitter since last pps in 200 ns counts (adj for missed pps)
long timerCountsDiff;                     
long timerCountsAcc;                      // net jitter since start
long timer_us;                            // microsec since 1 pps

// lock tracking variables
const int lockPPSlimit = 250;             // was 100  
const int lockPPSfactor = 5;
unsigned long lockPPScounter = 0;
boolean PPSlocked = false;

// data reporting variables
int i = 0; // counter for 300secs before storing temp and dac readings average
int j = 0; // counter for stored 300sec readings
int k = 0; // counter for stored 3hour readings
unsigned int StoreTIC_A[144];      //300sec storage
unsigned int StoreTIC_A_Old[144];  //previous 12 hrs of 5-min averages

//unsigned int StoreTempA[144];
unsigned int StoreDAC_A[144];
long sumTIC;                    // for current 5 min TIC average
long sumTIC2;                   // for current 3-hr average
//long sumTemp;
//long sumTemp2;
unsigned long sumDAC;           // for current 5 min DAC average
unsigned long sumDAC2;          // for current 3-hr average

//const byte led = 13;                          //LED pin for lock indicator
const byte txPin = 6;                           //GPS serial output
volatile boolean PPS_ReadFlag = false;          //set by PPS ISR
enum Modes {hold, run};
Modes opMode = run;                             //operating mode
unsigned int holdValue;                         //DAC value for Hold mode

 // helper functions for new serial monitor

// uppercase strings
char * sUpper(char *pStr)
{
    char *pStart = pStr;
    while(*pStr)
    {
        *pStr = toupper(*pStr);
        *pStr++;
    }
    return pStart;
}
// formatter for serial monitor - NOTE the length info checked by the serial monitor
void SMprint(const char *sCmd, const char *sText)
{
    printf("%s~%s$%d\n", sCmd, sText, strlen(sText) + strlen(sCmd) + 1);
    return;
}
// maps printf family to Serial
int serial_putc(char c, FILE *) 
{
  Serial.write(c);
  return c;
}
// begin printf family support
void printf_begin(void) 
{
  fdevopen(&serial_putc, 0);
}

ISR (TIMER1_OVF_vect)           // Timer1 overflow interrupt handler - called every 13.1 ms
{
   ++overflowCount;             //just bump the count
}

// Timer Capture interrupt routine - this runs at rising edge of 1 PPS on Pin 8
ISR (TIMER1_CAPT_vect)          
  {                    
  timer1CounterValue = ICR1;
  
  //timerCounts = (overflowCount * 65536) + timer1CounterValue;
  timerCounts = ((unsigned long)overflowCount) << 16 | timer1CounterValue;  //faster than above?
  // read TIC - maybe move this sooner and/or just kick off the ADC?
  TIC_Value = analogRead(A0);  
  
  PPS_ReadFlag = true;         //set the flag so the main loop knows it's time to run
}

// 1 pps interrupt routine - replaced by input capture above
/*
  void pps() {                    
  timer1CounterValue = TCNT1;  // grab counter value - original
  timer1CounterValue = ICR1;   // 11/2/14 read ICR instead
  
  //disabled 11/1/14 - missed overflow addressed in main loop
  //if (TIFR1 & TOV1) {          // wrong?? should be (TIFR1 & _BV(TOV1)) or (bitRead(TIFR1, TOV1))
  //  overflowCount++;
  //}
  
  timerCounts = (overflowCount * 65536) + timer1CounterValue;
  TIC_Value = analogRead(A0);  // read TIC - maybe move this sooner and/or just kick off the ADC?
  
  PPS_ReadFlag = true;      //set the flag
}
*/

void calculation() {

  timerCountsDiff = timerCounts - timerCountsOld;    // should be close to 5,000,000

  time = time + (timerCountsDiff + 250000) / 500000; // need to add 250000 to avoid truncation errors
  
  //reduce by 1 sec worth of counts at a time to correct for missed pps
  while (timerCountsDiff > 2500000) {
      timerCountsDiff -= 5000000;
  } 
  if (abs(timerCountsDiff + 65536) < 100) {         // corrects for rollover or missed overflow. not nice but..
      timerCountsDiff = timerCountsDiff + 65536; 
      timerCounts += 65536;
   }
  
  timerCountsAcc -= timerCountsDiff;  // net pps jitter in 200 ns ticks
  
  if (abs(timerCountsAcc) > 100) // reset if more than +-20uS. Should maybe add reset at start and maybe no reset in hold mode??
    {
      timerCountsAcc = (TIC_Value + 500)/200; // Not good to just check one sample. Should probably have some averaging.
    }
    
  timer_us = (timerCountsAcc - TIC_Value / 200) / 5;
  
  //detect missed pps
  if (time - timeOld > 11) // may give error after about 5000days (13 years) without reset!
    {
      //missedPPS++;
      missedPPS += (time - timeOld)/10 - 1;
    }
    
  timerCountsOld = timerCounts;  
  timeOld = time;
  
  //detect lock
  // criterion is smoothed TIC value (TC=16 sec) must be within lockPPSlimit (250 counts) of nominal (500) for at least
  // lockPPSfactor (5) * timeConst secs.
  if (time > 15) {
    TIC_ValueSmoothed = TIC_ValueSmoothed + (TIC_Value * 16 - TIC_ValueSmoothed) / 16; // Low Pass Filter for PPS lock
    lockPPScounter++;
  }
  else {
    TIC_ValueSmoothed = TIC_Value * 16;      //initialize TIC_ValueSmoothed
  }
  
   // clearer version of original
   if (abs(TIC_ValueSmoothed / 16 - TIC_Offset) > lockPPSlimit) {
      lockPPScounter = 0;
   }
   
   if (lockPPScounter > timeConst * lockPPSfactor) {
      PPSlocked = true;
   } 
   else {
      PPSlocked = false;
   }
     
    //digitalWrite(ppsLockedLED,PPSlocked); // turn on LED 13 if "locked"

    //filterConst = timeConst/filterDiv;

    if(time > 105 && opMode == run) {       // don't recalculate filters for the first 10 secs or if in Hold mode
      
      //LPF on TIC_Value - settles to Value * filterConst
      //TIC_ValueFiltered = TIC_ValueFiltered + (TIC_Value * filterConst - TIC_ValueFiltered)/filterConst;    //original
      //new - reduces roundoff error
      //TIC_ValueFiltered = TIC_ValueFiltered + (TIC_Value * filterConst - TIC_ValueFiltered + filterConst/2)/filterConst;
      // float version
      TIC_ValueFiltered = TIC_ValueFiltered + (TIC_Value * filterConst - TIC_ValueFiltered)/filterConst;
  
      //remove TIC offset and integrate (corr for time)
      //dacValue = dacValue + ((TIC_ValueFiltered - TIC_Offset * filterConst) * 100 / filterConst * gain / damping / timeConst);    //original
  
      // this version reduces overflow and rounding problems
      //dacValue = dacValue + ((TIC_ValueFiltered - TIC_Offset * filterConst) * gain + filterConst2/2)/filterConst2;
      //use this with dacValue as float
      dacValue = dacValue + ((TIC_ValueFiltered - TIC_Offset * filterConst) * gain)/filterConst2;
      
      //corr for frequency??
      //dacValue = dacValue + ((TIC_ValueFiltered - TIC_ValueOld) * 100 / filterConst * gain / 100);
      
      //reduce roundoff problems
      //dacValue = dacValue +((TIC_ValueFiltered - TIC_ValueOld) * gain + filterConst/2) / filterConst;
 
      //use this with float
      dacValue = dacValue +((TIC_ValueFiltered - TIC_ValueOld) * gain) / filterConst;
     }
   else {
     //do this at startup or if in hold mode
     TIC_ValueFiltered = TIC_ValueSmoothed * filterConst/16;
   }
     
     TIC_ValueOld = TIC_ValueFiltered;                     //keep for next time
     dacValue = constrain(dacValue, 0, 4095*timeConst);    //limit dacValue to valid range
      
     dacValueOut = (dacValue / timeConst) + 0.5;       //round and convert to integer for output
 
     if (opMode == hold) {
       dacValueOut = holdValue;
     }
     
     dacValueOut = constrain(dacValueOut, 0, 4095);
     dac.setVoltage(dacValueOut, false);


   // Storage of average readings for later printing

  
  sumTIC = sumTIC + (TIC_Value * 10);
  //sumTemp = sumTemp + (tempADC2 * 10);
  sumDAC = sumDAC + dacValueOut;
  
  if (++i == 300) // 300sec ** needs fix for missed pps
  {
    StoreTIC_A_Old[j] = StoreTIC_A[j];  //remember the data from 12 hrs ago
    StoreTIC_A[j]= sumTIC / i;
    sumTIC2 = sumTIC2 + sumTIC / i;
    sumTIC = 0;
    //StoreTempA[j]= sumTemp / i;
    //sumTemp2 = sumTemp2 + sumTemp / i;
    //sumTemp = 0;
    StoreDAC_A[j]= sumDAC / i;
    sumDAC2 = sumDAC2 + sumDAC / i;
    sumDAC = 0;
    i = 0;
    if (++j % 36 == 0) // store every 36 x 300sec (3 hours)
    {
      sumTIC2 = sumTIC2 / 36;
      EEPROM.write(k, highByte(sumTIC2));
      EEPROM.write(k+144, lowByte(sumTIC2));
      sumTIC2 = 0;
      //sumTemp2= sumTemp2 / 36;
      //EEPROM.write(k+576, highByte(sumTemp2));
      //EEPROM.write(k+720, lowByte(sumTemp2));
      //sumTemp2 = 0;
      sumDAC2 = sumDAC2 / 36;
      EEPROM.write(k+288, highByte(sumDAC2));
      EEPROM.write(k+432, lowByte(sumDAC2));
      sumDAC2 = 0;
      if (j == 144) // 144 x 300sec (12 hours)
        {
        j = 0;
        }

      if (++k == 144) // 144 x 10800sec (18 days) ?Ã¤ndra till 160 = 20days?
      {
        k = 0;
      }
      EEPROM.write(1023, k); // store present k (index of 3 hour average, used in setup)
     
    }
  }
}

void printDataToSerial()
{
  const char tab = '\t';
  Serial.print((time/10));
  Serial.print(tab);              // prints a tab
  Serial.print(timer_us *1000 + TIC_Value);
  Serial.print(tab); 
  Serial.print(TIC_Value);
  Serial.print(tab);              // prints a tab
  Serial.print(TIC_ValueFiltered / filterConst);
  Serial.print(tab);              // prints a tab
  Serial.print(dacValue/timeConst, 3);  //use 3 decimal places
  Serial.print(tab);              // prints a tab
  Serial.print(timeConst);
  Serial.print(tab);              // prints a tab
  Serial.print(timerCountsAcc);
  Serial.print(tab);              // prints a tab
   Serial.print(timerCountsDiff);
  Serial.print(tab);              // prints a tab
  Serial.print(missedPPS);
  Serial.print(tab);              // prints a tab
  if (opMode == run){ 
   Serial.print(PPSlocked);
  }
  else {   
    Serial.print(F("Hold"));
  }
  Serial.print(tab);              // prints a tab
   
  if (i == 1)
  {
 
  Serial.print(F("Five minute averages: TIC+DAC"));
  Serial.print(tab);              // prints a tab
  }
  if (i == 2)
  {
  Serial.print(F("Now acquiring value: "));
  Serial.print(j);
  Serial.print(tab);              // prints a tab
  }
  if ((i >= 4) && (i <= 147))
  {
  Serial.print((i-4));
  Serial.print(tab);              // prints a tab
  Serial.print((StoreTIC_A[i-4]));
  Serial.print(tab);              // prints a tab
  Serial.print((StoreDAC_A[i-4]));
  Serial.print(tab);              // prints a tab
  //Serial.print((StoreTempA[i-4]));
  //Serial.print(tab);              // prints a tab  
  }
  if (i == 148)
  {
  Serial.print(F("Three hour averages: TIC+DAC"));
  Serial.print(tab);              // prints a tab
  }
  if (i == 149)
  {
  Serial.print(F("Now acquiring value: "));
  Serial.print(k);
  Serial.print(tab);              // prints a tab
  }
 if ((i >= 150) && (i <=293))
  {
  Serial.print((i-150+1000));
  Serial.print(tab);              // prints a tab
  Serial.print(word(EEPROM.read(i-150+0), EEPROM.read(i-150+144)));
  Serial.print(tab);   
  //unsigned int x = EEPROM.read(i-150+288)*256 + EEPROM.read(i-150+432); // not nice but...
  //Serial.print(x);
  Serial.print(word(EEPROM.read(i-150+288), EEPROM.read(i-150+432)));
  Serial.print(tab);              // prints a tab   
  //Serial.print((EEPROM.read(i-150+576)*256 + EEPROM.read(i-150+720)));  // temperature
  //Serial.print(tab);              // prints a tab
  } 
 if (i == 295)
  {
  Serial.print(F("TimeConst = "));
  Serial.print(timeConst);
  Serial.print(F(" sec"));
  Serial.print(tab);              // prints a tab
  }
 if (i == 296)
  {
  Serial.print(F("Prefilter = "));
  Serial.print(filterConst);
  Serial.print(F(" sec "));
  Serial.print(tab);              // prints a tab
  }
 if (i == 297)
  {
  Serial.print(F("Damping = "));
  Serial.print(damping);
  Serial.print(F("/100"));
  Serial.print(tab);              // prints a tab
  } 
 if (i == 298)
  {
  Serial.print(headline);
  Serial.print(tab);              // prints a tab
  }
 if (i == 299)
  {
  Serial.print(verNo); // Could be a string in the beginning of the program
  Serial.print(tab);              // prints a tab
  }
 
 
  Serial.println();      // prints a carriage return
 
 }

void printDataToNewSerial() {
  char buffer1[10];
  char buffer2[10];
  char buffer3[10];
  char buffer4[10];
  char output[40];
  long plotCycle;
  static long lastPlotCycle = 0;
  static boolean lastPPSlocked = false;
  static int oldTIC = TIC_Offset;
  
//update the plot every 86.4 sec so a 1000 point sweep will be one day
  plotCycle = time/864;
  if(plotCycle > lastPlotCycle) {
    itoa(TIC_Value, buffer1, 10);                                         // convert TIC_Value to string
    itoa((TIC_ValueFiltered + filterConst/2)/filterConst, buffer2, 10);   // normalize and convert TIC_ValueFiltered
    itoa(dacValueOut % 200 + 400, buffer3, 10);                             // convert DACValueOut and autorange 400->600 for display
    itoa(StoreTIC_A_Old[j]/10 - 100, buffer4, 10);                        //get yesterday's TIC value and move it down
    sprintf(output, "%s~%s~%s~%s", buffer1, buffer2, buffer3, buffer4);   //generate the sweep
    SMprint("#PLOTSWEEP", output);
    lastPlotCycle++;
  }

  // every 5 minutes if the loop is locked add the average TIC value to the histogram
  if (i==0 && PPSlocked==1) {        
 /*
    if (j==0)
      itoa(StoreTIC_A[143], buffer1, 10);
    
    else 
      itoa(StoreTIC_A[j-1], buffer1, 10);
 */
   // note j has already been bumped so we have to back up one
    itoa(StoreTIC_A[(j+143) % 144], buffer1, 10);
    
    SMprint("#HISTOGRAM", buffer1);
  }
      
  if (time > 105 && abs(timerCountsDiff) > 4) {  //alert if pps interrupt at unexpected time
    ltoa(timerCountsDiff, buffer1, 10);
    ltoa(time, buffer2, 10);
    sprintf(output, "%s~Time jump at %s", buffer1, buffer2);
    SMprint("#ALERT", output);
  }
  
  if (time > 105 && abs(oldTIC - TIC_Value) > 75) {    // alert if a big jump in TICValue
    itoa(oldTIC - TIC_Value, buffer1, 10);
    sprintf(output, "%s~%s", buffer1, "TIC jump");
    SMprint("#ALERT", output);
  }
  oldTIC = TIC_Value;
   
   if (PPSlocked != lastPPSlocked) {              // alert if Lock is established or lost
    if (PPSlocked) SMprint("#ALERT", "100~Locked");
    else SMprint("#ALERT", "100~Lost Lock");
  lastPPSlocked = PPSlocked;
  }
 }

void getCommand()
{
  char ch;
  enum Command {                      //this is the command set
    h = 'h', H = 'H',                 // hold (followed by a DAC value)
    r = 'r', R = 'R'                  // run
  }; 
  
  if (Serial.available() > 0) {         //process if something is there
    ch = Serial.read();
    
    switch(ch) {
      case H:                                // hold command
      case h:
        opMode = hold;
        holdValue = Serial.parseInt();
        //Serial.println(holdValue);
        
        break;
      case R:                                // run command
      case r:
        //Serial.println("Run");
        opMode = run;
        break;
      default:
        Serial.println(F("No valid command"));
        break;
    };
    
    while(Serial.available() > 0) {
      ch = Serial.read();                //flush rest of line
    }
  }
}
    
void setup () {
    Serial.begin(9600);
    Serial.print(headline);
    Serial.print(' ');
    Serial.println(verNo);
    
    //attachInterrupt(0, pps, RISING);

    dac.begin(0x62);                            //initialize DAC
     
    k = EEPROM.read (1023);                     // last index of stored 3 hour averages
    
    // get last stored 3 hour average DAC value
    unsigned int y = EEPROM.read(((k+143) % 144) + 288)*256 + EEPROM.read(((k+143) % 144) + 432);
    
    //if non-zero, use it to set intital dacValue, otherwise keep the default
    if (y != 0)
    {
      dacValueOut = y;
    }
    
    //set initial DAC value and remember it for next power-up of oscillator
    dac.setVoltage(dacValueOut, true);          
   
    //pinMode(led, OUTPUT);
    pinMode(txPin, INPUT);
  
    dacValue = dacValueOut * timeConst;
    //filterConst = timeConst/filterDiv;
    //TIC_ValueOld = TIC_Offset*filterConst;
    //TIC_ValueFiltered = TIC_Offset*filterConst;
    
    analogReference(INTERNAL);
    delay(100);                          //wait for the reference to settle
    TIC_Value = analogRead(A0);          //do some dummy reads to settle the mux
    delay(100);
    TIC_Value = analogRead(A0);
    delay(100);
    
// for new serial monitor
    printf_begin();
    SMprint("#plotsweep", "set~500~100~1000~nsec~Arduino GPSDO~TIC_Value~TIC_ValueFiltered~dacOut~TIC yesterday");
    SMprint("#ALERT", "0~Run Started");
//done with new serial monitor

    cli();          //turn off interrupts while messing with registers 
    // stop Timer 0 so that timer1 not misses counts due to timer0 interrupts
    // note: millis() delay()) etc will not work!
    TCCR0A = 0;             
    TCCR0B = 0;  
    // reset Timer 1 control register
    TCCR1A = 0;             
    TCCR1B = 0;              
    // Timer 1 - counts events on pin D5
    // 5MHz external clock source on T1 pin (D5). Clock on rising edge.
    
    //TIMSK1 = _BV (TOIE1);   // interrupt on Timer 1 overflow only (original)
    TIMSK1 = _BV (TOIE1) | _BV (ICIE1);     // interrupt on Timer 1 overflow or Capture
    
    TCNT1 = 0;              // reset counter 1 to zero
    
    //CS10, 11, 12 select external clock, rising edge
    //ICES1 enables Input Capture
    //ICNC1 enables Noise Canceller (w/ slight delay)
    TCCR1B =  _BV (CS10) | _BV (CS11) | _BV (CS12) | _BV (ICES1) | _BV (ICNC1);
    sei();
}
  
void loop () {
  
  if(PPS_ReadFlag) { //wait for pps interrupt
    //Serial.println("pps detected");
    calculation();
    printDataToSerial();
    getCommand();
    printDataToNewSerial();
    PPS_ReadFlag = false; // Main loop will do nothing until Timer Capture interrupt sets this flag again   
   }
}
