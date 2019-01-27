/* A basic datalogger script from the Cave Pearl Project 
that sleeps the datalogger and wakes from DS3231 RTC alarms*/

//This code to support the online build tutorial at: 
// https://thecavepearlproject.org/2019/01/11/pro-mini-logger-project-for-the-classroom-edu-version-2-2019/
//but it will run on any of the Pro Mini dataloggers described at 
// https://thecavepearlproject.org/how-to-build-an-arduino-data-logger/

//updated 20190118 with better support for running unregulated systems directly from 2xAA lithium batteries

#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>     // library from https://github.com/MrAlvin/RTClib  Note: there are many other DS3231 libs availiable
#include <LowPower.h>   // https://github.com/rocketscream/Low-Power and https://github.com/rocketscream/Low-Power 
#include <SdFat.h>      // needs 512 byte ram buffer! see https://github.com/greiman/SdFat

#define SampleIntervalMinutes 15  // Whole numbers 1-30 only, must be a divisor of 60
// this is the number of minutes the loggers sleeps between each sensor reading

//#define ECHO_TO_SERIAL // this define to enable debugging output to the serial monitor
//comment out this define in for field deployments where you have no USB cable connected

//uncomment ONLY ONE OF THESE TWO! - depending on how you are powering your logger
#define voltageRegulated  // if you connect the power supply through the Raw & GND pins which uses the system regulator
//#define unregulatedOperation  // if you've remvoved the regulator from the Pro Mini and are running from 2XAA lithium batteries

SdFat sd; /*Create the objects to talk to the SD card*/
SdFile file;
const int chipSelect = 10;    //CS moved to pin 10 on the arduino

RTC_DS3231 RTC; // creates an RTC object in the code
// variables for reading the RTC time & handling the INT(0) interrupt it generates
#define DS3231_I2C_ADDRESS 0x68
#define RTC_INTERRUPT_PIN 2
byte Alarmhour;
byte Alarmminute;
byte Alarmday;
char CycleTimeStamp[ ] = "0000/00/00,00:00"; //16 ascii characters (without seconds)
volatile boolean clockInterrupt = false;  //this flag is set to true when the RTC interrupt handler is executed
//variables for reading the DS3231 RTC temperature register
float rtc_TEMP_degC;
byte tMSB = 0;
byte tLSB = 0;

char FileName[12] = "data000.csv"; 
const char codebuild[] PROGMEM = __FILE__;  // loads the compiled source code directory & filename into a varaible
const char compileDate[] PROGMEM = __DATE__; 
const char compileTime[] PROGMEM = __TIME__;
const char compilerVersion[] PROGMEM = __VERSION__; // https://forum.arduino.cc/index.php?topic=158014.0
const char dataCollumnLabels[] PROGMEM = "Time Stamp,Battery(mV),RTC temp(C),Rail(mV),AnalogRead(Raw),"; //gets written to second line of datafiles

//track the rail voltage using the internal 1.1v bandgap trick
uint16_t VccBGap = 9999; 
uint16_t systemShutdownVoltage = 2850; 
//if running of of 2x AA cells (with no regulator) the input cutoff voltage should be 2850 mV (or higher)
//if running a unit with the voltage regulator the input cutoff voltage should be 3400 mV

//example variables for analog pin reading
int analogPin = A0;
int AnalogReading = 0;
int BatteryPin = A6;
int BatteryReading = 9999;
float batteryVoltage = 9999.9;
int preSDsaveBatterycheck = 0;
//Global variables
//******************
byte bytebuffer1 =0;
//byte bytebuffer2 =0;
//int intbuffer=0;  //not used yet

//indicator LED pins - change these defs to suit actual
#define RED_PIN 4
#define GREEN_PIN 5
#define BLUE_PIN 6 
#define LED_GROUND_PIN 3 //some loggers need a pin grounded for pre-made led modules - but not if LED has true GND connect


//======================================================================================================================
//  *  *   *   *   *   *   SETUP   *   *   *   *   *
//======================================================================================================================

void setup() {
 
  DIDR0 = 0x0F;  // disables digital input on analog lines 0..3 (analog 4&5 are used for I2C bus)
  //important for builds that jumper A4->A2 and A5->A3 to bring the I2C bus to the side screw terminals 

  #ifdef LED_GROUND_PIN
  pinMode(LED_GROUND_PIN, OUTPUT);   //units using pre-made LED boards sometimes need to set
  digitalWrite(LED_GROUND_PIN, LOW);  //another pin to sink current - depending on the wireing
  #endif

  #ifdef unregulatedOperation
  systemShutdownVoltage = 2850; // minimum Battery voltage when running from 2xAA's
  #endif

  #ifdef voltageRegulated
  systemShutdownVoltage = 3500; // minimum Battery voltage when running from 3 or 4 AA's supplying power to the Mic5205 regulator
  #endif
  
  // Setting the SPI pins high helps some sd cards go into sleep mode 
  // the following pullup resistors only needs to be enabled for the ProMini builds - not the UNO loggers
  pinMode(chipSelect, OUTPUT); digitalWrite(chipSelect, HIGH); //ALWAYS pullup the ChipSelect pin with the SD library
  //and you may need to pullup MOSI/MISO, usually MOSIpin=11, and MISOpin=12 if you do not already have hardware pulls
  pinMode(11, OUTPUT);digitalWrite(11, HIGH); //pullup the MOSI pin on the SD card module
  pinMode(12, INPUT_PULLUP); //pullup the MISO pin on the SD card module
  // NOTE: In Mode (0), the SPI interface holds the CLK line low when the bus is inactive, so DO NOT put a pullup on it.
  // NOTE: when the SPI interface is active, digitalWrite() cannot affect MISO,MOSI,CS or CLK

  // 24 second time delay here to compile & upload if you just connected the UART
  // this delay also stabilizes system after power connection - the cap on the main battery voltage divider needs>2s to charge up 
  // also prevents writing multiple file headers
  pinMode(BLUE_PIN, OUTPUT);  digitalWrite(BLUE_PIN, LOW);
  pinMode(GREEN_PIN, OUTPUT);  digitalWrite(GREEN_PIN, LOW);
  pinMode(RED_PIN, OUTPUT);  digitalWrite(RED_PIN, HIGH);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  digitalWrite(BLUE_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  digitalWrite(GREEN_PIN, LOW); 
  digitalWrite(RED_PIN, HIGH); 
  
  Serial.begin(9600);    // Open serial communications and wait for port to open:
  Wire.begin();          // start the i2c interface for the RTC
  TWBR = 2;//speeds up I2C bus to 400 kHz bus - ONLY Use this on 8MHz Pro Mini's
  // and remove TWBR = 2; if you have sensor reading problems on the I2C bus
  // onboard AT24c256 eeprom also ok @ 400kHz http://www.atmel.com/Images/doc0670.pdf  

  pinMode(RTC_INTERRUPT_PIN,INPUT_PULLUP);// RTC alarms low, so need pullup on the D2 line 
  //Note using the internal pullup is not needed if you have hardware pullups on SQW line, and most RTC modules do.
  RTC.begin();  // RTC initialization:
  clearClockTrigger(); //stops RTC from holding the interrupt low after power reset occured
  RTC.turnOffAlarm(1);
  DateTime now = RTC.now();
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());

#ifdef ECHO_TO_SERIAL
  Serial.println(F("Initializing SD card..."));
#endif
// print lines in the setup loop only happen once
// see if the card is present and can be initialized
  if (!sd.begin(chipSelect,SPI_FULL_SPEED)) {   // some cards may need SPI_HALF_SPEED
    #ifdef ECHO_TO_SERIAL
    Serial.println(F("Card failed, or not present"));
    Serial.flush();
    #endif
    error(); //if you cant initialise the SD card, you can't save any data - so shut down the logger
    return;
  }
#ifdef ECHO_TO_SERIAL
  Serial.println(F("card initialized."));
#endif
  delay(50); //sd.begin hits the power supply pretty hard
  
// Find the next availiable file name
  // 2 GB or smaller cards should be used and formatted FAT16 - FAT16 has a limit of 512 files entries in root
  // O_CREAT = create the file if it does not exist,  O_EXCL = fail if the file exists, O_WRITE - open for write
  if (!file.open(FileName, O_CREAT | O_EXCL | O_WRITE)) { // note that restarts often generate empty log files!
    for (int i = 1; i < 512; i++) {
      delay(5);
      snprintf(FileName, sizeof(FileName), "data%03d.csv", i);//concatenates the next number into the filename
      if (file.open(FileName, O_CREAT | O_EXCL | O_WRITE)) // O_CREAT = create file if not exist, O_EXCL = fail if file exists, O_WRITE - open for write
      {
        break; //if you can open a file with the new name, break out of the loop
      }
    }
  }
  delay(25);
  //write the header information in the new file
  file.print(F("Filename:"));
  file.println((__FlashStringHelper*)codebuild); // writes the entire path + filename to the start of the data file
  file.print(F("Compiled:,"));
  file.print((__FlashStringHelper*)compileDate);
  file.print(F(","));
  file.print((__FlashStringHelper*)compileTime);
  file.print(F(","));
  file.print((__FlashStringHelper*)compilerVersion);
  file.println();
  file.println();file.println((__FlashStringHelper*)dataCollumnLabels);
  file.close(); delay(5);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
  //Note: SD cards can continue drawing system power for up to 1 second after file close command
  digitalWrite(RED_PIN, LOW);
  
#ifdef ECHO_TO_SERIAL
  Serial.print(F("Data Filename:")); Serial.println(FileName); Serial.println(); Serial.flush();
#endif

  DIDR0 = 0x0F;  // This disables the digital inputs on analog lines 0..3 (analog 4&5 are used for I2C bus)

//setting any unused digital pins to input pullup reduces noise & risk of accidental short
//D2 = RTC alarm interrupts, D456 = RGB led
//pinMode(3,INPUT_PULLUP); //only if you do not have anything connected to this pin
pinMode(8,INPUT_PULLUP); //only if you do not have anything connected to this pin
pinMode(9,INPUT_PULLUP); //only if you do not have anything connected to this pin
#ifndef ECHO_TO_SERIAL
 pinMode(0,INPUT_PULLUP); //but not if we are on usb - then these pins are needed for RX & TX 
 pinMode(1,INPUT_PULLUP);
#endif
digitalWrite(GREEN_PIN, HIGH); 
  
//====================================================================================================
}   //   terminator for setup
//=====================================================================================================

// ========================================================================================================
//      *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
//========================================================================================================

void loop() {
  digitalWrite(GREEN_PIN, HIGH); 
  DateTime now = RTC.now(); //this reads the time from the RTC
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  //loads the time into a string variable - don’t record seconds in the time stamp because the interrupt to time reading interval is <1s, so seconds are always ’00’  
  // We set the clockInterrupt in the ISR, deal with that now:
  if (clockInterrupt) {
    if (RTC.checkIfAlarm(1)) {       //Is the RTC alarm still on?
      RTC.turnOffAlarm(1);              //then turn it off.
    }
#ifdef ECHO_TO_SERIAL
   Serial.print("RTC Alarm on INT-0 triggered at ");  //(optional) debugging message
   Serial.println(CycleTimeStamp);
#endif
    clockInterrupt = false;                //reset the interrupt flag to false
  }//—————————————————————–
  // read the RTC temp register - Note: the DS3231 temp registers (11h-12h) are only updated every 64seconds
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);                     //the register where the temp data is stored
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);   //ask for two bytes of data
  if (Wire.available()) {
  tMSB = Wire.read();            //2’s complement int portion
  tLSB = Wire.read();             //fraction portion
  rtc_TEMP_degC = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0;  // Allows for readings below freezing: thanks to Coding Badly
  //rtc_TEMP_degC = (rtc_TEMP_degC * 1.8) + 32.0; // To Convert Celcius to Fahrenheit
}
else {
  rtc_TEMP_degC = 0;  //if rtc_TEMP_degC contains zero, then you had a problem reading from the RTC!
}
#ifdef ECHO_TO_SERIAL
Serial.print(F(" TEMPERATURE from RTC is: "));
Serial.print(rtc_TEMP_degC);
Serial.println(F(" Celsius"));
#endif

// You could read in other variables here …like the analog pins, I2C sensors, etc
analogReference(DEFAULT);
AnalogReading = analogRead(analogPin);delay(1);  //throw away the first reading
AnalogReading = analogRead(analogPin);

//========== Pre SD saving battery check ===========
#ifdef unregulatedOperation 
preSDsaveBatterycheck = getRailVoltage(); //If running with no regulator then vcc = the battery voltage
if (preSDsaveBatterycheck < (systemShutdownVoltage+100)) { //on older lithium batteries the SD save can cause a 100mv drop
    error(); //shut down the logger because of low voltage reading
}
#endif

#ifdef voltageRegulated 
analogRead(BatteryPin);delay(5);  //throw away the first reading, high impedance divider
batteryVoltage = float(analogRead(BatteryPin));
batteryVoltage = (batteryVoltage+0.5)*(3.3/1024.0)*4.030303; // 4.0303 = (Rhigh+Rlow)/Rlow for 10M/3.3M resistor combination
preSDsaveBatterycheck=int(batteryVoltage*1000.0);
if (preSDsaveBatterycheck < (systemShutdownVoltage+300)) {  //on old alkaline batteries the 100-200mA SD save events can cause a 300mv drop
    error(); //shut down the logger because of low voltage reading
}
#endif

digitalWrite(GREEN_PIN, LOW); digitalWrite(RED_PIN, HIGH); 
//========== Battery Good? then write the data to the SD card ===========
file.open(FileName, O_WRITE | O_APPEND); // open the file for write at end like the Native SD library
    delay(20);
    file.print(CycleTimeStamp);
    file.print(","); 
    file.print(BatteryReading);
    file.print(",");    
    file.print(rtc_TEMP_degC);
    file.print(",");    
    file.print(VccBGap);
    file.print(",");
    file.print(AnalogReading);
    file.println(",");
    file.close();

//========== POST SD saving battery check ===========
//the SD card can pull up to 200mA, and so more representative battery readings are those taken after this load
#ifdef unregulatedOperation  /If you are running from raw battery power (with no regulator) vcc = the battery voltage
VccBGap = getRailVoltage(); //takes this reading immediately directly after the SD save event
BatteryReading = VccBGap;
  if (VccBGap < systemShutdownVoltage) { 
    error(); //shut down the logger because of low voltage reading
  }
#endif

#ifdef voltageRegulated 
analogReference(DEFAULT);
AnalogReading = analogRead(BatteryPin);delay(5);  //throw away the first reading, high impedance divider
analogRead(BatteryPin);delay(5);  //throw away the first reading, high impedance divider
batteryVoltage = float(analogRead(BatteryPin));
batteryVoltage = (batteryVoltage+0.5)*(3.3/1024.0)*4.030303 ; // 4.0303 = (Rhigh+Rlow)/Rlow for 10M/3.3M resistor combination
BatteryReading = int(batteryVoltage*1000.0);

if (BatteryReading < systemShutdownVoltage) { 
    error(); //shut down the logger because of low voltage reading
}

VccBGap = getRailVoltage(); //if your system is regulated, take this reading after the main battery - it should be very stable!
#endif

digitalWrite(RED_PIN, LOW); digitalWrite(BLUE_PIN, HIGH); 

#ifdef ECHO_TO_SERIAL  // print to the serial port too:
    Serial.print(CycleTimeStamp);
    Serial.print(","); 
    Serial.print(BatteryReading);
    Serial.print(",");      
    Serial.print(rtc_TEMP_degC);
    Serial.print(",");    
    Serial.print(VccBGap);
    Serial.print(",");
    Serial.println(AnalogReading);
    Serial.flush();
#endif

//============Set the next alarm time =============
Alarmhour = now.hour();
Alarmminute = now.minute() + SampleIntervalMinutes;
Alarmday = now.day();

// check for roll-overs
if (Alarmminute > 59) { //error catching the 60 rollover!
  Alarmminute = 0;
  Alarmhour = Alarmhour + 1;
  if (Alarmhour > 23) {
    Alarmhour = 0;
    // put ONCE-PER-DAY code here -it will execute on the 24 hour rollover
  }
}
// then set the alarm
RTC.setAlarm1Simple(Alarmhour, Alarmminute);
RTC.turnOnAlarm(1);
if (RTC.checkAlarmEnabled(1)) {
  //you would comment out most of this message printing
  //if your logger was actually being deployed in the field

#ifdef ECHO_TO_SERIAL
  Serial.print(F("RTC Alarm Enabled!"));
  Serial.print(F(" Going to sleep for : "));
  Serial.print(SampleIntervalMinutes);
  Serial.println(F(" minutes"));
  Serial.println();
  Serial.flush();//adds a carriage return & waits for buffer to empty
#endif
}

  //delay(5); //this optional delay is only here so we can see the LED’s otherwise the entire loop executes so fast you might not see it.
  digitalWrite(GREEN_PIN, LOW);digitalWrite(RED_PIN, LOW);digitalWrite(BLUE_PIN, LOW);
  // Note: Normally you would NOT leave a red indicator LED on during sleep! This is just so you can see when your logger is sleeping, & when it's awake
  //digitalWrite(RED_PIN, HIGH);  // Turn on red led as our indicator that the Arduino is sleeping. Remove this before deployment.
  //——– sleep and wait for next RTC alarm ————–
  // Enable interrupt on pin2 & attach it to rtcISR function:
attachInterrupt(0, rtcISR, LOW);
  // Enter power down state with ADC module disabled to save power:
LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  //processor starts HERE AFTER THE RTC ALARM WAKES IT UP
detachInterrupt(0); // immediately disable the interrupt on waking
//Interupt woke processor, now go back to the start of the main loop
}
//============================= END of the MAIN LOOP =================================

// This is the Interrupt subroutine that only executes when the RTC alarm goes off:
void rtcISR() {
    clockInterrupt = true;
  }
//====================================================================================
void clearClockTrigger()    // from http://forum.arduino.cc/index.php?topic=109062.0
{
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231
  Wire.write(0x0F);               //Tell the device which address we want to read or write
  Wire.endTransmission();         //Before you can write to and clear the alarm flag you have to read the flag first!
  Wire.requestFrom(0x68,1);       //Read one byte
  bytebuffer1=Wire.read();        //In this example we are not interest in actually using the bye
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231 
  Wire.write(0x0F);               //status register
  Wire.write(0b00000000);         //Write the byte.  The last 0 bit resets Alarm 1 //is it ok to just set these all to zeros?
  Wire.endTransmission();
  clockInterrupt=false;           //Finally clear the flag we use to indicate the trigger occurred
}
//====================================================================================
int getRailVoltage()    // from http://forum.arduino.cc/index.php/topic,38119.0.html
{
  int result; 
  const long InternalReferenceVoltage = 1100L; //note your can read the band-gap voltage & set this value more accurately: https://forum.arduino.cc/index.php?topic=38119.0

  for (int i = 0; i < 4; i++) { // have to loop at least 4 times before it yeilds consistent results - the cap on aref needs to settle

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // For mega boards
    // const long InternalReferenceVoltage = 1100L;  // Adjust this value to your boards specific internal BG voltage x1000
    // REFS1 REFS0          --> 0 1, AVcc internal ref.
    // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#else
    // For 168/328 boards
    // const long InternalReferenceVoltage = 1100L;
    // Adust this value to your boards specific internal BG voltage x1000
    // REFS1 REFS0          --> 0 1, AVcc internal ref.
    // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
#endif
    delay(1);  // voltage droop/recover from pulse load in lithiums takes >10ms, so this delay is ok  http://data.energizer.com/pdfs/l91.pdf
    // Start a conversion
    ADCSRA |= _BV( ADSC );
    // Wait for it to complete
    while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );
    // Scale the value
    result = (((InternalReferenceVoltage * 1023L) / ADC) + 5L); //scale Rail voltage in mV
    // note that you can tune the accuracy of this function by changing InternalReferenceVoltage to match your board
    // just tweak the constant till the reported rail voltage matches what you read with a DVM!
  }     // end of for (int i=0; i < 4; i++) loop

  ADMUX = bit (REFS0) | (0 & 0x07); analogRead(A0); // cleanup: select input port A0 + engage new Aref at rail voltage

//  if (result < LowestVcc) {LowestVcc = result;} //as the batteries loose capacity, the delta btween these two numbers increases
//  if (result > HighestVcc) {HighestVcc = result;}
  
  return result;
  
}  // terminator for getRailVoltage()

//========================================================================================
void error(){
    digitalWrite(GREEN_PIN, LOW);digitalWrite(RED_PIN, LOW);digitalWrite(BLUE_PIN, LOW);
    for (int CNTR = 0; CNTR < 50; CNTR++) { //spend some time flashing red indicator light on error before shutdown!
    digitalWrite(RED_PIN, HIGH);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
    digitalWrite(RED_PIN, LOW);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
  }
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON); //note that the BOD is on here
}


