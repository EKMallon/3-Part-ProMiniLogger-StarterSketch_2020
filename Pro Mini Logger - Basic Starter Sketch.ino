/* A basic datalogger script from the Cave Pearl Project 
that sleeps the datalogger and wakes from DS3231 RTC alarms*/

//This code supports the online build tutorial at: https://thecavepearlproject.org/2019/01/11/pro-mini-logger-project-for-the-classroom-edu-version-2-2019/
//but it will run on any of the Pro Mini dataloggers described at https://thecavepearlproject.org/how-to-build-an-arduino-data-logger/

//updated 20190118 with better support for running unregulated systems directly from 2xAA lithium batteries
//updated 20190204 with better with dynamically adjusted preSDsaveBatterycheck safety factor
//updated 20190219 with support for using indicator LED as a light sensor

#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>     // https://github.com/MrAlvin/RTClib         // Note: there are many other DS3231 libs availiable
#include <LowPower.h>   // https://github.com/rocketscream/Low-Power //for low power sleeping between readings
#include <SdFat.h>      // https://github.com/greiman/SdFat          //needs 512 byte ram buffer!

//============ CONFIGURATION SETTINGS =============================
#define SampleIntervalMinutes 1  // Options: 1,2,3,4,5,6,10,12,15,20,30,60 ONLY (must be a divisor of 60)
// this is the number of minutes the loggers sleeps between each sensor reading

#define ECHO_TO_SERIAL // this define enables debugging output to the serial monitor when your logger is powered via USB/UART
// comment out this define when you are deploying the logger and running on batteries

//uncomment ONLY ONE of following -> depending on how you are powering your logger
//#define voltageRegulated  // if you connect the power supply through the Raw & GND pins which uses the system regulator
#define unregulated2xLithiumAA  // define this if you've remvoved the regulator from the Pro Mini and are running from 2xAA lithium batteries

#define LED_GROUND_PIN 3 //to use the indicator LED as a light sensor, it must be grounded on Pin D3
#define readLEDsensor ON // enabling readLEDsensor define ADDS LED AS A SENSOR readings to the loggers default operation 
#define RED_PIN 6  //change these numbers to suit your actual connections
#define GREEN_PIN 5
#define BLUE_PIN 4 
// Note: I always turn on indicator LEDs via INPUT_PULLUP, rather than HIGH,
// to save power & add short circuit safety in case an LED is connected without limiter

SdFat sd; /*Create the objects to talk to the SD card*/
SdFile file;
const int chipSelect = 10;    //CableSelect moved to pin 10 in this build

RTC_DS3231 RTC; // creates an RTC object in the code
// variables for reading the RTC time & handling the INT(0) interrupt it generates
#define DS3231_I2C_ADDRESS 0x68
#define DS3231_CONTROL_REG 0x0E
#define RTC_INTERRUPT_PIN 2
byte Alarmhour;
byte Alarmminute;
byte Alarmday;
char CycleTimeStamp[ ] = "0000/00/00,00:00"; //16 ascii characters (without seconds because they are always zeros on wakeup)
volatile boolean clockInterrupt = false;  //this flag is set to true when the RTC interrupt handler is executed
//variables for reading the DS3231 RTC temperature register
float rtc_TEMP_degC;
byte tMSB = 0;
byte tLSB = 0;

char FileName[12] = "data000.csv"; //note: this gets updated to a new number if the file aready exists on the SD card
const char codebuild[] PROGMEM = __FILE__;  // loads the compiled source code directory & filename into a varaible
const char compileDate[] PROGMEM = __DATE__; 
const char compileTime[] PROGMEM = __TIME__;
const char compilerVersion[] PROGMEM = __VERSION__; // https://forum.arduino.cc/index.php?topic=158014.0
const char dataCollumnLabels[] PROGMEM = "TimeStamp,Battery(mV),Rail(mV),SDsaveDelta(mV),RTC Temp(C),A0(Raw),"; //gets written to second line of datafiles
const char noteToSelf[] PROGMEM = "Notes to yourself about your code here."; 

uint16_t VccBGap = 9999;  //the rail voltage is read using the internal 1.1v bandgap
int BatteryReading = 9999; //usually read from the 10M/3.3M voltage divider, but could aso be VccBGap
int preSDsaveBatterycheck = 9999;  //to check that there is enough power before Data saving
uint16_t safetyMargin4SDsave = 100; // updated dynamically in the code after every SD save event
uint16_t systemShutdownVoltage = 2850; // updated later depending on how you power your logger
//if running from 2x AA cells (with no regulator) the input cutoff voltage should be 2850 mV (or higher)
//if running a unit with the voltage regulator the absolute minumum input cutoff voltage is 3400 mV (or higher)

//example variables for analog pin reading
#define analogPinA0 A0
int AnalogPinA0reading = 0;
#define BatteryPin A6


//Global variables
//******************
byte bytebuffer1 =0;
float floatbuffer = 9999.9;
int intbuffer=0;  //not used yet


//======================================================================================================================
//  *  *   *   *   *   *   SETUP   *   *   *   *   *
//======================================================================================================================

void setup() {

// builds that jumper A4->A2 and A5->A3 to bring the I2C bus to the screw terminals MUST DISABLE digital I/O on these two pins
// If you are doing only ADC conversions on some of the analog inputs you can disable the digital buffers on those, to save power
bitSet (DIDR0, ADC0D);  // disable digital buffer on A0
bitSet (DIDR0, ADC1D);  // disable digital buffer on A1
bitSet (DIDR0, ADC2D);  // disable digital buffer on A2
bitSet (DIDR0, ADC3D);  // disable digital buffer on A3
//Once the input buffer is disabled, a digitalRead on those A-pins will always read zero.

  #ifdef LED_GROUND_PIN
  pinMode(LED_GROUND_PIN, OUTPUT);   //units using pre-made LED boards sometimes need to set
  digitalWrite(LED_GROUND_PIN, LOW);  //another pin to sink current - depending on the wireing
  #endif

  #ifdef voltageRegulated
  systemShutdownVoltage = 3600; // 3400 is the minimum allowd input to the Mic5205 regulator - alkalines often drop by 200mv or more under load
  #endif
  
  #if defined (unregulated2xLithiumAA) || defined(ECHO_TO_SERIAL) // two situations with no voltage on the A6 resistor divider
  systemShutdownVoltage = 2850; // minimum Battery voltage when running from 2x LITHIUM AA's
  #endif
  
  // Setting the SPI pins high helps some sd cards go into sleep mode 
  // the following pullup resistors only needs to be enabled for the ProMini builds - not the UNO loggers
  pinMode(chipSelect, OUTPUT); digitalWrite(chipSelect, HIGH); //ALWAYS pullup the ChipSelect pin with the SD library
  //and you may need to pullup MOSI/MISO, usually MOSIpin=11, and MISOpin=12 if you do not already have hardware pulls
  pinMode(11, OUTPUT);digitalWrite(11, HIGH); //pullup the MOSI pin on the SD card module
  pinMode(12, INPUT_PULLUP); //pullup the MISO pin on the SD card module
  // NOTE: In Mode (0), the SPI interface holds the CLK line low when the bus is inactive, so DO NOT put a pullup on it.
  // NOTE: when the SPI interface is active, digitalWrite() cannot affect MISO,MOSI,CS or CLK

  // 24 second time delay
  // this delay also stabilizes system after power connection
  // the cap on the main battery voltage divider needs>2s to charge up 
  // also prevents writing multiple file headers with power connection stutters
  pinMode(BLUE_PIN, OUTPUT);  digitalWrite(BLUE_PIN, LOW);
  pinMode(GREEN_PIN, OUTPUT);  digitalWrite(GREEN_PIN, LOW);
  pinMode(RED_PIN,INPUT_PULLUP); //I use INPUT_PULLUP instead of HIGH as a safety factor in case students connect a RAW led
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  digitalWrite(RED_PIN, LOW);
  pinMode(BLUE_PIN, INPUT_PULLUP);//note that to reduce power, you can also light blue & green LEDs with the pullup
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  digitalWrite(BLUE_PIN, LOW);
  pinMode(GREEN_PIN, INPUT_PULLUP); //green led is 4x as bright as the others...so can light it with pullup resistor to save power
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  digitalWrite(GREEN_PIN, LOW); 
  pinMode(RED_PIN,INPUT_PULLUP); // red is usually dimmest color, so can't use the pullup trick
  
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

  enableRTCAlarmsonBackupBattery(); // this is only needed if you cut the VCC pin supply on the DS3231

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
  
// Find the next availiable file name // from https://learn.adafruit.com/adafruit-feather-32u4-adalogger/using-the-sd-card
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
  file.print((__FlashStringHelper*)noteToSelf);
  file.println();
  file.println();file.print((__FlashStringHelper*)dataCollumnLabels);
#ifdef  readLEDsensor
  file.print(F("RedLED,GreenLED,BlueLED"));
#endif
  file.println();file.close();delay(5);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
  //Note: SD cards can continue drawing system power for up to 1 second after file close command
  digitalWrite(RED_PIN, LOW);
  
#ifdef ECHO_TO_SERIAL
  Serial.print(F("Data Filename:")); Serial.println(FileName); Serial.println(); Serial.flush();
#endif

//setting unused digital pins to input pullup reduces noise & risk of accidental short
//D2 = RTC alarm interrupts, D456 = RGB led
pinMode(7,INPUT_PULLUP); //only if you do not have anything connected to this pin
pinMode(8,INPUT_PULLUP); //only if you do not have anything connected to this pin
pinMode(9,INPUT_PULLUP); //only if you do not have anything connected to this pin
#ifndef ECHO_TO_SERIAL
 pinMode(0,INPUT_PULLUP); //but not if we are connected to usb - then these pins are needed for RX & TX 
 pinMode(1,INPUT_PULLUP);
#endif
pinMode(GREEN_PIN,INPUT_PULLUP); 
  
//====================================================================================================
}   //   terminator for setup
//=====================================================================================================

// ========================================================================================================
//      *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
//========================================================================================================

void loop() {
  pinMode(GREEN_PIN,INPUT_PULLUP); 
  DateTime now = RTC.now(); //this reads the time from the RTC
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  //loads the time into a string variable - don’t record seconds in the time stamp because the interrupt to time reading interval is <1s, so seconds are always ’00’  
  // We set the clockInterrupt in the ISR, deal with that now:

if (clockInterrupt) {
    if (RTC.checkIfAlarm(1)) {       //Is the RTC alarm still on?
      RTC.turnOffAlarm(1);              //then turn it off.
    }
#ifdef ECHO_TO_SERIAL
   Serial.print("RTC Alarm:INT0 at ");  //(optional) debugging message
   Serial.println(CycleTimeStamp);
#endif
    clockInterrupt = false;                //reset the interrupt flag to false
}//=========================end of if (clockInterrupt) =========================
  
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

// Read A0 - You could read in other variables here …like the analog pins, I2C sensors, etc
analogReference(DEFAULT); // cange this to suit your sensors
analogRead(analogPinA0);delay(5);  //delay lets ADC input cap adjust
AnalogPinA0reading = analogRead(analogPinA0);


#ifdef readLEDsensor  
//========  Read Light Level with the indicator LED as a sensor =====================================
//===================================================================================================
// this code is modfied from  //https://playground.arduino.cc/Learning/LEDSensor
// with an explaination of the reverse-bias technique at https://www.sparkfun.com/news/2161
// LED capacitor charges very quickly in 100-200ns so the reversal does not have to be held very long
// The brighter the light, the faster the LED will cause the INPUT to change from HIGH to LOW

//this code assumes D3 is the ground connection of the common cathode LED
//also assumes the color connections are BLUE=D4, Green=D5, RED=D6 per the pre-made 5050 LED module
//I use INPUT_PULLUP to light the LEDs, just for the extra saftey factor in case someone connected a raw LED.

// Prep pin states
pinMode(LED_GROUND_PIN,OUTPUT);digitalWrite(LED_GROUND_PIN,LOW);
pinMode(RED_PIN,OUTPUT);digitalWrite(RED_PIN,LOW);
pinMode(GREEN_PIN,OUTPUT);digitalWrite(GREEN_PIN,LOW);
pinMode(RED_PIN,OUTPUT);digitalWrite(RED_PIN,LOW);

  long j; //used for all three loops
  //READ red channel of LED //BLUE=D4, Green=D5, RED=D6
  pinMode(LED_GROUND_PIN,OUTPUT);digitalWrite(LED_GROUND_PIN,LOW);
  pinMode(BLUE_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(BLUE_PIN,LOW);//channel not being read
  pinMode(GREEN_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(GREEN_PIN,LOW);//channel not being read
  pinMode(RED_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(RED_PIN,LOW);
  pinMode(RED_PIN,OUTPUT);digitalWrite(RED_PIN,LOW);
  pinMode(LED_GROUND_PIN,INPUT_PULLUP);delayMicroseconds(200); //(D3 =sensing pin) Reverses Polarity on the LED to charge its internal capacitor
  digitalWrite(3,LOW); //now D3 is in INPUT mode - listening to the voltage on the D3 pin
  for (j = 0; j < 1000000; j++) { // Counts how long it takes the LED to sink back down to a logic 0 voltage level
    if ((PIND & B00001000) == 0) break; //this is equivalent to: "if (digitalRead(LED_GROUND_PIN)==0) break;"  but much faster
    //using PIND speeds the loop - increasing the sensitivity of the sensor.
  } 
  long redLEDreading=j;
  
#ifdef ECHO_TO_SERIAL
   Serial.print(F("RedLED= "));
   Serial.print(redLEDreading);
#endif

  //READ Green channel of LED   //BLUE=D4, Green=D5, RED=D6
  pinMode(LED_GROUND_PIN,OUTPUT);digitalWrite(LED_GROUND_PIN,LOW);
  pinMode(RED_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(RED_PIN,LOW);//channel not being read
  pinMode(BLUE_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(BLUE_PIN,LOW);//channel not being read
  pinMode(GREEN_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(GREEN_PIN,LOW);
  pinMode(GREEN_PIN,OUTPUT);digitalWrite(GREEN_PIN,LOW);
  pinMode(LED_GROUND_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(LED_GROUND_PIN,LOW);
  for (j = 0; j < 1000000; j++) {
    if ((PIND & B00001000) == 0) break;
  }
 long greenLEDreading=j;
 
  #ifdef ECHO_TO_SERIAL
   Serial.print(F("  GreenLED= "));
   Serial.print(greenLEDreading);
  #endif

//READ Blue channel of LED //BLUE=D4, Green=D5, RED=D6
  pinMode(3,OUTPUT);digitalWrite(3,LOW);
  pinMode(RED_PIN,INPUT_PULLUP);digitalWrite(RED_PIN,LOW);//channel not being read
  pinMode(GREEN_PIN,INPUT_PULLUP);digitalWrite(GREEN_PIN,LOW);//channel not being read
  pinMode(BLUE_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(BLUE_PIN,LOW);
  pinMode(BLUE_PIN,OUTPUT);digitalWrite(BLUE_PIN,LOW);
  pinMode(LED_GROUND_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(LED_GROUND_PIN,LOW);
  for (j = 0; j < 1000000; j++) {
    if ((PIND & B00001000) == 0) break;
  }
 long blueLEDreading=j;
 
  #ifdef ECHO_TO_SERIAL
   Serial.print(F("  BlueLED= "));
   Serial.println(blueLEDreading);Serial.flush();
  #endif

#endif // #ifdef readLEDsensor 

//== END OF ======  Read Light Level with the indicator LED as a sensor =============================
//===================================================================================================


// ========== Pre SD saving battery checks ===========

#ifdef voltageRegulated 
analogReference(DEFAULT);
analogRead(BatteryPin);delay(5);  //throw away the first reading, high impedance divider!
floatbuffer = float(analogRead(BatteryPin));
floatbuffer = (floatbuffer+0.5)*(3.3/1024.0)*4.030303; // 4.0303 = (Rhigh+Rlow)/Rlow for 10M/3.3M resistor combination
preSDsaveBatterycheck=int(floatbuffer*1000.0);
#endif

#if defined(unregulated2xLithiumAA) || defined(ECHO_TO_SERIAL) // two situations with no voltage on the A6 resistor divider
preSDsaveBatterycheck = getRailVoltage(); //If running with no regulator then rail voltage = battery voltage
#endif

if (preSDsaveBatterycheck < (systemShutdownVoltage+safetyMargin4SDsave+100)) {
    error(); //shut down the logger because of low voltage reading
}

//========== If battery is OK then it's safe to write the data ===========
// The variables saved here should match the order stored in dataCollumnLabels[] at the start of the code
file.open(FileName, O_WRITE | O_APPEND); // open the file for write at end
    file.print(CycleTimeStamp);
    file.print(","); 
    file.print(BatteryReading);
    file.print(",");
    file.print(VccBGap);
    file.print(",");  
    file.print(safetyMargin4SDsave);
    file.print(",");  
    file.print(rtc_TEMP_degC);
    file.print(",");    
    file.print(AnalogPinA0reading);
    file.print(",");
#ifdef readLEDsensor 
    file.print(redLEDreading);
    file.print(",");  
    file.print(greenLEDreading);
    file.print(",");    
    file.print(blueLEDreading);
    file.println(",");
#endif   
    file.close();

//========== POST SD saving battery check ===========
//the SD card can pull up to 200mA, and so a more representative battery reading is one taken AFTER this load

#ifdef voltageRegulated 
analogRead(BatteryPin);//throw away the first reading, high impedance divider
floatbuffer = float(analogRead(BatteryPin));
floatbuffer = (floatbuffer+0.5)*(3.3/1024.0)*4.030303 ; // 4.0303 = (Rhigh+Rlow)/Rlow for 10M/3.3M resistor combination
BatteryReading = int(floatbuffer*1000.0);
#endif

VccBGap = getRailVoltage(); //if your logger is regulated the rail voltage should be VERY stable 

#if defined(unregulated2xLithiumAA) || defined(ECHO_TO_SERIAL) // two situations with no voltage on the A6 input
BatteryReading = VccBGap; //If you are running from raw battery power (with no regulator) VccBGap IS the battery voltage
#endif

if (BatteryReading < systemShutdownVoltage) { 
    error(); //shut down the logger if you get a voltage reading below the cut-off
  }

digitalWrite(RED_PIN, LOW); pinMode(BLUE_PIN,INPUT_PULLUP); //I use RED led to indicate SD events, and BLUE to indicate RTC events
  
//SDsaveVoltageDelta safety margin increases every time a larger voltage drop is recorded:
if ((preSDsaveBatterycheck-BatteryReading)>safetyMargin4SDsave) {
  safetyMargin4SDsave= preSDsaveBatterycheck-BatteryReading;
}

//Note: SD card controllers sometimes generate "internal housekeeping events" that draw MUCH more power from the batteries than normal data saves
//so the value in SDsaveVoltageDelta is usually set by these occasional big power drain events
//this delta increases as your batteries run down, AND if the temperature falls low enough to reduce the amount of power your batteries can delvier

#ifdef ECHO_TO_SERIAL  // print to the serial port only if ECHO_TO_SERIAL is defined
    Serial.print("Data Saved: "); 
    Serial.print(CycleTimeStamp);
    Serial.print(","); 
    Serial.print(BatteryReading);
    Serial.print(",");  
    Serial.print(VccBGap);
    Serial.print(",");
    Serial.print(safetyMargin4SDsave);
    Serial.print(",");     
    Serial.print(rtc_TEMP_degC);
    Serial.print(",");    
    Serial.print(AnalogPinA0reading);
#ifdef readLEDsensor 
    Serial.print(",");
    Serial.print(redLEDreading);
    Serial.print(",");  
    Serial.print(greenLEDreading);
    Serial.print(",");    
    Serial.print(blueLEDreading);
#endif   
    Serial.println(","); Serial.flush();
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
  Serial.println(F(" minute(s)"));
  Serial.println();
  Serial.flush();//adds a carriage return & waits for buffer to empty
#endif
}

  //delay(5); //this optional delay is only here so we can see the LED’s otherwise the entire loop executes so fast you might not see it.
digitalWrite(GREEN_PIN, LOW);digitalWrite(RED_PIN, LOW);digitalWrite(BLUE_PIN, LOW);
  //——– sleep and wait for next RTC alarm ————–
  // Enable interrupt on pin2 & attach it to rtcISR function:
attachInterrupt(0, rtcISR, LOW);
  // Enter power down state with ADC module disabled to save power:
LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  //processor starts HERE AFTER THE RTC ALARM WAKES IT UP
detachInterrupt(0); // immediately disable the interrupt on waking
//Interupt woke processor, now go back to the start of the main loop
} //============================= END of the MAIN LOOP =================================

// This is the Interrupt subroutine that only executes when the RTC alarm goes off:
void rtcISR() {
    clockInterrupt = true;
  }

//====================================================================================
// Enable Battery-Backed Square-Wave Enable on the RTC module: 
/* Bit 6 (Battery-Backed Square-Wave Enable) of DS3231_CONTROL_REG 0x0E, can be set to 1 
 * When set to 1, it forces the wake-up alarms to occur when running the RTC from the back up battery alone. 
 * [note: This bit is usually disabled (logic 0) when power is FIRST applied]
 */
  void enableRTCAlarmsonBackupBattery(){
  Wire.beginTransmission(DS3231_I2C_ADDRESS);// Attention RTC 
  Wire.write(DS3231_CONTROL_REG);            // move the memory pointer to CONTROL_REG
  Wire.endTransmission();                    // complete the ‘move memory pointer’ transaction
  Wire.requestFrom(DS3231_I2C_ADDRESS,1);    // request data from register
  byte resisterData = Wire.read();           // byte from registerAddress
  bitSet(resisterData, 6);                   // Change bit 6 to a 1 to enable
  Wire.beginTransmission(DS3231_I2C_ADDRESS);// Attention RTC
  Wire.write(DS3231_CONTROL_REG);            // target the register
  Wire.write(resisterData);                  // put changed byte back into CONTROL_REG
  Wire.endTransmission();
  }

//====================================================================================
void clearClockTrigger()   // from http://forum.arduino.cc/index.php?topic=109062.0
{
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231
  Wire.write(0x0F);               //Tell the device which address we want to read or write
  Wire.endTransmission();         //Before you can write to and clear the alarm flag you have to read the flag first!
  Wire.requestFrom(0x68,1);       //Read one byte
  bytebuffer1=Wire.read();        //In this example we are not interest in actually using the byte
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231 
  Wire.write(0x0F);               //Status Register: Bit 3: zero disables 32kHz, Bit 7: zero enables the main oscilator
  Wire.write(0b00000000);         //Bit1: zero clears Alarm 2 Flag (A2F), Bit 0: zero clears Alarm 1 Flag (A1F)
  Wire.endTransmission();
  clockInterrupt=false;           //Finally clear the flag we used to indicate the trigger occurred
}
//========================================================================================
void error(){
    digitalWrite(GREEN_PIN, LOW);digitalWrite(RED_PIN, LOW);digitalWrite(BLUE_PIN, LOW);
    //spend some time flashing red indicator light on error before shutdown!
    for (int CNTR = 0; CNTR < 50; CNTR++) { 
    pinMode(RED_PIN,INPUT_PULLUP);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
    digitalWrite(RED_PIN, LOW);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
  }
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON); //note that the BOD is on here because our batteries are LOW
}

//====================================================================================
int getRailVoltage()    // from http://forum.arduino.cc/index.php/topic,38119.0.html
{
  int result; 
  const long InternalReferenceVoltage = 1100L; 
  //note your can read the band-gap voltage & set this value more accurately: https://forum.arduino.cc/index.php?topic=38119.0

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



