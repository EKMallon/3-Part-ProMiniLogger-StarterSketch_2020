/* A basic datalogger script from the Cave Pearl Project 
that sleeps the datalogger and wakes from DS3231 RTC alarms*/

//This code supports the online build tutorial at: https://thecavepearlproject.org/2019/01/11/pro-mini-logger-project-for-the-classroom-edu-version-2-2019/
//but it will run on any of the Pro Mini dataloggers described at https://thecavepearlproject.org/how-to-build-an-arduino-data-logger/

//updated 20190118 with support for unregulated systems running directly from 2xAA lithium batteries
//updated 20190204 with dynamic preSDsaveBatterycheck safety check
//updated 20190219 with support for using indicator LED as a light sensor
//updated 20190720 with support for DS18b20 Temperature sensor, set to 12-bit
//updated 20190720 with support for tweaking the internal vref constant

#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>     // https://github.com/MrAlvin/RTClib         // Note: there are many other DS3231 libs availiable
#include <LowPower.h>   // https://github.com/rocketscream/Low-Power //for low power sleeping between readings
#include <SdFat.h>      // https://github.com/greiman/SdFat          //needs 512 byte ram buffer!

//============ CONFIGURATION SETTINGS =============================
const char deploymentDetails[] PROGMEM = "Ed's Logger:#23,LED used as sensor,1134880L constant,UTC time set,if found contact: name@email.edu"; 
const char dataCollumnLabels[] PROGMEM = "TimeStamp,Battery(mV),SDsaveDelta(mV),RTCtemp(C),A0(Raw),DS18b20(C),RedLED,GreenLED,BlueLED"; //gets written to second line of datafiles
//change the text in the Details/Labels variables above to suit your configuration
//more information on storing data with the PROGMEM modifier @ http://www.gammon.com.au/progmem

#define SampleIntervalMinutes 1  // Options: 1,2,3,4,5,6,10,12,15,20,30 ONLY (must be a divisor of 60)
                                 // number of minutes the loggers sleeps between each sensor reading

#define ECHO_TO_SERIAL // this enables debugging output to the serial monitor when your logger is powered via USB/UART
                         // comment out this define when you are deploying the logger in the field

//uncomment ONLY ONE of following -> depending on how you are powering your logger
//#define voltageRegulated  // if you connect the battery supply through the Raw & GND pins & use the ProMini's regulator
#define unregulated2xLithiumAA  // define this if you've removed the regulator and are running directly from 2xAA lithium batteries

#define InternalReferenceConstant 1098946L  //used for reading the rail voltage reading
//The "default" value is 1126400L, This assumes the internal vref is perfect 1.1v (i.e. 1100mV times 1024 ADC levels is 1126400)
//but in reality the internal ref. varies by ±10% - to make the Rail/Battery readings more accurate use the CalVref utility from OpenEnergyMonitor
// https://github.com/openenergymonitor/emontx2/blob/master/firmware/CalVref/CalVref.ino to get the constant for your particular Arduino

// note this code assumes you have a common cathode RGB led as the indicator on your logger and are reading all three channels
// however you can read only one LED color channel by disabling the other two color defines here:
#define readRedLEDsensor ON // enabling readLEDsensor define ADDS LED AS A SENSOR readings to the loggers default operation
#define readGreenLEDsensor ON // enabling readLEDsensor define ADDS LED AS A SENSOR readings to the loggers default operation 
#define readBlueLEDsensor ON // enabling readLEDsensor define ADDS LED AS A SENSOR readings to the loggers default operation 
 
#define LED_GROUND_PIN 3 // to use the indicator LED as a light sensor you must ground it through a digital I/O pin from D3 to D7
#define RED_PIN 4   //change these numbers to suit the way you connected the indicator LED
#define GREEN_PIN 5
#define BLUE_PIN 6 
// Note: I always turn on indicator LEDs via INPUT_PULLUP, rather than simply setting the pin to OUTPUT & HIGH,
// this saves power & adds short circuit safety in case the LED was connected without limit resistor - but the light is dim

SdFat sd; /*Create the objects to talk to the SD card*/
SdFile file;
const int chipSelect = 10;    //CableSelect moved to pin 10 in this build

// variables for reading the RTC time & handling the D2=INT(0) alarm interrupt signal it generates
RTC_DS3231 RTC; // creates an RTC object in the code
#define DS3231_I2C_ADDRESS 0x68
#define DS3231_CONTROL_REG 0x0E
#define RTC_INTERRUPT_PIN 2
byte Alarmhour;
byte Alarmminute;
byte Alarmday;
char TimeStamp[ ] = "0000/00/00,00:00"; //16 ascii characters (without seconds because they are always zeros on wakeup)
volatile boolean clockInterrupt = false;  //this flag is set to true when the RTC interrupt handler is executed
float rtc_TEMP_degC;

char FileName[12] = "data000.csv"; //note: this gets updated to a new number if the file aready exists on the SD card
const char codebuild[] PROGMEM = __FILE__;  // loads the compiled source code directory & filename into a varaible
const char compileDate[] PROGMEM = __DATE__; 
const char compileTime[] PROGMEM = __TIME__;

int BatteryReading = 9999; //often read from a 10M/3.3M voltage divider, but could aso be same as VccBGap when unregulated
int safetyMargin4SDsave = 100; // grows dynamically in the code after SD save events - slowly gets larger as batteries age
int systemShutdownVoltage = 2850; // updated later depending on how you power your logger
//if running from 2x AA cells (with no regulator) the input cutoff voltage should be 2850 mV (or higher)
//if running a unit with the voltage regulator the absolute minumum input cutoff voltage is 3400 mV (or higher)
#ifdef voltageRegulated
#define BatteryPin A6  //only used if you have a voltage divider to put the input on A6
#endif

//example variables for analog pin reading
#define analogInputPin A0
int analogPinReading = 0;

//TEMPERATURE Sensors enable if installed
//=======================================
//TEMP Sensor: comment this out if not connected:
#define TS_DS18B20 8    //set this define to the INPUT PIN connected to the sensors DATA wire
// & don't forget you need a 4K7 pullup resistor (joining data line to the high rail) for the DS18b20 to operate properly

#if defined(TS_DS18B20)      // variables for DS18B20 temperature sensor only included if #define TS_DS18B20 exists
#include <OneWire.h>         // this sensor library from  http://www.pjrc.com/teensy/td_libs_OneWire.html
OneWire ds(TS_DS18B20);      
 byte addr[8];
 int ds18b20_TEMP_Raw = 0;
 float ds18b20_TEMP_degC= 0.0;
#endif

//Global variables
//******************
//bool bitBuffer;         // for fuctions that return an on/off true/false state
byte bytebuffer1 = 0;     // for functions that return a byte  //usually I2C comms with sensors
byte bytebuffer2 = 0;     // second buffer for 16-bit registers
//byte bytebuffer3 = 0;      // third buffer for 24-bit registers (MS5803)
int integerBuffer = 9999;    // for temp-swapping ADC readings
int integerBuffer2 = 9999;   // for temp-swapping ADC readings
float floatBuffer=9999.9;    // for temporary float calculations
// char stringBuffer[9];      // for conversion of float values to strings
// eg:  stringBuffer[0] = '\0';dtostrf((b280_hum_act),6,2,stringBuffer); // BME280 Resolution: 0.008 %RH
//      str.print(stringBuffer);str.print(F(",")); //7 chars is max for stringBuffer[8]!

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
//Once the input buffer is disabled, a digitalRead on those A-pins will always be zero.

  #if defined (unregulated2xLithiumAA) || defined(ECHO_TO_SERIAL) // two situations with no voltage on the A6 resistor divider
  systemShutdownVoltage = 2750; // minimum Battery voltage when running from 2x LITHIUM AA's
  #endif
  #ifdef voltageRegulated
  systemShutdownVoltage = 3500; // 3400 is the minimum allowd input to the Mic5205 regulator - alkalines often drop by 200mv or more under load
  #endif
  
  // Setting the SPI pins high helps some sd cards go into sleep mode 
  pinMode(chipSelect, OUTPUT); digitalWrite(chipSelect, HIGH); //ALWAYS pullup the ChipSelect pin with the SD library
  //and you may need to pullup MOSI/MISO, usually MOSIpin=11, and MISOpin=12 if you do not already have hardware pulls
  pinMode(11, OUTPUT);digitalWrite(11, HIGH); //pullup the MOSI pin on the SD card module
  pinMode(12, INPUT_PULLUP); //pullup the MISO pin on the SD card module
  // NOTE: In Mode (0), the SPI interface holds the CLK line low when the bus is inactive, so DO NOT put a pullup on it.
  // NOTE: when the SPI interface is active, digitalWrite() cannot affect MISO,MOSI,CS or CLK

  // 24 second time delay - stabilizes system after power connection
  // the 104 cap on the main battery voltage divider needs > 2s to charge up (on regulated systems)
  // delay also prevents writing multiple file headers with brief accidental power connections
  digitalWrite(BLUE_PIN, LOW); digitalWrite(GREEN_PIN, LOW);digitalWrite(RED_PIN, LOW); 
  #ifdef LED_GROUND_PIN
  digitalWrite(LED_GROUND_PIN, LOW);  //another pin to sink current - depending on the wireing
  pinMode(LED_GROUND_PIN, OUTPUT);   //units using pre-made LED boards sometimes need to set
  #endif
  
  pinMode(RED_PIN,INPUT_PULLUP);     // Using INPUT_PULLUP instead of HIGH lets you connect a raw LED safely
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
  digitalWrite(RED_PIN, LOW);
  pinMode(BLUE_PIN, INPUT_PULLUP);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
  digitalWrite(BLUE_PIN, LOW);
  pinMode(GREEN_PIN, INPUT_PULLUP); //green led is usually 4x as bright as the others
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
  digitalWrite(GREEN_PIN, LOW); 
  pinMode(RED_PIN,INPUT_PULLUP);    // red is usually dimmest color
  
  Serial.begin(9600);    // Open serial communications and wait for port to open:
  Wire.begin();          // start the i2c interface for the RTC
  RTC.begin();  // RTC initialization:
  clearClockTrigger(); //stops RTC from holding the interrupt low after power reset occured
  RTC.turnOffAlarm(1);
  pinMode(RTC_INTERRUPT_PIN,INPUT_PULLUP);  //not needed if you have hardware pullups on SQW line, most RTC modules do but some do not
  DateTime now = RTC.now();
  sprintf(TimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  //enableRTCAlarmsonBackupBattery(); // only needed if you cut the pin supplying power to the DS3231

  #if defined (unregulated2xLithiumAA) || defined(ECHO_TO_SERIAL) 
  BatteryReading=getRailVoltage(); //If you are running from raw battery power (with no regulator) VccBGap IS the battery voltage
  #else  // #ifdef voltageRegulated:
  analogReference(DEFAULT);analogRead(BatteryPin); delay(5);  //throw away the first reading when using high impedance voltage dividers!
  floatbuffer = float(analogRead(BatteryPin));
  floatbuffer = (floatbuffer+0.5)*(3.3/1024.0)*4.030303; // 4.0303 = (Rhigh+Rlow)/Rlow for a 10M/3.3M voltage divider combination
  BatteryReading=int(floatbuffer*1000.0);
  #endif
  
  if (BatteryReading < (systemShutdownVoltage+safetyMargin4SDsave+100)) {
    error(); //if the battery voltage is too low to create a log file, shut down the system
  }

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
// O_CREAT = create the file if it does not exist,  O_EXCL = fail if the file exists, O_WRITE - open for write
  if (!file.open(FileName, O_CREAT | O_EXCL | O_WRITE)) { // note that every system restart will generate new log files!
    for (int i = 1; i < 500; i++) {  // FAT16 has a limit of 512 files entries in root directory
      delay(5);
      snprintf(FileName, sizeof(FileName), "data%03d.csv", i);//concatenates the next number into the filename
      if (file.open(FileName, O_CREAT | O_EXCL | O_WRITE)) 
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
  file.println();file.println((__FlashStringHelper*)deploymentDetails);
  file.println();file.println((__FlashStringHelper*)dataCollumnLabels);
  file.close();delay(5);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
  //Note: SD cards can continue drawing system power for up to 1 second after file close command
  digitalWrite(RED_PIN, LOW);
  
#ifdef ECHO_TO_SERIAL
  Serial.print(F("Data Filename:")); Serial.println(FileName); Serial.println(); Serial.flush();
#endif

//====================================================================
//DS18b20 initialization 
//====================================================================
#ifdef TS_DS18B20
  if ( !ds.search(addr))
  {
    Serial.println(F("ERROR: Did not find the DS18B20 Temp Sensor!"));Serial.flush();
    return;
  }
  else
  { 
    //set the DS18b20 to 12 bit (high resolution) mode
    ds.reset();             // rest 1-Wire
    ds.select(addr);        // select DS18B20
    ds.write(0x4E);         // write on scratchPad
    ds.write(0x00);         // User byte 0 - Unused
    ds.write(0x00);         // User byte 1 - Unused
    ds.write(0x7F);         // set up en 12 bits (0x7F)
    ds.reset();             // reset 1-Wire
    ds.select(addr);        // select DS18B20 
    ds.write(0x48);         // copy scratchpad to EEPROM
    delay(15);              // wait for end of EEPROM write
    
    Serial.print(F("DS18B20 found @ ROM addr:"));
    for (uint8_t i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    Serial.println();Serial.flush();
  }
#endif  //for #ifdef TS_DS18B20

//setting UNUSED digital pins to input pullup reduces noise & risk of accidental short
//D2 = RTC alarm interrupts, D456 = RGB led
//pinMode(7,INPUT_PULLUP); //only if you do not have anything connected to this pin
//pinMode(8,INPUT_PULLUP); //only if you do not have anything connected to this pin
//pinMode(9,INPUT_PULLUP); // are you using this pin for the DS18b20?
#ifndef ECHO_TO_SERIAL
 pinMode(0,INPUT_PULLUP); //but not if we are connected to usb - then these pins are needed for RX & TX 
 pinMode(1,INPUT_PULLUP);
#endif

#ifdef voltageRegulated 
analogReference(DEFAULT);
analogRead(BatteryPin);delay(5);  //throw away the first reading, high impedance divider!
floatbuffer = float(analogRead(BatteryPin));
floatbuffer = (floatbuffer+0.5)*(3.3/1024.0)*4.030303; // 4.0303 = (Rhigh+Rlow)/Rlow for 10M/3.3M resistor combination
BatteryReading=int(floatbuffer*1000.0);
#else
BatteryReading=getRailVoltage(); //If you are running from raw battery power (with no regulator) VccBGap IS the battery voltage
#endif
  
//====================================================================================================
}   //   terminator for setup
//=====================================================================================================


// ========================================================================================================
//      *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
//========================================================================================================

void loop() {
  pinMode(BLUE_PIN,INPUT_PULLUP); 
  DateTime now = RTC.now(); //this reads the time from the RTC
  sprintf(TimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  //loads the time into a string variable - don’t record seconds in the time stamp because the interrupt to time reading interval is <1s, so seconds are always ’00’  
  // We set the clockInterrupt in the ISR, deal with that now:

if (clockInterrupt) {
    if (RTC.checkIfAlarm(1)) {       //Is the RTC alarm still on?
      RTC.turnOffAlarm(1);           //then turn it off.
    }
clockInterrupt = false;                //reset the interrupt flag to false
   
#ifdef ECHO_TO_SERIAL
   Serial.print("RTC Alarm:INT0 at ");  //(optional) debugging message
   Serial.println(TimeStamp);Serial.flush();
#endif

}//=========================end of if (clockInterrupt) =========================
  
// read the RTC temp register - Note: the DS3231 temp registers (11h-12h) are only updated every 64seconds
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);     //the register where the temp data is stored
  Wire.endTransmission(); // nothing actually gets 'written' until the complier hits the .endTransmission command
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);   //ask for two bytes of data
  if (Wire.available()) {
  byte tMSB = Wire.read();            //2’s complement int portion
  byte tLSB = Wire.read();             //fraction portion
  rtc_TEMP_degC = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0;  // Allows for readings below freezing: thanks to Coding Badly
  //rtc_TEMP_degC = (rtc_TEMP_degC * 1.8) + 32.0; // To Convert Celcius to Fahrenheit
}
else {
  rtc_TEMP_degC = 0;  //if rtc_TEMP_degC contains zero, then you had a problem reading from the RTC!
}
#ifdef ECHO_TO_SERIAL
Serial.print(F(" TEMPERATURE from RTC is: "));
Serial.print(rtc_TEMP_degC); // you can just print floats directly like this, but it takes >1K of compiled program memory 
Serial.println(F(" Celsius"));Serial.flush();
#endif

digitalWrite(BLUE_PIN, LOW); //end of RTC communications
pinMode(GREEN_PIN,INPUT_PULLUP); //indicates sensor readings

//============================================================
// Read Analog Input
analogReference(DEFAULT);analogRead(analogInputPin); //always throw away the first reading
delay(5);  //optional 5msec delay lets ADC input cap adjust if needed

//now you can do a single analog reading once with 
analogPinReading = analogRead(analogInputPin);

// OR you can read the analog input line multiple times, and feed those readings into an averaging or smoothing filter
// One of my favorites for getting rid of "single spike" errors from noisy sensor inputs is median3 which takes three values/readings as input
analogPinReading = median_of_3( analogRead(analogInputPin), analogRead(analogInputPin), analogRead(analogInputPin));
//you can use this filter with any sensor that generates only positive integer values

//=====================================
//Read the DS18b20 temperature Sensor:
#ifdef TS_DS18B20    
  ds18b20_TEMP_Raw = readDS18B20Temp();// Note: 750msec of sleep is embedded in this function while waiting for data!
  #ifdef ECHO_TO_SERIAL
  ds18b20_TEMP_degC =(float)ds18b20_TEMP_Raw*0.0625; //many 12 bit sensors use this same calculation
  integerBuffer = (int)ds18b20_TEMP_degC;
  //integerBuffer2= int((ds18b20_TEMP_degC*1000) - abs(integerBuffer*1000)); // Float split into 2 intergers so print funtions dont eat memory  
  integerBuffer2= ((int)(ds18b20_TEMP_degC*1000)%1000); // Float split into 2 intergers so print funtions dont eat memory  
  Serial.print(F("DS18b20 Temp is: "));
  Serial.print(integerBuffer);Serial.print(F("."));Serial.print(integerBuffer2);Serial.println(F(" Celsius"));Serial.flush();
  #endif
#endif
digitalWrite(GREEN_PIN, LOW);
 
//========  Read Light Level with the indicator LED as a sensor =====================================
//===================================================================================================
// this code is modfied from  //https://playground.arduino.cc/Learning/LEDSensor  I added PIND for speed
// with an explaination of the reverse-bias LED reading technique at https://www.sparkfun.com/news/2161
// the readings get smaller as the amount of light increases and the response is logarithmic

long j; //this variable is used for all three LED reading loops
  
#ifdef readRedLEDsensor 
// Prep pin states - discharge any existing capacitance
  digitalWrite(LED_GROUND_PIN,LOW);pinMode(LED_GROUND_PIN,OUTPUT);
  digitalWrite(BLUE_PIN,LOW);pinMode(BLUE_PIN,OUTPUT);
  digitalWrite(GREEN_PIN,LOW);pinMode(GREEN_PIN,OUTPUT);
  digitalWrite(RED_PIN,LOW);pinMode(RED_PIN,OUTPUT);
//READ red channel of LED
  pinMode(BLUE_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(BLUE_PIN,LOW);//channel not being read in input mode
  pinMode(GREEN_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(GREEN_PIN,LOW);//channel not being read in input mode
  pinMode(RED_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(RED_PIN,LOW);
  pinMode(RED_PIN,OUTPUT);//digitalWrite(RED_PIN,LOW);
  pinMode(LED_GROUND_PIN,INPUT_PULLUP);delayMicroseconds(200); //Reverses Polarity on the LED to charge its internal capacitor
  digitalWrite(LED_GROUND_PIN,LOW); //now (GROUND_PIN =sensing pin) is in INPUT mode - listening to the voltage on the LED
  for (j = 0; j < 1000000; j++) {   // Counts how long it takes the LED to fall to the logic 0 voltage level
    if ((PIND & (1 << LED_GROUND_PIN)) == 0) break; // equivalent to: "if (digitalRead(LED_GROUND_PIN)=LOW) stop looping"
    //but PIND uses port manipulation so executes much faster than digitalRead-> increasing the resolution of the sensor
  } 
  long redLEDreading=j; //note that the maximum value here is 1000000 but that is an arbitrary limit (set so the system does not try to count forever)
  
#ifdef ECHO_TO_SERIAL
   Serial.print(F("RedLED= "));
   Serial.print(redLEDreading);
#endif
#endif  //for #ifdef readRedLEDsensor 

#ifdef readGreenLEDsensor   //READ Green channel of LED  
// Prep pin states
  digitalWrite(LED_GROUND_PIN,LOW);pinMode(LED_GROUND_PIN,OUTPUT);
  digitalWrite(BLUE_PIN,LOW);pinMode(BLUE_PIN,OUTPUT);
  digitalWrite(GREEN_PIN,LOW);pinMode(GREEN_PIN,OUTPUT);
  digitalWrite(RED_PIN,LOW);pinMode(RED_PIN,OUTPUT);
// READ green channel of LED
  pinMode(RED_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(RED_PIN,LOW);//channel not being read
  pinMode(BLUE_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(BLUE_PIN,LOW);//channel not being read
  pinMode(GREEN_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(GREEN_PIN,LOW);
  pinMode(GREEN_PIN,OUTPUT);//digitalWrite(GREEN_PIN,LOW);
  pinMode(LED_GROUND_PIN,INPUT_PULLUP);delayMicroseconds(200);//charge led internal capacitor
  digitalWrite(LED_GROUND_PIN,LOW);
  for (j = 0; j < 1000000; j++) {
    if ((PIND & (1 << LED_GROUND_PIN)) == 0) break; 
  }
 long greenLEDreading=j;
 
  #ifdef ECHO_TO_SERIAL
   Serial.print(F("  GreenLED= "));
   Serial.print(greenLEDreading);
  #endif

//READ Blue channel of LED //BLUE=D4, Green=D5, RED=D6
// Prep pin states
  digitalWrite(LED_GROUND_PIN,LOW);pinMode(LED_GROUND_PIN,OUTPUT);
  digitalWrite(BLUE_PIN,LOW);pinMode(BLUE_PIN,OUTPUT);
  digitalWrite(GREEN_PIN,LOW);pinMode(GREEN_PIN,OUTPUT);
  digitalWrite(RED_PIN,LOW);pinMode(RED_PIN,OUTPUT);
// READ blue channel of LED
  pinMode(RED_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(RED_PIN,LOW);//channel not being read
  pinMode(GREEN_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(GREEN_PIN,LOW);//channel not being read
  pinMode(BLUE_PIN,INPUT_PULLUP);delayMicroseconds(200);digitalWrite(BLUE_PIN,LOW);
  pinMode(BLUE_PIN,OUTPUT);//digitalWrite(BLUE_PIN,LOW);
  pinMode(LED_GROUND_PIN,INPUT_PULLUP);delayMicroseconds(200);//charge led internal capacitor
  digitalWrite(LED_GROUND_PIN,LOW);
  for (j = 0; j < 1000000; j++) {
    if ((PIND & (1 << LED_GROUND_PIN)) == 0) break; 
  }
  long blueLEDreading=j;
  #ifdef ECHO_TO_SERIAL
   Serial.print(F("  BlueLED= "));
   Serial.println(blueLEDreading);Serial.flush();
  #endif
#endif // #ifdef readBlueLEDsensor 

//== END OF ======  Read Light Level with the indicator LED as a sensor =============================
//===================================================================================================
  
pinMode(RED_PIN,INPUT_PULLUP);  //saving data to SD card
  
// ========== Pre SD saving battery checks ==========
#if defined (unregulated2xLithiumAA) || defined(ECHO_TO_SERIAL) 
  int preSDsaveBatterycheck=getRailVoltage(); //If you are running from raw battery power (with no regulator) VccBGap IS the battery voltage
  #else  // #ifdef voltageRegulated:
  analogReference(DEFAULT);analogRead(BatteryPin); delay(5);  //throw away the first reading when using high impedance voltage dividers!
  floatbuffer = float(analogRead(BatteryPin));
  floatbuffer = (floatbuffer+0.5)*(3.3/1024.0)*4.030303; // 4.0303 = (Rhigh+Rlow)/Rlow for a 10M/3.3M voltage divider combination
  preSDsaveBatterycheck=int(floatbuffer*1000.0);
  #endif

if (preSDsaveBatterycheck < (systemShutdownVoltage+safetyMargin4SDsave+100)) {
    error(); //shut down the logger because the voltage is too low for SD saving
}

//========== If battery is OK then it's safe to write the data ===========
// The variables saved here should match the order stored in dataCollumnLabels[] at the start of the code
    file.open(FileName, O_WRITE | O_APPEND); // open the file for write at end
    file.print(TimeStamp);
    file.print(","); 
    file.print(BatteryReading);
    file.print(","); 
    file.print(safetyMargin4SDsave);
    file.print(",");  

    //split floats into 2 intergers so print funtions dont eat up your variable memory
    integerBuffer = (int)rtc_TEMP_degC; 
    //integerBuffer2 = int((rtc_TEMP_degC * 100) - abs(integerBuffer * 100)); // only extract two decimal digits here or calc gives x.224 & x.238 error  
    integerBuffer2= ((int)(rtc_TEMP_degC*100.0)%100); 
    file.print(integerBuffer);file.print(".");file.print(integerBuffer2);
    file.print(",");   
    file.print(analogPinReading); 
    file.print(",");   

#ifdef TS_DS18B20
    ds18b20_TEMP_degC = ds18b20_TEMP_Raw * 0.0625; //many 12 bit sensors use this same calculation
    integerBuffer = (int)ds18b20_TEMP_degC;        //whole number portion // The (int) cast from float to int keeps only the integer part
    //integerBuffer2= int((ds18b20_TEMP_degC*1000) - abs(integerBuffer*1000)); //here we are extracting three significant digits
    integerBuffer2= ((int)(ds18b20_TEMP_degC*1000.0)%1000);  //here we are extracting three digits
    //the cast to (int) trims multiplied to exact #of digits you want & then %=modulus(remainder)=the decimal portion // https://stackoverflow.com/questions/499939/extract-decimal-part-from-a-floating-point-number-in-c
    file.print(integerBuffer);file.print(".");file.print(integerBuffer2);
    file.print(",");
#endif
   
#ifdef readRedLEDsensor 
    file.print(redLEDreading);
    file.print(",");
#endif 
#ifdef readGreenLEDsensor   
    file.print(greenLEDreading);
    file.print(","); 
#endif 
#ifdef readBlueLEDsensor   
    file.print(blueLEDreading);
    file.print(","); 
#endif 
    file.println(); 
    file.close();

//========== POST SD saving battery check ===========
//the SD card can pull up to 200mA, and so a more representative battery reading is one taken AFTER this load

  #if defined (unregulated2xLithiumAA) || defined(ECHO_TO_SERIAL) 
  BatteryReading=getRailVoltage(); //If you are running from raw battery power (with no regulator) Rail voltage = battery voltage
  #else  // if voltageRegulated:
  analogReference(DEFAULT);analogRead(BatteryPin); delay(5);  //throw away the first reading when using high impedance voltage dividers!
  floatbuffer = float(analogRead(BatteryPin));
  floatbuffer = (floatbuffer+0.5)*(3.3/1024.0)*4.030303; // 4.0303 = (Rhigh+Rlow)/Rlow for a 10M/3.3M voltage divider combination
  BatteryReading=int(floatbuffer*1000.0);
  #endif

//Note: SD card controllers sometimes generate "internal housekeeping events" that draw MUCH more power from the batteries than normal data saves
//so the value in SDsaveVoltageDelta is usually set by these occasional big power drain events
//this delta increases as your batteries run down, AND if the temperature falls low enough to reduce the amount of power your batteries can delvier
if ((preSDsaveBatterycheck-BatteryReading)>safetyMargin4SDsave) {
  safetyMargin4SDsave= preSDsaveBatterycheck-BatteryReading;
  }
if (BatteryReading < systemShutdownVoltage) { 
    error(); //shut down the logger if you get a voltage reading below the cut-off
  }
digitalWrite(RED_PIN, LOW);  // SD saving is over
pinMode(BLUE_PIN,INPUT_PULLUP); // BLUE to indicate RTC events

// OPTIONAL debugging output: only if ECHO_TO_SERIAL is defined
#ifdef ECHO_TO_SERIAL
    Serial.print("Data Saved: "); 
    Serial.print(TimeStamp);
    Serial.print(","); 
    Serial.print(BatteryReading);
    Serial.print(",");  
    Serial.print(safetyMargin4SDsave);
    Serial.print(", ");    
    Serial.print(analogPinReading);
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

digitalWrite(GREEN_PIN, LOW);digitalWrite(RED_PIN, LOW);digitalWrite(BLUE_PIN, LOW);
  
//============================================================
//   ----- NOW sleep and wait for next RTC wakeup alarm ------
//============================================================
// Enable interrupt on pin2 & attach it to rtcISR function:
  attachInterrupt(0, rtcISR, LOW);
// Enter power down state with ADC module disabled to save power:
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
//processor starts HERE AFTER THE RTC ALARM WAKES IT UP
  detachInterrupt(0); // immediately disable the interrupt on waking
//==============================================================
//now go to the start of the main loop and start the cycle again
} //============================= END of the MAIN LOOP =================================


//====================================================================================
// Stand alone functions called from the main loop:
//====================================================================================
// This is the Interrupt subroutine that only executes when the RTC alarm goes off:
void rtcISR() {                      //called from attachInterrupt(0, rtcISR, LOW);
    clockInterrupt = true;
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
//====================================================================================
// Enable Battery-Backed Square-Wave Enable on the DS3231 RTC module: 
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
  
//========================================================================================
void error(){
    digitalWrite(GREEN_PIN, LOW);digitalWrite(RED_PIN, LOW);digitalWrite(BLUE_PIN, LOW);
    // spend some time flashing red indicator light on error before shutdown!
    for (int CNTR = 0; CNTR < 100; CNTR++) { 
    pinMode(RED_PIN,INPUT_PULLUP);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
    digitalWrite(RED_PIN, LOW);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
  }
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON); //BOD is left on here to protect the processor
}

//====================================================================================
int getRailVoltage()    // from http://forum.arduino.cc/index.php/topic,38119.0.html
{
  int result; //gets passed back to main loop
  int value;  //temp variable for calculation

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

    // changing the ADC reference from default rail/Vcc voltage to internal 1.1V bandgap can take 10 msec
    // due to the fact that the capacitor connected to the aref pin (on the Promini) needs to discharge down to the lower reference level
    ADCSRA |= _BV( ADSC ); //engage Mux settings & throw away the first ADC reading
    while ( ( (ADCSRA & (1 << ADSC)) != 0 ) ); // wait for throw away reading to occur
    delay(3);
  
    for (int i = 0; i < 4; i++) { //loop 4-6 times for consistent results
    ADCSRA |= _BV( ADSC ); // Start an ADC conversion
    while ( ( (ADCSRA & (1 << ADSC)) != 0 ) ); // Wait for it to complete
    value = ADC;
    delay(1);
    }      // terminates for(int i=0; i <= 5; i++) loop

    result = (((InternalReferenceConstant) / (long)value)); //scale the reading into milliVolts

    // post reading cleanup: select input port A0 + re-engage default rail voltage as Aref
    ADMUX = bit (REFS0) | (0 & 0x07); analogRead(A0);  
  
  return result;
  
}  // terminator for getRailVoltage() function

//====================================================================================
// DS18B20  ONE WIRE TEMPERATURE reading function
// this function from library at http://www.pjrc.com/teensy/td_libs_OneWire.html
// also see Dallas Temperature Control library by Miles Burton: http://milesburton.com/Dallas_Temperature_Control_Library

#if defined(TS_DS18B20)
int readDS18B20Temp()
{
  byte data[2]; //byte data[12]; there are more bytes of data to be read...
  ds.reset();
  ds.select(addr);
  ds.write(0x44); // start conversion, read temperature and store it in the scratchpad
  //The time needed between the CONVERT_T command and the READ_SCRATCHPAD command has to be at least
  //750 millisecs (but can be shorter if using a D18B20 set to resolution < 12 bits)
  //if you start getting "85" all the time you did not wait long enough
  // power saving during sleep from http://www.gammon.com.au/forum/?id=11497
  // no need to keep processor awake for that time:
  LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_ON);
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
  delay(6); //regulator stabilization after uC startup
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad
  for (int i = 0; i < 2; i++)
  {
    data[i] = ds.read();
  }
  byte MSB = data[1];
  byte LSB = data[0];
  int tempRaw = ((MSB << 8) | LSB); //using two's compliment //TEMP_degC = tempRaw / 16;
  return tempRaw;
}
#endif

//================================================================================================
//  SIGNAL PROCESSING FUNCTIONS
//================================================================================================
/* 
This median3 filter is pretty good at getting rid of single NOISE SPIKES from flakey sensors
(It is better than any low pass filter, moving average, weighted moving average, etc. 
IN TERMS OF ITS RESPONSE TIME and its ability  to ignore such single-sample noise spike outliers. 
The median-of-3 requires very little CPU power, and is quite fast.
*/
// pass three separate positive integer readings into this filter:
// for more on bitwise xor operator see https://www.arduino.cc/reference/en/language/structure/bitwise-operators/bitwisexor/  
int median_of_3( int a, int b, int c ){  // created by David Cary 2014-03-25
    int the_max = max( max( a, b ), c );
    int the_min = min( min( a, b ), c );
    int the_median = the_max ^ the_min ^ a ^ b ^ c;
    return( the_median );
}                                        // teriminator for median_of_3

/*
// for continuous readings, drop oldest int value and shift in latest reading before calling this function:
oldest = recent;
recent = newest;
newest = analogRead(A0);
*/

//================================================================================================
// NOTE: for more complex signal filtering, look into the digitalSmooth function with outlier rejection
// by Paul Badger at  http://playground.arduino.cc/Main/DigitalSmooth  works well with acclerometers, etc


