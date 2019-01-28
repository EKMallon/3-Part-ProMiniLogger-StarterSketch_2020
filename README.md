<img src="https://github.com/EKMallon/The_Cave_Pearl_Project_CURRENT_codebuilds/blob/master/images/CavePearlProjectBanner_130x850px.jpg">

# Pro-Mini-Datalogger---Basic-Starter-Sketch
A starter script for the Pro Mini based Classroom Data Logger build

This repository contains a basic data logger script that will run on all of the Pro-Mini based "Modules &amp; Jumper Wires" logger described in the Sensors paper: http://www.mdpi.com/1424-8220/18/2/530 but was updated specifically for the 2019  **Pro Mini Logger Project for the Classroom [ EDU v2: 2019 ]** post at:
https://thecavepearlproject.org/2019/01/11/pro-mini-logger-project-for-the-classroom-edu-version-2-2019/

<img src="https://github.com/EKMallon/The_Cave_Pearl_Project_CURRENT_codebuilds/blob/master/images/UNObreadboard_600pix.jpg">

In general you only have to do four things to add a new sensor to this logger base code:

1) Download an #include the library that drives your sensor. This is usually provided by the sensor vendor (Adafruit, Sparkfun, etc) 
2) Connect your sensor as appropriate. The easiest ones to work with are often I2C sensors, which should be connected in parallel with the RTC module (since is also an I2C device)
3) Add commands to take a reading from that sensor and put it into a variable at the beginning of the main loop. This is usually means adding:  YourSensorVariable=readsensor();  with the functions provided by the library
4) In the middle of the code where the data is written to the SD card add:

**file.print(YourNewReading);** // ad this to the series of print statements between file.open & file.close();

**file.print(",");** //add a comma to separate new data - the last line should say file.println

The code then saves all the ascii characters in dataString to the SD card as CSV text. The code automatically creates a new empty data file that is named "dataXXX.csv" on the SD card waiting for that data save. To retrive your data, unplug the battery, and then exchange the SD card with a new one. Then import the CSV files into any spreadsheet like Excel, Google sheets, or Open Office. 

You will find an introduction to the different types of sensors that you can use with the UNO logger at:

**Arduino Tutorial: Adding Sensors to Your Data Logger**
https://thecavepearlproject.org/2017/12/17/adding-sensors-to-an-arduino-data-logger/

If you are using the UNO, check that the sensor you want to use can handle the UNO's 5V positive rail.

**Also note:** that I have a directory of little utility scripts that come in very handy when testing your Arduino based data loggers at: https://github.com/EKMallon/Utilities

