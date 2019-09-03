<img src="https://github.com/EKMallon/The_Cave_Pearl_Project_CURRENT_codebuilds/blob/master/images/CavePearlProjectBanner_130x850px.jpg">

# Pro Mini Datalogger - Basic Starter Sketch
A starter script for the 2019 Cave Pearl Project Classroom Logger build

This repository contains a basic data logger script that will run on all of the Pro-Mini based "Modules &amp; Jumper Wires" loggers described in the Sensors paper: http://www.mdpi.com/1424-8220/18/2/530 but was updated specifically for the 2019  **Pro Mini Logger Project for the Classroom [ EDU v2: 2019 ]** post at:
https://thecavepearlproject.org/2019/01/11/pro-mini-logger-project-for-the-classroom-edu-version-2-2019/

<img src="https://github.com/EKMallon/Pro-Mini-Datalogger---Basic-Starter-Sketch/blob/master/images/_CavePearl_ClassroomLogger_2019_640pix.jpg">

In general you only have to do four things to add a new sensor to this logger base code:

1) Download and #include the library that drives your sensor. This is usually provided by the sensor vendor (Adafruit, Sparkfun, etc) but there a plenty of other sources if you google it. 
2) Connect your sensor as appropriate. I2C sensors are often the easiest ones to work with, and should be connected in parallel with the RTC module (since is also an I2C device)
3) Add commands to take a reading from that sensor and store it into a variable at the beginning of the main loop. This is usually means adding something like: int YourSensorVariable=readsensor();  with whatever functions are provided by the library
4) In the middle of the code where the data is written to the SD card add:

**file.print(YourSensorVariable);** // add this to the series of print statements between file.open & file.close();

**file.println(",");** //add a comma to separate data varaibles - only the last line should use .println

The code then saves all the ascii characters to the SD card as CSV text. This script automatically creates a new empty data file that is named "dataXXX.csv" on the SD card at startup. To retrive your data, unplug the battery, and then exchange the SD card with a new one. Then import the CSV files into any spreadsheet like Excel, Google sheets, or Open Office. 

You will find an introduction to the different types of sensors that you can use with the logger at:

**Arduino Tutorial: Adding Sensors to Your Data Logger**
https://thecavepearlproject.org/2017/12/17/adding-sensors-to-an-arduino-data-logger/

**Also note:** that I have a directory of little utility scripts that come in very handy when testing your Arduino based data loggers at: https://github.com/EKMallon/Utilities

<img src="https://github.com/EKMallon/Pro-Mini-Datalogger---Basic-Starter-Sketch/blob/master/images/LeafTransmittanceTrials@UTJ_300pixw.jpg" align="right">
For a sense of what you can do with the classroom logger, heres a post about creating a custom ‘Leaf Transmittance Index’ based on readings from an IR LED and the red channel of the RGB indicator already on the logger: https://thecavepearlproject.org/2019/08/30/creating-a-normalized-vegetation-index-sensor-with-two-leds/ Although using generic LED’s introduces non-optimal aspects wrt frequency & bandwidth, the trial successfully distinguished ‘healthy’ vs ‘unhealthy’ plant leaves where a simple visual inspection could not. Creating a pseudo NDVI with transmittance rather than reflectance bends the rules a bit: but keep in mind this was meant from the start to be a conceptual proof of concept, rather than a rigorous tool. From a teaching point of view, students would have to create "custom calibrations" for their one-of-a-kind LED combination & for each plant species they work on - but that's actually a useful teaching exercise in and of itself. This fun little hack really has legs in that educational context, and it wouldn't take much to tune the idea to a more specific application with better LED selection.
