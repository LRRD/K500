# K500
Repository for the Emriver K500 digital flow controller

For the K500 to be reprogrammed with custom hydrographs first find the latest version of our code here: https://github.com/LRRD/K500/blob/master/K500_3-8/K500_3-8.ino 
Install Arduino IDE: https://www.arduino.cc/en/Main/Software
Open the code in Arduino IDE.
I would recommend editing an existing hydrograph rather than creating a new one from scratch unless you're very familiar with Arduino.

To edit existing hydrograph:
Travel to line 171.
Replace numbers in the array with desired flow values (25 - 210 ml/s).
Each number specifies desired flow in 30s increments.
The maximum hydrograph length is 30m (60 array values).
(Optional) 
Change the text displayed for the hydrograph.
Travel to line 887.
Replace text with desired (maximum length of 16 characters).
Travel to line 890.
Replace text with desired (maximum length of 16 characters).

To upload code:
Inside Arduino IDE: Tools -> Board -> Boards Manager -> Arduino megaAVR board -> Install
Tools -> Board -> Arduino megaAVR Boards -> Arduino Nano Every
Tools -> Port
Note attached port numbers.
Plug in K500 using provided cable.
Choose the port number that just appeared.
Upload using the right-pointing arrow or Sketch -> Upload

To create new hydrograph from scratch (advanced):
Insert new line at 177.
const uint8_t newhydrograph[] = { x, y, z...}
Copy lines 883 - 911.
Paste at line 1043.
Replace menu '8' references with '14'.
Replace text with desired.
Replace 'flashy' references with 'newhydrograph'.
Travel to line 138.
Increase menuhigh by 1.

If you wish to increase the maximum hydrograph length you must accept a tradeoff of less frequent flow updates.
Travel to line 397.
Replace 30000 (30s in milliseconds) with a larger number.
