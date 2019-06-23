# Maze-Runner-MVP-Firmware
Firmware loaded inside Maze Runner custom RPI hat<br>
This firmware is meant to provide the minimum features needed to have Maze Runner work. THe MVP demo revealed multiple sever shortcomings that are going to be addressed in coming firmware releases.<br>

:<br>
[VIDEO of the Maze Runner MVP Demo in action](https://www.youtube.com/watch?v=rEVTI9Kiidc)
<br>
Features: <br>
1) Uniparser V3 decode an ascii message in the form 'VR%dL%d\0' to set the speed of the left and right servo motors<br>
-13 is the maximum backward speed, +13 is the maximum forward speed


2) LCD library V2 controls a LCD 16x2 display with miminum resourse usage and snappy refresh rate<br>

BUGS:<br>
1) Uniparser V3 has a bug which chrashes the firmware if arguments bigger than 99 are used due to a differing implementation of MALLOC with the ATMEL studio 7 compiler.<br>
2) Something is wrong with the uart channel Hat->RPI. It doesn't matter for the demo.<br>

TODO:<br>
1) Write a Uniparser V4 library to remove dynamic memory allocation and use callbacks<br>
2) fix the uart channel Hat->RPI<br>
3) add bootloader capability<br>
4) upgrade to the latest generation atmel microcontroller AT4809<br>
5) add power profiling options<br>
