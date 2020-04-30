# JustMyStuff

Contents

ThreeDScan:
===========
* Small Arudino Software project to read data on a Bosch PLR15 laser range finder.
* On the back of the laser range finder the service interface is used to trigger measurements and to read data back. There is a combined Serial Tx/Rx line which is connected to the serial line on arduino. Some level adjustment might be required
* The Arduions software triggers ~ 2measurements in a second and plots the result on a small i2c display
* Further the measured data is printed on the console on the PC
* Idea was to use the PLR15 as laser module to build up 3d scanner with low budget

Brachiograph:
=============
* Project written in python on basis of existing brachiograph software
* My program is adjusted it to use the PCA9685 instead directly driving the servos on the raspi

