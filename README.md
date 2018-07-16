# sigfox_examples
Custom made GPS tracker and accelerometer data examples for low-powered Sigfox module TD1205P.

## Used information

* Telecom Design SDK given for their Sigfox modules - https://github.com/Telecom-Design
* Basics and message formating from this tutorial - http://www.instructables.com/id/Sigfox-GPS-Tracker/

## General Message Fromat
  In all of the examples is used 12 bytes message format for transceiving the information from the module(GPS and accelerometer data). Message is formed by using some of the code in above tutorial:
  4 bytes - GPS latitude
  4 bytes - GPS longitude
  1 byte - Voltage
  1 byte - Temperature
  2 bytes - Free or used to tranceive the minimum value of one of the accelerometer axis  

## Example acc_calc.c
##### Calculates and prints minimum, maximum, the average and standard deviation of the accelerometer data for x, y, z at given interval
### Commands
* AT$ACCFREQ= 
##### If empty prints the current value
##### (value: frequency) - (1: 1Hz), (2: 10Hz), (3: 25Hz), (4: 50Hz), (5: 100Hz), (6: 200Hz), (7: 400Hz), (8: 1.25KHz)
##### Default value - 10Hz
* AT$ACCSCALE=
##### If empty prints the current value
##### (value: scale) - (1: 2G), (2: 4G), (3, 8G), (4: 16G)
##### Default value - 2G
### Baud rate - 
