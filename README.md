# sigfox_examples
Custom made GPS tracker and accelerometer data examples for low-powered Sigfox module TD1205P.

## Useful information

* Telecom Design SDK given for their Sigfox modules - https://github.com/Telecom-Design
* Basics and message formating from this tutorial - http://www.instructables.com/id/Sigfox-GPS-Tracker/
* To configure your environment use the tutorial.
* The examples are only built and tested for TD1205P module.
* To see the recieved messages use Sigfox backend - https://backend.sigfox.com
* To build one of the examples you need to check the "Exclude resource from build" in Build Properties of all the other src files!!!

## General Message Fromat
  In all of the examples is used 12 bytes message format for transceiving the information from the module(GPS and accelerometer data). Message is formed by using some of the code in above tutorial:
 - 4 bytes - GPS latitude
 - 4 bytes - GPS longitude
 - 1 byte - Voltage
 - 1 byte - Temperature
 - 2 bytes - Free or used to tranceive the minimum value of one of the accelerometer axis  

## Baud rate
  The default baud rate is 9600. Even though, if you want to use accelerometer at higher rates than 10Hz and to get the data through the serial port, you need to set the baud rate to different values. The highest rate is 460800 bps. In the examples acc_calc and acc_data the rate is 460800 bps.
  
## Example acc_calc.c
 Calculates and prints minimum, maximum, the average and standard deviation of the accelerometer data for x, y, z at given interval
### Commands
#### AT$ACCFREQ= 
- If empty prints the current value
- (value: frequency) - (1: 1Hz), (2: 10Hz), (3: 25Hz), (4: 50Hz), (5: 100Hz), (6: 200Hz), (7: 400Hz), (8: 1.25KHz)
- Default value - 10Hz
#### AT$ACCSCALE=
- If empty prints the current value
- (value: scale) - (1: 2G), (2: 4G), (3, 8G), (4: 16G)
- Default value - 2G
#### AT$ACCDATA=
- 0: turn off the accelerometer
- 1: turn on the accelerometer
- 2: enable/disable printing of current accelerometer data
#### AT$CALC=
- 0: turn off the accelerometer and calculation of the data
- interval in seconds: turn on the accelerometer and calculate the data in some interval

## Example acc_data.c
Configures the accelerometer and prints 3 axes data
### Commands
#### AT$ACCFREQ= 
- If empty prints the current value
- (value: frequency) - (1: 1Hz), (2: 10Hz), (3: 25Hz), (4: 50Hz), (5: 100Hz), (6: 200Hz), (7: 400Hz), (8: 1.25KHz)
- Default value - 1Hz
#### AT$ACCSCALE=
- If empty prints the current value
- (value: scale) - (1: 2G), (2: 4G), (3, 8G), (4: 16G)
- Default value - 2G
#### AT$ACCDATA=
- 1: turn on/off the accelerometer
- 2: enable/disable printing of current accelerometer data

## Example pw_test.c
Configures GPS and sends messages at given interval
* Added custom commands for GPS timeout, GPS fixing interval and GPS fixing mode
* Defaults: 180 seconds timeout, 2 hours fixing interval, TD_GEOLOC_HW_BCKP GPS mode
* 12 bytes message - 4 bytes latitude, 4 bytes longitude, 1 byte voltage, 1 byte temperature
* Sends keepalive message at every boot
* At every boot waits 180 seconds for commands through the serial(if connected), after that starts fixing with default values.
### Commands
#### AT$MODE=
- sets GPS sleep mode
- (value: mode) - (0: TD_GEOLOC_OFF), (1: TD_GEOLOC_HW_BCKP), (2: TD_GEOLOC_POWER_SAVE_MODE)
#### AT$TIMEOUT=
- sets GPS timeout
- timeout in seconds
#### AT$PWTEST=
- manually starts the fixing with given interval
- fixing interaval in seconds

## Example pw_test_acc.c
Configures GPS, sends messages at given interval, saves the minimum value of axis Z during the sleep GPS interval. The Z value is saved in the 11th and 12th bytes.
#### AT$MODE=
- sets GPS sleep mode
- (value: mode) - (0: TD_GEOLOC_OFF), (1: TD_GEOLOC_HW_BCKP), (2: TD_GEOLOC_POWER_SAVE_MODE)
#### AT$TIMEOUT=
- sets GPS timeout
- timeout in seconds
#### AT$ACC=
- 0/1 - turn off/on the accelerometer and saving the minimum Z value
#### AT$PWTEST=
- manually starts the fixing with given interval
- fixing interaval in seconds
