# sigfox_tracker
Custom made low-powered GPS tracker application for Sigfox module TD1205P.

* Added custom commands for GPS timeout, GPS fixing interval and GPS fixing mode
* Defaults: 180 seconds timeout, 2 hours fixing interval, TD_GEOLOC_HW_BCKP GPS mode
* 12 bytes message - 4 bytes latitude, 4 bytes longitude, 1 byte voltage, 1 byte temperature
* Sends keepalive message at every boot
* At every boot waits 180 seconds for commands through the serial(if connected), after that starts fixing with default values.

## Used information

* Telecom Design SDK given for their Sigfox modules - https://github.com/Telecom-Design
* Basics and message formating from this tutorial - http://www.instructables.com/id/Sigfox-GPS-Tracker/

## Commands

* AT_USER_TIMEOUT= {timeout in seconds}
* AT_USER_MODE= {GPS mode - 0: TD_GEOLOC_OFF, 1: TD_GEOLOC_HW_BCKP, 2: TD_GEOLOC_POWER_SAVE_MODE}
* AT_USER_PWTEST= {fixing interval(starts immediately)}