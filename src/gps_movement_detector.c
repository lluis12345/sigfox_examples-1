/******************************************************************************
 * @file
 * @brief Simple accelerometer event monitor application for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
  ******************************************************************************/

#include "config.h"
#include "at_user.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <efm32.h>

#include <td_core.h>
#include <td_rtc.h>
#include <td_uart.h>
#include <td_printf.h>
#include <td_stream.h>
#include <td_flash.h>
#include <td_scheduler.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_measure.h>

#include <at_parse.h>
#include <at_radio.h>
#include <at_sigfox.h>

#include <td_accelero.h>
#include <td_geoloc.h>
#include <td_sensor.h>
#include <sensor_data_geoloc.h>

#include <td_sigfox.h>

/*******************************************************************************
 ******************************  DEFINES ***************************************
 ******************************************************************************/

/** Manufacturer */
#define MANUFACTURER		"Telecom Design"

/** Product */
#define PRODUCT				"TDxxxx"

/** Hardware revision */
#define HARDWARE_VERSION	"0F"

/** Software version */
#define SOFTWARE_VERSION	"SOFTxxxx"

/** Release data */
#define RELEASE_DATE		"M10+2012"

/** Telecom Design 12-digit serial number */
#define SERIAL_NUMBER		"123456789012"

#include <td_config.h>

/** Flash variable version ID */
#define VARIABLES_VERSION 0

/** Acceptable minimum horizontal accuracy, 800 to be very accurate */
#define FIX_HDOP 800

/** Boot monitoring, 1 to enable */
#define BOOT_MONITORING 0

/** Keepalive monitoring interval in hours, 0 to disable
 * If you wish to send a keepalive frame remember to add a scheduler as well in the setup function */
#define KEEPALIVE_INTERVAL 0

/**************************************************************************************
 *************************  GLOBAL VARIABLES   ****************************************
 **************************************************************************************/

TD_STREAM_t *out_stream; // Pointer to UART stream structure
uint8_t gps_mode = TD_GEOLOC_HW_BCKP; // Default GPS sleep mode
int ds_id; // Delayed start scheduler id
int fix_timeout = 120; // Maximum waiting time of the device for finding GPS position
int fix_interval_movement = 3600; // Default interval used to send data if there is movement (t1 period)
int nb_acc_events = 0; // Variable which counts the number of accelerometer events during one fix_interval_movement
int nb_no_acc_events = 0; // Variable which counts the number of consequential intervals (which last t1 period) without movement detection
int type_message = 0; // In case of no movement, default sending of temperature and voltage
int com_t2_period = 3; // Variable for choosing the t2 period, multiple of t1 period (fix_interval_movement)
                       // Default value is 3, and t2 period = com_t2_period * fix_interval_movement
uint8_t last_GPS_message[8]; // Last GPS coordinates

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** User AT command tokens */
typedef enum user_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,	///< First extension token
	// Below here, please try to keep the enum in alphabetical order!!!
	AT_USER_INTERVAL,
	AT_USER_MODE,
	AT_USER_T2_PERIOD,
	AT_USER_TIMEOUT,
	AT_USER_TYPE,
} user_tokens;

/*******************************************************************************
 *************************  CONSTANTS   ****************************************
 ******************************************************************************/

/** User AT command set */
static AT_command_t const user_commands[] = {
	{"AT$INTERVAL=", AT_USER_INTERVAL},
	{"AT$MODE=", AT_USER_MODE},
	{"AT$T2PERIOD=", AT_USER_T2_PERIOD},
	{"AT$TIMEOUT=", AT_USER_TIMEOUT},
	{"AT$TYPE=", AT_USER_TYPE},
	{0, 0}
};

/*******************************************************************************
 ******************************  FUNCTIONS  ************************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *  GPS fix callback
 *
 * @param[in] fix
 *   The GPS fix data structure.
 *
 * @param[in] timeout
 *   Flag that indicates whether a timeout occurred if set to true.
 ******************************************************************************/
static void GPSFix(TD_GEOLOC_Fix_t * fix, bool timeout) {
	int i;
	uint8_t bytes[12], temp;

	unsigned long latitude, longitude;
	int size;

	// Message init - Set to zero not to get random unwanted values
	for(i=0;i<12;i++) {
		bytes[i] = 0x00;
	}

	size = sizeof(bytes)/sizeof(bytes[0]);

	// If GPS fix has been found
	if (fix->type >= TD_GEOLOC_2D_FIX && fix->hard.rtc_calibrated) {

		// Voltage measure
		uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);

		// Temperature measure
		int32_t mv2 = TD_MEASURE_VoltageTemperatureExtended(true) / 10;

		// Stop GPS
		TD_GEOLOC_StopFix(gps_mode);

		// Now we add the received data in the bytes array:

		// Latitude
		// Hint: divide the return value by 10 will make it understandable by Sigfox backend
		if (fix->position.latitude < 0) {
			  latitude = (int32_t)(-1)* ((int32_t)fix->position.latitude / 10) ;
			  // latitude_direction = 'S';
			  latitude = latitude| 0x80000000;
		} else {
			  latitude = fix->position.latitude / 10;
			  // latitude_direction = 'N';
		}
		bytes[0] = (latitude >> 24) & 0xFF;
		bytes[1] = (latitude >> 16) & 0xFF;
		bytes[2] = (latitude >> 8) & 0xFF;
		bytes[3] = latitude & 0xFF;

		// Longitude
		if (fix->position.longitude < 0) {
			  longitude = (int32_t)(-1)* ((int32_t)fix->position.longitude / 10) ;
			  // longitude_direction = 'W';
			  longitude = longitude| 0x80000000;
		} else {
			  longitude = fix->position.longitude / 10;
			  // longitude_direction = 'E';
		}
		bytes[4] = (longitude >> 24) & 0xFF;
		bytes[5] = (longitude >> 16) & 0xFF;
		bytes[6] = (longitude >> 8) & 0xFF;
		bytes[7] = longitude & 0xFF;

		// Save GPS message (latitude and longitude)
		for(i=0;i<8;i++) {
			last_GPS_message[i] = bytes[i];
		}

		// Battery
		for(i=0; i<255; i++) {
			if(mv>=i*15 && mv<=(i+1)*15) {
				bytes[8] = bytes[8] |  i;
			}
			else if(mv>3825) {
				bytes[8] = bytes[8] | 0xFF;
			}
		}

		// Temperature
		if (mv2 < 0) {
			temp = (int32_t)(-1) * (int32_t)mv2; // If the temperature is below 0;
			temp = temp | 0x80;
		}
		else {
			temp = mv2;
		}
		bytes[9] = temp & 0xFF;

		// Sending the message
		TD_SIGFOX_Send(bytes, size, 2);
		tfp_printf("A message was sent on the Sigfox network\r\n");
		tfp_printf("GPS Fix OK\r\n");
	}
	else if (timeout) {
		// If no GPS fix has been found, we still send the voltage and the temperature

		// Voltage measure
		uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);

		// Temperature measure
		int32_t mv2 = TD_MEASURE_VoltageTemperatureExtended(true) / 10;

		// Set the device in its sleep mode
		TD_GEOLOC_StopFix(gps_mode);

		// Battery
		for(i=0; i<255; i++) {
			if(mv>=i*15 && mv<=(i+1)*15) {
				bytes[8] = bytes[8] |  i;
			}
			else if(mv>3825) {
				bytes[8] = bytes[8] | 0xFF;
			}
		}

		// Temperature
		if (mv2 < 0) {
			temp = (int32_t)(-1) * (int32_t)mv2; // Temperature below 0;
			temp = temp | 0x80;
		}
		else {
			temp = mv2;
		}
		bytes[9] = temp & 0xFF;

		// Sending the message
		TD_SIGFOX_Send(bytes, 12, 2);
		tfp_printf("A message was sent on the Sigfox network\r\n");
		tfp_printf("GPS Fix KO\r\n");
	}
}

/***************************************************************************//**
 * @brief
 *  Start fixing periodically.
 *
 * @param[in] arg
 *  Generic argument set by the user that is passed along to the callback
 *  function.
 *
 * @param[in] repeat_count
 *  Updated repeat count, decremented at each timer trigger, unless it is an
 *  infinite timer.
 ******************************************************************************/

static void StartFixing(uint32_t arg, uint8_t repeat_count)  {

	// Test message which is sent if there is no movement during one fix_interval_movement
	// uint8_t test[1];
	// test[0]=0xAA;

	// If there is movement during one fix_interval_movement
	if(nb_acc_events >= 1) {
		tfp_printf("There were accelerometer events during the last %d seconds (or 180 seconds if it is the first value)\r\n", fix_interval_movement);
		tfp_printf("Trying to send GPS fix using Sigfox...\r\n");

		// Trying to send GPS coordinates, and temperature, voltage
		TD_GEOLOC_TryToFix(TD_GEOLOC_NAVIGATION, fix_timeout, GPSFix);

		// Reset of variable, because there was movement
		nb_no_acc_events = 0;
	}
	// If there is no movement during one fix_interval_movement
	else {
		// Sending the test message
		// TD_SIGFOX_Send(test, 1, 2);

		tfp_printf("No accelerometer events during the last %d seconds (or 180 seconds if it is the first value)\r\n", fix_interval_movement);

		// Incrementation of the variable which counts the number of consequential intervals without movement
		nb_no_acc_events++;

		// If there was no movement during "T2 period" seconds
		if(nb_no_acc_events == com_t2_period) {

			int i;
			uint8_t no_movement_message[12], temp;

			// Message init - Set to zero not to get random unwanted values
			for(i=0;i<12;i++) {
				no_movement_message[i] = 0x00;
			}

			// Voltage measure
			uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);

			// Temperature measure
			int32_t mv2 = TD_MEASURE_VoltageTemperatureExtended(true) / 10;

			// Battery
			for(i=0; i<255; i++) {
				if(mv>=i*15 && mv<=(i+1)*15) {
					no_movement_message[8] = no_movement_message[8] |  i;
				}
				else if(mv>3825) {
					no_movement_message[8] = no_movement_message[8] | 0xFF;
				}
			}

			// Temperature
			if (mv2 < 0) {
				temp = (int32_t)(-1) * (int32_t)mv2; // Temperature below 0;
				temp = temp | 0x80;
			}
			else {
				temp = mv2;
			}
			no_movement_message[9] = temp & 0xFF;

			// If the user chooses to send the last GPS position too
			if(type_message == 1) {
				for(i=0;i<8;i++) {
					no_movement_message[i] = last_GPS_message[i];
				}
			}

			// Sending the message
			TD_SIGFOX_Send(no_movement_message, 12, 2);
			tfp_printf("A message was sent on the Sigfox network\r\n");

			// Reset
			nb_no_acc_events = 0;
		}
	}
	// Reset of variable, because there will a calculation on a new interval
	nb_acc_events = 0;
}

/***************************************************************************//**
 * @brief
 *  delayed_start callback invokes StartFixing and close all terminal connections
 *
 * @param[in] arg
 *  Generic argument set by the user that is passed along to the callback
 *  function.
 *
 * @param[in] repeat_count
 *  Updated repeat count, decremented at each timer trigger, unless it is an
 *  infinite timer.
 ******************************************************************************/
static void delayed_start(uint32_t arg, uint8_t repeat_count) {

	// Stop the use of serial communication
	TD_UART_DisableExtended(out_stream);

	// Start the GPS immediately
	StartFixing(0, 0);

	// Message init - Set to zero not to get random unwanted values
	int i;
	for(i=0;i<8;i++){
		last_GPS_message[i] = 0x00;
	}

	// Printing the T1 period and the T2 period
	// tfp_printf("Fix_Interval T1 = %d\r\n", fix_interval_movement);
	// tfp_printf("Fix_Interval T2 = %d\r\n", com_t2_period*fix_interval_movement);

	// StartFixing function will executed every fix_interval_movement period
	TD_SCHEDULER_Append(fix_interval_movement, 0, 0, TD_SCHEDULER_INFINITE, StartFixing, 0);
}

/***************************************************************************//**
 * @brief
 *   Callback function called when the accelerometer generates an IRQ.
 *
 * @param[in] source
 *   The (possibly) simultaneous IRQ sources.
 ******************************************************************************/
static void EventCallback(uint8_t source) {

	// Filter out low IRQs, as low these are always set in accelerometer registers
	source &= TD_ACCELERO_ALL_HIGH_IRQ;

	// Incrementation of the variable which counts the accelerometer events
	nb_acc_events++;
	tfp_printf("Accelerometer event: %d\r\n", nb_acc_events);
	tfp_printf("%02lX\r\n", source);
	if (source & TD_ACCELERO_IRQ_XL) {
		tfp_printf("x low\r\n");
	}
	if (source & TD_ACCELERO_IRQ_XH) {
		tfp_printf("x high\r\n");
	}
	if (source & TD_ACCELERO_IRQ_YL) {
		tfp_printf("y low\r\n");
	}
	if (source & TD_ACCELERO_IRQ_YH) {
		tfp_printf("y high\r\n");
	}
	if (source & TD_ACCELERO_IRQ_ZL) {
		tfp_printf("z low\r\n");
	}
	if (source & TD_ACCELERO_IRQ_ZH) {
		tfp_printf("z high\r\n");
	}
}

/****************************************************************************//**
 * @brief
 * 	Adds functionality of newly added commands
 *
 * @param[in] token
 *
 *******************************************************************************/
static int8_t user_parse(uint8_t token) {

	int8_t result = AT_OK;
	int interval, timeout, mode, type, multiplier;

	switch (token) {

	// Changes the fix_interval_movement (t1 period)
	case AT_USER_INTERVAL:
		if (AT_argc == 0) {
			tfp_printf("GPS Fix Interval: %d\r\n", fix_interval_movement);
		}
		else if (AT_argc == 1) {
			interval = AT_atoll(AT_argv[0]);
			if (interval > 0) {
				fix_interval_movement = interval;
			}
			else {
				return AT_ERROR;
			}
		}
		else {
			result= AT_ERROR;
		}
		break;

	// Changes the default timeout used for GPS fixing
	case AT_USER_TIMEOUT:
		if (AT_argc == 0) {
			tfp_printf("GPS Fix Timeout: %d\r\n", fix_timeout);
		}
		else if (AT_argc == 1) {
			timeout = AT_atoll(AT_argv[0]);
			if (timeout > 0) {
				fix_timeout = timeout;
			}
			else {
				result = AT_ERROR;
			}
		}
		else {
			result = AT_ERROR;
		}
		break;

	// Changes the default GPS mode either to TD_GEOLOC_OFF or TD_GEOLOC_HW_BCKP
	case AT_USER_MODE:
		if (AT_argc == 1){
			mode = AT_atoll(AT_argv[0]);
			if (mode == 0) {
				gps_mode = TD_GEOLOC_OFF;
				tfp_printf("The GPS Mode which is used is : TD_GEOLOC_OFF\r\n");
			}
			else if(mode == 1) {
				gps_mode = TD_GEOLOC_HW_BCKP;
				tfp_printf("The GPS Mode which is used is : TD_GEOLOC_HW_BCKP\r\n");
			}
			else {
				result = AT_ERROR;
			}
		}
		else {
			result = AT_ERROR;
		}
		break;

	// Changes the type of message which is sent when there is no movement (every t2 period)
	case AT_USER_TYPE:
		if (AT_argc == 0) {
			if(type_message) {
				tfp_printf("Type message: %d: Last GPS Position, Temperature and Voltage\r\n", type_message);
			}
			else {
				tfp_printf("Type message: %d: Only Temperature and Voltage\r\n", type_message);
			}
		}
		else if (AT_argc == 1) {
			type = AT_atoll(AT_argv[0]);
			if (type != type_message) {
				if((type == 0) || (type == 1)) {
					type_message = type;
				}
				else {
					result = AT_ERROR;
				}
			}
			else {
				result = AT_ERROR;
			}
		}
		else {
			result = AT_ERROR;
		}
		break;

	// Change the t2 period (period of time after which the device has to send a message if there is no movement)
	case AT_USER_T2_PERIOD:
		if (AT_argc == 0) {
			tfp_printf("T2 period value: %d\r\n", com_t2_period * fix_interval_movement);
		}
		else if (AT_argc == 1) {
			multiplier = AT_atoll(AT_argv[0]);
			if (multiplier > 0) {
				com_t2_period = multiplier;
			}
			else {
				result = AT_ERROR;
			}
		}
		else {
			result = AT_ERROR;
		}
		break;

	default:
		result = AT_NOTHING;
		break;
	}

	return result;
}

AT_extension_t user_extension = {
	.commands = user_commands,		// Pointer to the list of extension commands
	.parse = user_parse,			// Pointer to the extension parse function
};

/***************************************************************************//**
 * @brief
 *  User Setup function.
 ******************************************************************************/
void TD_USER_Setup(void) {

	TD_UART_Options_t options = {LEUART_DEVICE, LEUART_LOCATION, 9600, 8, 'N',
	1, false};

	// Open an I/O stream using LEUART0
	out_stream = TD_UART_Open(&options, TD_STREAM_RDWR);

	// Set flash variables version
	TD_FLASH_DeleteVariables();

	// Add the RF information AT extension
	AT_AddExtension(&user_extension);

	// Initialize the AT command parser
	AT_Init();

	// Set the device as a Sensor transmitter
	TD_SENSOR_Init(SENSOR_TRANSMITTER, 0, 0);

	// Geoloc and accelerometer initialization
	TD_GEOLOC_Init();
	TD_ACCELERO_Init();

	// Monitor accelerometer events (movements)
	TD_ACCELERO_MonitorEvent(true,	// Monitoring enabled
		TD_ACCELERO_10HZ, 			// Sampling rate 10 Hz
		TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
		TD_ACCELERO_2G,				// Scale 2 g
		TD_ACCELERO_ALL_HIGH_IRQ,	// Only monitor high IRQs, as low IRQs are always set in
									// accelerometer registers
		10,							// Threshold in mg = 10 * 2 g / 127 = +- 160 mg (with scale 2 g)
		1,							// Duration in ms = 1 * (1 / 10 Hz) = 100 ms (with rate 10 Hz)
		1,							// High-pass filter enabled
		EventCallback);

	// Delayed start, waiting for commands during 180 seconds
	ds_id = TD_SCHEDULER_Append(0, 0, 180, TD_SCHEDULER_ONE_SHOT, delayed_start, 0);

	// Send a keep-alive frame immediately
	TD_SENSOR_MonitorKeepAlive(true, 0);

}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void) {

	// Process Sensor events
	TD_SENSOR_Process();

	// Process Geoloc events
	TD_GEOLOC_Process();

	// Process Accelerometer events
	TD_ACCELERO_Process();

	int c;
	// While there are characters
	while ((c = TD_UART_GetChar()) >= 0) {
		// Parse them using the AT interpreter
		AT_Parse(c);
	}
}
