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
int acc_print = 1; // Variable for choosing if the user wants to print data on serial, 1 is ok
uint8_t acc_freq = TD_ACCELERO_1HZ;
uint8_t acc_scale = TD_ACCELERO_2G;

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** User AT command tokens */
typedef enum user_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,	///< First extension token
	// Below here, please try to keep the enum in alphabetical order!!!
	AT_USER_ACCDATA,
	AT_USER_ACCFREQ,
	AT_USER_ACCSCALE,
	AT_USER_INTERVAL,
	AT_USER_MODE,
	AT_USER_TIMEOUT,
} user_tokens;

/*******************************************************************************
 *************************  CONSTANTS   ****************************************
 ******************************************************************************/

/** User AT command set */
static AT_command_t const user_commands[] = {
	{"AT$ACCFREQ=", AT_USER_ACCFREQ},
	{"AT$ACCSCALE=", AT_USER_ACCSCALE},
	{"AT$ACCDATA=", AT_USER_ACCDATA},
	{"AT$INTERVAL=", AT_USER_INTERVAL},
	{"AT$MODE=", AT_USER_MODE},
	{"AT$TIMEOUT=", AT_USER_TIMEOUT},
	{0, 0}
};

/*******************************************************************************
 ******************************  FUNCTIONS  ************************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Callback function called when the accelerometer data changes.
 *
 * @param[in] x
 *   X-axis acceleration value.
 *
 * @param[in] y
 *   Y-axis acceleration value.
 *
 * @param[in] z
 *   Z-axis acceleration value.
 ******************************************************************************/
void DataCallback(TD_ACCELERO_Data_t data[32], uint8_t count, bool overrun) {
	int i;
	for (i = 0; i < count; i++) {
		if(acc_print) {
			tfp_printf("%d\t%d\t%d\r\n", data[i].x, data[i].y, data[i].z);
		}
	}
	if (overrun) {
		tfp_printf("overrun\r\n");
	}
}

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

	// Disable monitor accelerometer event (movement)
	TD_ACCELERO_MonitorData(false,	// Monitoring disabled
		false,						// Low-power mode disabled
		acc_freq, 			        // Sampling rate
		TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
		acc_scale,				    // Scale
		0,							// High-pass filter disabled
		TD_ACCELERO_STREAM,			// FIFO stream mode
		1,							// Update watermark enabled (32 max, 0 is real-time)
		DataCallback);

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
	}

	// Enable monitor accelerometer event (movement)
	TD_ACCELERO_MonitorData(true,	// Monitoring enabled
		false,						// Low-power mode disabled
		acc_freq, 			        // Sampling rate
		TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
		acc_scale,				    // Scale
		0,							// High-pass filter disabled
		TD_ACCELERO_STREAM,			// FIFO stream mode
		1,							// Update watermark enabled (32 max, 0 is real-time)
		DataCallback);
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

	// Trying to send GPS coordinates, and temperature, voltage
	TD_GEOLOC_TryToFix(TD_GEOLOC_NAVIGATION, fix_timeout, GPSFix);
}

/***************************************************************************//**
 * @brief
 *  delayed_start callback invokes StartFixing
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

	// Enable monitor accelerometer event (movement)
	TD_ACCELERO_MonitorData(true,	// Monitoring enabled
		false,						// Low-power mode disabled
		acc_freq, 			        // Sampling rate
		TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
		acc_scale,				    // Scale
		0,							// High-pass filter disabled
		TD_ACCELERO_STREAM,			// FIFO stream mode
		1,							// Update watermark enabled (32 max, 0 is real-time)
		DataCallback);

	// Start the GPS immediately
	StartFixing(0, 0);

	// StartFixing function will executed every fix_interval_movement period
	TD_SCHEDULER_Append(fix_interval_movement, 0, 0, TD_SCHEDULER_INFINITE, StartFixing, 0);

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
	int value, interval, mode, timeout;

	switch (token) {

	// For choosing the frequency to retrieve accelerometer data
	case AT_USER_ACCFREQ:
		if (AT_argc == 0){
			tfp_printf("Accelerometer's frequency is: %d\r\n", acc_freq);
		}
		else if (AT_argc == 1) {
			value = AT_atoll(AT_argv[0]);
			switch(value){

			case 1:
				acc_freq = TD_ACCELERO_1HZ;
				break;
			case 2:
				acc_freq = TD_ACCELERO_10HZ;
				break;
			case 3:
				acc_freq = TD_ACCELERO_25HZ;
				break;
			case 4:
				acc_freq = TD_ACCELERO_50HZ;
				break;
			case 5:
				acc_freq = TD_ACCELERO_100HZ;
				break;
			case 6:
				acc_freq = TD_ACCELERO_200HZ;
				break;
			case 7:
				acc_freq = TD_ACCELERO_400HZ;
				break;
			case 8:
				acc_freq = TD_ACCELERO_1_25KHZ;
				break;
			default:
				acc_freq = TD_ACCELERO_1HZ;
				break;
			}
		}
		else result = AT_ERROR;
		break;

	// For choosing the accelerometer scale
	case AT_USER_ACCSCALE:
		if (AT_argc == 0){
			tfp_printf("Accelerometer's scale is: %d\r\n", acc_scale);
			}
		else if (AT_argc == 1) {
			value = AT_atoll(AT_argv[0]);
			switch(value){
			case 1:
				acc_scale = TD_ACCELERO_2G;
				break;
			case 2:
				acc_scale = TD_ACCELERO_4G;
				break;
			case 3:
				acc_scale = TD_ACCELERO_8G;
				break;
			case 4:
				acc_scale = TD_ACCELERO_16G;
				break;
			default:
				acc_scale = TD_ACCELERO_2G;
				break;
			}
		}
		else {
			result = AT_ERROR;
		}
		break;

	// For choosing if accelerometer will be printed on the serial console
	case AT_USER_ACCDATA:
		if (AT_argc == 0) {
			if(acc_print == 1) {
				tfp_printf("Print accelerometer data on serial: Yes\r\n");
			}
			else {
				tfp_printf("Print accelerometer data on serial: No\r\n");
			}
		}
		else if (AT_argc == 1) {
			value = AT_atoll(AT_argv[0]);
			if (value != acc_print) {
				if((value == 0) || (value == 1)) {
					acc_print = value;
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
			result = AT_ERROR;
		}
		break;

	// Changes the default GPS mode either to TD_GEOLOC_OFF, or TD_GEOLOC_HW_BCKP or TD_GEOLOC_POWER_SAVE_MODE
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
			else if(mode == 2) {
				gps_mode = TD_GEOLOC_POWER_SAVE_MODE;
				tfp_printf("The GPS Mode which is used is : TD_GEOLOC_POWER_SAVE_MODE\r\n");
			}
			else {
				result = AT_ERROR;
			}
		}
		else {
			result = AT_ERROR;
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
	while ((c = TD_UART_GetChar()) >= 0 ) {
		// Parse them using the AT interpreter
		AT_Parse(c);
	}
}
