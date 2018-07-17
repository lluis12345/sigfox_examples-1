/******************************************************************************
 * @file
 * @brief Simple GPS periodic fix application for the TDxxxx RF modules.
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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <efm32.h>

#include <td_core.h>
#include <td_uart.h>
#include <td_stream.h>
#include <at_parse.h>
#include <td_flash.h>
#include <td_scheduler.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_measure.h>
#include <td_printf.h>

#include <td_sensor.h>
#include <sensor_data_geoloc.h>

#include <at_radio.h>
#include <at_sigfox.h>
#include <td_geoloc.h>
#include <td_sigfox.h>

#include "at_user.h"

/*******************************************************************************
 ******************************  DEFINES ****************************
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
int fix_timeout = 1, fix_interval = 1*300;
int16_t minimum_z;
uint8_t gps_mode = TD_GEOLOC_HW_BCKP; // GPS Sleep Mode
int ds_id; // Delayed start scheduler id
bool acc_test = true, is_started = false, first_element = true;

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** User AT command tokens */
typedef enum user_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,	///< First extension token
	// Below here, please try to keep the enum in alphabetical order!!!
	AT_USER_MODE,
	AT_USER_ACC,
	AT_USER_PWTEST,
	AT_USER_TIMEOUT,
} user_tokens;

/*******************************************************************************
 *************************  CONSTANTS   ****************************************
 ******************************************************************************/

/** User AT command set */
static AT_command_t const user_commands[] = {
	{"AT$ACC=", AT_USER_ACC},
	{"AT$PWTEST=", AT_USER_PWTEST},
	{"AT$TIMEOUT=", AT_USER_TIMEOUT},
	{"AT$MODE=", AT_USER_MODE},
	{0, 0}
};

/*******************************************************************************
 ******************************  GLOBAL FUNCTIONS  ****************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Callback function called when the accelerometer data changes.
 *   Saves minimum value of Z axis.
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
void DataCallback(TD_ACCELERO_Data_t data[32], uint8_t count, bool
	overrun)
{
	int i;
	for (i = 0; i < count; i++) {
		tfp_printf("%d\t%d\t%d\r\n", data[i].x, data[i].y, data[i].z);
		if (first_element){			// if it is the first element, it is min
			minimum_z = data[i].z;
			first_element = false;
		}
		else{
			if (data[i].z < minimum_z){
				minimum_z = data[i].z;
			}
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
static void GPSFix(TD_GEOLOC_Fix_t * fix, bool timeout)
{
	int i;
	uint8_t bytes[12], temp;
	uint16_t min;

	unsigned long latitude, longitude;
	int size;

	//Message init - Set to zero not to get random unwanted values
	for(i = 0; i < 12; i++){
		bytes[i] = 0x00;
	}

	size = sizeof(bytes)/sizeof(bytes[0]);

	if (fix->type >= TD_GEOLOC_2D_FIX && fix->hard.rtc_calibrated) {
		uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);
		int32_t mv2 = TD_MEASURE_VoltageTemperatureExtended(true) / 10;

		//Stop GPS
		TD_GEOLOC_StopFix(gps_mode);

		//Now we add the received data in the bytes array:

		//Latitude
		//Hint: divide the return value by 10 will make it understandable by Sigfox backend
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

		//Longitude
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

		//Battery
		for(i=0; i<255; i++){
			if(mv>=i*15 && mv<=(i+1)*15){
				bytes[8] = bytes[8] |  i;
			}else if(mv>3825){
				bytes[8] = bytes[8] | 0xFF;
			}
		}

		// Temperature
		if (mv2 < 0){
			temp = (int32_t)(-1) * (int32_t)mv2; // If the temperature is below 0;
			temp = temp | 0x80;
		}
		else {
			temp = mv2;
		}
		bytes[9] = temp & 0xFF;

		// Minimum Z
		if (minimum_z < 0){
			min = (int16_t)(-1) * (int16_t)minimum_z;
			min = min | 0x8000;
		}
		else {
			min = minimum_z;
		}
		bytes[10] = (min >> 8) & 0xFF;
		bytes[11] = min  & 0xFF;
		first_element = true;

		//Sending the message
		TD_SIGFOX_Send(bytes, size, 2);

		// Start accelerometer
		if (acc_test && !is_started) {
			TD_ACCELERO_MonitorData(true,						// Monitoring enabled
									false,						// Low-power mode disabled
									TD_ACCELERO_1HZ, 			// Sampling rate 10 Hz
									TD_ACCELERO_AXIS_Z,			// Axis mask: all axis
									TD_ACCELERO_2G,				// Scale 2 g
									0,							// High-pass filter disabled
									TD_ACCELERO_STREAM,			// FIFO stream mode
									1,							// Update watermark enabled (32 max, 0 is real-time)
									DataCallback);
			is_started = true;
		}

	} else if (timeout) {
		//If no GPS fix has been found, we still send the voltage and the last appropriate GPS position
		uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);
		int32_t mv2 = TD_MEASURE_VoltageTemperatureExtended(true) / 10;

		//Set the device in its sleep mode
		TD_GEOLOC_StopFix(gps_mode);

		// Battery
		for(i=0; i<255; i++){
			if(mv>=i*15 && mv<=(i+1)*15){
				bytes[8] = bytes[8] |  i;
			}else if(mv>3825){
				bytes[8] = bytes[8] | 0xFF;
			}
		}

		//Temperature
		if (mv2 < 0){
			temp = (int32_t)(-1) * (int32_t)mv2; // Temperature below 0;
			temp = temp | 0x80;
		}
		else {
			temp = mv2;
		}
		bytes[9] = temp & 0xFF;

		// Minimum Z
		if (minimum_z < 0){
			min = (int16_t)(-1) * (int16_t)minimum_z;
			min = min | 0x8000;
		}
		else {
			min = minimum_z;
		}
		bytes[10] = (min >> 8) & 0xFF;
		bytes[11] = min  & 0xFF;
		first_element = true;

		//Sending the message
		TD_SIGFOX_Send(bytes, 12, 2);

		// Start accelerometer
		if (acc_test && !is_started) {
			TD_ACCELERO_MonitorData(true,						// Monitoring enabled
									false,						// Low-power mode disabled
									TD_ACCELERO_1HZ, 			// Sampling rate 10 Hz
									TD_ACCELERO_AXIS_Z,			// Axis mask: all axis
									TD_ACCELERO_2G,				// Scale 2 g
									0,							// High-pass filter disabled
									TD_ACCELERO_STREAM,			// FIFO stream mode
									1,							// Update watermark enabled (32 max, 0 is real-time)
									DataCallback);
			is_started = true;
		}
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

static void StartFixing(uint32_t arg, uint8_t repeat_count) {

	// Stop accelerometer
	if (is_started){
		TD_ACCELERO_MonitorData(false,						// Monitoring enabled
								false,						// Low-power mode disabled
								TD_ACCELERO_1HZ, 			// Sampling rate 10 Hz
								TD_ACCELERO_AXIS_Z,			// Axis mask: all axis
								TD_ACCELERO_2G,				// Scale 2 g
								0,							// High-pass filter disabled
								TD_ACCELERO_STREAM,			// FIFO stream mode
								1,							// Update watermark enabled (32 max, 0 is real-time)
								DataCallback);
		is_started = false;
	}
	TD_GEOLOC_TryToFix(TD_GEOLOC_NAVIGATION, fix_timeout, GPSFix);
}

// Disable serial connection and start fixing
static void delayed_start(uint32_t arg, uint8_t repeat_count){
	TD_UART_DisableExtended(out_stream);
	StartFixing(0, 0);
	TD_SCHEDULER_Append(fix_interval, 0, 0, TD_SCHEDULER_INFINITE, StartFixing, 0);
}

static int8_t user_parse(uint8_t token)
{
	int8_t result = AT_OK;
	int interval, timeout, mode, acc;

	switch (token) {

	// Turn ON/OFF accelerometer
	case AT_USER_ACC:
		if (AT_argc == 1) {
			acc = AT_atoll(AT_argv[0]);
			if (acc == 1){
				acc_test = true;
			}
			else if (acc == 0){
				acc_test = false;
			}
			else {
				return AT_ERROR;
			}
		}
		else {
			return AT_ERROR;
		}
		break;

	// Sets fixing interval and start fixing immediately
	case AT_USER_PWTEST:
		if (AT_argc == 1) {
			interval = AT_atoll(AT_argv[0]);
			if (interval > 0){
				fix_interval = interval;
				tfp_printf("\nSTART TEST: %d %d %d\r\n", fix_interval, fix_timeout, gps_mode);
				TD_SCHEDULER_Remove(ds_id);
				// Disable serial connection and start fixing
				TD_UART_DisableExtended(out_stream);
				StartFixing(0, 0);
				TD_SCHEDULER_Append(fix_interval, 0, 0, TD_SCHEDULER_INFINITE, StartFixing, 0);
			}
			else {
				return AT_ERROR;
			}
		}
		else {
			return AT_ERROR;
		}
		break;

	// Sets GPS timeout
	case AT_USER_TIMEOUT:
			if (AT_argc == 1){
				timeout = AT_atoll(AT_argv[0]);
				if (timeout > 0){
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

	// Sets GPS mode
	case AT_USER_MODE:
			if (AT_argc == 1){
				mode = AT_atoll(AT_argv[0]);
				if (mode == 0){
					gps_mode = TD_GEOLOC_OFF;
				}
				else if(mode == 1){
					gps_mode = TD_GEOLOC_HW_BCKP;
				}
				else if(mode == 2){
					gps_mode = TD_GEOLOC_POWER_SAVE_MODE;
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
	.commands = user_commands,		///< Pointer to the list of extension commands
	.parse = user_parse,				//< Pointer to the extension parse function
};
/***************************************************************************//**
 * @brief
 *  User Setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	TD_UART_Options_t options = {LEUART_DEVICE, LEUART_LOCATION, 9600, 8, 'N',
	  1, false};

	// Open an I/O stream using LEUART0
	out_stream = TD_UART_Open(&options, TD_STREAM_RDWR);
	// Add the RF information AT extension
	AT_AddExtension(&user_extension);
	// Initialize the AT command parser
	AT_Init();

	// Senson initialization
	TD_SENSOR_Init(SENSOR_TRANSMITTER, 0, 0);

	// Geoloc initialization
	TD_GEOLOC_Init();

	// Geoloc and accelerometer initialization
	TD_ACCELERO_Init();

	// Delayed Start
	ds_id = TD_SCHEDULER_Append(0, 0, 1, TD_SCHEDULER_ONE_SHOT, delayed_start, 0);
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
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
