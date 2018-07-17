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

#include <efm32.h>

#include <td_core.h>
#include <td_uart.h>
#include <td_printf.h>
#include <td_stream.h>
#include <at_parse.h>
#include <td_flash.h>
#include <td_scheduler.h>
#include <td_watchdog.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_measure.h>

#include <td_sensor.h>
#include <sensor_data_geoloc.h>

#include <at_radio.h>
#include <at_sigfox.h>
#include <td_accelero.h>
#include <td_geoloc.h>
#include <td_sigfox.h>

#include "at_user.h"

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

/** Boot monitoring, 1 to enable */
#define BOOT_MONITORING 0

/** Keepalive monitoring interval in hours, 0 to disable
 * If you wish to send a keepalive frame remember to add a scheduler as well in the setup function */
#define KEEPALIVE_INTERVAL 0


// Save the last fixed position, get the GPS scheduler id
uint8_t acc_scheduler_id=0;
bool is_started = false, acc_print = false;
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
} user_tokens;

/*******************************************************************************
 *************************  CONSTANTS   ****************************************
 ******************************************************************************/

/** User AT command set */
static AT_command_t const user_commands[] = {
	{"AT$ACCFREQ=", AT_USER_ACCFREQ},
	{"AT$ACCSCALE=", AT_USER_ACCSCALE},
	{"AT$ACCData=", AT_USER_ACCDATA},
	{0, 0}
};

/*******************************************************************************
 ******************************  GLOBAL FUNCTIONS  ****************************
 ******************************************************************************/

void DataCallback(TD_ACCELERO_Data_t data[32], uint8_t count, bool
	overrun)
{
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

static int8_t user_parse(uint8_t token)
{
	int8_t result = AT_OK;
	int value;

	switch (token) {

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

	case AT_USER_ACCSCALE:
		if (AT_argc == 0){
			tfp_printf("Accelerometer's frequency is: %d\r\n", acc_freq);
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
			break;

	case AT_USER_ACCDATA:
		if (AT_argc == 1) {
			value = AT_atoll(AT_argv[0]);
			if (value == 1 && is_started != true) {
				is_started = !is_started;
				TD_ACCELERO_MonitorData(is_started,					// Monitoring enabled
										false,						// Low-power mode disabled
										acc_freq, 					// Sampling rate 10 Hz
										TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
										acc_scale,					// Scale 2 g
										0,							// High-pass filter disabled
										TD_ACCELERO_STREAM,			// FIFO stream mode
										1,							// Update watermark enabled (32 max, 0 is real-time)
										DataCallback);
				}
			else if(value == 2){
				acc_print = !acc_print;
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
	.commands = user_commands,		///< Pointer to the list of extension commands
	.parse = user_parse,				//< Pointer to the extension parse function
};
/***************************************************************************//**
 * @brief
 *  User Setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	TD_UART_Options_t options = {LEUART_DEVICE, LEUART_LOCATION, 460800, 8, 'N',
		1, false};

	// Open an I/O stream using LEUART0
	TD_UART_Open(&options, TD_STREAM_RDWR);
	// Add the RF information AT extension
	AT_AddExtension(&user_extension);
	// Initialize the AT command parser
	AT_Init();

	TD_SENSOR_Init(SENSOR_TRANSMITTER, 0, 0);

	// Geoloc and accelerometer initialization
	TD_ACCELERO_Init();


#if BOOT_MONITORING

	// Will only send a boot monitor frame on NEXT reboot
	TD_SENSOR_MonitorBoot(true, 0);
#endif

#if KEEPALIVE_INTERVAL > 0

	// Send a keep-alive frame immediately, then at given interval
	TD_SENSOR_MonitorKeepAlive(true, KEEPALIVE_INTERVAL);
#endif
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	// Process Sensor events
	TD_SENSOR_Process();

	// Process Accelerometer events
	TD_ACCELERO_Process();

	int c;
	// While there are characters
	while ((c = TD_UART_GetChar()) >= 0) {
		// Parse them using the AT interpreter
		AT_Parse(c);
	}
}
