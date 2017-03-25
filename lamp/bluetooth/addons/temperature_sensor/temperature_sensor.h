/*!
 * \defgroup temperature_sensor Temperature Sensor
 * @{
 * Temperature sensor module implements functions to obtain the internal chip 
 * temperature of the SoC by reading the temperature sensor ADC channel. 
 ******************************************************************************/
/*!
* @file temperature_sensor.h
*
* @author  
*
* @version 1.0
*
* @date Mar-15-2016
*
* @brief Driver for Temperature Sensor interface
*
********************************************************************************
*
* Copyright (c) 2016, Freescale Semiconductor.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _TEMPERATURE_SENSOR_H_
#define _TEMPERATURE_SENSOR_H_

/******************************************************************************
* Header files
******************************************************************************/
// Stack header files
#include "EmbeddedTypes.h"
#include "ble_general.h"

/******************************************************************************
* User definitions
*******************************************************************************/
#define TEMPERATURE_SENSOR_ADC_INSTANCE         0       /*!< ADC module instance connected to the internal temperature sensor */

#define TEMPERATURE_SENSOR_V_BANDGAP_mV         1000    /*!< Bandgap ADC channel voltage (in mV) */
#define TEMPERATURE_SENSOR_ADC_RESOLUTION       32768   /*!< Expanded resolution value 2^Resolution */
#define TEMPERATURE_SENSOR_VTEMP25_mV           716     /*!< Temperature sensor voltage @25C defined by the datasheet */
#define TEMPERATURE_SENSOR_SLOPE_uV             1620    /*!< Temperature sensor slope (in uV) defined by the datasheet */

/******************************************************************************
* Type definitions
******************************************************************************/

/*! Temperature sensor status */
typedef enum temperatureSensorStatus{
  kTemperatureSensorOk,                                 /*!< No error */
  kTemperatureInitError,                                /*!< Initialization error */
  kTemperatureSensorConversionStartError = 0xFFFF,      /*!< Error starting the channel conversion */
}temperature_sensor_status_t;

typedef union lamp_TempVoltage_tag {
	uint32_t raw;
	struct {	
          /* core temperature, exponent -2 */
          int16_t gCoreTemperature;
          /* core voltage reference, exponent -3 */
          int16_t g_vReference;
	} int16;
} chip_TempVoltage_t;


/******************************************************************************
* Globals
*******************************************************************************/


/* core temperature at witch the sistem should switch off all outputs, exponent -2 */
#ifndef gCoreTemperatureFaliure_d 
  #define gCoreTemperatureFaliure_d    7500
#endif

/* core temperature at upper limit, exponent -2 */
#define gCoreTemperatureFaliureUL_d    15000

/* core temperature at lower limit, exponent -2 */
//#define gCoreTemperatureFaliureLL_d    3500
#define gCoreTemperatureFaliureLL_d    2000


/******************************************************************************
* Configuration options
******************************************************************************/

/******************************************************************************
* Public Function prototypes
*******************************************************************************/

/*!***************************************************************************
* \brief        Initializes the temperature sensor
*
* \param[in]    None
*
* \return       @ref temperature_sensor_status_t Error status
****************************************************************************/
temperature_sensor_status_t temperature_sensor_init (void);

/*!***************************************************************************
* \brief        Returns the current chip temperature with a 0.01 °C resolution.
*
* \param[in]    None
*
* \return       Temperature measurement with a 0.01 °C resolution or 
*               0xFFFF if an error occurred to global gCoreTemperature
*               Core Voltage reference g_vReference
****************************************************************************/
void measure_chip_temperature (void);

bleResult_t set_chip_critical_temperature (int16_t new_critical_temperature);

/*!
 * @} End of temperature_sensor
 */

#endif
/* End of file */
