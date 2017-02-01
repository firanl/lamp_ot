/*!
*@defgroup input_report Input Report
*
* Input Report implements functions to acquire users input when a button is pressed (GPIO) or a capacitive sensor is touched (TSI).
*
* Input Report functions are divided in two sub-modules. Keyboard module is part of the Connectivity Software stack and implements 
* functions to handle GPIO inputs. TSI Sensor module includes functions to acquire user input when pressing capacitive touch sensors.
*
* Keyboard module is explained in the Connectivity Software stack documentation. Please refer to the Connectivity Framwork Reference Manual 
* (CONNFWRKRM) for more information on this module.
*
* @{
*******************************************************************************/
/*!
* @file tsi_sensor.h
*
* @author  
*
* @version 1.0
*
* @date Mar-09-2016
*
* @brief Driver for TSI Sensor interface
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

/*!
 * \defgroup tsi_sensor TSI Sensor
 * @{
 *  TSI Sensor implements functions to detect touch events in capacitive pads
 *****************************************************************************/

#ifndef _TSI_SENSOR_H_
#define _TSI_SENSOR_H_

/******************************************************************************
* Header files
******************************************************************************/
// KSDK header files
#include "fsl_tsi_driver.h"

/******************************************************************************
* User definitions
*******************************************************************************/
#define TSI_SENSOR_THRESHOLD_ADDER      10       /*!< Threshold value to detect a touch event */

/******************************************************************************
* Type definitions
******************************************************************************/
/*!
 * TSI Electrode flags array
 */
typedef union tsiSensorElectrodeFlags{
  uint16_t overallFlagStatus;   /*!< Status of all TSI flags */
  struct {
    uint16_t electrode1 : 1;
    uint16_t electrode2 : 1;
    uint16_t electrode3 : 1;
    uint16_t electrode4 : 1;
    uint16_t electrode5 : 1;
    uint16_t electrode6 : 1;
    uint16_t electrode7 : 1;
    uint16_t electrode8 : 1;
    uint16_t electrode9 : 1;
    uint16_t electrode10 : 1;
    uint16_t electrode11 : 1;
    uint16_t electrode12 : 1;
    uint16_t electrode13 : 1;
    uint16_t electrode14 : 1;
    uint16_t electrode15 : 1;
    uint16_t electrode16 : 1;  
  }activeFlag;                 /*!< Status of each TSI flag */
}tsi_sensor_electrode_flags_t;

/*!
 * TSI Sensor return status for functions
 */
typedef enum tsiSensorStatus{
  kTsiOk,               /*!< No error */
  kTsiInitError,        /*!< Error initializing the module */
  kTsiStartError,       /*!< Error starting the measurements */
}tsi_sensor_status_t;

/*!
 * TSI electrode data structure
 */
typedef struct tsiSensorElectrodeData{
  uint8_t       channel;        /*!< Electrode channel */
  uint16_t      threshold;      /*!< Electrode threshold */
}tsi_sensor_electrode_data_t;

/*!
 * TSI Sensor callback function type
 */
typedef void (*tsi_sensor_callback_t) (tsi_sensor_electrode_flags_t* pElectrodeFlags);

/******************************************************************************
* Globals
*******************************************************************************/
extern tsi_sensor_electrode_flags_t tsiSensorActiveElectrodeFlag; /*!< TSI electrode flags */

/******************************************************************************
* Configuration options
******************************************************************************/

/******************************************************************************
* Public Function prototypes
*******************************************************************************/

/*!***************************************************************************
* \brief        Initializes the TSI sensor.
*
* \param[in]    Function to execute when a TSI touch is sensed.
*
* \return       @ref tsi_sensor_status_t Error status.
****************************************************************************/
tsi_sensor_status_t tsi_sensor_init (tsi_sensor_callback_t pCallbackFunc);

/*!***************************************************************************
* \brief        Starts a single TSI sensor measurement.
*
* \param[in]    None.
*
* \return       @ref tsi_sensor_status_t Error status.
****************************************************************************/
tsi_sensor_status_t tsi_sensor_start_single_measurement (void);

/*!
 * @} End of tsi_sensor
 */

/*!
 * @} End of input_report
 */

#endif
/* End of file */
