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
  

/******************************************************************************
* Type definitions
******************************************************************************/


typedef struct tsi_touch_tag {
    uint16_t low;
    uint16_t sensitivity;
} tsi_touch_t;

/*!
 * TSI Sensor return status for functions
 */
typedef enum tsiSensorStatus{
  kTsiOk,               /*!< No error */
  kTsiInitError,        /*!< Error initializing the module */
  kTsiStartError,       /*!< Error starting the measurements */
}tsi_sensor_status_t;


/*!
 * TSI Sensor callback function type
 */
typedef void (*tsi_sensor_callback_t) (uint8_t events);

/*
 * Name: tsi_event_t
 * Description: tsi event call (see the following enumerations)
 */
typedef uint8_t tsi_event_t;

/*
 * Description: TSI push events defintion
 */
enum
{
    gTSI_EventShortPush_c = 1,         /* Short push button  */
    gTSI_EventLongPush_c,              /* Long push button  */
};

/******************************************************************************
* Globals
*******************************************************************************/


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
void TSI_Init(void);

/*!***************************************************************************
* \brief        Starts a single TSI sensor measurement.
*
* \param[in]    None.
*
* \return       @ref tsi_sensor_status_t Error status.
****************************************************************************/
void TSI_MeasureOnce(void);

/*!
 * @} End of tsi_sensor
 */


#endif
/* End of file */
