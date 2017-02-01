/*!
 * \defgroup buzzer Buzzer
 * @{
 * Buzzer module implements functions to control a DC buzzer using PWM.
 ******************************************************************************/
/*!
* @file buzzer_driver.h
*
* @author  
*
* @version 1.0
*
* @date Mar-14-2016
*
* @brief Driver for Buzzer Driver interface
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

#ifndef _BUZZER_DRIVER_H_
#define _BUZZER_DRIVER_H_

/******************************************************************************
* Header files
******************************************************************************/
// Stack header files
#include "EmbeddedTypes.h"

/******************************************************************************
* User definitions
*******************************************************************************/
#define BUZZER_DRIVER_PWM_CHANNEL       1       /*!< TPM PWM channel to use */
#define BUZZER_DRIVER_TPM_MODULE        1       /*!< TPM module to use */
#define BUZZER_DRIVER_FREQUENCY_HZ      1000    /*!< Buzzer signal frequency in Hz */

/******************************************************************************
* Type definitions
******************************************************************************/
/*! Buzzer driver status */
typedef enum buzzerDriverStatus{
  kBuzzerDriverOk,              /*!< No error */
  kBuzzerDriverInitError,       /*!< Error during initialization */
  kBuzzerPwmStartError,         /*!< Error starting the PWM channel */
}buzzer_driver_status_t;
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
* \brief        Initializes the Buzzer driver
*
* \param[in]    None
*
* \return       @ref buzzer_driver_status_t Error status
****************************************************************************/
buzzer_driver_status_t buzzer_driver_init (void);

/*!***************************************************************************
* \brief        Change the current buzzer status
*
* \param[in]    buzzerStatus The buzzer status to set (0: Off, 1: On)
*
* \return       @ref buzzer_driver_status_t Error status
****************************************************************************/
buzzer_driver_status_t buzzer_driver_change_buzzer_status (uint8_t buzzerStatus);

/*!
 * @} End of buzzer
 */


#endif
/* End of file */
