/*! *********************************************************************************
 * \addtogroup Lampster Custom Profile
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2016, FiranL.
 * All rights reserved.
 *
 * \file lamp_interface.h
 *
 * This file is the interface file for the Lamp Service
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#ifndef _LAMP_INTERFACE_H_
#define _LAMP_INTERFACE_H_

/* ***********************************************************************************
* Include
*********************************************************************************** */
#include "stdbool.h"
#include "board.h"

/* ***********************************************************************************
* Public constants & macros
*********************************************************************************** */

#define timerOff_d 0
#define timerOn_d  1



/* ***********************************************************************************
* Public type definitions
*********************************************************************************** */

/*! Lamp Service - Configuration */
typedef struct lasConfig_tag
{
    uint16_t    serviceHandle;
    
} lasConfig_t;



/* ***********************************************************************************
* Public memory declarations
*********************************************************************************** */

/* ***********************************************************************************
* Public prototypes
*********************************************************************************** */

#ifdef __cplusplus
extern "C" {
#endif

/*!**********************************************************************************
* \brief        Starts Lamp Service functionality
*
* \param[in]    pServiceConfig  Pointer to structure that contains server 
*                               configuration information.
*
* \return       gBleSuccess_c or error.
************************************************************************************/
bleResult_t Las_Start(lasConfig_t *pServiceConfig);

/*!**********************************************************************************
* \brief        Stops Lamp Service functionality
*
* \param[in]    pServiceConfig  Pointer to structure that contains server 
*                               configuration information.
*
* \return       gBleSuccess_c or error.
************************************************************************************/
bleResult_t Las_Stop(lasConfig_t *pServiceConfig);

/*!**********************************************************************************
* \brief        Subscribes a GATT client to the Lamp service
*
* \param[in]    pClient  Client Id in Device DB.
*
* \return       gBleSuccess_c or error.
************************************************************************************/
bleResult_t Las_Subscribe(deviceId_t clientDeviceId);

/*!**********************************************************************************
* \brief        Unsubscribes a GATT client from the Lamp service
*
* \return       gBleSuccess_c or error.
************************************************************************************/
bleResult_t Las_Unsubscribe();



bleResult_t Las_RecordMeasurementTV (uint16_t serviceHandle);

bleResult_t Las_SetLampControl (uint16_t serviceHandle, lamp_control_t control, bool notify, uint16_t speedMs);

bleResult_t Las_SetLampWhite (uint16_t serviceHandle, uint8_t warmW, uint8_t coldW, bool notify, bool showMax);

bleResult_t Las_SetLampRGB (uint16_t serviceHandle, uint8_t red, uint8_t green, uint8_t blue, bool notify);

bleResult_t Las_SetOnTimer  (uint16_t serviceHandle, uint8_t* pSeconds);

/* get remaining time of on timer and update record */
bleResult_t Las_GetOnTimer(uint16_t serviceHandle);

bleResult_t Las_SetOffTimer (uint16_t serviceHandle, uint8_t* pSeconds);

/* get remaining time of off timer and update record */
bleResult_t Las_GetOffTimer(uint16_t serviceHandle);


#ifdef __cplusplus
}
#endif

#endif /* _LAMP_INTERFACE_H_ */

/*! **********************************************************************************
 * @}
 ************************************************************************************/
