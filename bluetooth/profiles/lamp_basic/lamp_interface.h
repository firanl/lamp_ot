/*! *********************************************************************************
 * \addtogroup Lampster Custom Profile
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2016, FiranL.
 * All rights reserved.
 * \file lamp_interface.h
 * This file is the interface file for the Temperature Service
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

/* ***********************************************************************************
* Public constants & macros
*********************************************************************************** */

/*! Temperature Service - Invalid Value */
#define gTms_InvalidTemperatureValue_c     0x1000

/*! Temperature Service - Maximum Value ( -273.15 C)*/
#define gTms_MinimumTemperatureValue_c     0x954D

/*! Temperature Service - Minimum Value ( -273.15 C)*/
#define gTms_MaximumTemperatureValue_c     0x8FFF

/* ***********************************************************************************
* Public type definitions
*********************************************************************************** */

/*! Lamp Service - Configuration */
typedef struct lasConfig_tag
{
    uint16_t    serviceHandle;
    uint8_t     lampControl;   
    uint16_t     lampWhite;  
    uint32_t     lampRGB;  
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

/*!**********************************************************************************
* \brief        Records Lamp control settings on a specified service handle.
*
* \param[in]    serviceHandle   Service handle.
* \param[in]    value           Lamp control bits.
*
* \return       gBleSuccess_c or error.
************************************************************************************/
bleResult_t Las_RecordLampControl (uint16_t serviceHandle, uint8_t control_1B);
bleResult_t Las_RecordLampWhite   (uint16_t serviceHandle, uint16_t white_2B);
bleResult_t Las_RecordLampRGB     (uint16_t serviceHandle, uint32_t rgb_3B);

bleResult_t Las_RecordMeasurementTV (uint16_t serviceHandle);

#ifdef __cplusplus
}
#endif

#endif /* _LAMP_INTERFACE_H_ */

/*! **********************************************************************************
 * @}
 ************************************************************************************/
