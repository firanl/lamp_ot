/*! *********************************************************************************
 * \addtogroup Temperature Custom Profile
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * \file temperature_service.c
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

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "FunctionLib.h"
#include "ble_general.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h" // Include this file for the 128 bit characteristic UUIDs. Do not access the handles directlty!
#include "gatt_server_interface.h"
#include "gap_interface.h"
#include "lamp_interface.h"
/************************************************************************************
*************************************************************************************
* Private constants & macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/*! Temperature Service - Subscribed Client*/
static deviceId_t mLas_SubscribedClientId;

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/
// Short BTN press changes
static void Hls_LampControlNotification
(
    uint16_t handle
);

// Long BTN press canges
static void Hls_LampWhiteNotification 
(
    uint16_t handle
);
/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
bleResult_t Las_Start (lasConfig_t *pServiceConfig)
{    
    bleResult_t result;
      
    mLas_SubscribedClientId = gInvalidDeviceId_c;
    
    result = Las_RecordLampControl (pServiceConfig->serviceHandle, 
                                             pServiceConfig->lampControl);
    if(result != gBleSuccess_c) return result;
    
    result = Las_RecordLampWhite (pServiceConfig->serviceHandle, 
                                             pServiceConfig->lampWhite);
    if(result != gBleSuccess_c) return result; 
    
    result = Las_RecordLampRGB (pServiceConfig->serviceHandle, 
                                             pServiceConfig->lampRGB);
    if(result != gBleSuccess_c) return result;   
    
    return result;
}

bleResult_t Las_Stop (lasConfig_t *pServiceConfig)
{
    return Las_Unsubscribe();
}

bleResult_t Las_Subscribe(deviceId_t deviceId)
{
    mLas_SubscribedClientId = deviceId;

    return gBleSuccess_c;
}

bleResult_t Las_Unsubscribe()
{
    mLas_SubscribedClientId = gInvalidDeviceId_c;
    
    return gBleSuccess_c;
}

bleResult_t Las_RecordLampControl (uint16_t serviceHandle, uint8_t control_1B)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid = (bleUuid_t*)&uuid_char_lamp_Control;

    /* Get handle of Temperature characteristic */
    result = GattDb_FindCharValueHandleInService(serviceHandle,
        gBleUuidType128_c, pUuid, &handle);

    if (result != gBleSuccess_c)
        return result;
    
    /* Update characteristic value */
    result = GattDb_WriteAttribute(handle, sizeof(uint8_t), (uint8_t*)&control_1B);

    if (result != gBleSuccess_c)
        return result;

    Hls_LampControlNotification(handle);

    return gBleSuccess_c;
}

bleResult_t Las_RecordLampWhite (uint16_t serviceHandle, uint16_t white_2B)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid = (bleUuid_t*)&uuid_char_lamp_White;

    /* Get handle of Temperature characteristic */
    result = GattDb_FindCharValueHandleInService(serviceHandle,
        gBleUuidType128_c, pUuid, &handle);

    if (result != gBleSuccess_c)
        return result;
    
    /* Update characteristic value */
    result = GattDb_WriteAttribute(handle, sizeof(uint16_t), (uint8_t*)&white_2B);

    if (result != gBleSuccess_c)
        return result;

    Hls_LampControlNotification(handle);

    return gBleSuccess_c;
}

bleResult_t Las_RecordLampRGB (uint16_t serviceHandle, uint32_t rgb_3B)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid = (bleUuid_t*)&uuid_char_lamp_RGB;

    /* Get handle of Temperature characteristic */
    result = GattDb_FindCharValueHandleInService(serviceHandle,
        gBleUuidType128_c, pUuid, &handle);

    if (result != gBleSuccess_c)
        return result;
    
    /* Update characteristic value */
    result = GattDb_WriteAttribute(handle, 3, (uint8_t*)&rgb_3B);

    if (result != gBleSuccess_c)
        return result;

    Hls_LampControlNotification(handle);

    return gBleSuccess_c;
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
// Short BTN press changes
static void Hls_LampControlNotification
(
  uint16_t handle
)
{
    uint16_t  hCccd;
    bool_t isNotificationActive;

    /* Get handle of CCCD */
    if (GattDb_FindCccdHandleForCharValueHandle(handle, &hCccd) != gBleSuccess_c)
        return;

    if (gBleSuccess_c == Gap_CheckNotificationStatus
        (mLas_SubscribedClientId, hCccd, &isNotificationActive) &&
        TRUE == isNotificationActive)
    {
        GattServer_SendNotification(mLas_SubscribedClientId, handle);
    }
}

// Long BTN press canges
static void Hls_LampWhiteNotification 
(
  uint16_t handle
)
{
    uint16_t  hCccd;
    bool_t isNotificationActive;

    /* Get handle of CCCD */
    if (GattDb_FindCccdHandleForCharValueHandle(handle, &hCccd) != gBleSuccess_c)
        return;

    if (gBleSuccess_c == Gap_CheckNotificationStatus
        (mLas_SubscribedClientId, hCccd, &isNotificationActive) &&
        TRUE == isNotificationActive)
    {
        GattServer_SendNotification(mLas_SubscribedClientId, handle);
    }
}

//BOARD_chip_temperature();

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
