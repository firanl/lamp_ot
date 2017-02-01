/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MWS.h
* This is the header file for the Mobile Wireless Standard interface.
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

#ifndef _MWS_H_
#define _MWS_H_ 

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"


/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#ifndef gMWS_Enabled_d
#define gMWS_Enabled_d    1
#endif

#ifndef gMWS_BLE_Priority_d
#define gMWS_BLE_Priority_d 0
#endif 

#ifndef gMWS_ZIGBEE_Priority_d
#define gMWS_ZIGBEE_Priority_d 1
#endif

#ifndef gMWS_BLE_DenyThreshold_d
#define gMWS_BLE_DenyThreshold_d 0
#endif 

#ifndef gMWS_ZIGBEE_DenyThreshold_d
#define gMWS_ZIGBEE_DenyThreshold_d 1
#endif

#ifndef gMWS_BLE_AbortThreshold_d
#define gMWS_BLE_AbortThreshold_d 0
#endif 

#ifndef gMWS_ZIGBEE_AbortThreshold_d
#define gMWS_ZIGBEE_AbortThreshold_d 1
#endif

#ifndef gMWS_ProtocolCount_d 
#define gMWS_ProtocolCount_d 2
#endif 

/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */
typedef enum
{
  gMWS_ErrorNone_c = 0,
  gMWS_Denied_c,
  gMWS_InvalidParameter_c,
  gMWS_Error_c
}mwsStatus_t;

typedef enum
{
  gMWS_BLE_c = 0,
  gMWS_ZIGBEE_c,
  gMWS_None_c
}mwsProtocols_t;
  
typedef enum
{
  gMWS_Idle_c = 0,
  gMWS_BleActive_c,
  gMWS_ZigbeeActive_c,
  gMWS_Abort_c,
  gMWS_Deny_c,
  gMWS_AbortThresholdReached_c,
  gMWS_DenyThresholdReached_c
}mwsProtocolStates_t;

typedef void ( *pfMwsCallBack_t ) ( mwsProtocolStates_t state );

/* Defines the MWS interface structure */
typedef struct mws_tag{    
    uint8_t     priority;
    uint8_t     denyCount;
    uint8_t     denyThreshold;
    uint8_t     abortCount;
    uint8_t     abortThreshold;
    pfMwsCallBack_t callback;
}mws_t;

uint32_t BLE_InactivityDuration(void);


#endif /* _MWS_H_ */