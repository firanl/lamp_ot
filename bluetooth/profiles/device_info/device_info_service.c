/*! *********************************************************************************
* \addtogroup Device Information Service
* @{
********************************************************************************** */
/*!
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * \file device_info_service.c
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
* Include
************************************************************************************/
#include <stdio.h>

#include "ble_general.h"
#include "gatt_db_app_interface.h"
#include "gatt_server_interface.h"
#include "gap_interface.h"
#include "device_info_interface.h"
#include "OtaSupport.h"

#include "board.h"

/************************************************************************************
* Private constants & macros
************************************************************************************/

/* Hardware Revision String */
#if defined(__IAR_SYSTEMS_ICC__)
  #pragma location = "HwRevStringMem"
  const uint8_t HRS[4] = 
#elif defined(__GNUC__)
  const uint8_t HRS[4]  __attribute__ ((section(".HwRevStringMem"))) = 
#else
  const uint8_t HRS[4] = 
#endif
{ DI_HardwareRevisionString };

/* Serial Number */
#if defined(__IAR_SYSTEMS_ICC__)
  #pragma location = "SerialNumberMem"
  const serial_number_t SerialNumber = 
#elif defined(__GNUC__)
  const serial_number_t SerialNumber  __attribute__ ((section(".SerialNumberMem"))) = 
#else
  const serial_number_t SerialNumber = 
#endif
{ DI_SerialNumberMem };


/************************************************************************************
* Private type definitions
************************************************************************************/



/************************************************************************************
* Private memory declarations
************************************************************************************/
extern const bootInfo_t gBootFlags;
extern uint32_t SERIAL_NUMBER_ADDR[];

/************************************************************************************
* Private functions prototypes
************************************************************************************/
static bleResult_t Dis_UpdateDB(uint16_t serviceHandle, bleUuid_t uuid, uint8_t stringLength, const uint8_t *pUtf8s);


/************************************************************************************
* Public functions
************************************************************************************/

bleResult_t Dis_Start (disConfig_t *pServiceConfig)
{
    bleResult_t result = gBleSuccess_c;
    bleUuid_t uuid;
    char SerialNumberString[15] = {0};
    uint8_t a;

    /* convert serial number from memory 8 bytes to ASCII */
    sprintf(SerialNumberString,"%02d-%04d-%05d", SerialNumber.param.Prod, SerialNumber.param.DOY, SerialNumber.param.UnitId);
    
    uuid.uuid16 = gBleSig_SerialNumberString_d;
    result = Dis_UpdateDB(pServiceConfig->serviceHandle, uuid, sizeof(DI_SerialNumberString), (uint8_t*) SerialNumberString);

    uuid.uuid16 = gBleSig_HardwareRevisionString_d;
    result = Dis_UpdateDB(pServiceConfig->serviceHandle, uuid, 4, HRS);
    
    uuid.uuid16 = gBleSig_FirmwareRevisionString_d;
    result = Dis_UpdateDB(pServiceConfig->serviceHandle, uuid, 2, gBootFlags.version);

    return result;
}

bleResult_t Dis_Stop (disConfig_t *pServiceConfig)
{
    return gBleSuccess_c;
}

/************************************************************************************
* Private functions
************************************************************************************/

static bleResult_t Dis_UpdateDB(uint16_t serviceHandle, bleUuid_t uuid, uint8_t stringLength, const uint8_t *pUtf8s)
{
    uint16_t  handle;
    bleResult_t result = gBleSuccess_c;


    /* Get handle of characteristic */
    result = GattDb_FindCharValueHandleInService(serviceHandle,
        gBleUuidType16_c, &uuid, &handle);

    if (result != gBleSuccess_c)
        return result;

    /* Update characteristic value*/
    result = GattDb_WriteAttribute(handle, stringLength, (uint8_t*) pUtf8s);

    return result;
}


/*! *********************************************************************************
* @}
********************************************************************************** */
