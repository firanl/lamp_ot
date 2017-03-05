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
* Include
************************************************************************************/
#include "FunctionLib.h"
#include "ble_general.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h" // Include this file for the 128 bit characteristic UUIDs. Do not access the handles directlty!
#include "gatt_server_interface.h"
#include "gap_interface.h"
#include "lamp_interface.h"
#include "temperature_sensor.h"
#include "board.h"
/* TPM PWM */
#include "tpm_pwm_led_ctrl.h"
/************************************************************************************
* Private constants & macros
************************************************************************************/

/* core temperature T, signed exponent -2 */
/* core voltage reference V, signed exponent -3 */
extern chip_TempVoltage_t g_chip_TV;

/* lamp control light data */
extern lamp_NVdata_t lamp_NVdata;

/************************************************************************************
* Private type definitions
************************************************************************************/

/* ***********************************************************************************
* Private memory declarations
*********************************************************************************** */

/*! Lamp basic Service - Subscribed Client*/
static deviceId_t mLas_SubscribedClientId;

static deviceId_t mLas_serviceHandle;


/************************************************************************************
* Private functions prototypes
************************************************************************************/
// Short BTN press changes
static void Hls_LampControlNotification(uint16_t handle );
// Long BTN press canges
static void Hls_LampWhiteNotification(uint16_t handle);
static bleResult_t  Las_RecordLampControl (uint16_t serviceHandle, uint8_t notify);
static bleResult_t  Las_RecordLampWhite   (uint16_t serviceHandle, uint8_t notify);
static bleResult_t  Las_RecordLampRGB     (uint16_t serviceHandle);

/************************************************************************************
* Public functions
************************************************************************************/
bleResult_t Las_Start (lasConfig_t *pServiceConfig)
{    
    bleResult_t result;
      
    mLas_SubscribedClientId = gInvalidDeviceId_c;
    
//
    /* switch to pre-reset light */
    if(lamp_NVdata.lampControl.bit.OnOff)
    {
      /* the lamp is on */
      if(lamp_NVdata.lampControl.bit.White)
      {
        /* white light is on */
        TPM_PWM_WarmWhite(lamp_NVdata.lampWhite.uint8.warmW);
        TPM_PWM_ColdWhite(lamp_NVdata.lampWhite.uint8.coldW);
      }
      else
      {
        TPM_PWM_WarmWhiteOff();
        TPM_PWM_ColdWhiteOff();
      }
      
      if(lamp_NVdata.lampControl.bit.Color)
      {
         /* color RGB light is on */
         TPM_PWM_Red  (lamp_NVdata.lampRGB.uint8.r);
         TPM_PWM_Green(lamp_NVdata.lampRGB.uint8.g);
         TPM_PWM_Blue (lamp_NVdata.lampRGB.uint8.b);
      } 
      else
      {
         TPM_PWM_RedOff  ();
         TPM_PWM_GreenOff();
         TPM_PWM_BlueOff ();
      }
      
    }
    else  
    {
      TPM_PWM_Off();
    }
//
    //TODO entry change 
    //Las_SetLampWhite (uint16_t serviceHandle, uint8_t warmW, uint8_t coldW, uint8_t notify)
    //Las_SetLampRGB (uint16_t serviceHandle, uint8_t red, uint8_t green, uint8_t blue)
    //Las_SetLampControl (mLas_serviceHandle, control, TRUE);
    
    result = Las_RecordLampControl (pServiceConfig->serviceHandle, FALSE);
    if(result != gBleSuccess_c) return result;

    result = Las_RecordLampWhite (pServiceConfig->serviceHandle, FALSE);
    if(result != gBleSuccess_c) return result; 

    result = Las_RecordLampRGB (pServiceConfig->serviceHandle);
    if(result != gBleSuccess_c) return result;   
    
    mLas_serviceHandle = pServiceConfig->serviceHandle;
    
    return result;
}

bleResult_t Las_Stop (lasConfig_t *pServiceConfig)
{
    return Las_Unsubscribe();
}


bleResult_t Las_Subscribe(deviceId_t deviceId)
{
   uint8_t control;
   
    mLas_SubscribedClientId = deviceId;
   
    
    /* if conection enable flag turn lamp on */
    if(lamp_NVdata.lampControl.bit.BTcon)
    {
      /* and lamp was set before */
      if(lamp_NVdata.lampControl.bit.White || lamp_NVdata.lampControl.bit.Color)
      {
        control = lamp_NVdata.lampControl.raw8 | 0x80;
        Las_SetLampControl (mLas_serviceHandle, control, TRUE);
      }
    }   

    return gBleSuccess_c;
}

bleResult_t Las_Unsubscribe()
{
   uint8_t control;

    mLas_SubscribedClientId = gInvalidDeviceId_c;
    
    //if conection enable flag turn lamp off
    if(lamp_NVdata.lampControl.bit.BTcon)
    {
      /* and lamp was set before */
      if(lamp_NVdata.lampControl.bit.White || lamp_NVdata.lampControl.bit.Color)
      {
        control = lamp_NVdata.lampControl.raw8 & 0x7F;
        Las_SetLampControl (mLas_serviceHandle, control, FALSE);   
      }
    }
    
    return gBleSuccess_c;
}


bleResult_t Las_RecordMeasurementTV (uint16_t serviceHandle)
{
    uint16_t  handle;
    uint16_t  hCccd;
    bool_t isNotificationActive;
    bleResult_t result;
    bleUuid_t uuidT = Uuid16(gBleSig_Temperature_d);
    bleUuid_t* pUuidV = (bleUuid_t*)&uuid_char_core_voltage;

    /* Get handle of CoreTemperature characteristic */
    result = GattDb_FindCharValueHandleInService(serviceHandle,
        gBleUuidType16_c, &uuidT, &handle);

    if (result != gBleSuccess_c) return result;

    /* Update characteristic value  */
    result = GattDb_WriteAttribute(handle, sizeof(int16_t), (uint8_t*) &g_chip_TV.int16.gCoreTemperature);

    if (result != gBleSuccess_c) return result;
   
    /* Get handle of CoreVoltage characteristic */
    result = GattDb_FindCharValueHandleInService(serviceHandle,
        gBleUuidType128_c, pUuidV, &handle);
    
    if (result != gBleSuccess_c) return result;
    
    /* Update characteristic value */
    result = GattDb_WriteAttribute(handle, sizeof(int16_t), (uint8_t*)&g_chip_TV.int16.g_vReference);

    if (result != gBleSuccess_c) return result;
    
    // if ctitical or warning values send notification
    if(g_chip_TV.int16.gCoreTemperature > gCoreTemperatureNotify_d)
    {
        /* Get handle of CCCD */
        if (GattDb_FindCccdHandleForCharValueHandle(serviceHandle, &hCccd) == gBleSuccess_c)
        {
          if (gBleSuccess_c == Gap_CheckNotificationStatus
              (mLas_SubscribedClientId, hCccd, &isNotificationActive) &&
              TRUE == isNotificationActive)
          {
              GattServer_SendNotification(mLas_SubscribedClientId, serviceHandle);
          }
        }
    }

    return gBleSuccess_c;
}




bleResult_t Las_SetLampControl (uint16_t serviceHandle, uint8_t control, uint8_t notify)
{
  bleResult_t result = gBleSuccess_c;
  lamp_control_t lamp_ctrl;

  lamp_ctrl.raw8 = control;
  
  /* see some differences from old control */   
  if (lamp_ctrl.raw8 != lamp_NVdata.lampControl.raw8)
  { 
    lamp_NVdata.lampControl.bit.BTcon = lamp_ctrl.bit.BTcon;
    lamp_NVdata.lampControl.bit.mix = lamp_ctrl.bit.mix; 
    
    /* the on off state is changed */
    if(lamp_ctrl.bit.OnOff != lamp_NVdata.lampControl.bit.OnOff)
    {
      lamp_NVdata.lampControl.bit.White = lamp_ctrl.bit.White; 
      lamp_NVdata.lampControl.bit.Color = lamp_ctrl.bit.Color; 
      
      /* turn lamp on - stat fade on */
      if(lamp_ctrl.bit.OnOff)
      {      
        ///===== fade on
          if(lamp_NVdata.lampControl.bit.White)
          {
            /* white light is on */
            TPM_PWM_WarmWhite(lamp_NVdata.lampWhite.uint8.warmW);
            TPM_PWM_ColdWhite(lamp_NVdata.lampWhite.uint8.coldW);
          }
          else
          {
            TPM_PWM_WarmWhiteOff();
            TPM_PWM_ColdWhiteOff();
          }
          
          if(lamp_NVdata.lampControl.bit.Color)
          {
             /* color RGB light is on */
             TPM_PWM_Red  (lamp_NVdata.lampRGB.uint8.r);
             TPM_PWM_Green(lamp_NVdata.lampRGB.uint8.g);
             TPM_PWM_Blue (lamp_NVdata.lampRGB.uint8.b);
          } 
          else
          {
             TPM_PWM_RedOff  ();
             TPM_PWM_GreenOff();
             TPM_PWM_BlueOff ();
          }
        ///=====      
      }
      /* turn lamp off - stat fade off */
      else
      {
      ///===== fade off
        TPM_PWM_Off();
      ///=====
      }
      
      lamp_NVdata.lampControl.bit.OnOff = lamp_ctrl.bit.OnOff; 
    }
    /* the White or RGB state is changed */
    else
    {
      lamp_NVdata.lampControl.bit.White = lamp_ctrl.bit.White; 
      lamp_NVdata.lampControl.bit.Color = lamp_ctrl.bit.Color; 
      
      if(lamp_NVdata.lampControl.bit.White)
      {
        /* white light is on */
        TPM_PWM_WarmWhite(lamp_NVdata.lampWhite.uint8.warmW);
        TPM_PWM_ColdWhite(lamp_NVdata.lampWhite.uint8.coldW);
      }
      else
      {
        TPM_PWM_WarmWhiteOff();
        TPM_PWM_ColdWhiteOff();
      }
      
      if(lamp_NVdata.lampControl.bit.Color)
      {
         /* color RGB light is on */
         TPM_PWM_Red  (lamp_NVdata.lampRGB.uint8.r);
         TPM_PWM_Green(lamp_NVdata.lampRGB.uint8.g);
         TPM_PWM_Blue (lamp_NVdata.lampRGB.uint8.b);
      } 
      else
      {
         TPM_PWM_RedOff  ();
         TPM_PWM_GreenOff();
         TPM_PWM_BlueOff ();
      }
    
    }
    
    /* update DB */
    result = Las_RecordLampControl(serviceHandle, notify);
  }
  
  
  return result;
}

  
bleResult_t Las_SetLampWhite (uint16_t serviceHandle, uint8_t warmW, uint8_t coldW, uint8_t notify)
{
  bleResult_t result = gBleSuccess_c;
  uint8_t upd_db = 0;
  
  if (warmW != lamp_NVdata.lampWhite.uint8.warmW)
  {
    upd_db = 1;
    lamp_NVdata.lampWhite.uint8.warmW = warmW;
    if (lamp_NVdata.lampControl.bit.OnOff && lamp_NVdata.lampControl.bit.White)
    {
      TPM_PWM_WarmWhite(lamp_NVdata.lampWhite.uint8.warmW);
    }
  }
  
  if (coldW != lamp_NVdata.lampWhite.uint8.coldW)
  {
    upd_db = 1;
    lamp_NVdata.lampWhite.uint8.coldW = coldW;
    if (lamp_NVdata.lampControl.bit.OnOff && lamp_NVdata.lampControl.bit.White)
    {
      TPM_PWM_ColdWhite(lamp_NVdata.lampWhite.uint8.coldW); 
    }
  }  
  
  if(upd_db)
  {
    /* update DB */
    result = Las_RecordLampWhite(serviceHandle, notify);  
  }
  
  return result;
}


bleResult_t Las_SetLampRGB (uint16_t serviceHandle, uint8_t red, uint8_t green, uint8_t blue)
{
  bleResult_t result = gBleSuccess_c;
  uint8_t upd_db = 0;
  
  if (red != lamp_NVdata.lampRGB.uint8.r)
  {
    upd_db = 1;
    lamp_NVdata.lampRGB.uint8.r = red;
    if (lamp_NVdata.lampControl.bit.OnOff && lamp_NVdata.lampControl.bit.Color)
    {
      TPM_PWM_Red  (lamp_NVdata.lampRGB.uint8.r);
    }
  }  
  
  if (green != lamp_NVdata.lampRGB.uint8.g)
  {
    upd_db = 1;
    lamp_NVdata.lampRGB.uint8.g = green;
    if (lamp_NVdata.lampControl.bit.OnOff && lamp_NVdata.lampControl.bit.Color)
    {
      TPM_PWM_Green(lamp_NVdata.lampRGB.uint8.g);
    }
  }  

  if (blue != lamp_NVdata.lampRGB.uint8.b)
  {
    upd_db = 1;
    lamp_NVdata.lampRGB.uint8.b = blue;
    if (lamp_NVdata.lampControl.bit.OnOff && lamp_NVdata.lampControl.bit.Color)
    {
      TPM_PWM_Blue (lamp_NVdata.lampRGB.uint8.b);
    }
  }    
  
  if(upd_db)
  {
    /* update DB */
    result = Las_RecordLampRGB(serviceHandle);  
  } 
  
    return result;
}


             
             

/************************************************************************************
* Private functions
************************************************************************************/

static bleResult_t Las_RecordLampControl (uint16_t serviceHandle, uint8_t notify)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid = (bleUuid_t*)&uuid_char_lamp_Control;

    
    if(mLas_SubscribedClientId != gInvalidDeviceId_c)
    {
      /* Get handle of Temperature characteristic */
      result = GattDb_FindCharValueHandleInService(serviceHandle,
          gBleUuidType128_c, pUuid, &handle);

      if (result != gBleSuccess_c)
          return result;
      
      /* Update characteristic value */
      result = GattDb_WriteAttribute(handle, sizeof(uint8_t), (uint8_t*)&lamp_NVdata.lampControl.raw8);

      if (result != gBleSuccess_c)
          return result;

      if (notify) Hls_LampControlNotification(handle);
    }

    return gBleSuccess_c;
}

static bleResult_t Las_RecordLampWhite (uint16_t serviceHandle, uint8_t notify)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid = (bleUuid_t*)&uuid_char_lamp_White;

    if(mLas_SubscribedClientId != gInvalidDeviceId_c)
    {
      /* Get handle of Temperature characteristic */
      result = GattDb_FindCharValueHandleInService(serviceHandle,
          gBleUuidType128_c, pUuid, &handle);

      if (result != gBleSuccess_c)
          return result;
      
      /* Update characteristic value */
      result = GattDb_WriteAttribute(handle, sizeof(uint16_t), (uint8_t*)&lamp_NVdata.lampWhite.raw16);

      if (result != gBleSuccess_c)
          return result;

      if (notify) Hls_LampWhiteNotification(handle);
    }

    return gBleSuccess_c;
}

static bleResult_t Las_RecordLampRGB (uint16_t serviceHandle)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid = (bleUuid_t*)&uuid_char_lamp_RGB;

    if(mLas_SubscribedClientId != gInvalidDeviceId_c)
    {
      /* Get handle of Temperature characteristic */
      result = GattDb_FindCharValueHandleInService(serviceHandle,
          gBleUuidType128_c, pUuid, &handle);

      if (result != gBleSuccess_c)
          return result;
      
      /* Update characteristic value */
      result = GattDb_WriteAttribute(handle, 3, (uint8_t*)&lamp_NVdata.lampRGB.raw32);

      if (result != gBleSuccess_c)
          return result;
    }

    return gBleSuccess_c;
}

// Short BTN press changes
static void Hls_LampControlNotification( uint16_t handle )
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
static void Hls_LampWhiteNotification( uint16_t handle )
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



/*! *********************************************************************************
 * @}
 ********************************************************************************** */
