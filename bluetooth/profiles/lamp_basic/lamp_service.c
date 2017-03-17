/*! *********************************************************************************
 * \addtogroup Temperature Custom Profile
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2017, FiranL
 * All rights reserved.
 *
 * \file lamp_service.c
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
#include "TimersManager.h"
/* TPM PWM */
#include "tpm_pwm_led_ctrl.h"
/************************************************************************************
* Private constants & macros
************************************************************************************/

#define MAXOnOffSecondsDay  (60*60*24+1)

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

/* timers 3 */
  /* Fade in out lamp control */
  static tmrTimerID_t tmrFadeId;
  /* Set on lamp timer */
  static tmrTimerID_t tmrOn_secondsId;
  /* Set off lamp timer */
  static tmrTimerID_t tmrOff_secondsId;



/************************************************************************************
* Private functions prototypes
************************************************************************************/
static void Hls_LampNotification(uint16_t handle);


static bleResult_t  Las_RecordLampControl (uint16_t serviceHandle, uint8_t notify);
static bleResult_t  Las_RecordLampWhite   (uint16_t serviceHandle, uint8_t notify);
static bleResult_t  Las_RecordLampRGB     (uint16_t serviceHandle);
static bleResult_t  Las_RecordOnTimer     (uint16_t serviceHandle, uint8_t timerOnOff, uint32_t seconds);

static void FadeTimerCallback(void* pParam);
static void OnTimerCallback(void* pParam);
static void OffTimerCallback(void* pParam);

/************************************************************************************
* Extern functions
************************************************************************************/
extern void ResetMCU(void);

/************************************************************************************
* Public functions
************************************************************************************/

bleResult_t Las_Start (lasConfig_t *pServiceConfig)
{    
    bleResult_t result;
      
    mLas_SubscribedClientId = gInvalidDeviceId_c;
  
    tmrFadeId             = TMR_AllocateTimer();

    
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
    
    // if ctitical or warning values send notification
    //if(g_chip_TV.int16.gCoreTemperature > gCoreTemperatureNotify_d)
    {
      //Hls_LampNotification(handle);
    }

    Hls_LampNotification(handle);
    
   
    /* Get handle of CoreVoltage characteristic */
    result = GattDb_FindCharValueHandleInService(serviceHandle,
        gBleUuidType128_c, pUuidV, &handle);
    
    if (result != gBleSuccess_c) return result;
    
    /* Update characteristic value */
    result = GattDb_WriteAttribute(handle, sizeof(int16_t), (uint8_t*)&g_chip_TV.int16.g_vReference);

    if (result != gBleSuccess_c) return result;
    
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
    /* check if reset */
    /* TODO: send it twice ? */
    if(lamp_ctrl.raw8 == 0x04) { ResetMCU(); return result; }
    
    
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


bleResult_t Las_SetOnTimer(uint16_t serviceHandle, uint8_t* pSeconds)
{
    tmrErrCode_t tmrerr = gTmrInvalidId_c;
    uint32_t seconds=0;
    
    seconds= *( (uint32_t*) pSeconds); 
    
    if(seconds>0)
    {
      tmrOn_secondsId  = TMR_AllocateTimer();
      tmrerr = TMR_StartSingleShotTimer(tmrOn_secondsId, TmrSeconds(seconds), OnTimerCallback, NULL);
    }
    else
    {
      tmrerr = TMR_FreeTimer(tmrOn_secondsId);
    }
    
    Las_RecordOnTimer(serviceHandle, 1, seconds);
    
    return gBleSuccess_c;
}

/* get remaining time of on timer and update record */
bleResult_t Las_GetOnTimer(uint16_t serviceHandle)
{
    uint32_t seconds=0;
    
    seconds = TMR_GetRemainingTime(tmrOn_secondsId)  / 1000;
    Las_RecordOnTimer(serviceHandle, 1, seconds);
    
    return gBleSuccess_c;
}

bleResult_t Las_SetOffTimer(uint16_t serviceHandle, uint8_t* pSeconds)
{
    tmrErrCode_t tmrerr = gTmrInvalidId_c;
    uint32_t seconds=0;
    
    seconds= *( (uint32_t*) pSeconds); 
    
    if(seconds>0)
    {
      tmrOff_secondsId  = TMR_AllocateTimer();
      tmrerr = TMR_StartSingleShotTimer(tmrOff_secondsId, TmrSeconds(seconds), OffTimerCallback, NULL);
    }
    else
    {
       tmrerr = TMR_FreeTimer(tmrOff_secondsId);
    }

    Las_RecordOnTimer(serviceHandle, 0, seconds);
    
    return gBleSuccess_c;
}
             
/* get remaining time of off timer and update record */
bleResult_t Las_GetOffTimer(uint16_t serviceHandle)
{
    uint32_t seconds=0;
    
    seconds = TMR_GetRemainingTime(tmrOff_secondsId) / 1000;
    Las_RecordOnTimer(serviceHandle, 0, seconds);
    
    return gBleSuccess_c;
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

      if (notify) Hls_LampNotification(handle);
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

      if (notify) Hls_LampNotification(handle);
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
      
      /* Update characteristic value - only first 3 bytes */
      result = GattDb_WriteAttribute(handle, 3, (uint8_t*)&lamp_NVdata.lampRGB.raw32);

      if (result != gBleSuccess_c)
          return result;
    }

    return gBleSuccess_c;
}

static bleResult_t Las_RecordOnTimer (uint16_t serviceHandle, uint8_t timerOnOff, uint32_t seconds)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid ;

    if(timerOnOff)
      pUuid = (bleUuid_t*)&uuid_char_lamp_on_sec;
    else
      pUuid = (bleUuid_t*)&uuid_char_lamp_off_sec;
    
    if(mLas_SubscribedClientId != gInvalidDeviceId_c)
    {
      /* Get handle of Temperature characteristic */
      result = GattDb_FindCharValueHandleInService(serviceHandle,
          gBleUuidType128_c, pUuid, &handle);

      if (result != gBleSuccess_c)
          return result;
      
      /* Update characteristic value */
      result = GattDb_WriteAttribute(handle, sizeof(uint32_t), (uint8_t*)&seconds);

      if (result != gBleSuccess_c)
          return result;

    }

    return gBleSuccess_c;
}


/* notifications */

static void Hls_LampNotification( uint16_t handle )
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
* \brief        On Timer for lamp to start after some seconds.
*
* \param[in]    pParam        Callback parameters.
********************************************************************************** */
static void OnTimerCallback(void * pParam)
{
    uint8_t control;
    
    /*  lamp was set before */

    /* turn on lamp */
    control = lamp_NVdata.lampControl.raw8 | 0x80;
    Las_SetLampControl (mLas_serviceHandle, control, TRUE);
}

/*! *********************************************************************************
* \brief        Off Timer for lamp stop after some seconds.
*
* \param[in]    pParam        Callback parameters.
********************************************************************************** */
static void OffTimerCallback(void * pParam)
{
    uint8_t control;

    /* turn off lamp and notify */
    control = lamp_NVdata.lampControl.raw8 & 0x7F;
    Las_SetLampControl (mLas_serviceHandle, control, TRUE);  
}

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
