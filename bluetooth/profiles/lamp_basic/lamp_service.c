/*! *********************************************************************************
 * \addtogroup Lampster Custom Profile
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

/* lamp cfg params */
extern lamp_config_t lamp_cfg;

/************************************************************************************
* Private type definitions
************************************************************************************/

/* ***********************************************************************************
* Private memory declarations
*********************************************************************************** */

/*! Lamp basic Service - Subscribed Client*/
static deviceId_t mLas_SubscribedClientId;

static deviceId_t mLas_serviceHandle;

static bool FadeOnMutex;

/* timers 3 */
  /* Fade in out lamp control */
  static tmrTimerID_t tmrFadeId;
  /* Set on lamp timer */
  static tmrTimerID_t tmrOn_secondsId;
  /* Set off lamp timer */
  static tmrTimerID_t tmrOff_secondsId;

static lamp_NVdata_t temp_lamp_NVdata; 

/************************************************************************************
* Private functions prototypes
************************************************************************************/
static void Hls_LampNotification(uint16_t handle);


static bleResult_t  Las_RecordLampControl (uint16_t serviceHandle, uint8_t notify);
static bleResult_t  Las_RecordLampWhite   (uint16_t serviceHandle, uint8_t notify);
static bleResult_t  Las_RecordLampRGB     (uint16_t serviceHandle, uint8_t notify);
static bleResult_t  Las_RecordOnTimer     (uint16_t serviceHandle, uint8_t timerOnOff, uint32_t seconds);

static void FadeTimerCallback(void* pParam);
static void BlinkTimerCallback(void * pParam);
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
    lamp_control_t control;
      
    mLas_SubscribedClientId = gInvalidDeviceId_c;
    
    TPM_PWM_Off();
    
    FadeOnMutex=false;
    temp_lamp_NVdata.lampControl.raw8 = 0;

    /* TODO: get data from Flash using some board.c function to read */  
    result = Las_SetLampWhite   (pServiceConfig->serviceHandle, LA_LAMP_WARM_WHITE, LA_LAMP_COLD_WHITE, FALSE, FALSE);
    result = Las_SetLampRGB     (pServiceConfig->serviceHandle, LA_LAMP_R, LA_LAMP_G, LA_LAMP_B, FALSE);
    control.raw8 = LA_LAMP_CONTROL;
    result = Las_SetLampControl (pServiceConfig->serviceHandle, control, FALSE, lamp_cfg.fadeTimeMs);
    
    
    mLas_serviceHandle = pServiceConfig->serviceHandle;
    
    return result;
}

bleResult_t Las_Stop (lasConfig_t *pServiceConfig)
{
    return Las_Unsubscribe();
}


bleResult_t Las_Subscribe(deviceId_t deviceId)
{
   lamp_control_t control;
   
    mLas_SubscribedClientId = deviceId;
   
    
    /* if conection enable flag turn lamp on */
    if(lamp_NVdata.lampControl.bit.BTcon)
    {
      /* and lamp was set before */
      if( (!lamp_NVdata.lampControl.bit.OnOff) && (lamp_NVdata.lampControl.bit.White || lamp_NVdata.lampControl.bit.Color) )
      {
        control = lamp_NVdata.lampControl;
        control.bit.OnOff = 1;
        Las_SetLampControl (mLas_serviceHandle, control, TRUE, lamp_cfg.fadeTimeMs);
      }
    }   

    return gBleSuccess_c;
}

bleResult_t Las_Unsubscribe()
{
   lamp_control_t control;

    mLas_SubscribedClientId = gInvalidDeviceId_c;
    
    //if conection enable flag turn lamp off
    if(lamp_NVdata.lampControl.bit.BTcon)
    {
      /* and lamp was set before */
      if(lamp_NVdata.lampControl.bit.OnOff && ( lamp_NVdata.lampControl.bit.White || lamp_NVdata.lampControl.bit.Color) )
      {
        control = lamp_NVdata.lampControl;
        control.bit.OnOff = 0;
        Las_SetLampControl (mLas_serviceHandle, control, FALSE, lamp_cfg.fadeTimeMs);   
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

    /* notify temperature */
    Hls_LampNotification(handle);
    
   
    /* Get handle of CoreVoltage characteristic */
    result = GattDb_FindCharValueHandleInService(serviceHandle,
        gBleUuidType128_c, pUuidV, &handle);
    
    if (result != gBleSuccess_c) return result;
    
    /* Update characteristic value */
    result = GattDb_WriteAttribute(handle, sizeof(int16_t), (uint8_t*)&g_chip_TV.int16.g_vReference);

    if (result != gBleSuccess_c) return result;
    
    /* notify voltage */
    Hls_LampNotification(handle);    
    
    return gBleSuccess_c;
}




bleResult_t Las_SetLampControl (uint16_t serviceHandle, lamp_control_t control, bool notify, uint8_t speedMs)
{
  bleResult_t result = gBleSuccess_c;


  /* check if reset */
  /* TODO: send it twice ? */
  if(control.raw8 == 0x04) { ResetMCU(); return gBleSuccess_c; }
  
  /* if fade timer is on do not enter control */ 
  if(FadeOnMutex) { return gBleUnavailable_c; }

  if (control.raw8 != lamp_NVdata.lampControl.raw8)
  {
    temp_lamp_NVdata.lampControl.raw8 = 0;
    lamp_NVdata.lampControl.bit.BTcon = control.bit.BTcon;
    lamp_NVdata.lampControl.bit.mix   = control.bit.mix; 
    
    /* do not permit special states */
      /* 000x xxxx */
      if( !(control.raw8 & 0xE0) ) { return gBleUnavailable_c; }
      /* 100x xxxx */
      if( (control.raw8 & 0xE0) == 0x80 ) { return gBleUnavailable_c; }
  
    /* turn off lamp - off state 0xx ( and 000 ) */
    if( !control.bit.OnOff ) 
    {
        lamp_NVdata.lampControl.bit.White = control.bit.White; 
        lamp_NVdata.lampControl.bit.Color = control.bit.Color; 
        lamp_NVdata.lampControl.bit.OnOff = control.bit.OnOff;          
        
        if (lamp_NVdata.lampControl.bit.White)
        {
          temp_lamp_NVdata.lampWhite.uint8.warmW = lamp_NVdata.lampWhite.uint8.warmW;
          temp_lamp_NVdata.lampWhite.uint8.coldW = lamp_NVdata.lampWhite.uint8.coldW; 
          temp_lamp_NVdata.lampControl.raw8 = 1;
        }
        
        if (lamp_NVdata.lampControl.bit.Color)
        {
          temp_lamp_NVdata.lampRGB.uint8.r = lamp_NVdata.lampRGB.uint8.r;
          temp_lamp_NVdata.lampRGB.uint8.g = lamp_NVdata.lampRGB.uint8.g;
          temp_lamp_NVdata.lampRGB.uint8.b = lamp_NVdata.lampRGB.uint8.b; 
          temp_lamp_NVdata.lampControl.raw8 += 2;
        }
          
      /* start fade timer - turn off */
      FadeOnMutex=true;
      tmrFadeId  = TMR_AllocateTimer();
      TMR_StartTimer(tmrFadeId, gTmrIntervalTimer_c, TmrMilliseconds(speedMs), FadeTimerCallback, NULL);   
    } else 
    {
        
      /* turn on */
      if( control.bit.OnOff  && (!lamp_NVdata.lampControl.bit.OnOff) )
      {
        lamp_NVdata.lampControl.bit.White = control.bit.White; 
        lamp_NVdata.lampControl.bit.Color = control.bit.Color; 
        lamp_NVdata.lampControl.bit.OnOff = control.bit.OnOff;
        
        if (lamp_NVdata.lampControl.bit.White)
        {
          temp_lamp_NVdata.lampWhite.uint8.warmW = 0;
          temp_lamp_NVdata.lampWhite.uint8.coldW = 0; 
          temp_lamp_NVdata.lampControl.raw8 = 4;
        }
        
        if (lamp_NVdata.lampControl.bit.Color)
        {
          temp_lamp_NVdata.lampRGB.uint8.r = 0;
          temp_lamp_NVdata.lampRGB.uint8.g = 0;
          temp_lamp_NVdata.lampRGB.uint8.b = 0; 
          temp_lamp_NVdata.lampControl.raw8 += 5;
        }      
        
        /* start fade timer - turn on*/
        FadeOnMutex=true;
        tmrFadeId  = TMR_AllocateTimer();
        TMR_StartTimer(tmrFadeId, gTmrIntervalTimer_c, TmrMilliseconds(speedMs), FadeTimerCallback, NULL);  
      } else
      {
         
        /* change on state  + White  01 -> 11 */  
        if( (control.bit.White)  && (!lamp_NVdata.lampControl.bit.White) )
        {
            lamp_NVdata.lampControl.bit.White = control.bit.White;
            
            temp_lamp_NVdata.lampWhite.uint8.warmW = 0;
            temp_lamp_NVdata.lampWhite.uint8.coldW = 0; 
            temp_lamp_NVdata.lampControl.raw8 = 10;
        } 
        
        /* change on state  - White  11 -> 01 */  
        if( (!control.bit.White)  && (lamp_NVdata.lampControl.bit.White) )
        {
            lamp_NVdata.lampControl.bit.White = control.bit.White;
          
            temp_lamp_NVdata.lampWhite.uint8.warmW = lamp_NVdata.lampWhite.uint8.warmW;
            temp_lamp_NVdata.lampWhite.uint8.coldW = lamp_NVdata.lampWhite.uint8.coldW; 
            temp_lamp_NVdata.lampControl.raw8 = 11;
        }   
        
        /* change on state  + RGB  10 -> 11 */  
        if( (control.bit.Color)  && (!lamp_NVdata.lampControl.bit.Color) )
        {
            lamp_NVdata.lampControl.bit.Color = control.bit.Color; 
          
            temp_lamp_NVdata.lampRGB.uint8.r = 0;
            temp_lamp_NVdata.lampRGB.uint8.g = 0;
            temp_lamp_NVdata.lampRGB.uint8.b = 0; 
            temp_lamp_NVdata.lampControl.raw8 += 12;
        } 
        
        /* change on state  - RGB   11 -> 10  */  
        if( (!control.bit.Color)  && (lamp_NVdata.lampControl.bit.Color) )
        {
            lamp_NVdata.lampControl.bit.Color = control.bit.Color; 
          
            temp_lamp_NVdata.lampRGB.uint8.r = lamp_NVdata.lampRGB.uint8.r;
            temp_lamp_NVdata.lampRGB.uint8.g = lamp_NVdata.lampRGB.uint8.g;
            temp_lamp_NVdata.lampRGB.uint8.b = lamp_NVdata.lampRGB.uint8.b; 
            temp_lamp_NVdata.lampControl.raw8 += 14;
        } 

        /* start fade timer - change state*/
        FadeOnMutex=true;
        tmrFadeId  = TMR_AllocateTimer();
        TMR_StartTimer(tmrFadeId, gTmrIntervalTimer_c, TmrMilliseconds(speedMs), FadeTimerCallback, NULL);         
        
      }
    
    }
    
    
    /* update DB */
    result = Las_RecordLampControl(serviceHandle, notify);
  }
  
  return result;
}

  
bleResult_t Las_SetLampWhite (uint16_t serviceHandle, uint8_t warmW, uint8_t coldW, bool notify, bool showMax)
{
  bleResult_t result = gBleSuccess_c;
  uint8_t upd_db = 0;
 
  /* if fade timer is on do not enter control */ 
  if(FadeOnMutex) { return gBleUnavailable_c; }
  
  /* do not permit false white on state */
  if( (warmW + coldW) == 0) { warmW = 1; }
    
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
    
    /* when max white or rgb is reach - make a blink */
    if (showMax)
    {
      /* start fade timer - blink */
      FadeOnMutex=true;
      tmrFadeId  = TMR_AllocateTimer();
      temp_lamp_NVdata.lampControl.raw8 = lamp_cfg.blinkCnt;
      TMR_StartTimer(tmrFadeId, gTmrIntervalTimer_c, TmrMilliseconds(lamp_cfg.blinkTimeMs), BlinkTimerCallback, NULL); 
    } // end showMax
    
  } // end upd_db
  
  return result;
}


bleResult_t Las_SetLampRGB (uint16_t serviceHandle, uint8_t red, uint8_t green, uint8_t blue, bool notify)
{
  bleResult_t result = gBleSuccess_c;
  uint8_t upd_db = 0;

  /* if fade timer is on do not enter control */ 
  if(FadeOnMutex) { return gBleUnavailable_c; }  
  
  /* do not permit false color on state */
  if( (red + green + blue) == 0) { green = 1; }
  
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
    result = Las_RecordLampRGB(serviceHandle, notify);  
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

    
    //if(mLas_SubscribedClientId != gInvalidDeviceId_c)
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

    //if(mLas_SubscribedClientId != gInvalidDeviceId_c)
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

static bleResult_t Las_RecordLampRGB (uint16_t serviceHandle, uint8_t notify)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid = (bleUuid_t*)&uuid_char_lamp_RGB;

    //if(mLas_SubscribedClientId != gInvalidDeviceId_c)
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
      
      if (notify) Hls_LampNotification(handle);
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
    
    //if(mLas_SubscribedClientId != gInvalidDeviceId_c)
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
    lamp_control_t control;
    
    /*  lamp was set before */

    /* turn on lamp */
    control = lamp_NVdata.lampControl;
    control.bit.OnOff = 1;
    Las_SetLampControl (mLas_serviceHandle, control, TRUE, FALSE);
}

/*! *********************************************************************************
* \brief        Off Timer for lamp stop after some seconds.
*
* \param[in]    pParam        Callback parameters.
********************************************************************************** */
static void OffTimerCallback(void * pParam)
{
    lamp_control_t control;

    /* turn off lamp and notify */
    control = lamp_NVdata.lampControl;
    control.bit.OnOff = 0;
    Las_SetLampControl (mLas_serviceHandle, control, TRUE, FALSE);  
}


/*! *********************************************************************************
* \brief        Fade Timer for lamp start stop slow.
*
* \param[in]    pParam        Callback parameters.
********************************************************************************** */
static void FadeTimerCallback(void * pParam)
{

     switch(temp_lamp_NVdata.lampControl.raw8)
     {
       case 11:
       case 1: // white fade off 
         {
            if(temp_lamp_NVdata.lampWhite.uint8.warmW > 0) { temp_lamp_NVdata.lampWhite.uint8.warmW--; }
            if(temp_lamp_NVdata.lampWhite.uint8.coldW > 0) { temp_lamp_NVdata.lampWhite.uint8.coldW--; }
            TPM_PWM_WarmWhite(temp_lamp_NVdata.lampWhite.uint8.warmW);
            TPM_PWM_ColdWhite(temp_lamp_NVdata.lampWhite.uint8.coldW);
            if( temp_lamp_NVdata.lampWhite.raw16 == 0 ) 
            { temp_lamp_NVdata.lampControl.raw8 = 0; }
 
         } break;
         
       case 14: 
       case 2: // rgb fade off 
         {
            if(temp_lamp_NVdata.lampRGB.uint8.r > 0) { temp_lamp_NVdata.lampRGB.uint8.r--;  }
            if(temp_lamp_NVdata.lampRGB.uint8.g > 0) { temp_lamp_NVdata.lampRGB.uint8.g--;  }
            if(temp_lamp_NVdata.lampRGB.uint8.b > 0) { temp_lamp_NVdata.lampRGB.uint8.b--;  }   
            TPM_PWM_Red  (temp_lamp_NVdata.lampRGB.uint8.r);
            TPM_PWM_Green(temp_lamp_NVdata.lampRGB.uint8.g);
            TPM_PWM_Blue (temp_lamp_NVdata.lampRGB.uint8.b);
            if( temp_lamp_NVdata.lampRGB.raw32 == 0 ) 
            { temp_lamp_NVdata.lampControl.raw8 = 0; }
         } break;    
         
        case 3: // white + rgb fade off 
         {
            if(temp_lamp_NVdata.lampWhite.uint8.warmW > 0) { temp_lamp_NVdata.lampWhite.uint8.warmW--; }
            if(temp_lamp_NVdata.lampWhite.uint8.coldW > 0) { temp_lamp_NVdata.lampWhite.uint8.coldW--; }          
            if(temp_lamp_NVdata.lampRGB.uint8.r > 0) { temp_lamp_NVdata.lampRGB.uint8.r--;  }
            if(temp_lamp_NVdata.lampRGB.uint8.g > 0) { temp_lamp_NVdata.lampRGB.uint8.g--;  }
            if(temp_lamp_NVdata.lampRGB.uint8.b > 0) { temp_lamp_NVdata.lampRGB.uint8.b--;  }   

            TPM_PWM_WarmWhite(temp_lamp_NVdata.lampWhite.uint8.warmW);
            TPM_PWM_ColdWhite(temp_lamp_NVdata.lampWhite.uint8.coldW);            
            TPM_PWM_Red  (temp_lamp_NVdata.lampRGB.uint8.r);
            TPM_PWM_Green(temp_lamp_NVdata.lampRGB.uint8.g);
            TPM_PWM_Blue (temp_lamp_NVdata.lampRGB.uint8.b);
            
            if( (temp_lamp_NVdata.lampRGB.raw32 == 0) && ( temp_lamp_NVdata.lampWhite.raw16 == 0 ) ) 
            { temp_lamp_NVdata.lampControl.raw8 = 0; }
         } break;         

        case 10: 
        case 4: // white fade on 
         {
            if(temp_lamp_NVdata.lampWhite.uint8.warmW < lamp_NVdata.lampWhite.uint8.warmW) { temp_lamp_NVdata.lampWhite.uint8.warmW++; }
            if(temp_lamp_NVdata.lampWhite.uint8.coldW < lamp_NVdata.lampWhite.uint8.coldW) { temp_lamp_NVdata.lampWhite.uint8.coldW++; }
            TPM_PWM_WarmWhite(temp_lamp_NVdata.lampWhite.uint8.warmW);
            TPM_PWM_ColdWhite(temp_lamp_NVdata.lampWhite.uint8.coldW);
            if( temp_lamp_NVdata.lampWhite.raw16 == lamp_NVdata.lampWhite.raw16 ) 
            { temp_lamp_NVdata.lampControl.raw8 = 0; }
 
         } break;
         
       case 12:  
       case 5: // rgb fade on 
         {
            if(temp_lamp_NVdata.lampRGB.uint8.r < lamp_NVdata.lampRGB.uint8.r) { temp_lamp_NVdata.lampRGB.uint8.r++;  }
            if(temp_lamp_NVdata.lampRGB.uint8.g < lamp_NVdata.lampRGB.uint8.g) { temp_lamp_NVdata.lampRGB.uint8.g++;  }
            if(temp_lamp_NVdata.lampRGB.uint8.b < lamp_NVdata.lampRGB.uint8.b) { temp_lamp_NVdata.lampRGB.uint8.b++;  }   
            TPM_PWM_Red  (temp_lamp_NVdata.lampRGB.uint8.r);
            TPM_PWM_Green(temp_lamp_NVdata.lampRGB.uint8.g);
            TPM_PWM_Blue (temp_lamp_NVdata.lampRGB.uint8.b);
            if( temp_lamp_NVdata.lampRGB.raw32 == lamp_NVdata.lampRGB.raw32 ) 
            { temp_lamp_NVdata.lampControl.raw8 = 0; }
         } break;    
         
        case 9: // white + rgb fade on 
         {
            if(temp_lamp_NVdata.lampWhite.uint8.warmW < lamp_NVdata.lampWhite.uint8.warmW) { temp_lamp_NVdata.lampWhite.uint8.warmW++; }
            if(temp_lamp_NVdata.lampWhite.uint8.coldW < lamp_NVdata.lampWhite.uint8.coldW) { temp_lamp_NVdata.lampWhite.uint8.coldW++; }          
            if(temp_lamp_NVdata.lampRGB.uint8.r < lamp_NVdata.lampRGB.uint8.r) { temp_lamp_NVdata.lampRGB.uint8.r++;  }
            if(temp_lamp_NVdata.lampRGB.uint8.g < lamp_NVdata.lampRGB.uint8.g) { temp_lamp_NVdata.lampRGB.uint8.g++;  }
            if(temp_lamp_NVdata.lampRGB.uint8.b < lamp_NVdata.lampRGB.uint8.b) { temp_lamp_NVdata.lampRGB.uint8.b++;  }   

            TPM_PWM_WarmWhite(temp_lamp_NVdata.lampWhite.uint8.warmW);
            TPM_PWM_ColdWhite(temp_lamp_NVdata.lampWhite.uint8.coldW);            
            TPM_PWM_Red  (temp_lamp_NVdata.lampRGB.uint8.r);
            TPM_PWM_Green(temp_lamp_NVdata.lampRGB.uint8.g);
            TPM_PWM_Blue (temp_lamp_NVdata.lampRGB.uint8.b);
            
            if( (temp_lamp_NVdata.lampRGB.raw32 == lamp_NVdata.lampRGB.raw32) && ( temp_lamp_NVdata.lampWhite.raw16 == lamp_NVdata.lampWhite.raw16 ) ) 
            { temp_lamp_NVdata.lampControl.raw8 = 0; }
         } break; 
         
        case 23: // white fade off + rgb fade on
          {
            if(temp_lamp_NVdata.lampWhite.uint8.warmW > 0) { temp_lamp_NVdata.lampWhite.uint8.warmW--; }
            if(temp_lamp_NVdata.lampWhite.uint8.coldW > 0) { temp_lamp_NVdata.lampWhite.uint8.coldW--; }          
            if(temp_lamp_NVdata.lampRGB.uint8.r < lamp_NVdata.lampRGB.uint8.r) { temp_lamp_NVdata.lampRGB.uint8.r++;  }
            if(temp_lamp_NVdata.lampRGB.uint8.g < lamp_NVdata.lampRGB.uint8.g) { temp_lamp_NVdata.lampRGB.uint8.g++;  }
            if(temp_lamp_NVdata.lampRGB.uint8.b < lamp_NVdata.lampRGB.uint8.b) { temp_lamp_NVdata.lampRGB.uint8.b++;  }   

            TPM_PWM_WarmWhite(temp_lamp_NVdata.lampWhite.uint8.warmW);
            TPM_PWM_ColdWhite(temp_lamp_NVdata.lampWhite.uint8.coldW);            
            TPM_PWM_Red  (temp_lamp_NVdata.lampRGB.uint8.r);
            TPM_PWM_Green(temp_lamp_NVdata.lampRGB.uint8.g);
            TPM_PWM_Blue (temp_lamp_NVdata.lampRGB.uint8.b);
            
            if( (temp_lamp_NVdata.lampRGB.raw32 == lamp_NVdata.lampRGB.raw32) && ( temp_lamp_NVdata.lampWhite.raw16 == 0 ) ) 
            { temp_lamp_NVdata.lampControl.raw8 = 0; }            
          } break; 
         
        case 24: // white fade on + rgb fade off
         {
            if(temp_lamp_NVdata.lampWhite.uint8.warmW < lamp_NVdata.lampWhite.uint8.warmW) { temp_lamp_NVdata.lampWhite.uint8.warmW++; }
            if(temp_lamp_NVdata.lampWhite.uint8.coldW < lamp_NVdata.lampWhite.uint8.coldW) { temp_lamp_NVdata.lampWhite.uint8.coldW++; }          
            if(temp_lamp_NVdata.lampRGB.uint8.r > 0) { temp_lamp_NVdata.lampRGB.uint8.r--;  }
            if(temp_lamp_NVdata.lampRGB.uint8.g > 0) { temp_lamp_NVdata.lampRGB.uint8.g--;  }
            if(temp_lamp_NVdata.lampRGB.uint8.b > 0) { temp_lamp_NVdata.lampRGB.uint8.b--;  }  

            TPM_PWM_WarmWhite(temp_lamp_NVdata.lampWhite.uint8.warmW);
            TPM_PWM_ColdWhite(temp_lamp_NVdata.lampWhite.uint8.coldW);            
            TPM_PWM_Red  (temp_lamp_NVdata.lampRGB.uint8.r);
            TPM_PWM_Green(temp_lamp_NVdata.lampRGB.uint8.g);
            TPM_PWM_Blue (temp_lamp_NVdata.lampRGB.uint8.b);
            
            if( (temp_lamp_NVdata.lampRGB.raw32 == 0) && ( temp_lamp_NVdata.lampWhite.raw16 == lamp_NVdata.lampWhite.raw16 ) ) 
            { temp_lamp_NVdata.lampControl.raw8 = 0; }
         } break;         
      }  
     

     if( temp_lamp_NVdata.lampControl.raw8 == 0 )
     {
       FadeOnMutex=false;
       temp_lamp_NVdata.lampRGB.raw32 = 0;
       temp_lamp_NVdata.lampWhite.raw16 = 0;
       TMR_FreeTimer(tmrFadeId);
     }
  
}

/*! *********************************************************************************
* \brief        Fade Timer for lamp blink when white max is reached.
*
* \param[in]    pParam        Callback parameters.
********************************************************************************** */
static void BlinkTimerCallback(void * pParam)
{
  
    if(!temp_lamp_NVdata.lampControl.raw8)// exit
    {
        FadeOnMutex=false;
        TPM_PWM_WarmWhite( lamp_NVdata.lampWhite.uint8.warmW );
        TPM_PWM_ColdWhite( lamp_NVdata.lampWhite.uint8.coldW ); 
        TMR_FreeTimer(tmrFadeId);
    } else 
    {
      if( temp_lamp_NVdata.lampControl.raw8 % 2 )
      {
        TPM_PWM_WarmWhite( 0 );
        TPM_PWM_ColdWhite( 0 ); 
      } else
      {
        TPM_PWM_WarmWhite( 30 );
        TPM_PWM_ColdWhite( 30 );         
      }
      
      if(temp_lamp_NVdata.lampControl.raw8 > 0) { temp_lamp_NVdata.lampControl.raw8--; }
    
    }
 
}

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
