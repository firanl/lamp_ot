/*!
* @file tsi_sensor.c
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

/******************************************************************************
*                               Header Files
*******************************************************************************/

// Stack header files
#include "EmbeddedTypes.h"

// Application header files
#include "tsi_sensor.h"

// KSDK header files
#include "fsl_port_hal.h"
#include "fsl_tsi_driver.h"
#include "TimersManager.h"


/******************************************************************************
* Locals
*******************************************************************************/
tsi_state_t tsiSensorState;
tsi_sensor_callback_t userCallbackFunction;

tsi_touch_t tsi;

/* The TPM instance/channel used for board */
#define BOARD_TSI_INSTANCE        0
  /*!< Electrode channel */
  #define BOARD_TSI_BTN_CHANNEL               15u



/* Touch Sensing sensor timer  */
static tmrTimerID_t tmrTsiId;  

static uint8_t tsi_event;


/******************************************************************************
* Globals
*******************************************************************************/



/******************************************************************************
* Private Function prototypes
******************************************************************************/
static void TsiTimerCallback(void* pParam);
static void tsiIrqCallback(uint32_t instance, void* usrData);
static tsi_status_t TsiCalibrate(void);

/************************************************************************************
* Extern functions
************************************************************************************/

extern void TSI_DRV_IRQHandler(uint32_t instance);
extern void BleApp_HandleTouch(uint8_t* pEvent);

/******************************************************************************
* Public Functions
******************************************************************************/

void TSI_Init ()
{
  tsi_status_t result;
  tmrErrCode_t tmrerr = gTmrInvalidId_c;
  
  uint32_t  lowestSignal;
  
  /* Set up the HW configuration for normal mode of TSI */
  /* FSL_FEATURE_TSI_VERSION == 4 */
  static const tsi_config_t tsi_HwConfig =
  {
    .ps = kTsiElecOscPrescaler_2div,
    .extchrg = kTsiExtOscChargeCurrent_16uA,
    .refchrg = kTsiRefOscChargeCurrent_16uA,
    .nscn = kTsiConsecutiveScansNumber_8time,
    .mode = kTsiAnalogModeSel_Capacitive,
    .dvolt = kTsiOscVolRails_Dv_029,
    .thresh = 100,
    .thresl = 200,
  };
  
  /* Set up the configuration of initialization structure */
  static const tsi_user_config_t tsiSensorConfig =
  {
    .config = (tsi_config_t*)&tsi_HwConfig,
    .pCallBackFunc = tsiIrqCallback,
    .usrData = (void*)0x00,
  };
  
  /*!< Threshold value to detect a touch event */
  tsi.sensitivity = 200; //10
  /*!< TSI update time in ms */
  tsi.tmr = 12; // 15
  
  tsi.InitHitCnt = 4;         /* Intermediate state hit cnt, time is  tmr*InitHitCnt ms */
  tsi.InitIdleCnt = 4;        /* Intermediate state idle cnt */
  tsi.InitHitHitCnt = 4;      /* Long Press Series hit cnt */
  tsi.InitHitIdleCnt = 6;     /* Intermediate not used state idle cnt  */
      
  tsi.InitIdleHitCnt = 5;     /* Intermediate state hit cnt */
  tsi.InitIdleIdleCnt = 8;    /* Idle state idle cnt  */
  tsi.InitIdleHitHitCnt = 45;  /* First Long Press hit cnt */
  tsi.InitIdleHitIdleCnt = 5; /* Short Press  idle cnt */  
  
  /*!< TSI  Long Press Series succesive count that triggers recalibration - 5 minutes */
  tsi.stuckBtnCntMax = (5 * 60 * 1000) / ( (tsi.InitHitCnt + tsi.InitHitHitCnt) * tsi.tmr );
  
  
  //configure_tsi_pins
  /* TSI0_CH15 */
  PORT_HAL_SetMuxMode(PORTC, 3u, kPortPinDisabled);  
  
  /* Initialize TSI module */
  result = TSI_DRV_Init(BOARD_TSI_INSTANCE, &tsiSensorState, &tsiSensorConfig);
  
  /* Configure all electrode channels */
  result = TSI_DRV_EnableElectrode(BOARD_TSI_INSTANCE, BOARD_TSI_BTN_CHANNEL, true);

  /* Calibrate all electrode channels */
  result = TsiCalibrate();
  
  tmrTsiId = TMR_AllocateTimer(); /* TSI */
  
  /* Start TSI timer for capacitive touch BTN */     
  tmrerr = TMR_StartTimer(tmrTsiId, gTmrIntervalTimer_c, tsi.tmr, TsiTimerCallback, NULL);  
  tsi_event = 0;
}

void TSI_MeasureOnce(void)
{
  tsi_status_t result;
  
  /* Start measurements */
  result = TSI_DRV_Measure(BOARD_TSI_INSTANCE);
  
}

/******************************************************************************
* Private Functions
******************************************************************************/
static void tsiIrqCallback(uint32_t instance, void* usrData)
{
  static uint8_t hitCnt = 0;
  static uint8_t idleCnt = 0;
  static tsi_states_t state = kTsiInit;
  uint16_t tsiChannelReading;
  #if (gUseStuckButtonCounter_d)
    /* Stuck Button counter */
    static uint16_t stuckBtnCnt = 0;
  #endif


     
  if( (!tsi_event) )
  {
    
   //Read current measurement
   TSI_DRV_GetCounter(BOARD_TSI_INSTANCE, BOARD_TSI_BTN_CHANNEL, &tsiChannelReading); 
   if( tsiChannelReading > tsi.low ) hitCnt++;
   else idleCnt++;
   
   switch (state)
   {
    case kTsiInit:
     {
      if(hitCnt >= tsi.InitHitCnt) //4
      {
        /* Intermediate state */
        state = kTsiHit; idleCnt = 0; hitCnt = 0;
      }
      else if (idleCnt >= tsi.InitIdleCnt) //4
      {
        /* Intermediate state */
        state = kTsiIdle; idleCnt = 0; hitCnt = 0;
        #if (gUseStuckButtonCounter_d)
         stuckBtnCnt=0;
        #endif
      }
     } break;
     
    case kTsiHit:
     {
      if(hitCnt >= tsi.InitHitHitCnt) // 3
      {
       /*  Confirm Long Press Series state hit hit*/ 
        tsi_event = gTSI_EventLongPush_c;
        idleCnt = 0; hitCnt = 0; state=kTsiInit;
        BleApp_HandleTouch(&tsi_event); 
        
        #if (gUseStuckButtonCounter_d)
          stuckBtnCnt++;
          /* trigger recalibration */
          if (stuckBtnCnt==tsi.stuckBtnCntMax)
          {          
            /* Calibrate electrode channel - try lower treshold */
            if(tsi.low > tsi.sensitivity)  { tsi.low = tsi.low - tsi.sensitivity; }
            else { TsiCalibrate(); }
            stuckBtnCnt=0;
            state = kTsiIdle; idleCnt = 0; hitCnt = 0;
          }
        #endif
      }
      else if (idleCnt >= tsi.InitHitIdleCnt)  // 6
      {
        /* Intermediate state hit idle */
        idleCnt = 0; hitCnt = 0; state=kTsiInit;
      }
     } break;     
     
    case kTsiIdle:
     {
      if(hitCnt >= tsi.InitIdleHitCnt) // 5
      {
        /* Intermediate state idle hit */
        idleCnt = 0; hitCnt = 0; state=kTsiIdle_Hit;
      }
      else if (idleCnt >= tsi.InitIdleIdleCnt) // 8
      {
        /* Confirm Idle state idle idle */
        if( tsi_event != gTSI_EventIdle_c )
        {
          tsi_event = gTSI_EventIdle_c;
          idleCnt = 0; hitCnt = 0; state=kTsiInit;
          BleApp_HandleTouch(&tsi_event); 
        }
      }
     } break; 
     
    case kTsiIdle_Hit:
     {
      if(hitCnt >= tsi.InitIdleHitHitCnt) // 45
      {
        /*  Confirm First Long Press state */ 
        tsi_event = gTSI_EventLongPush_c;
        idleCnt = 0; hitCnt = 0; state=kTsiInit;
        BleApp_HandleTouch(&tsi_event); 
      }
      else if (idleCnt >= tsi.InitIdleHitIdleCnt) // 5
      {
        /* Confirm Short Press state */ 
        tsi_event = gTSI_EventShortPush_c;
        idleCnt = 0; hitCnt = 0; state=kTsiInit;
        BleApp_HandleTouch(&tsi_event); 
      }
     } break; 
     
   }

  }

 
}

tsi_status_t TsiCalibrate(void)
{
  tsi_status_t result;
  
  //TODO use TSI_DRV_Recalibrate
    
  /* Calibrate all electrode channels */
  result = TSI_DRV_MeasureBlocking(BOARD_TSI_INSTANCE);
  result = TSI_DRV_GetCounter(BOARD_TSI_INSTANCE, BOARD_TSI_BTN_CHANNEL, &tsi.low);
  //TSI_DRV_Recalibrate(BOARD_TSI_INSTANCE, &lowestSignal);
  
  tsi.low += tsi.sensitivity;

  return result;
}

/*! *********************************************************************************
 * \brief        Handles TSI sensor timer callback
 *
 * \param[in]    pParam        Callback parameters.
********************************************************************************** */
static void TsiTimerCallback(void* pParam)
{
  /* Start a new TSI single measurement */
  TSI_MeasureOnce();
}

/*! *********************************************************************************
 * @brief Implementation of TSI0 handler named in startup code.
 *
 * Passes instance to generic TSI IRQ handler.
********************************************************************************** */
void TSI0_IRQHandler(void)
{
    TSI_DRV_IRQHandler(0);
}

/* End of file */