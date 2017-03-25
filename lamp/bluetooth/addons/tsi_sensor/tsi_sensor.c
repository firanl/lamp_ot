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

/* The TPM instance/channel used for board */
#define BOARD_TSI_INSTANCE        0
  /*!< Electrode channel */
  #define BOARD_TSI_BTN_CHANNEL               15u


/*!< Threshold value to detect a touch event */
uint16_t TSI_SENSOR_THRESHOLD_ADDER = 10; 
/*!< TSI update time in mS */
#define gTsiUpdateTime_c                15  

#define gTsiShortPTime_c                50  

#define gTsiLongPTime_c                 230 





static uint16_t BOARD_TSI_BTN_treshold;

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
  
  //configure_tsi_pins
  /* TSI0_CH15 */
  PORT_HAL_SetMuxMode(PORTC, 3u, kPortPinDisabled);  
  
  /* Initialize TSI module */
  result = TSI_DRV_Init(BOARD_TSI_INSTANCE, &tsiSensorState, &tsiSensorConfig);
  
  /* Configure all electrode channels */
  result = TSI_DRV_EnableElectrode(BOARD_TSI_INSTANCE, BOARD_TSI_BTN_CHANNEL, true);

  /* Calibrate all electrode channels */
  result = TSI_DRV_MeasureBlocking(BOARD_TSI_INSTANCE);
  result = TSI_DRV_GetCounter(BOARD_TSI_INSTANCE, BOARD_TSI_BTN_CHANNEL, &BOARD_TSI_BTN_treshold);
  //TSI_DRV_Recalibrate(BOARD_TSI_INSTANCE, &lowestSignal);
  
  BOARD_TSI_BTN_treshold += TSI_SENSOR_THRESHOLD_ADDER;
  
  
  tmrTsiId = TMR_AllocateTimer(); /* TSI */
  
  /* Start TSI timer for capacitive touch BTN */     
  tmrerr = TMR_StartTimer(tmrTsiId, gTmrIntervalTimer_c, gTsiUpdateTime_c, TsiTimerCallback, NULL);  
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
  static bool_t isCalibrated = TRUE;
  static uint8_t hitCnt = 0;
  static uint8_t missCnt = 0;
  static uint8_t state = 0;
  uint16_t tsiChannelReading;
  bool_t hit = 0;



      
  if( (!tsi_event) && isCalibrated   )
  {
    
  //Read current measurement
  TSI_DRV_GetCounter(BOARD_TSI_INSTANCE, BOARD_TSI_BTN_CHANNEL, &tsiChannelReading); 
  
  if(tsiChannelReading > BOARD_TSI_BTN_treshold)
  {
      hitCnt++;
      if( hitCnt >= (gTsiLongPTime_c/gTsiUpdateTime_c) )
      {
        tsi_event = gTSI_EventLongPush_c;
        missCnt = 0; hitCnt = 0; state=0;
        BleApp_HandleTouch(&tsi_event);
      }
      
      if(state==1)
      {
        if ( hitCnt >= (gTsiShortPTime_c/gTsiUpdateTime_c) )
        { state = 2; missCnt = 0; }
      }
      
  }
  else 
  {
      missCnt++;
      if( missCnt >= (gTsiLongPTime_c/gTsiUpdateTime_c) )
      {
        tsi_event = gTSI_EventIdle_c;
        missCnt = 0; hitCnt = 0; state=0;
        BleApp_HandleTouch(&tsi_event);
      }
      
      if (state==0)
      {
        if ( missCnt >= ( ( gTsiShortPTime_c)/gTsiUpdateTime_c) )
        {
          state=1; hitCnt = 0;
        }
      } else if (state==2)
      {
        if ( missCnt >= ( (gTsiLongPTime_c - gTsiShortPTime_c)/gTsiUpdateTime_c) )
        {
	  tsi_event = gTSI_EventShortPush_c;
          missCnt = 0; hitCnt = 0; state=0;
          BleApp_HandleTouch(&tsi_event);
        }
      
      }
  }
 

  
    
    
    
  }
  /* start recalibrate */
  else
  {
  }
  
  
  
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

/*!
 * @brief Implementation of TSI0 handler named in startup code.
 *
 * Passes instance to generic TSI IRQ handler.
 */
void TSI0_IRQHandler(void)
{
    TSI_DRV_IRQHandler(0);
}

/* End of file */