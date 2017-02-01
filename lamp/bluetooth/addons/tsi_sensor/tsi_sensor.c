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
#include "fsl_tsi_driver.h"


/******************************************************************************
* Locals
*******************************************************************************/
tsi_state_t tsiSensorState;
tsi_sensor_callback_t userCallbackFunction;

/* List of TSI electrodes to scan */
tsi_sensor_electrode_data_t tsiSensorElectrodeList[] = {
  {
    .channel = 10,
  },
  {
    .channel = 11,
  },
};

tsi_sensor_electrode_flags_t tsiSensorActiveElectrodeFlag = {0x0000};

/******************************************************************************
* Globals
*******************************************************************************/

/******************************************************************************
* Private Function prototypes
******************************************************************************/
static void tsiIrqCallback(uint32_t instance, void* usrData);

/******************************************************************************
* Public Functions
******************************************************************************/

tsi_sensor_status_t tsi_sensor_init (tsi_sensor_callback_t pCallbackFunc){
  tsi_status_t result;
  
  /* Set up the HW configuration for normal mode of TSI */
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
  
  /* Initialize TSI module */
  result = TSI_DRV_Init(0, &tsiSensorState, &tsiSensorConfig);
  
  if(result != kStatus_TSI_Success)
    return kTsiInitError;
  
  /* Configure all electrode channels */
  uint8_t electrodeCount = (sizeof(tsiSensorElectrodeList)/sizeof(tsi_sensor_electrode_data_t));
  uint8_t electrodeIndex;
  
  for(electrodeIndex=0; electrodeIndex<electrodeCount; electrodeIndex++)
  {
    result = TSI_DRV_EnableElectrode(0, tsiSensorElectrodeList[electrodeIndex].channel, true);
    if(result != kStatus_TSI_Success)
    return kTsiInitError;
  }
  
  /* Store user callback function */
  userCallbackFunction = pCallbackFunc;
  
  /* Start measurements */
  return tsi_sensor_start_single_measurement();
}

tsi_sensor_status_t tsi_sensor_start_single_measurement (void){
  tsi_status_t result;
  
  /* Start measurements */
  result = TSI_DRV_Measure(0);
  
  if(result != kStatus_TSI_Success)
  {
    return kTsiStartError;
  }
  
  return kTsiOk;
}

/******************************************************************************
* Private Functions
******************************************************************************/
void tsiIrqCallback(uint32_t instance, void* usrData){
  static bool_t isTsiSensorCalibrated = FALSE;
  uint16_t tsiChannelReading;
  uint8_t electrodeCount = (sizeof(tsiSensorElectrodeList)/sizeof(tsi_sensor_electrode_data_t));
  uint8_t electrodeIndex;
    
  if(isTsiSensorCalibrated == FALSE){
    /* Calibrate all electrode channels */
    for(electrodeIndex=0; electrodeIndex<electrodeCount; electrodeIndex++)
    {
      TSI_DRV_GetCounter(0, tsiSensorElectrodeList[electrodeIndex].channel, &tsiSensorElectrodeList[electrodeIndex].threshold);
      tsiSensorElectrodeList[electrodeIndex].threshold += TSI_SENSOR_THRESHOLD_ADDER;
    }
    /* Clear sensor calibration flag */
    isTsiSensorCalibrated = TRUE;
  }
  else{
    for(electrodeIndex=0; electrodeIndex<electrodeCount; electrodeIndex++)
    {
      TSI_DRV_GetCounter(0, tsiSensorElectrodeList[electrodeIndex].channel, &tsiChannelReading); //Read current measurement
      if(tsiChannelReading > tsiSensorElectrodeList[electrodeIndex].threshold) //Compare measurement with thresshold
        tsiSensorActiveElectrodeFlag.overallFlagStatus |= (1<<electrodeIndex); //Set proper flag
    }
    //Check if some electrode was pressed. Executes callback if true
    if(tsiSensorActiveElectrodeFlag.overallFlagStatus != 0)
      userCallbackFunction(&tsiSensorActiveElectrodeFlag);
  }
}

/* End of file */