/*!
* @file buzzer_driver.c
*
* @author  
*
* @version 1.0
*
* @date Mar-14-2016
*
* @brief Driver for Buzzer Driver interface
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


/*******************************************************************************
*                               Header Files
*******************************************************************************/

// Stack header files
#include "EmbeddedTypes.h"

// Application header files
#include "buzzer_driver.h"

// KSDK header files
#include "fsl_tpm_driver.h"

/******************************************************************************
* Locals
*******************************************************************************/
tpm_pwm_param_t buzzerPwmParameters = {
  .mode = kTpmEdgeAlignedPWM,
  .edgeMode = kTpmHighTrue,
  .uFrequencyHZ = BUZZER_DRIVER_FREQUENCY_HZ,
  .uDutyCyclePercent = 50,
};

/******************************************************************************
* Globals
*******************************************************************************/

/******************************************************************************
* Private Function prototypes
******************************************************************************/

/******************************************************************************
* Public Functions
******************************************************************************/

buzzer_driver_status_t buzzer_driver_init (void){
  tpm_status_t result;
  
  /* Initialize TPM module for PWM generation */
  tpm_general_config_t buzzerTpmConfig = {      //TPM configuration structure
    .isDBGMode = TRUE,
    .isGlobalTimeBase = FALSE,
    .isTriggerMode = FALSE,
    .isStopCountOnOveflow = FALSE,
    .isCountReloadOnTrig = FALSE,
    .triggerSource = kTpmTrigSel0,
  };
  
  result = TPM_DRV_Init(BUZZER_DRIVER_TPM_MODULE, &buzzerTpmConfig);            //Initialize TPM module
  
  if(result != kStatusTpmSuccess)
    return kBuzzerDriverInitError;
  
  /* Set TPM clock */
  TPM_DRV_SetClock (BUZZER_DRIVER_TPM_MODULE, kTpmClockSourceModuleClk, kTpmDividedBy2);
  
  return kBuzzerDriverOk;
}

buzzer_driver_status_t buzzer_driver_change_buzzer_status (uint8_t buzzerStatus){
  tpm_status_t result;
  if(buzzerStatus){
    result = TPM_DRV_PwmStart(BUZZER_DRIVER_TPM_MODULE, &buzzerPwmParameters, BUZZER_DRIVER_PWM_CHANNEL);
    
    if(result != kStatusTpmSuccess)
      return kBuzzerPwmStartError;
  }
  else{
    TPM_DRV_PwmStop(BUZZER_DRIVER_TPM_MODULE, &buzzerPwmParameters, BUZZER_DRIVER_PWM_CHANNEL);
  }
  
  return kBuzzerDriverOk;
}

/******************************************************************************
* Private Functions
******************************************************************************/


/* End of file */