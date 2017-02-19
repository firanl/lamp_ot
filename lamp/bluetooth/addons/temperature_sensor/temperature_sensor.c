/*!
* @file temperature_sensor.c
*
* @author  
*
* @version 1.0
*
* @date Mar-15-2016
*
* @brief Driver for Temperature Sensor interface
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
#include "temperature_sensor.h"

// KSDK header files
#include "fsl_adc16_driver.h"
#include "fsl_pmc_hal.h"

/******************************************************************************
* Locals
*******************************************************************************/

/*! Temperarture sensor ADC configuration structure */
const adc16_converter_config_t temperatureSensorAdcConfig= {
  .lowPowerEnable = FALSE,
  .clkDividerMode = kAdc16ClkDividerOf4,
  .longSampleTimeEnable = FALSE,
  .resolution = kAdc16ResolutionBitOfDiffModeAs16,
  .clkSrc = kAdc16ClkSrcOfBusClk,
  .asyncClkEnable = FALSE,
  .highSpeedEnable = FALSE, 
  .longSampleCycleMode = kAdc16LongSampleCycleOf24,
  .hwTriggerEnable = FALSE,
  .refVoltSrc = kAdc16RefVoltSrcOfVref,
  .continuousConvEnable = FALSE,
};

/*! Channel configuration for internal temperature sensor */
const adc16_chn_config_t temperatureSensorChipTemperatureChannel = {
  .chnIdx = kAdc16Chn26,
  .convCompletedIntEnable = FALSE,
  .diffConvEnable = TRUE,
};

/*! Channel configuration for bandgap voltage */
const adc16_chn_config_t temperatureSensorBandgapVoltageChannel = {
  .chnIdx = kAdc16Chn27,
  .convCompletedIntEnable = FALSE,
  .diffConvEnable = TRUE,
};

/******************************************************************************
* Globals
*******************************************************************************/

/* core temperature, exponent -2 */
/* core voltage reference, exponent -3 */
chip_TempVoltage_t g_chip_TV;

/******************************************************************************
* Private Function prototypes
******************************************************************************/

/******************************************************************************
* Public Functions
******************************************************************************/

temperature_sensor_status_t temperature_sensor_init (void){
  
  adc16_status_t result;
  adc16_calibration_param_t adcCalibrationParameters;
  pmc_bandgap_buffer_config_t pmcBandgapConfiguration = {
    .enable = TRUE,
  };
  
  /* Initialize ADC driver */
  result = ADC16_DRV_Init(TEMPERATURE_SENSOR_ADC_INSTANCE, &temperatureSensorAdcConfig);
  
  if (result != kStatus_ADC16_Success)
    return kTemperatureInitError;
  
  /* Calibrate ADC module */
  result = ADC16_DRV_GetAutoCalibrationParam(TEMPERATURE_SENSOR_ADC_INSTANCE, &adcCalibrationParameters);
  
  if (result != kStatus_ADC16_Success)
    return kTemperatureInitError;
  
  result =  ADC16_DRV_SetCalibrationParam(TEMPERATURE_SENSOR_ADC_INSTANCE, &adcCalibrationParameters);
  
  if (result != kStatus_ADC16_Success)
    return kTemperatureInitError;
  
  /* Enable bandgap voltage channel for reference measurement */
  PMC_HAL_BandgapBufferConfig(PMC, &pmcBandgapConfiguration);
  
  return kTemperatureSensorOk;
}

void measure_chip_temperature (void){
  adc16_status_t result;
  int16_t bandgapVoltageAdcReading, temperatureChannelAdcReading;
  int16_t vTemperatureSensor;

  
  /* Start Bandgap Voltage Measurements */
  result = ADC16_DRV_ConfigConvChn(TEMPERATURE_SENSOR_ADC_INSTANCE, 0, &temperatureSensorBandgapVoltageChannel);
  
  if(result != kStatus_ADC16_Success) {
    g_chip_TV.int16.gCoreTemperature = kTemperatureSensorConversionStartError;
    g_chip_TV.int16.g_vReference     = kTemperatureSensorConversionStartError;
  }
  else {
        /* Wait for bandgap voltage measurement reading */
        ADC16_DRV_WaitConvDone (TEMPERATURE_SENSOR_ADC_INSTANCE, 0);
        
        /* Get bandgap measurement */
        bandgapVoltageAdcReading = ADC16_DRV_GetConvValueSigned (TEMPERATURE_SENSOR_ADC_INSTANCE, 0);  
        
        /* Start Temperature Channel Measurements */
        result = ADC16_DRV_ConfigConvChn(TEMPERATURE_SENSOR_ADC_INSTANCE, 0, &temperatureSensorChipTemperatureChannel);
        
        if(result != kStatus_ADC16_Success) {
          g_chip_TV.int16.gCoreTemperature = kTemperatureSensorConversionStartError;
          g_chip_TV.int16.g_vReference     = kTemperatureSensorConversionStartError;
        }
        else { 
              /* Wait for temperature channel measurement reading */
              ADC16_DRV_WaitConvDone (TEMPERATURE_SENSOR_ADC_INSTANCE, 0);
              
              /* Get temperature channel measurement */
              temperatureChannelAdcReading = ADC16_DRV_GetConvValueSigned (TEMPERATURE_SENSOR_ADC_INSTANCE, 0);
              
              /* Calculate Reference Voltage */
              g_chip_TV.int16.g_vReference = (int16_t)((TEMPERATURE_SENSOR_V_BANDGAP_mV * TEMPERATURE_SENSOR_ADC_RESOLUTION)/bandgapVoltageAdcReading);
              
              /* Calculate Temperature Sensor Voltage */
              vTemperatureSensor = (int16_t)((g_chip_TV.int16.g_vReference*temperatureChannelAdcReading)/TEMPERATURE_SENSOR_ADC_RESOLUTION);
              
              /* Obtain temperature measurement*/
              g_chip_TV.int16.gCoreTemperature = 2500 - (((vTemperatureSensor - TEMPERATURE_SENSOR_VTEMP25_mV)*1000*100)/TEMPERATURE_SENSOR_SLOPE_uV); 
              
              /* If failure temperature is reached stop all PWM TPM outputs and try to BT notify */
              if(g_chip_TV.int16.gCoreTemperature > gCoreTemperatureFaliure_d)
              {
                  //TPM_PWM_Off();
              }
        }
  }
  
}

/******************************************************************************
* Private Functions
******************************************************************************/

/* End of file */