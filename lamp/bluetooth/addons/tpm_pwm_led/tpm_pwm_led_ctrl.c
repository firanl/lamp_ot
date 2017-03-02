/*!
* @file tpm_pwm_led_ctrl.c
*
* @author  firanl 
*
* @version 1.0
*
* @date Feb-02-2017
*
* @brief PWM TPM led control RGBWWCW
*
********************************************************************************
*/

/*******************************************************************************
*                               Header Files
*******************************************************************************/

// Stack header files
#include "EmbeddedTypes.h"

// Application header files
#include "tpm_pwm_led_ctrl.h"

// KSDK header files
#include "fsl_port_hal.h"
#include "fsl_tpm_driver.h"


/************************************************************************************
* Private type definitions and macros
************************************************************************************/

/* The TPM instance/channel used for board */
#define BOARD_TPM_INSTANCE_WHITE        1
 #define BOARD_TPM_CHANNEL_WHITE_WARM               0
 #define BOARD_TPM_CHANNEL_WHITE_COLD               1
#define BOARD_TPM_INSTANCE_RGB        0
 #define BOARD_TPM_CHANNEL_RGB_RED                  0
 #define BOARD_TPM_CHANNEL_RGB_GREEN                1
 #define BOARD_TPM_CHANNEL_RGB_BLUE                 2

/* Set PWM output signal type - options  kTpmCenterAlignedPWM and kTpmEdgeAlignedPWM  */
#define BOARD_TPM_MODE  kTpmEdgeAlignedPWM

#define BOARD_TPM_FRQ_HZ    120000u  



/************************************************************************************
* Private memory declarations
************************************************************************************/
  /* Initialize TPM module for PWM generation */
  tpm_general_config_t driverInfo_0 = {      //TPM 0 configuration structure 
        .isDBGMode = FALSE,
        .isGlobalTimeBase = FALSE,
        .isTriggerMode = FALSE,
        .isStopCountOnOveflow = FALSE,
        .isCountReloadOnTrig = FALSE,
        .triggerSource = kTpmTrigSel0,
  };

  tpm_general_config_t driverInfo_1 = {      //TPM 1 configuration structure
        .isDBGMode = FALSE,
        .isGlobalTimeBase = FALSE,
        .isTriggerMode = FALSE,
        .isStopCountOnOveflow = FALSE,
        .isCountReloadOnTrig = FALSE,
        .triggerSource = kTpmTrigSel0,
  }; 
  
  tpm_pwm_param_t paramWW = {
            .mode              = BOARD_TPM_MODE, 
            .edgeMode          = kTpmHighTrue,
            .uFrequencyHZ      = BOARD_TPM_FRQ_HZ,
            .uDutyCyclePercent = PWM_factor_Init // 0 to 100
  };
  
  tpm_pwm_param_t paramCW = {
            .mode              = BOARD_TPM_MODE, 
            .edgeMode          = kTpmHighTrue,
            .uFrequencyHZ      = BOARD_TPM_FRQ_HZ,
            .uDutyCyclePercent = PWM_factor_Init // 0 to 100
  };  
  
   tpm_pwm_param_t paramR = {
            .mode              = BOARD_TPM_MODE, 
            .edgeMode          = kTpmHighTrue,
            .uFrequencyHZ      = BOARD_TPM_FRQ_HZ,
            .uDutyCyclePercent = PWM_factor_Init // 0 to 100
  };
  
  tpm_pwm_param_t paramG = {
            .mode              = BOARD_TPM_MODE, 
            .edgeMode          = kTpmHighTrue,
            .uFrequencyHZ      = BOARD_TPM_FRQ_HZ,
            .uDutyCyclePercent = PWM_factor_Init // 0 to 100
  };  
  
  tpm_pwm_param_t paramB = {
            .mode              = BOARD_TPM_MODE, 
            .edgeMode          = kTpmHighTrue,
            .uFrequencyHZ      = BOARD_TPM_FRQ_HZ,
            .uDutyCyclePercent = PWM_factor_Init // 0 to 100
  };    
  
/************************************************************************************
* Private functions prototypes
************************************************************************************/

/************************************************************************************
* Public functions prototypes
************************************************************************************/
void TPM_PWM_Init(void);
void TPM_PWM_WarmWhite(uint8_t dutyCycleWarmWhite);
void TPM_PWM_ColdWhite(uint8_t dutyCycleColdWhite);
void TPM_PWM_Red(uint8_t dutyCycleRed);
void TPM_PWM_Green(uint8_t dutyCycleGreen);
void TPM_PWM_Blue(uint8_t dutyCycleBlue);
void TPM_PWM_Off(void);

/************************************************************************************
* Public functions
************************************************************************************/

void TPM_PWM_Init(void)
{
 
  #ifndef DEBUG
    PORT_HAL_SetMuxMode(PORTA, 0u,kPortMuxAlt5); /* TPM1_CH0 warm white*/
    //PORT_HAL_SetMuxMode(PORTA, 1u,kPortMuxAlt5); /* TPM1_CH1 cold white old versions */
  #endif  
    
  PORT_HAL_SetMuxMode(PORTB, 3u, kPortMuxAlt5); /* TPM1_CH1  cold white, not present in old version */
  PORT_HAL_SetMuxMode(PORTB,18u, kPortMuxAlt5); /* TPM0_CH0 */
  PORT_HAL_SetMuxMode(PORTC, 0u, kPortMuxAlt5); /* TPM0_CH1 */
  PORT_HAL_SetMuxMode(PORTC, 1u, kPortMuxAlt5); /* TPM0_CH2 */


  // Init TPM WHITE
  TPM_DRV_Init(BOARD_TPM_INSTANCE_WHITE, &driverInfo_1);
  // Set clock for TPM WHITE
  TPM_DRV_SetClock(BOARD_TPM_INSTANCE_WHITE, kTpmClockSourceModuleClk, kTpmDividedBy2);

   
  // Init TPM COLOR 
  TPM_DRV_Init(BOARD_TPM_INSTANCE_RGB, &driverInfo_0);
  // Set clock for TPM COLOR
  TPM_DRV_SetClock(BOARD_TPM_INSTANCE_RGB, kTpmClockSourceModuleClk, kTpmDividedBy2);
  
}


void TPM_PWM_WarmWhite(uint8_t dutyCycleWarmWhite)
{
  paramWW.uDutyCyclePercent = dutyCycleWarmWhite;
  #ifndef DEBUG
    TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_WHITE, &paramWW, BOARD_TPM_CHANNEL_WHITE_WARM);
  #endif		
}

void TPM_PWM_ColdWhite(uint8_t dutyCycleColdWhite)
{
 paramCW.uDutyCyclePercent = dutyCycleColdWhite;
 TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_WHITE, &paramCW, BOARD_TPM_CHANNEL_WHITE_COLD);
}


void TPM_PWM_Red(uint8_t dutyCycleColorRed)
{
  paramR.uDutyCyclePercent = dutyCycleColorRed;
  TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_RGB, &paramR,  BOARD_TPM_CHANNEL_RGB_RED);
}

void TPM_PWM_Green(uint8_t dutyCycleColorGreen)
{
  paramG.uDutyCyclePercent = dutyCycleColorGreen;
  TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_RGB, &paramG,  BOARD_TPM_CHANNEL_RGB_GREEN);
}

void TPM_PWM_Blue(uint8_t dutyCycleColorBlue)
{
  paramB.uDutyCyclePercent = dutyCycleColorBlue;
  TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_RGB, &paramB,  BOARD_TPM_CHANNEL_RGB_BLUE); 
}

void TPM_PWM_Off(void)
{
  TPM_PWM_WarmWhiteOff();
  TPM_PWM_ColdWhiteOff();
  TPM_PWM_RedOff();
  TPM_PWM_GreenOff();
  TPM_PWM_BlueOff();	
}

void TPM_PWM_On(void)
{
  TPM_PWM_WarmWhiteOn();
  TPM_PWM_ColdWhiteOn();
  TPM_PWM_RedOn();
  TPM_PWM_GreenOn();
  TPM_PWM_BlueOn();	
}

           



/************************************************************************************
* Private functions
************************************************************************************/
