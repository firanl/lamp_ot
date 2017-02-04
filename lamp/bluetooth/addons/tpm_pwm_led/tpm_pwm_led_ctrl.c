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

/* Set uMod 'magic' constant number generated in DEBUG mode;
 look inside TPM_DRV_PwmStart in debug mode, depends on BOARD_TPM_FRQ_HZ and uC clocks */
#if (BOARD_TPM_MODE==kTpmEdgeAlignedPWM)
  static const uint16_t uMod = 132;
#else 
  static const uint16_t uMod = 66;
#endif

/************************************************************************************
* Private memory declarations
************************************************************************************/

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
  
   tpm_pwm_param_t param = {
              .mode              = BOARD_TPM_MODE, 
              .edgeMode          = kTpmHighTrue,
              .uFrequencyHZ      = BOARD_TPM_FRQ_HZ,
              .uDutyCyclePercent = PWM_factor_Init // 0 to 100
   };
	  
    
    #ifndef DEBUG
      PORT_HAL_SetMuxMode(PORTA, 0u,kPortMuxAlt5); /* TPM1_CH0 warm white*/
      //PORT_HAL_SetMuxMode(PORTA, 1u,kPortMuxAlt5); /* TPM1_CH1 cold white old versions */
    #endif  
      
    PORT_HAL_SetMuxMode(PORTB, 3u, kPortMuxAlt5); /* TPM1_CH1  cold white */
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
     
      
    // Init PWM module with configuration.
    #ifndef DEBUG
		TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_WHITE, &param, BOARD_TPM_CHANNEL_WHITE_WARM);
    #endif
    
    TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_WHITE, &param, BOARD_TPM_CHANNEL_WHITE_COLD);
    
    TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_RGB, &param,  BOARD_TPM_CHANNEL_RGB_RED);
    TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_RGB, &param,  BOARD_TPM_CHANNEL_RGB_GREEN);
    TPM_DRV_PwmStart(BOARD_TPM_INSTANCE_RGB, &param,  BOARD_TPM_CHANNEL_RGB_BLUE);
}



void TPM_PWM_WarmWhite(uint8_t dutyCycleWarmWhite)
{
	uint16_t uCnvWW;	
	TPM_Type *tpmBaseWHITE = g_tpmBase[BOARD_TPM_INSTANCE_WHITE];
	
    uCnvWW = uMod * dutyCycleWarmWhite / 100;
    /* For 100% duty cycle */
    if(uCnvWW >= uMod) {  uCnvWW = uMod + 1;  }  	
	
    #ifndef DEBUG
		TPM_HAL_SetChnCountVal(tpmBaseWHITE, BOARD_TPM_CHANNEL_WHITE_WARM, uCnvWW); 
    #endif		
}

void TPM_PWM_ColdWhite(uint8_t dutyCycleColdWhite)
{
	uint16_t uCnvCW;	
	TPM_Type *tpmBaseWHITE = g_tpmBase[BOARD_TPM_INSTANCE_WHITE];
	
    uCnvCW = uMod * dutyCycleColdWhite / 100;
    /* For 100% duty cycle */
    if(uCnvCW >= uMod) {  uCnvCW = uMod + 1;  }  	
	
    TPM_HAL_SetChnCountVal(tpmBaseWHITE, BOARD_TPM_CHANNEL_WHITE_COLD, uCnvCW);	
}


void TPM_PWM_Red(uint8_t dutyCycleColorRed)
{
	uint16_t uCnvR;	
	TPM_Type *tpmBaseRGB   = g_tpmBase[BOARD_TPM_INSTANCE_RGB];
	
    uCnvR = uMod * dutyCycleColorRed / 100;
    /* For 100% duty cycle */
    if(uCnvR >= uMod) {  uCnvR = uMod + 1;  }	
	
    TPM_HAL_SetChnCountVal(tpmBaseRGB, BOARD_TPM_CHANNEL_RGB_RED, uCnvR);
}

void TPM_PWM_Green(uint8_t dutyCycleColorGreen)
{
	uint16_t uCnvG;	
	TPM_Type *tpmBaseRGB   = g_tpmBase[BOARD_TPM_INSTANCE_RGB];
	
     uCnvG = uMod * dutyCycleColorGreen / 100;
    /* For 100% duty cycle */
    if(uCnvG >= uMod) {  uCnvG = uMod + 1;  }	
	
    TPM_HAL_SetChnCountVal(tpmBaseRGB, BOARD_TPM_CHANNEL_RGB_GREEN, uCnvG);
}

void TPM_PWM_Blue(uint8_t dutyCycleColorBlue)
{
	uint16_t uCnvB;	
	TPM_Type *tpmBaseRGB   = g_tpmBase[BOARD_TPM_INSTANCE_RGB];
	
    uCnvB = uMod * dutyCycleColorBlue / 100;
    /* For 100% duty cycle */
    if(uCnvB >= uMod) {  uCnvB = uMod + 1;  }   
	
    TPM_HAL_SetChnCountVal(tpmBaseRGB, BOARD_TPM_CHANNEL_RGB_BLUE, uCnvB);
}

void TPM_PWM_Off(void)
{
	TPM_PWM_WarmWhiteOff();
	TPM_PWM_ColdWhiteOff();
	TPM_PWM_RedOff();
	TPM_PWM_GreenOff();
	TPM_PWM_BlueOff();	
}




/************************************************************************************
* Private functions
************************************************************************************/
