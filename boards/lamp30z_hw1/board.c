/*
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
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
#include "EmbeddedTypes.h"
#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_smc_hal.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "fsl_adc16_driver.h"
#include "fsl_pmc_hal.h"
#include "Flash_Adapter.h"
#include "gpio_pins.h"



#if cPWR_UsePowerDownMode
#include "PWR_Interface.h"
#endif

#if gDCDC_Enabled_d
#include "DCDC.h"
#endif


/************************************************************************************
* Private type definitions and macros
************************************************************************************/
#define ADC16_INSTANCE                (0)   /* ADC instance */
#define ADC16_CHN_GROUP               (0)   /* ADC group configuration selection */
#define ADC16_POTENTIOMETER_CHN       (kAdc16Chn0) /* Potentiometer channel */

#define ADC16_BATLVL_CHN              (kAdc16Chn23) /* Potentiometer channel */
#define ADC16_BL_LOWER_LIMIT          (0) /* min percentage of battery charge */
#define ADC16_BL_UPPER_LIMIT          (100) /* max percentage of battery charge */
#define ADC16_BL_DYNAMIC_RANGE        (ADC16_BL_UPPER_LIMIT - ADC16_BL_LOWER_LIMIT) /* Range = [ADC16_HB_LOWER_LIMIT .. ADC16_HB_LOWER_LIMIT + ADC16_HB_DYNAMIC_RANGE] */

#define ADC16_BANDGAP_CHN             (kAdc16Chn27) /* ADC channel of BANDGAP Voltage reference*/

#define MIN_VOLT_BUCK 180
#define MAX_VOLT_BUCK 310
#define FULL_BAT      100
#define EMPTY_BAT     0

/************************************************************************************
* Private memory declarations
************************************************************************************/
uint32_t offsetVdd = 0;               
adc16_converter_config_t adcUserConfig;   // structure for user config

static uint32_t adcValue = 0; /* ADC value */
static adc16_converter_config_t adcUserConfig; /* structure for user config */





/* Declare Input GPIO pins */
gpio_input_pin_user_config_t switchPins[] = {
    {
        .pinName = kGpioSW1,
        .config.isPullEnable = true,
        .config.pullSelect = kPortPullUp,
        .config.isPassiveFilterEnabled = false,
        .config.interrupt = kPortIntFallingEdge,
    },
    {
        .pinName = kGpioSW2,
        .config.isPullEnable = true,
        .config.pullSelect = kPortPullUp,
        .config.isPassiveFilterEnabled = false,
        .config.interrupt = kPortIntFallingEdge
    },
    {
        .pinName = GPIO_PINS_OUT_OF_RANGE,
    }
};

/* Declare Output GPIO pins */
gpio_output_pin_user_config_t ledPins[] = {
    {
        .pinName = kGpioLED1,
        .config.outputLogic = 1,
        .config.slewRate = kPortSlowSlewRate,
        .config.driveStrength = kPortLowDriveStrength,
    },
    {
        .pinName = kGpioLED2,
        .config.outputLogic = 1,
        .config.slewRate = kPortSlowSlewRate,
        .config.driveStrength = kPortLowDriveStrength,
    },
    {
        .pinName = kGpioLED3,
        .config.outputLogic = 1,
        .config.slewRate = kPortSlowSlewRate,
        .config.driveStrength = kPortLowDriveStrength,
    },
    {
        .pinName = kGpioLED4,
        .config.outputLogic = 1,
        .config.slewRate = kPortSlowSlewRate,
        .config.driveStrength = kPortLowDriveStrength,
    },
    {
        .pinName = GPIO_PINS_OUT_OF_RANGE,
    }
};




/************************************************************************************
* Private functions prototypes
************************************************************************************/
static void ADC16_CalibrateParams(void);
static inline uint32_t ADC16_Measure(void);
static inline uint32_t ADC16_BatLvl(void);
static inline uint32_t ADC16_BgLvl(void);
static uint16_t ADC16_ReadValue(adc16_chn_t chnIdx, uint8_t diffMode);
static void DCDC_AdjustVbatDiv4();
static void CLOCK_SetBootConfig(clock_manager_user_config_t const* config);
/************************************************************************************
* Public functions prototypes
************************************************************************************/


/************************************************************************************
* Public functions
************************************************************************************/

void hardware_init(void) {

  if((PMC->REGSC & PMC_REGSC_ACKISO_MASK) != 0x00U)
  {
    PMC->REGSC |= PMC_REGSC_ACKISO_MASK; /* Release hold with ACKISO:  Only has an effect if recovering from VLLSx.*/
    /*clear power management registers after recovery from vlls*/
    SMC_BWR_STOPCTRL_LLSM(SMC, 0);
    SMC_BWR_PMCTRL_STOPM(SMC, 0);
    SMC_BWR_PMCTRL_RUNM(SMC, 0);
  }
  /* enable clock for PORTs */
  CLOCK_SYS_EnablePortClock(PORTA_IDX);
  CLOCK_SYS_EnablePortClock(PORTB_IDX);
  CLOCK_SYS_EnablePortClock(PORTC_IDX);

  /* Init board clock */
  BOARD_ClockInit();
  
  
  NV_ReadHWParameters(&gHardwareParameters);
}

/* Function to initialize OSC0 base on board configuration. */
void BOARD_InitOsc0(void)
{
    MCG_WR_C2_RANGE(MCG,kOscRangeHigh);
    g_xtal0ClkFreq = 32000000U;
}

/* Function to initialize RTC external clock base on board configuration. */
void BOARD_InitRtcOsc(void)
{
    rtc_osc_user_config_t rtcOscConfig =
    {
        .freq                = RTC_XTAL_FREQ,
        .enableCapacitor2p   = RTC_SC2P_ENABLE_CONFIG,
        .enableCapacitor4p   = RTC_SC4P_ENABLE_CONFIG,
        .enableCapacitor8p   = RTC_SC8P_ENABLE_CONFIG,
        .enableCapacitor16p  = RTC_SC16P_ENABLE_CONFIG,
        .enableOsc           = RTC_OSC_ENABLE_CONFIG,
    };

    CLOCK_SYS_RtcOscInit(0U, &rtcOscConfig);
}

void BOARD_InitAdc(void)
{
  SIM_HAL_EnableClock(SIM, kSimClockGateDcdc);
  CLOCK_SYS_EnableAdcClock(0);
  ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
  adcUserConfig.resolution = kAdc16ResolutionBitOfDiffModeAs13;
  adcUserConfig.refVoltSrc = kAdc16RefVoltSrcOfVref;
  ADC16_DRV_Init(ADC16_INSTANCE, &adcUserConfig);
  ADC16_CalibrateParams();
  
}

uint16_t BOARD_GetBatteryLevel(void)
{
    uint16_t batVal, bgVal, batVolt, bgVolt = 0; /*cV*/
    
    bgVal = ADC16_BgLvl();
    DCDC_AdjustVbatDiv4(); /* Bat voltage  divided by 4 */
    batVal = ADC16_BatLvl() * 4; /* Need to multiply the value by 4 because the measured voltage is divided by 4*/
    
    batVolt = bgVolt * batVal / bgVal;
    
    return batVolt;    
}


/* Initialize clock. */
void BOARD_ClockInit(void)
{
    /* Configuration for enter RUN mode. Core clock = 16MHz / 32MHz. */
    const clock_manager_user_config_t g_defaultClockConfigRun =
    {
        .mcgConfig =
        {
            .mcg_mode           = kMcgModeBLPE, // Work in BLPE mode.
            .irclkEnable        = true,  // MCGIRCLK enable.
            .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
            .ircs               = kMcgIrcSlow, // Select IRC32k.
            .fcrdiv             = 0U,    // FCRDIV is 0.

            .frdiv   = 5U,
            .drs     = kMcgDcoRangeSelLow,  // Low frequency range
            .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%
            .oscsel  = kMcgOscselOsc,       // Select 
        },
        .simConfig =
        {
            .pllFllSel = kClockPllFllSelFll,    // PLLFLLSEL select FLL.
            .er32kSrc  = kClockEr32kSrcOsc0,     // ERCLK32K selection, use OSC0.
    #if CLOCK_INIT_CONFIG == CLOCK_RUN_16        
            .outdiv1   = 1U,
            .outdiv4   = 0U,
    #else
            .outdiv1   = 0U,
            .outdiv4   = 1U,
    #endif        
        }
    };
    
    /* Set allowed power mode, allow all. */
    SMC_HAL_SetProtection(SMC, kAllowPowerModeAll);

    /* Setup board clock source. */
    // Setup OSC0 if used.
    // Configure OSC0 pin mux.
    PORT_HAL_SetMuxMode(EXTAL32K_PORT, EXTAL32K_PIN, EXTAL32K_PINMUX);
    PORT_HAL_SetMuxMode( XTAL32K_PORT,  XTAL32K_PIN,  XTAL32K_PINMUX);

    // OSC0 has not configuration register, only set frequency
    BOARD_InitOsc0();


    /* Set system clock configuration. */
    CLOCK_SetBootConfig(&g_defaultClockConfigRun);
    
    /* set TPM clock */
    CLOCK_SYS_SetTpmSrc(0, kClockTpmSrcOsc0erClk);
}





/************************************************************************************
* Private functions
************************************************************************************/

/*!
 * @brief Parameters calibration: VDD and ADCR_TEMP25
 *
 * This function used BANDGAP as reference voltage to measure vdd and
 * calibrate V_TEMP25 with that vdd value.
 */
static const adc16_hw_average_config_t adcHwAverageConfig =
{
  .hwAverageEnable = true, /*!< Enable the hardware average function. */
  .hwAverageCountMode = kAdc16HwAverageCountOf16 /*!< Select the count of conversion result for accumulator. */
} ;


void ADC16_CalibrateParams(void)
{
    adc16_calibration_param_t adcCalibraitionParam;   

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    ADC16_DRV_GetAutoCalibrationParam(ADC16_INSTANCE, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC16_INSTANCE, &adcCalibraitionParam);
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
    
  ADC16_DRV_ConfigHwAverage(0, &adcHwAverageConfig);
  
    pmc_bandgap_buffer_config_t pmcBandgapConfig = {
        .enable = true,
#if FSL_FEATURE_PMC_HAS_BGEN
        .enableInLowPower = false,
#endif
#if FSL_FEATURE_PMC_HAS_BGBDS
        .drive = kPmcBandgapBufferDriveLow,
#endif
    };
    
    // Enable BANDGAP reference voltage
    PMC_HAL_BandgapBufferConfig(PMC_BASE_PTR, &pmcBandgapConfig);
}


/*!
 * @brief Gets the current voltage of the battery
 *
 * This function measure the ADC channel corresponding to the battery
 */
static inline uint32_t ADC16_BatLvl(void)
{
    adcValue = ADC16_ReadValue((adc16_chn_t)ADC16_BATLVL_CHN, false);
    return adcValue;
}

/*!
 * @brief Gets the current bandgap voltage
 *
 * This function measure the ADC channel corresponding to the bandgap
 */
static inline uint32_t ADC16_BgLvl(void)
{
    adcValue = ADC16_ReadValue((adc16_chn_t)ADC16_BANDGAP_CHN, false);
    return adcValue;
}


/*!
 * @brief Reads the ADC value from the channel given as input
 *
 * This function measure the ADC channel given as input
 */
static uint16_t ADC16_ReadValue(adc16_chn_t chnIdx, uint8_t diffMode)
{
  adc16_chn_config_t chnConfig;

    /* Configure the conversion channel */
    chnConfig.chnIdx     = chnIdx;
#if FSL_FEATURE_ADC16_HAS_DIFF_MODE
    chnConfig.diffConvEnable = diffMode;
#endif
    chnConfig.convCompletedIntEnable  = false;

    /* Software trigger the conversion */
    ADC16_DRV_ConfigConvChn(ADC16_INSTANCE, ADC16_CHN_GROUP, &chnConfig);

    /* Wait for the conversion to be done */
    ADC16_DRV_WaitConvDone(ADC16_INSTANCE, ADC16_CHN_GROUP);

    /* Fetch the conversion value */
    adcValue = (diffMode) ? ADC16_DRV_GetConvValueSigned(ADC16_INSTANCE, ADC16_CHN_GROUP) : ADC16_DRV_GetConvValueRAW(ADC16_INSTANCE, ADC16_CHN_GROUP);

    /* Calculates adcValue in 16bit resolution from 12bit resolution 
    in order to convert to reading */
#if (FSL_FEATURE_ADC16_MAX_RESOLUTION < 16)
    adcValue = adcValue << 4;
#endif
    /* Pause the conversion */
    ADC16_DRV_PauseConv(ADC16_INSTANCE, ADC16_CHN_GROUP);
    
    return adcValue;
}

static void DCDC_AdjustVbatDiv4()
{
  const uint8_t vBatDiv = 3;
  DCDC_BWR_REG0_DCDC_VBAT_DIV_CTRL(DCDC_BASE_PTR, vBatDiv);  
}

static void CLOCK_SetBootConfig(clock_manager_user_config_t const* config)
{
    CLOCK_SYS_SetSimConfigration(&config->simConfig);

    CLOCK_SYS_SetMcgMode(&config->mcgConfig);

    SystemCoreClock = CORE_CLOCK_FREQ;
}


/*******************************************************************************
 * EOF
 ******************************************************************************/

 
