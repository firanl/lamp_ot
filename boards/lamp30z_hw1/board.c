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
//#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "fsl_adc16_driver.h"
#include "fsl_pmc_hal.h"
#include "Flash_Adapter.h"
#include "gpio_pins.h"
#include "FunctionLib.h"

/* core temperature measurement, voltage reference measurement */
#include "temperature_sensor.h"


#if cPWR_UsePowerDownMode
  #include "PWR_Interface.h"
#endif

#if gDCDC_Enabled_d
  #include "DCDC.h"
#endif


/************************************************************************************
* Private type definitions and macros
************************************************************************************/
extern uint32_t FREESCALE_PROD_DATA_BASE_ADDR[];


#if (initConstPublicDeviceAddress_d)
  extern const uint8_t gBDAddress_c[];
#else
  extern uint8_t gBDAddress_c[];
#endif

/* lamp control light data */
lamp_NVdata_t lamp_NVdata;

/************************************************************************************
* Private memory declarations
************************************************************************************/




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


/* ***********************************************************************************
* Private functions prototypes
*********************************************************************************** */
static void BOARD_InitRtcOsc(void);
static void BOARD_InitOsc0(void);
static void BOARD_ClockInit(void);
static void CLOCK_SetBootConfig(clock_manager_user_config_t const* config);
static void initHardwareParameters(void);

static void clone_RSIM_private_static_MAC(uint8_t bluetooth_address[]);

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
  
  
  /* core temperature measurement init */
  temperature_sensor_init ();
  /* init core temperature value and voltage reference */
  measure_chip_temperature();
  


  
  initHardwareParameters();
 

  
 
}


/* ***********************************************************************************
* Private functions
*********************************************************************************** */


/* init lamp NV data - lamp status, bluetooth adress, device info data */
static void initHardwareParameters(void)
{ 
  /* init lamp data */
  lamp_NVdata.lampControl.raw8 = LA_LAMP_CONTROL;
  lamp_NVdata.lampWhite.uint8.warmW = LA_LAMP_WARM_WHITE;
  lamp_NVdata.lampWhite.uint8.coldW = LA_LAMP_COLD_WHITE;
  lamp_NVdata.lampRGB.uint8.r = LA_LAMP_R;
  lamp_NVdata.lampRGB.uint8.g = LA_LAMP_G;
  lamp_NVdata.lampRGB.uint8.b = LA_LAMP_B;
  
  /* init MAC ADRESS */  
  clone_RSIM_private_static_MAC(gBDAddress_c);

   
}


static void clone_RSIM_private_static_MAC(uint8_t bluetooth_address[] )
{
    uint32_t *ptr;

    ptr = (uint32_t *)(RSIM_BASE + 0x0C); // RSIM_MAC_LSB
    bluetooth_address[0] = (uint8_t) *ptr;
    bluetooth_address[1] = (uint8_t) ( *ptr >> 8); 
    bluetooth_address[2] = (uint8_t) ( *ptr >> 16); 
    bluetooth_address[3] = (uint8_t) ( *ptr >> 24);  
    ptr = (uint32_t *)(RSIM_BASE + 0x08); // RSIM_MAC_MSB
    bluetooth_address[4] = (uint8_t) *ptr;
    bluetooth_address[5] = 0xC0;   // MSB is 11xx xxxx
}



static void clone_SIM_UIDL_private_static_MAC(uint8_t bluetooth_address[])
{
    uint32_t *ptr;
  
    ptr = (uint32_t *)(SIM_BASE  + 0x1060); // Unique Identification Register Low (SIM_UIDL)
    bluetooth_address[0] = (uint8_t) *ptr;
    bluetooth_address[1] = (uint8_t) ( *ptr >> 8); 
    bluetooth_address[2] = (uint8_t) ( *ptr >> 16); 
    bluetooth_address[3] = (uint8_t) ( *ptr >> 24);  
    ptr = (uint32_t *) (SIM_BASE + 0x105C); // Unique Identification Register Mid Low (SIM_UIDML)
    bluetooth_address[4] = (uint8_t) *ptr;
    bluetooth_address[5] = 0xC0;   // MSB is 11xx xxxx
}


static uint8_t test_MAC(uint8_t bluetooth_address[])
{
    int8_t i=6, j=0;
      
    /* check if bluetooth_address is valid */
    while(i--)
    {
       if(bluetooth_address[i] == 0xFF) j++;
        else if(bluetooth_address[i] == 0x00) j--;
    }
           
    if( (j==6) || (j==(-6)) )
    {
       return 0;
    }
    
    return 1;
}

/* Function to initialize RTC external clock base on board configuration. */
static void BOARD_InitRtcOsc(void)
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

/* Function to initialize OSC0 base on board configuration. */
static void BOARD_InitOsc0(void)
{
    MCG_WR_C2_RANGE(MCG,kOscRangeHigh);
    g_xtal0ClkFreq = 32000000U;
}

static void CLOCK_SetBootConfig(clock_manager_user_config_t const* config)
{
    CLOCK_SYS_SetSimConfigration(&config->simConfig);

    CLOCK_SYS_SetMcgMode(&config->mcgConfig);

    SystemCoreClock = CORE_CLOCK_FREQ;
}

/* Function to initialize clock base on board configuration. */
static void BOARD_ClockInit(void)
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
            .outdiv1   = 0U,
            .outdiv4   = 1U,        
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









/*******************************************************************************
 * EOF
 ******************************************************************************/

 
