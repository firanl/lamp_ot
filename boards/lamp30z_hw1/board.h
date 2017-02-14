/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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

#if !defined(__BOARD_H__)
#define __BOARD_H__

#include <stdint.h>
#include "pin_mux.h"
#include "gpio_pins.h"

/*************************************************************************************
* Public macros
*************************************************************************************/

/* The board name */
#define BOARD_NAME                      "FRDM-KW40Z"


/* Config clock */

  /* Clock variants */
  #define CLOCK_VLPR    1U
  #define CLOCK_RUN_16  2U
  #define CLOCK_RUN_32  3U
  #define CLOCK_NUMBER_OF_CONFIGURATIONS 3U

  /* Clock setup */
  #define CLOCK_INIT_CONFIG CLOCK_RUN_32


  #if (CLOCK_INIT_CONFIG == CLOCK_RUN_32)
      #define CORE_CLOCK_FREQ 32000000U
  #elif (CLOCK_INIT_CONFIG == CLOCK_RUN_16)
      #define CORE_CLOCK_FREQ 16000000U
  #else
      #define CORE_CLOCK_FREQ 4000000U    
  #endif



/* config hci_transport.h*/
  #define APP_SERIAL_INTERFACE_TYPE      (gSerialMgrLpuart_c)
  #define APP_SERIAL_INTERFACE_INSTANCE  (0)


/* RTC Setup */
  /* RTC pinout setup
  ALT0 kPortPinDisabled,  ALT1 kPortMuxAsGpio
  48QFN,32QFN,PinName,DEFAULT    ,ALT0     ,ALT1  ,ALT2 ,ALT3    ,ALT4 ,ALT5     ,ALT6 ,ALT7
  21,15,PTB16        ,EXTAL32K   ,EXTAL32K ,PTB16 ,—    ,I2C1_SCL,—    ,TPM2_CH0 ,—    ,—   
  22,16,PTB17        ,XTAL32K    ,XTAL32K  ,PTB17 ,—    ,I2C1_SDA,—    ,TPM2_CH1 ,—    ,—   
  */
  /* EXTAL0 PTB16 */
  #define EXTAL32K_PORT   PORTB
  #define EXTAL32K_PIN    16
  #define EXTAL32K_PINMUX kPortPinDisabled

  /* XTAL32K PTB17 */
  #define XTAL32K_PORT   PORTB
  #define XTAL32K_PIN    17
  #define XTAL32K_PINMUX kPortPinDisabled

  /* RTC external clock configuration. */
  #define RTC_XTAL_FREQ   32768U
  #define RTC_SC2P_ENABLE_CONFIG       false
  #define RTC_SC4P_ENABLE_CONFIG       false
  #define RTC_SC8P_ENABLE_CONFIG       false
  #define RTC_SC16P_ENABLE_CONFIG      false
  #define RTC_OSC_ENABLE_CONFIG        true

  /* The rtc instance used for rtc_func */
  #define BOARD_RTC_FUNC_INSTANCE         0


/* service_lamp db defaults */
#define LA_LAMP_CONTROL      0xA0
#define LA_LAMP_WHITE        0x00, 0x00
#define LA_LAMP_RGB          0x00, 0x05, 0x00



#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/************************************************************************************
* Public prototypes
************************************************************************************/

void hardware_init(void);


/* Function to initialize clock base on board configuration. */
void BOARD_ClockInit(void);

/* Function to initialize OSC0 base on board configuration. */
void BOARD_InitOsc0(void);

/* Function to initialize RTC external clock base on board configuration. */
void BOARD_InitRtcOsc(void);

/* Function to initialize ADC on board configuration. */
void BOARD_InitAdc(void);

/* Function to initialize DCDC on board configuration. */
void BOARD_DCDCInit(void);

/* Function to read battery level on board configuration. */
uint16_t BOARD_GetBatteryLevel(void);




#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __BOARD_H__ */
