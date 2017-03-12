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
   
#define LA_LAMP_GapAdShortenedLocalName  "FSL_OTAC"
#define LA_LAMP_GapAd_uuid_service       uuid_service_otap 
   //uuid_service_lamp
   
   
#define LA_LAMP_GapDeviceName            "FSL_BLE_OTAP_CLIENT"


#define DI_ManufacturerNameString  "Freescale"
#define DI_ModelNumberString       "OTAP Demo"
#define DI_SerialNumberString      "BLESN01"
#define DI_HardwareRevisionString  "FRDM-KW40Z"
#define DI_FirmwareRevisionString  "1.1.1"
#define DI_SoftwareRevisionString  "1.1.4"
#define DI_SystemId                "\x00\x00\x00\xFE\xFF\x9F\x04\x00"
#define DI_IeeeRcdl                0x00, 0x00, 0x00, 0x00
#define DI_PnpId                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

  
#define LA_LAMP_CONTROL      0xB0
#define LA_LAMP_WARM_WHITE   0x00
#define LA_LAMP_COLD_WHITE   0x00
#define LA_LAMP_R            0x00
#define LA_LAMP_G            0x01     
#define LA_LAMP_B            0x00 

/* temperature value - default 20.01 */
#define LA_LAMP_TEMP         0xD1, 0x07
/* core voltage - default 3.303 */
#define LA_LAMP_VCC          0xE4, 0x0C
// Year 1582 9999 defaulf 2017
#define LA_DATE_TIME_Y       0xE1, 0x07  
// Month 0 	12 values: 0 Month is not known; default 1 January
#define LA_DATE_TIME_M       0x01  
// Day 1 	31
#define LA_DATE_TIME_D       0x07
// Hours 0 	23
#define LA_DATE_TIME_H       0x09
// default 35
#define LA_DATE_TIME_MI      0x23
// default 
#define LA_DATE_TIME_S       0x01
 
  
   

typedef union prog_cycles_tag {
      uint32_t cnt;
      struct {
          uint8_t prog_cycles[4];
      } uint8;
  } prog_cycles_t;

 
/* Lamp basic control tag */
typedef union lamp_control_tag {
	uint8_t raw8;
	struct {
		uint8_t padding0   : 1; /*!< LSB   xxxx xxx1 - not used 0  */	
		uint8_t padding1   : 1; /*!<       xxxx xx1x - not used 0  */
		uint8_t reset      : 1; /*!<       0000 0100 - triggers a sw reset  */
		uint8_t mix        : 1; /*!<       xxxx 1xxx - White mix lock 1 / unlock 0 (long press btn functionality)  */
						
		uint8_t BTcon      : 1; /*!<       xxx1 xxxx - Lost connection enable switch on/off (byte 4 – On = 1 Off = 0)  */
		uint8_t Color      : 1; /*!<       xx1x xxxx - Lamp Color Light On Off - (byte 5 – On = 1 Off = 0)  */
		uint8_t White      : 1; /*!<       x1xx xxxx - Lamp White Light On Off - (byte 6 – On = 1 Off = 0)  */
		uint8_t OnOff      : 1; /*!<  MSB  1xxx xxxx - Lamp On Off - (byte 7 – On = 1 Off = 0)              */	
	} bit;
} lamp_control_t;

enum
{
  whiteMixLock_c = 0,
  whiteMixUnLock_c, 
};

typedef union lamp_white_tag {
	uint16_t raw16;
	struct {
		uint8_t warmW;   /*!<  LSB  warm white          */
		uint8_t coldW;   /*!<  MSB  cold white          */                
	} uint8;
} lamp_white_t;

typedef union lamp_color_tag {
	uint32_t raw32;
	struct {	
		uint8_t r;	  /*!< 	LSB  red          */
		uint8_t g;        /*!<       green         */
		uint8_t b;        /*!<       blue           */
		uint8_t padding;  /*!<  MSB	 - not used 0  */
	} uint8;
} lamp_color_t;

typedef struct lamp_NVdata_tag {
    lamp_control_t           lampControl;       // uint8_t
    lamp_white_t             lampWhite;         // uint16_t
    lamp_color_t             lampRGB;           // uint32_t
} lamp_NVdata_t;



#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/************************************************************************************
* Public prototypes
************************************************************************************/

void hardware_init(void);






#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __BOARD_H__ */
