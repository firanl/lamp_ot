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
#ifndef __FSL_GPIO_PINS_H__
#define __FSL_GPIO_PINS_H__

#include "fsl_gpio_driver.h"

/*! @file */
/*!*/
/*! This file contains gpio pin definitions used by gpio peripheral driver.*/
/*! The enums in _gpio_pins map to the real gpio pin numbers defined in*/
/*! gpioPinLookupTable. And this might be different in different board.*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief gpio pin names.*/
/*!*/ 
/*! This should be defined according to board setting.*/
enum _gpio_pins 
{
    //kGpioLED1        = GPIO_MAKE_PIN(GPIOC_IDX, 5),   /* FRDM-KW40Z4 Red LED 1 */
    kGpioLED1        = GPIO_MAKE_PIN(GPIOC_IDX, 1),   /* lamp Blue LED 1 */
    //kGpioLED2        = GPIO_MAKE_PIN(GPIOC_IDX, 4),   /* FRDM-KW40Z4 Red LED 2 */
    kGpioLED2        = GPIO_MAKE_PIN(GPIOC_IDX, 0),   /* lamp Green LED 2 */
    //kGpioLED3        = GPIO_MAKE_PIN(GPIOC_IDX, 1),   /* FRDM-KW40Z4 Red LED 3 */
    kGpioLED3        = GPIO_MAKE_PIN(GPIOB_IDX, 18),   /* lamp Red LED 3 */
    //kGpioLED4        = GPIO_MAKE_PIN(GPIOC_IDX, 0),   /* FRDM-KW40Z4 Red LED 4 */
    kGpioLED4        = GPIO_MAKE_PIN(GPIOB_IDX, 3),   /* lamp Cold White LED 4 */
    
    //kGpioSW1         = GPIO_MAKE_PIN(GPIOA_IDX, 18),  /* FRDM-KW40Z4 switchPin1 */
    kGpioSW1         = GPIO_MAKE_PIN(GPIOC_IDX, 3),  /* lamp CapacitiveTouch switchPin1 */
    //kGpioSW2         = GPIO_MAKE_PIN(GPIOA_IDX, 19),  /* FRDM-KW40Z4 switchPin2 */
    kGpioSW2         = GPIO_MAKE_PIN(GPIOB_IDX, 16),  /* lamp EXTAL32K switchPin2 */
    
    //kGpioI2cDAP      = GPIO_MAKE_PIN(GPIOB_IDX, 1),  
    kGpioI2cDAP      = GPIO_MAKE_PIN(GPIOC_IDX, 2),   /* lamp MIC_DATA    */
    //kGpioSpiDAP      = GPIO_MAKE_PIN(GPIOB_IDX, 1),
    kGpioSpiDAP      = GPIO_MAKE_PIN(GPIOC_IDX, 19),  /* lamp SPI_SS  AT45DB021E-SSHN-T  */
};

extern gpio_input_pin_user_config_t switchPins[];
extern gpio_output_pin_user_config_t ledPins[];

/* MKW30Z lamp pinout
32QFN,PinName,DEFAULT  , PCB function, used
 1,PTC19     ,DISABLED , SPI0_PCS0   , yes to AT45DB021E-SSHN-T  +10K pullup
 2,PTA0      ,SWD_DIO  , TPM1_CH0    , yes in debug SWD_DIO
 3,PTA1      ,SWD_CLK  , SWD_CLK     , yes in debug SWD_CLK
 4,PTA2      ,RESET_b  , RESET_b     , yes
 5,PSWITCH   ,PSWITCH  , PSWITCH     , yes tied to GND
 6,DCDC_CFG  ,DCDC_CFG , DCDC_CFG    , yes tied to 3.3V
 7,VDCDC_IN  ,VDCDC_IN , VDCDC_IN    , yes tied to 3.3V
 8,DCDC_GND  ,DCDC_GND , DCDC_GND    , yes tied to GND
 9,DCDC_LP   ,DCDC_LP  , DCDC_LP     , open
10,DCDC_LN   ,DCDC_LN  , DCDC_LN     , open
11,VDD_1P8OUT   ,VDD_1P8OUT ,VDD_1P8OUT ,yes tied to 3.3V
12,VDD_1P45OUT_PMCIN,VDD_1P45OUT_PMCIN,VDD_1P45OUT_PMCIN, yes tied to 3.3V
13,PTB3      ,ADC0_SE2/CMP0_IN4,TPM1_CH1, yes Cold White Leds channel BSS205
14,VDD_0     ,—        ,—            , yes tied to 3.3V
15,PTB16     ,EXTAL32K ,      ?      , open
16,PTB17     ,XTAL32K  ,      ?      , open
17,PTB18     ,NMI_b    ,TPM0_CH0     , yes Red Leds channel BSS205
18,VDDA      ,VDDA     ,VDDA         , yes tied to 3.3V
19,EXTAL_32M ,EXTAL_32M,EXTAL_32M    , yes to NXS3225SA-EXS00A-CS02368
20,XTAL_32M  ,XTAL_32M ,XTAL_32M     , yes to NXS3225SA-EXS00A-CS02368
21,VDD_RF2   ,VDD_RF2  ,VDD_RF2      , yes tied to 3.3V
22,RF_N      ,RF_N     ,RF_N         , yes to MURATA-LDB212G4005C-001
23,RF_P      ,RF_P     ,RF_P         , yes to MURATA-LDB212G4005C-001
24,VDD_RF1   ,VDD_RF1  ,VDD_RF1      , yes tied to 3.3V
25,PTC0      ,DISABLED ,TPM0_CH1     , yes Green Leds channel BSS205
26,PTC1      ,DISABLED ,TPM0_CH2     , yes Blue Leds channel BSS205
27,PTC2      ,DISABLED ,             , open
28,PTC3      ,DISABLED ,TSI0_CH15    , yes capacitive touch sensor
29,VDD_1     ,VDD      ,—            , yes tied to 3.3V
30,PTC16     ,DISABLED ,SPI0_SCK     , yes to AT45DB021E-SSHN-T  +10K pullup
31,PTC17     ,DISABLED ,SPI0_SOUT    , yes to AT45DB021E-SSHN-T  +10K pullup
32,PTC18     ,DISABLED ,SPI0_SIN     , yes to AT45DB021E-SSHN-T  
33-41,Ground ,NA       ,GND          , yes tied to GND
*/

/* MKW40Z MKW30Z pinout

48QFN,32QFN,PinName,DEFAULT    ,ALT0     ,ALT1 ,ALT2 ,ALT3 ,ALT4 ,ALT5 ,ALT6 ,ALT7
 1, 2,PTA0         ,SWD_DIO    ,TSI0_CH8 ,PTA0 ,SPI0_PCS1,— ,— ,TPM1_CH0,— ,SWD_DIO
 2, 3,PTA1         ,SWD_CLK    ,TSI0_CH9 ,PTA1 ,— ,— ,— ,TPM1_CH1,— ,SWD_CLK
 3, 4,PTA2         ,RESET_b    ,— ,PTA2 ,— ,— ,— ,TMP0_CH3,— ,RESET_b
 4, —,PTA16        ,DISABLED   ,TSI0_CH10 ,PTA16/LLWU_P4,SPI1_SOUT,— ,— ,TPM0_CH0,— ,—
 5, —,PTA17        ,DISABLED   ,TSI0_CH11 ,PTA17/LLWU_P5,SPI1_SIN,— ,— ,TPM_CLKIN1,— ,—
 6, —,PTA18        ,DISABLED   ,TSI0_CH12 ,PTA18/LLWU_P6,SPI1_SCK,— ,— ,TPM2_CH0,— ,—
 7, —,PTA19        ,DISABLED   ,TSI0_CH13 ,PTA19/LLWU_P7,SPI1_PCS0,— ,— ,TPM2_CH1,— ,—
 8, 5,PSWITCH      ,PSWITCH    ,PSWITCH ,— ,— ,— ,— ,— ,— ,—
 9, 6,DCDC_CFG     ,DCDC_CFG   ,DCDC_CFG ,— ,— ,— ,— ,— ,— ,—
10, 7,VDCDC_IN     ,VDCDC_IN   ,VDCDC_IN ,— ,— ,— ,— ,— ,— ,—
11, 9,DCDC_LP      ,DCDC_LP    ,DCDC_LP ,— ,— ,— ,— ,— ,— ,—
13, 8,DCDC_GND     ,DCDC_GND   ,DCDC_GND ,— ,— ,— ,— ,— ,— ,—
14,11,VDD_1P8OUT   ,VDD_1P8OUT ,VDD_1P8OUT ,— ,— ,— ,— ,— ,— ,—
12,10,DCDC_LN      ,DCDC_LN    ,DCDC_LN ,— ,— ,— ,— ,— ,— ,—
15,12,VDD_1P45OUT_PMCIN,VDD_1P45OUT_PMCIN,VDD_1P45OUT_PMCIN,— ,— ,— ,— ,— ,— ,—
16, —,PTB0         ,DISABLED   ,— ,PTB0/LLWU_P8,— ,I2C0_SCL,CMP0_OUT,TPM0_CH1,— ,CLKOUT
17, —,PTB1         ,ADC0_SE1/CMP0_IN5,ADC0_SE1/CMP0_IN5,PTB1 ,— ,I2C0_SDA,LPTMR0_ALT1,TPM0_CH2,— ,CMT_IRO
18, —,PTB2         ,ADC0_SE3/CMP0_IN3,ADC0_SE3/CMP0_IN3,PTB2 ,— ,— ,— ,TPM1_CH0,— ,—
19,13,PTB3         ,ADC0_SE2/CMP0_IN4,ADC0_SE2/CMP0_IN4,PTB3 ,— ,— ,CLKOUT,TPM1_CH1,— ,RTC_CLKOUT
20,14,VDD_0        ,—        ,—        ,— ,— ,— ,— ,— ,— ,—
21,15,PTB16        ,EXTAL32K ,EXTAL32K ,PTB16 ,— ,I2C1_SCL,— ,TPM2_CH0,— ,—
22,16,PTB17        ,XTAL32K  ,XTAL32K ,PTB17 ,— ,I2C1_SDA,— ,TPM2_CH1,— ,—
23,17,PTB18        ,NMI_b ,DAC0_OUT/ADC0_SE4/CMP0_IN2,PTB18 ,— ,I2C1_SCL,TPM_CLKIN0,TPM0_CH0,— ,NMI_b
24,— ,ADC0_DP0     ,ADC0_DP0/CMP0_IN0,ADC0_DP0/CMP0_IN0,— ,— ,— ,— ,— ,— ,—
25,— ,ADC0_DM0     ,—        ,ADC0_DM0/CMP0_IN1,— ,— ,— ,— ,— ,— ,—
26,— ,VSSA         ,VSSA     ,VSSA ,— ,— ,— ,— ,— ,— ,—
27,— ,VREFH        ,VREFH    ,VREFH ,— ,— ,— ,— ,— ,— ,—
28,18,VDDA         ,VDDA     ,VDDA ,— ,— ,— ,— ,— ,— ,—
29,19,EXTAL_32M    ,EXTAL_32M,EXTAL_32M ,— ,— ,— ,— ,— ,— ,—
30,20,XTAL_32M     ,XTAL_32M ,XTAL_32M ,— ,— ,— ,— ,— ,— ,—
31,— ,VDD_XTAL     ,VDD_XTAL ,VDD_XTAL ,— ,— ,— ,— ,— ,— ,—
32,21,VDD_RF2      ,VDD_RF2  ,VDD_RF2 ,— ,— ,— ,— ,— ,— ,—
33,22,RF_N         ,RF_N     ,RF_N ,— ,— ,— ,— ,— ,— ,—
34,23,RF_P         ,RF_P     ,RF_P ,— ,— ,— ,— ,— ,— ,—
35,24,VDD_RF1      ,VDD_RF1  ,VDD_RF1   ,— ,— ,— ,— ,— ,— ,—
36,25,PTC0         ,DISABLED ,—         ,PTC0/LLWU_P9,ANT_A ,I2C0_SCL,UART0_CTS_b,TPM0_CH1,— ,—
37,26,PTC1         ,DISABLED ,—         ,PTC1 ,ANT_B ,I2C0_SDA,UART0_RTS_b,TPM0_CH2,— ,BLE_ACTIVE
38,27,PTC2         ,DISABLED ,TSI0_CH14 ,PTC2/LLWU_P10,TX_SWITCH,I2C1_SCL,UART0_RX,CMT_IRO,— ,DTM_RX
39,28,PTC3         ,DISABLED ,TSI0_CH15 ,PTC3/LLWU_P11,RX_SWITCH,I2C1_SDA,UART0_TX,— ,— ,DTM_TX
40,— ,PTC4         ,DISABLED ,TSI0_CH0  ,PTC4/LLWU_P12,— ,EXTRG_IN,UART0_CTS_b,TPM1_CH0,— ,—
41,— ,PTC5         ,DISABLED ,TSI0_CH1  ,PTC5/LLWU_P13,— ,LPTMR0_ALT2,UART0_RTS_b,TPM1_CH1,— ,—
42,— ,PTC6         ,DISABLED ,TSI0_CH2  ,PTC6/LLWU_P14,— ,I2C1_SCL,UART0_RX,TPM2_CH0,— ,—
43,— ,PTC7         ,DISABLED ,TSI0_CH3  ,PTC7/LLWU_P15,SPI0_PCS2,I2C1_SDA,UART0_TX,TPM2_CH1,— ,—
44,29,VDD_1        ,VDD       ,—        ,— ,— ,— ,— ,— ,— ,—
45,30,PTC16        ,DISABLED ,TSI0_CH4  ,PTC16/LLWU_P0,SPI0_SCK,I2C0_SDA,UART0_RTS_b,TPM0_CH3,— ,—
46,31,PTC17        ,DISABLED ,TSI0_CH5  ,PTC17/LLWU_P1,SPI0_SOUT,— ,UART0_RX,— ,— ,DTM_RX
47,32,PTC18        ,DISABLED ,TSI0_CH6  ,PTC18/LLWU_P2,SPI0_SIN,— ,UART0_TX,— ,— ,DTM_TX
48, 1,PTC19        ,DISABLED ,TSI0_CH7  ,PTC19/LLWU_P3,SPI0_PCS0,I2C0_SCL,UART0_CTS_b,— ,— ,BLE_ACTIVE
49-64,33-41        ,Ground   ,NA        ,NA ,NA ,NA ,NA ,NA ,NA ,NA ,NA
*/

#endif /* __FSL_GPIO_PINS_H__ */
