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
    kGpioLED1        = GPIO_MAKE_PIN(GPIOC_IDX, 0),   /* USB-KW40Z4 Red LED 1 */
    kGpioLED2        = GPIO_MAKE_PIN(GPIOC_IDX, 1),   /* USB-KW40Z4 Red LED 2 */
    kGpioSW1         = GPIO_MAKE_PIN(GPIOB_IDX, 2),   /* USB-KW40Z4 switchPin1 */
    kGpioI2cDAP      = GPIO_MAKE_PIN(GPIOC_IDX, 19),  
    kGpioSpiDAP      = GPIO_MAKE_PIN(GPIOC_IDX, 18),
};

extern gpio_input_pin_user_config_t switchPins[];
extern gpio_output_pin_user_config_t ledPins[];

#endif /* __FSL_GPIO_PINS_H__ */
