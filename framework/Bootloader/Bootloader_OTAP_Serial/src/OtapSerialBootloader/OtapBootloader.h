/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file OtapBootloader.h
* OTAP Bootloader interface file
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

#ifndef __OTAPBOOTLOADER_H__
#define __OTAPBOOTLOADER_H__


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "Eeprom_Boot.h"



#if (MCU_MKW40Z160)
  #include "MKW40Z4.h"
  #include "MKW40Z160_cfg.h"
#else
  #error MCU not supported
#endif

#ifdef __cplusplus
    extern "C" {
#endif

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */

/*
 * Name: gSerialBootloaderEnable_c
 * Description: Enable UART Serial Boot. By default FALSE
 */
#ifndef gSerialBootloaderEnable_c
  #define gSerialBootloaderEnable_c FALSE
#endif

#ifndef gInitLeds_c
  #define gInitLeds_c TRUE
#endif

//#define gBootLoaderDebug_c TRUE

#define PortA_ClockEnable()    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
#define PortB_ClockEnable()    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
#define PortC_ClockEnable()    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

#define  WW_lo()   {GPIOA_PCOR |= 1 <<  0u;}
#define  WW_hi()   {GPIOA_PSOR |= 1 <<  0u;}
#define  CW_lo()   {GPIOB_PCOR |= 1 <<  3u;}
#define  CW_hi()   {GPIOB_PSOR |= 1 <<  3u;}
#define  R_lo()    {GPIOB_PCOR |= 1 << 18u;}
#define  R_hi()    {GPIOB_PSOR |= 1 << 18u;}
#define  G_lo()    {GPIOC_PCOR |= 1 <<  0u;}
#define  G_hi()    {GPIOC_PSOR |= 1 <<  0u;}
#define  B_lo()    {GPIOC_PCOR |= 1 <<  1u;}
#define  B_hi()    {GPIOC_PSOR |= 1 <<  1u;}      

/*
 * Name: gFlashProtection_c
 * Description: The value for FPROT register. By default the Flash is not Protected
 */
#define gFlashProtection_c 0xFFFFFFFF /* Flash is not write protected */
// #define gFlashProtection_c 0xFFFFFFFE /* The first Flash region is write protected */

/*
   FTFA_FOPT Bit 2
   NMI_DIS=1 NMI pin/interrupts reset default to enabled -> 0xFFFFFFFE 
   NMI_DIS=0 NMI interrupts are always blocked -> 0xFFFFFBFE 
*/
#define gFlash_NMI_DIS_0_c  0xFFFFFBFE
   

/*
 * Name: gMcuFlashBlocks_c
 * Description: The number of the internal flash blocks
 */

#define gMcuFlashBlocks_c FLASH_BLOCK_CNT
/*
 * Name: gMcuFlashSize_c
 * Description: The size of the internal FLASH
 */

#define gMcuFlashSize_c KINETIS_FLASH
/*
 * Name: gFlashErasePage_c
 * Description: The minimum amount of FLASH that can be erased
 */

#define gFlashErasePage_c  FLASH_ERASE_PAGE

/*
 * Name: gUserFlashStart_d
 * Description: Starting point of the application
 */
#define gUserFlashStart_d               ((uint32_t)(__region_BOOT_ROM_end__) + 1)

#if gMcuFlashBlocks_c == 2
/*
 * Name: gUserFlash2Start_d
 * Description: Start address of the second Flash block
*/
#define gUserFlash2Start_d     USER_FLASH_START_2
/*
 * Name: gMcuFlash2End_c
 * Description: End address of the second Flash block
*/
#define gMcuFlash2End_c        USER_FLASH_END_2
#endif
/*
 * Name: gBootImageFlagsAddress_c
 * Description: The flasg are located after the interrupt vector!
 */
#define gBootImageFlagsAddress_c        (gUserFlashStart_d + VECTOR_TABLE_SIZE)

/*
 * Name: gBootProductInfoAddress_c
 * Description: Address of the ProductInfo
 *              The Product info is located in the last sector of the FLASH
 */
#define gBootProductInfoAddress_c      (gMcuFlashSize_c - gFlashErasePage_c)

/*
 * Name: gInternalStorageStartAddress_c
 * Description: The start address of the internal storage
 */
#define gInternalStorageStartAddressOffset_c (0x44)



/* Defines used for implementing the boolean types when working with Flash */
#define gBootInvalidAddress_c          0xFFFFFFFF
#define gBootValueForTRUE_c            0x00
#define gBootValueForFALSE_c           0xFF

/* Init value for the mask used to look through each byte in the bitmap that
indicates which Flash sectors are write protected and should not be updated */
#define gBitMaskInit_c                       0x01
#define gBootData_ImageLength_Offset_c       0x00
#define gBootData_ImageLength_Size_c         0x04
#define gBootData_SectorsBitmap_Offset_c     gEepromAlignAddr_d(gBootData_ImageLength_Offset_c + \
                                                                gBootData_ImageLength_Size_c)
#define gBootData_SectorsBitmap_Size_c       (32)
#define gBootData_Image_Offset_c             gEepromAlignAddr_d(gBootData_SectorsBitmap_Offset_c + \
                                                                gBootData_SectorsBitmap_Size_c)



/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
/*
 * Name: FlashConfig_t
 * Description: Structure used to access Flash Configuration
 */
typedef union _FlashConfig_t
{
  uint32_t Data[4];
  struct {
    uint32_t BackDoorKey[2];
    uint32_t Protection;
    uint32_t Config;
  };
} FlashConfig_t;

/*
 * Name: bootFlags_t
 * Description: Structure used to inform the Bootloader if a new Image needs
 *              to be loaded.
 */
typedef PACKED_STRUCT
{
  uint8_t  newBootImageAvailable[gEepromParams_WriteAlignment_c];
  uint8_t  bootProcessCompleted[gEepromParams_WriteAlignment_c];
  uint8_t  bootVersion[2];
}bootInfo_t;

typedef void (*vector_entry)(void);
typedef void pointer(void);

/******************************************************************************
 *******************************************************************************
 * Public memory declarations
 *******************************************************************************
 ******************************************************************************/
#if defined(__IAR_SYSTEMS_ICC__)
    extern uint32_t __BOOT_STACK_ADDRESS[];
#elif defined(__GNUC__)
    extern uint32_t __SP_INIT[];
#endif

extern uint32_t __region_BOOT_ROM_start__[];
extern uint32_t __region_BOOT_ROM_end__[];

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
void __thumb_startup(void);
int main(int argc, char **argv);
void defaultISR(void);
void Boot_ResetMCU(void);
void Boot_Delay(uint32_t cnt);
void Boot_LedOK(void);
void Boot_LedNOK(void);
void Boot_LedRainbow(void);
void HandleBootError(void);

#ifdef __cplusplus
}
#endif

#endif /* __OTAPBOOTLOADER_H__ */
