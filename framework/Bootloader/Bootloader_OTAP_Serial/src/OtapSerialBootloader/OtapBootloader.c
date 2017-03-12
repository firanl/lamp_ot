/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file OtapBootloader.c
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


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "OtapBootloader.h"
#include "flash_boot_kinetis.h"
#include "Eeprom_Boot.h"
#include "UartBootloader.h"


/*! *********************************************************************************
*************************************************************************************
* Private Memory Declarations
*************************************************************************************
********************************************************************************** */
#if defined(__IAR_SYSTEMS_ICC__)
#pragma section = "IntVectTable"
#pragma location = "IntVectTable"
__root const vector_entry __vector_table[16] =
{
    (pointer*)__BOOT_STACK_ADDRESS,  /* Initial SP           */

#elif defined(__GNUC__)
const vector_entry __vector_table[16] __attribute__ ((section(".vectortable"))) =
{
    (pointer*)__SP_INIT,  /* Initial SP           */
#endif

    __thumb_startup, /* Initial PC           */
    defaultISR,      /* Non-maskable Interrupt (NMI) */
    defaultISR,      /* Hard Fault */
    defaultISR,      /* MemManage Fault */
    defaultISR,      /* Bus Fault */
    defaultISR,      /* Usage Fault */
    defaultISR,      /* Usage Fault */
    defaultISR,      /* Usage Fault */
    defaultISR,      /* Usage Fault */
    defaultISR,      /* Usage Fault */
    defaultISR,      /* Usage Fault */
    defaultISR,      /* Usage Fault */
    defaultISR,      /* Usage Fault */
    defaultISR,      /* Usage Fault */
    defaultISR       /* Usage Fault */
};

#if defined(__IAR_SYSTEMS_ICC__)
#pragma location = "FlashConfig"
__root const FlashConfig_t gFlashConfig @ "FlashConfig" =
#elif defined(__GNUC__)
const FlashConfig_t gFlashConfig __attribute__ ((section(".cfmconfig"))) =
#endif
{
    {0xFFFFFFFF,
     0xFFFFFFFF,
     gFlashProtection_c,
     gFlash_NMI_DIS_0_c }  
};

/* Variables used by the Bootloader */
volatile bootInfo_t *gpBootInfo;



/*! *********************************************************************************
*************************************************************************************
* Public Functions
*************************************************************************************
********************************************************************************** */


/*! *********************************************************************************
* \brief   The function resets the MCU
*
********************************************************************************** */
void Boot_ResetMCU(void)
{
    SCB_AIRCR = SCB_AIRCR_VECTKEY(0x5FA) | SCB_AIRCR_SYSRESETREQ_MASK;
    while(1);
}

void Boot_Delay(uint32_t cnt)
{
    while(cnt--){ asm("NOP"); };
}

void Boot_LedOK(void)
{
  int8_t cnt=3;
  
  while(cnt--)
  {
    G_hi();  
    Boot_Delay(0x00120000);
    G_lo(); 
    Boot_Delay(0x00120000);
  }
}

void Boot_LedNOK(void)
{
  int8_t cnt=3;
  
  while(cnt--)
  {
    R_hi();  
    Boot_Delay(0x00120000);
    R_lo(); 
    Boot_Delay(0x00120000);
  }
}

void Boot_LedRainbow(void)
{
  
    CW_hi();  
    Boot_Delay(0x0006F000);
    CW_lo(); R_hi();
    Boot_Delay(0x0009F000);
    R_lo(); WW_hi();
    Boot_Delay(0x0009F000);
    WW_lo(); G_hi();
    Boot_Delay(0x0007F000);    
    G_lo(); B_hi();
    Boot_Delay(0x0009F000);    
  
}

/*! *********************************************************************************
* \brief   Start the user application
*
* \param[in] userStartup  Address of the application's interrupt vector
*
********************************************************************************** */
static void JumpToApplication(volatile uint32_t userStartup)
{
    /* Enable ALL interrupts */
    asm("     cpsie   i       ");

    /* set up stack pointer */
    asm("LDR      r1,  [r0]");
    asm("MSR      MSP, r1");

    /* jump to application reset vector */
    asm("ADDS     r0,r0,#0x04 ");
    asm("LDR      r0, [r0]");
    asm("BX       r0");
}




/*! *********************************************************************************
* \brief   This function will copy the User Application from the external memory
*          into the program Flash
*
********************************************************************************** */
void Boot_LoadImage (void)
{
    static uint8_t bitmapBuffer[gBootData_SectorsBitmap_Size_c];
    bootInfo_t flags;
    uint8_t  buffer[gFlashErasePage_c];
    uint32_t remaingImgSize, len;
    uint32_t flashAddr      = 0;
    uint8_t  bitMask        = gBitMaskInit_c;
    uint8_t *pBitmap        = bitmapBuffer;



    /* Init the flash module */
    FlashInitialization();

    /* Init the external storage - if fails reset - true return point */
    if(EEPROM_Init() != ee_ok) 
         HandleBootError();

    /* Read image size - if fails reset - true return point  */
    if (EEPROM_ReadData(gBootData_ImageLength_Size_c,gBootData_ImageLength_Offset_c, (uint8_t*)(&remaingImgSize)) != ee_ok) 
        HandleBootError();

    /* Read sector bitmap - if fails reset - true return point */
    if (EEPROM_ReadData(gBootData_SectorsBitmap_Size_c, gBootData_SectorsBitmap_Offset_c, bitmapBuffer)  != ee_ok ) 
        HandleBootError();

    #if gInitLeds_c  
      Boot_LedOK();
    #endif    
      
    /* TODO: first sector to erase should contain bootInfo_t flags - setting them to FALSE, FALSE */
    /* implement field - how many times the device was flashed */
    
    /* Start writing the image. Do not alter the last sector which contains HW specific data! */
    while (flashAddr < (gMcuFlashSize_c - gFlashErasePage_c))
    {
        if (remaingImgSize > gFlashErasePage_c)
            len = gFlashErasePage_c;
        else
            len = remaingImgSize;

        /* Check if bitmap indicates that this sector is write protected and shouldn't be updated */
        if ((*pBitmap & bitMask) && (flashAddr >= gUserFlashStart_d))
        {
            /* Erase Flash sector */
            if (FLASH_OK != FLASH_EraseSector(flashAddr))
                  HandleBootError();

            if (len)
            {
                /* Read a new image block - if fails reset - false return point */
                if (EEPROM_ReadData(len, flashAddr + gBootData_Image_Offset_c, buffer)  != ee_ok )
                          HandleBootError();


                if( (flashAddr <= gBootImageFlagsAddress_c) && (flashAddr + len > gBootImageFlagsAddress_c) )
                {
                    uint32_t i, offset = gBootImageFlagsAddress_c - flashAddr;
                    
                    /* Program the Flash before boot flags  - if fails reset - false return point*/
                    if(FLASH_OK != FLASH_Program(flashAddr, (uint32_t)buffer, offset))
                            HandleBootError();

                    /* Keep the boot flags set  until the all image is downloaded */
                    for( i=0; i<gEepromParams_WriteAlignment_c; i++ )
                    {
                        flags.newBootImageAvailable[i] = gBootValueForFALSE_c;
                        flags.bootProcessCompleted[i] = gBootValueForTRUE_c;  
                    }
                    i = offset + 2 * gEepromParams_WriteAlignment_c;
                    flags.bootVersion[0] = buffer[i++];
                    flags.bootVersion[1] = buffer[i];
                    offset += gEepromAlignAddr_d(sizeof(bootInfo_t));

                    /* Program the Flash after the boot flags*/
                    if(FLASH_OK != FLASH_Program(flashAddr + offset, (uint32_t)(&buffer[offset]), len - offset))
                          HandleBootError();
                }
                else
                {
                    /* Program the image block to Flash */
                    if(FLASH_OK != FLASH_Program(flashAddr, (uint32_t)buffer, len))
                          HandleBootError();
                }
            }
        }

        /* Update Bitmask */
        bitMask <<= 1;
        if (bitMask == 0)
        {
            /* This was last bit in the current bitmap byte. Move to next bitmap byte */
            bitMask = gBitMaskInit_c;
            pBitmap++;
        }

        /* Update the current flash address */
        flashAddr += gFlashErasePage_c;

        /* Update the remaining bytes*/
        if (remaingImgSize)
            remaingImgSize -= len;
    } /* while */



    /* Set the bBootProcessCompleted Flag */
    if( FLASH_OK != FLASH_Program((uint32_t)gBootImageFlagsAddress_c, (uint32_t)&flags, sizeof(flags)) )
            HandleBootError();

#if gInitLeds_c    
    Boot_LedRainbow();
#endif
    
    /* Reseting MCU */
    Boot_ResetMCU();
}

/* Defines how the bootloader should handle errors */
void HandleBootError(void)
{
#if gInitLeds_c  
  Boot_LedNOK();
#endif
  
#ifdef gBootLoaderDebug_c
  {  while(1); }
#else
  { Boot_ResetMCU(); }
#endif
}

/*! *********************************************************************************
* \brief   This is the Bootloader's entry point
*
********************************************************************************** */
void __thumb_startup(void)
{
    (void)main(0, 0);
}

/*! *********************************************************************************
* \brief   This is the main Bootloader function.
*          It decides if it will load a new image or jump to the application
*
* \param[in] argc
* \param[in] argv
*
* \return int
*
********************************************************************************** */
int main(int argc, char **argv)
{
    /* Disable interrupt by default */
    asm("     CPSID   i       ");

    /* Set the start address of the interrupt vector*/
    SCB_VTOR = (uint32_t)__region_BOOT_ROM_start__;

    /* Disable watchdog */
#if defined(MCU_MKW40Z160)
    SIM_COPC = SIM_COPC_COPT(0);
#endif
#if gSerialBootloaderEnable_c
    BOOT_PIN_ENABLE_SIM_SCG_REG |= BOOT_PIN_ENABLE_SIM_SCG_MASK;
    PORT_PCR_REG(BOOT_PIN_ENABLE_PORT_BASE,BOOT_PIN_ENABLE_NUM) = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
    {
        vuint32_t delay;
        delay = 5;
        while(--delay);
    }
    if((GPIO_PDIR_REG(BOOT_PIN_ENABLE_GPIO_BASE) & (1 << BOOT_PIN_ENABLE_NUM)) == 0)
    {
        CheckForUartLoader();
    }
#endif

#if gInitLeds_c
    #ifndef gBootLoaderDebug_c
      PortA_ClockEnable(); 
      // Set Alternative 1 Mux WW
      PORT_PCR_REG(PORTA_BASE_PTR,  0u) &= ~PORT_PCR_MUX_MASK;
      PORT_PCR_REG(PORTA_BASE_PTR,  0u) |= PORT_PCR_MUX(1);
      // Set pin as output WW
      GPIOA_PDDR |= 1 << 0u;
    #endif
      
    PortB_ClockEnable();
    // Set Alternative 1 Mux CW
    PORT_PCR_REG(PORTB_BASE_PTR,  3u) &= ~PORT_PCR_MUX_MASK;
    PORT_PCR_REG(PORTB_BASE_PTR,  3u) |= PORT_PCR_MUX(1);
    // Set pin as output
    GPIOB_PDDR |= 1 << 3u;    
    // Set Alternative 1 Mux R
    PORT_PCR_REG(PORTB_BASE_PTR, 18u) &= ~PORT_PCR_MUX_MASK;
    PORT_PCR_REG(PORTB_BASE_PTR, 18u) |= PORT_PCR_MUX(1);
    // Set pin as output
    GPIOB_PDDR |= 1 << 18u; 
    
    PortC_ClockEnable();
    // Set Alternative 1 Mux  G
    PORT_PCR_REG(PORTC_BASE_PTR,  0u) &= ~PORT_PCR_MUX_MASK;
    PORT_PCR_REG(PORTC_BASE_PTR,  0u) |= PORT_PCR_MUX(1);
    // Set pin as output
    GPIOC_PDDR |= 1 << 0u;     
    // Set Alternative 1 Mux  B
    PORT_PCR_REG(PORTC_BASE_PTR,  1u) &= ~PORT_PCR_MUX_MASK;
    PORT_PCR_REG(PORTC_BASE_PTR,  1u) |= PORT_PCR_MUX(1);
    // Set pin as output
    GPIOC_PDDR |= 1 << 1u;     

    /* Set led pins to low */
    WW_lo();
    CW_lo();
    R_lo();
    G_lo();
    B_lo();
    
 #endif  
  
    
    gpBootInfo = (bootInfo_t*)gBootImageFlagsAddress_c;
    
    /* if new image available and bootProcessCompleted or not */
    if ( (gpBootInfo->newBootImageAvailable[0] == gBootValueForTRUE_c) )
    { 
        /* Write the new image */
        Boot_LoadImage();
    }
    else
    {
        /* Set the start address of the interrupt vector*/
        SCB_VTOR = gUserFlashStart_d;
        JumpToApplication(gUserFlashStart_d);       
    }

    return 0;
}

/*! *********************************************************************************
* \brief   Default ISR handler
*
********************************************************************************** */
void defaultISR(void)
{
    /* ISR code */
    HandleBootError();
}
//-----------------------------------------------------------------------------
