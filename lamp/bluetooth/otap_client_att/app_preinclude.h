/*! *********************************************************************************
 * \defgroup app
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * \file app_preinclude.h
 * This file is the app configuration file which is pre included.
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

#ifndef _APP_PREINCLUDE_H_
#define _APP_PREINCLUDE_H_

/*! *********************************************************************************
 * 	Framework Configuration
 ********************************************************************************** */
   
  /*! *********************************************************************************
   * 	Common
   ********************************************************************************** */  
  /* FwkInit.c */
  /* Defines a smaller FWK configuration */
  /* Range  default not defined */
  #define FWK_SMALL_RAM_CONFIG   
   
  /*! *********************************************************************************
   * 	DCDC
   ********************************************************************************** */   
   
  /*! *********************************************************************************
   * 	Flash
   ********************************************************************************** */
    /* Eeprom.h */
      /* Specifies the type of EEPROM available on the target board */
      /* Range 0,1,2,3,4 default gEepromDevice_None_c */
      #define gEepromType_d           gEepromDevice_AT45DB021E_c
   
      #define gUseNVMLink_d 1
      #define mAppUseNvm_d  1

  /*! *********************************************************************************
   * 	FSCI
   ********************************************************************************** */   
   
   /*! *********************************************************************************
   * 	FunctionLib
   ********************************************************************************** */  
   /* FunctionLib.h */
     /* Range   default 0 */
     #define gUseToolchainMemFunc_d 0

  /*! *********************************************************************************
   * 	GPIOIrq
   ********************************************************************************** */  
   
  /*! *********************************************************************************
   * 	Keyboard - Drivers Configuration
   ********************************************************************************** */

    /* GPIO KeyBoard support */
    #define gKeyBoardSupported_d    0
    /* Defines the number of available keys for the keyboard module */
    #define gKBD_KeysCount_c        1

  /*! *********************************************************************************
   * 	Led - Drivers Configuration
   ********************************************************************************** */

    /* GPIO Led support */
    #define gLEDSupported_d         0
    /* Specifies the number of physical LEDs on the target board */
    #define gLEDsOnTargetBoardCnt_c  4
    /* LED output normal or inverted */
    #define mLEDInvertedOut_c       0




  /*! *********************************************************************************
   * 	SerialManager
   ********************************************************************************** */
    /* SerialManager.h */
      /* Defines Num of Serial Manager interfaces */
      /* Range   default 1 */  
      #define gSerialManagerMaxInterfaces_c   0

      /* Defines Size for Serial Manager Task bytes */
      /* Range   default 1024 */  
      #define gSerialTaskStackSize_c  500


  /*! *********************************************************************************
   * 	TimersManager
   ********************************************************************************** */               
    /* TMR_Adapter.h */
      /* TPM instance 0,1,2 used to drive timer clock */
      #define gStackTimerInstance_c           2  
      /* Range   default 0 */       
      #define gStackTimerChannel_c            0       
             
    /* TimersManager.h */  
      /* Defines Size for Timer Task, align 8, default 600 */
      #define gTmrTaskStackSize_c  384
               
      /* Defines number of timers needed by the application, default 4 */
      #define gTmrApplicationTimers_c         6

      /* Defines number of timers needed by the protocol stack, default 4 */
      /* Range   default 4 */
      #define gTmrStackTimers_c               5

      /* Set this define TRUE if the PIT frequency is an integer number of MHZ */
      /* Range TRUE, FALSE default TRUE */
      #define gTMR_PIT_FreqMultipleOfMHZ_d    0

      /* Enables / Disables the precision timers platform component  */
      /* Range TRUE, FALSE default TRUE */
      #define gTimestamp_Enabled_d             0

      /* Enable/Disable Low Power Timer  */
      /* Range TRUE, FALSE default TRUE */
      #define gTMR_EnableLowPowerTimers_d       0
         
   
   /*! *********************************************************************************
   * 	 LowPower
   ********************************************************************************** */   
    /* PWR_Interface.h */
       /* Range TRUE, FALSE default FALSE */
       #define gAllowDeviceToSleep_c          0
                 
    /* PWR_Configuration.h */
      /* Enable/Disable PowerDown functionality in PwrLib */
      /* Range TRUE, FALSE default TRUE */
      #define cPWR_UsePowerDownMode           0

      /* Enable/Disable BLE Link Layer DSM */
      /* Range TRUE, FALSE default TRUE */
      #define cPWR_BLE_LL_Enable              0

      /* Default Deep Sleep Mode */
      /* Range 1,2,3,4,5,6 default 4 */
      #define cPWR_DeepSleepMode              3

  /*! *********************************************************************************
   * 	MemManager
   ********************************************************************************** */  
    /* MemManager.h */
      /* Defines pools by block size and number of blocks. Must be aligned to 4 bytes.*/
      #define PoolsDetails_c \
               _block_size_  32  _number_of_blocks_    8 _eol_  \
               _block_size_  64  _number_of_blocks_    5 _eol_  \
               _block_size_ 128  _number_of_blocks_    3 _eol_


  /*! *********************************************************************************
   * 	OSAbstraction - RTOS Configuration
   ********************************************************************************** */
    /* fsl_osa_ext_config.h */
      /* Defines number of OS main thread stack size in bytes */
      /* Range   default 1024  */           
      #define gMainThreadStackSize_c 1024
                 
      /* Defines number of OS semaphores used */
      /* Range   default 0 */
      #define osNumberOfSemaphores    1
                 
      /* Defines number of OS events used */
      /* Range   default 1 */ 
      #define osNumberOfEvents        4



          
/*! *********************************************************************************
 * 	bluetooth - BLE Stack Configuration
 ********************************************************************************** */

  /*! *********************************************************************************
   * 	controller
   ********************************************************************************** */
  /*! *********************************************************************************
   * 	hci_transport
   ********************************************************************************** */
    /* hci_transport.h*/
      #define APP_SERIAL_INTERFACE_TYPE      (gSerialMgrNone_c)
      #define APP_SERIAL_INTERFACE_INSTANCE  (0)                 
                 
  /*! *********************************************************************************
   * 	host
   ********************************************************************************** */
    /* ble_general.h */
      /* Range  0x0006 0xFFFF  default 40 <> 50ms */ 
      #define gcConnectionIntervalMinDefault_c            (40)
      /* Range  0x0000 0xFFFF  default 160 <> 200ms */ 
      #define gcConnectionIntervalMaxDefault_c            (160)
      #define gcConnectionSlaveLatencyDefault_c           (0)
      /*! Time = N * 10 ms */
      /* Range    default 2000 <> 20sec */                  
      #define gcConnectionSupervisionTimeoutDefault_c     (2000)
      /*! Time = N * 0.625 ms */                 
      #define gcConnectionEventMinDefault_c               (0) 
      /*! Time = N * 0.625 ms */                 
      #define gcConnectionEventMaxDefault_c               (0)                 
                 
  /*! *********************************************************************************
   * 	profiles
   ********************************************************************************** */
    /*  battery_service.c */
      #define gBatteryServiceSupported_d    0
                 
  /*! *********************************************************************************
   * 	app common
   ********************************************************************************** */
    /* ble_controller_task.c */                   
              
      #define BD_ADDR             0x13,0x00,0x00,0x9F,0x04,0x00
      #define BD_ADDR_FF          0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

    /* ble_controller_task_config.h */
      /* Range   default 900 */
      #define gControllerTaskStackSize_c 900
      /* Range   default 1 */           
      #define gControllerTaskPriority_c 1  
                 
    /* ble_host_task_config.h */  
      /* Range   default 1300 */
      #define gHost_TaskStackSize_c 1300  
      /* Range   default 5 */
      #define gHost_TaskPriority_c     5
      /* Range   default 800 */
      #define gL2ca_TaskStackSize_c  800
      /* Range   default 4 */
      #define gL2ca_TaskPriority_c     4
      /* Range time <= 10msec  default  0x08 <-> 5msec */               
      #define mcAdvertisingPacketInterval_c     0x08                     
      /* Range  0x01 - 0x07  default  0x07 */      
      #define mcScanChannelMap_c                0x07 
      /* Range  0x01 - 0x07  default  0x07 */                    
      #define mcInitiatorChannelMap_c           0x07 
      #define mcOffsetToFirstInstant_c          0xFFFF  
      /* Default Tx Power on the advertising channel.  */
      /* Range  0-15  default  5  */ 
      #define mAdvertisingDefaultTxPower_c      5  
      /* Default Tx Power on the connection channel.  */
      /* Range  0-15  default  5  */                  
      #define mConnectionDefaultTxPower_c       5                 
                 
/*! *********************************************************************************
 * 	RAM Stack Size
 ********************************************************************************** */

#define FW_Stack ( gMainThreadStackSize_c + gTmrTaskStackSize_c + gSerialTaskStackSize_c)
#define BT_Stack ( gControllerTaskStackSize_c + gHost_TaskStackSize_c + gL2ca_TaskStackSize_c)
#define Stack_Size ( FW_Stack + BT_Stack)


#endif /* _APP_PREINCLUDE_H_ */

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
