/*! *********************************************************************************
 * \addtogroup Lampster Custom Profile - Config Attribute 
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2017, FiranL
 * All rights reserved.
 *
 * \file lamp_att_cfg.c
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
#include "FunctionLib.h"
#include "ble_general.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h" // Include this file for the 128 bit characteristic UUIDs. Do not access the handles directlty!
#include "gatt_server_interface.h"
#include "gap_interface.h"

#include "board.h"
#include "TimersManager.h"
#include "lamp_interface.h"
#include "tsi_sensor.h"
#include "lamp_att_cfg.h"

#include "temperature_sensor.h"

/************************************************************************************
* Private constants & macros
************************************************************************************/



/************************************************************************************
* Extern memory
************************************************************************************/

/* Touch Sensing sensor timer  */
extern tmrTimerID_t tmrTsiId;

/* lamp cfg params */
extern lamp_config_t lamp_cfg;

/* lamp TSI cfg params */
extern tsi_touch_t tsi;

/* core temperature at witch the sistem should disable all outputs, exponent -2 */
extern int16_t gCoreTemperatureFaliure;

/************************************************************************************
* Private type definitions
************************************************************************************/

/* ***********************************************************************************
* Private memory declarations
*********************************************************************************** */



/************************************************************************************
* Private functions prototypes
************************************************************************************/
static bleResult_t Las_RecordValueToBeRead (uint16_t serviceHandle, uint16_t valLenght, uint8_t* pValue);
static bleResult_t Las_GetParamToBeRead(uint16_t serviceHandle, uint8_t paramID);

/************************************************************************************
* Extern functions
************************************************************************************/


/************************************************************************************
* Public functions
************************************************************************************/



bleResult_t Las_SetConfig (uint16_t serviceHandle, uint8_t cfg8, uint8_t  val8, uint16_t val16)
{
    bleResult_t result = gBleSuccess_c;
    
    val16 = val16<<8; 
    val16 += (uint16_t) val8;

    switch(cfg8)
    {
      /* TID tabel id pointer for witch value to be read */ 
      case gCFG_TID_c: 
        {
          return Las_GetParamToBeRead(serviceHandle, val8); 
        } break;
        
      /* TSI */ 
      /* uint16_t, TSI idle value sensor not pressed  */
      case gCFG_TSI_low_c: 
        {   
            /* check if value in limits */
            if( (val16 >= 0) && (val16 < 0xFFFF) )
            {
              tsi.low = val16;
            } else { return gBleOverflow_c; }
          
        } break;   
        
      /* uint8_t, TSI treshold add value, default 10  */
      case gCFG_TSI_sensitivity_c: 
        {           
            /* check if value in limits */
            if( (val8 > 0) && (val8 < 0xFF) )
            {
              /* substract old sensitivity and add new one*/
              tsi.low = (uint16_t) (tsi.low  + ( val8 - tsi.sensitivity )  );
              /* update sensitivity */
              tsi.sensitivity = val8;
            } else { return gBleOverflow_c; } 
        } break;  
        
      /* uint16_t, TSI highest value over time, max value */
      case gCFG_TSI_max_c: 
        {       
            /* check if value in limits */
            if( (val16 >= 0) && (val16 < 0xFFFF) )
            {
              tsi.max = val16;
            } else { return gBleOverflow_c; }
        } break;    
        
      /* uint16_t, TSI lowest value over time, min value */
      case gCFG_TSI_min_c: 
        {
            /* check if value in limits */
            if( (val16 >= 0) && (val16 < 0xFFFF) )
            {
              tsi.min = val16;
            } else { return gBleOverflow_c; }
        } break;        
        
      /* uint8_t, TSI update time in mili seconds, default 15 ms  */ 
      case  gCFG_TSI_tmr_c: 
        {  
            /* check if value in limits */
            if( (val8 > 4) && (val8 < 0xFF) )
            {
              tsi.tmr = val8;
              /* ReStart TSI timer for capacitive touch BTN with new interval */     
              TMR_StartTimer(tmrTsiId, gTmrIntervalTimer_c, tsi.tmr, TsiTimerCallback, NULL); 
            } else { return gBleOverflow_c; }
        } break;         
 
      /* uint8_t, Intermediate state hit cnt  */ 
      case  gCFG_TSI_InitHitCnt_c: 
        {
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 0xFF) )
            {
              tsi.InitHitCnt = val8;
            } else { return gBleOverflow_c; }
        } break;  
                 
      /* uint8_t, Intermediate state idle cnt */ 
      case  gCFG_TSI_InitIdleCnt_c: 
        {
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 0xFF) )
            {
              tsi.InitIdleCnt = val8;
            } else { return gBleOverflow_c; } 
        } break;     
                   
      /* uint8_t, Long Press Series hit cnt */
      case  gCFG_TSI_InitHitHitCnt_c: 
        {
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 0xFF) )
            {
              tsi.InitHitHitCnt = val8;
            } else { return gBleOverflow_c; }
        } break;     
                        
      /* uint8_t, Intermediate not used state idle cnt  */  
      case  gCFG_TSI_InitHitIdleCnt_c: 
        {  
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 0xFF) )
            {
              tsi.InitHitIdleCnt = val8;
            } else { return gBleOverflow_c; }
        } break;      
             
      /* uint8_t, Intermediate state hit cnt */ 
      case  gCFG_TSI_InitIdleHitCnt_c: 
        {
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 0xFF) )
            {
              tsi.InitIdleHitCnt = val8;
            } else { return gBleOverflow_c; }  
        } break;      
             
      /* uint8_t, Idle state idle cnt  */ 
      case  gCFG_TSI_InitIdleIdleCnt_c: 
        {
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 0xFF) )
            {
              tsi.InitIdleIdleCnt = val8;
            } else { return gBleOverflow_c; } 
        } break; 
                  
      /* uint8_t, First Long Press hit cnt */ 
      case  gCFG_TSI_InitIdleHitHitCnt_c: 
        {
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 0xFF) )
            {
              tsi.InitIdleHitHitCnt = val8;
            } else { return gBleOverflow_c; }    
        } break;      
              
      /* uint8_t, Short Press  idle cnt */ 
      case  gCFG_TSI_InitIdleHitIdleCnt_c: 
        {
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 0xFF) )
            {
              tsi.InitIdleHitIdleCnt = val8; 
            } else { return gBleOverflow_c; } 
        } break; 
        
      /* uint8_t, Start a low recalibration - TSI not pressed */ 
      case  gCFG_TSI_Recalibrate_low_c: 
        {
            /* check if value is 0x01 */
            if( val8 == 0x01 )
            {
              TsiCalibrate(); 
            } else { return gBleOverflow_c; }
        } break;        
        
      /* uint16_t, On / off time period of a blink in mili seconds, default 300 ms  */
      case gCFG_blinkTimeMs_c: 
        {
            /* check if value in limits */
            if( (val16 > 6) && (val16 < 5000) )
            {
              lamp_cfg.blinkTimeMs = val16;
            } else { return gBleOverflow_c; }
        } break;   
        
      /* uint8_t, How many blinks are performed, default 5  */
      case  gCFG_blinkCnt_c: 
        {
            /* check if value in limits */
            if( (val8 > 1) && (val8 < 15) )
            {
              lamp_cfg.blinkCnt = val8;
            } else { return gBleOverflow_c; }  
        } break;    
                           
      /* uint16_t, Fade refresh time, light increment, default 17 ms */
      case  gCFG_fadeTimeMs_c: 
        {
            /* check if value in limits */
            if( (val16 > 4) && (val16 < 200) )
            {
              lamp_cfg.fadeTimeMs = val16;
            } else { return gBleOverflow_c; }
        } break;      
                  
      /* uint16_t, Fade refresh time, light increment for temperature shutdown, default 5 ms */
      case  gCFG_fadeTimeCritMs_c: 
        {
            /* check if value in limits */
            if( (val16 > 2) && (val16 < 15) )
            {
              lamp_cfg.fadeTimeCritMs = val16;
            } else { return gBleOverflow_c; }  
        } break;      

      /* uint16_t, Fade on timer refresh time, light increment, default 30 ms */
      case  gCFG_fadeTimerOnTimeMs_c: 
        {
            /* check if value in limits */
            if( (val16 > 5) && (val16 < 10000) )
            {
              lamp_cfg.fadeTimerOnTimeMs = val16;
            } else { return gBleOverflow_c; }  
        } break;  

      /* uint16_t, Fade off timer refresh time, light decrement, default 20 ms */
      case  gCFG_fadeTimerOffTimeMs_c: 
        {
            /* check if value in limits */
            if( (val16 > 5) && (val16 < 10000) )
            {
              lamp_cfg.fadeTimerOffTimeMs = val16;
            } else { return gBleOverflow_c; }  
        } break;        
        
      /* uint16_t, core temperature at witch the sistem should disable all outputs, exponent -2  */
      case gCFG_ChipFailTemp_c: 
        {

            /* check if value in limits */
            if( (val16 >= gCoreTemperatureFaliureLL_d) && (val16 <= gCoreTemperatureFaliureUL_d) )
            {
              gCoreTemperatureFaliure = val16;
            } else { return gBleOverflow_c; }          
        } break;           
                                      
          
      default:  
        {
            return gBleInvalidParameter_c;
        }
    }

    
    return result;
}



/************************************************************************************
* Private functions
************************************************************************************/

static bleResult_t Las_GetParamToBeRead(uint16_t serviceHandle, uint8_t paramID)
{
    switch(paramID)
    {
     case gCFG_TID_c:                    
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &paramID);                   } break; 
     
     case gCFG_TSI_low_c:                
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t),
                                         (uint8_t*) &tsi.low);                } break; 
     case gCFG_TSI_sensitivity_c:       
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.sensitivity);        } break;
     case gCFG_TSI_min_c:                
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t),
                                         (uint8_t*) &tsi.min);                } break; 
     case gCFG_TSI_max_c:                
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t)
                                         , (uint8_t*) &tsi.max);                } break;                           
     case gCFG_TSI_tmr_c:                
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.tmr);                } break; 
     case gCFG_TSI_InitHitCnt_c:         
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.InitHitCnt);         } break;
     case gCFG_TSI_InitIdleCnt_c:        
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.InitIdleCnt);        } break;
     case gCFG_TSI_InitHitHitCnt_c:      
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.InitHitHitCnt);      } break;
     case gCFG_TSI_InitHitIdleCnt_c:     
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.InitHitIdleCnt);     } break;
     case gCFG_TSI_InitIdleHitCnt_c:     
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.InitIdleHitCnt);     } break;
     case gCFG_TSI_InitIdleIdleCnt_c:    
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.InitIdleIdleCnt);    } break;
     case gCFG_TSI_InitIdleHitHitCnt_c:  
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.InitIdleHitHitCnt);  } break;
     case gCFG_TSI_InitIdleHitIdleCnt_c: 
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &tsi.InitIdleHitIdleCnt); } break;
     case gCFG_TSI_Recalibrate_low_c:    
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &paramID);                } break;                 
     case gCFG_blinkTimeMs_c:            
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t),
                                         (uint8_t*) &lamp_cfg.blinkTimeMs);    } break;
     case gCFG_blinkCnt_c:               
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint8_t),
                                         (uint8_t*) &lamp_cfg.blinkCnt);       } break;
     case gCFG_fadeTimeMs_c:             
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t),
                                         (uint8_t*) &lamp_cfg.fadeTimeMs);     } break;
     case gCFG_fadeTimeCritMs_c:         
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t),
                                         (uint8_t*) &lamp_cfg.fadeTimeCritMs); } break;
     case gCFG_fadeTimerOnTimeMs_c:         
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t),
                                         (uint8_t*) &lamp_cfg.fadeTimerOnTimeMs); } break;
     case gCFG_fadeTimerOffTimeMs_c:         
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t),
                                         (uint8_t*) &lamp_cfg.fadeTimerOffTimeMs); } break;         
     case gCFG_ChipFailTemp_c:           
       { return Las_RecordValueToBeRead (serviceHandle, sizeof(uint16_t),
                                         (uint8_t*) &gCoreTemperatureFaliure); } break;

     default:                            
       { return gBleInvalidParameter_c; }                 
    }
}

static bleResult_t Las_RecordValueToBeRead (uint16_t serviceHandle, uint16_t valLenght, uint8_t* pValue)
{
    uint16_t  handle;
    bleResult_t result;
    bleUuid_t* pUuid = (bleUuid_t*)&uuid_char_lamp_config;

    

      /* Get handle of Temperature characteristic */
      result = GattDb_FindCharValueHandleInService(serviceHandle,
          gBleUuidType128_c, pUuid, &handle);

      if (result != gBleSuccess_c)
          return result;
      
      /* Update characteristic value */
      result = GattDb_WriteAttribute(handle, valLenght, pValue);

      if (result != gBleSuccess_c)
          return result;

    

    return gBleSuccess_c;
}




/*! *********************************************************************************
 * @}
 ********************************************************************************** */
