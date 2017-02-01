/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MWS.c
* This is the source file for the Mobile Wireless Standard Interface.
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
#include "MWS.h"
#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"
#include "controller_interface.h"

#if gMWS_Enabled_d
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private macros definitions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */    
mws_t maMWSTable[gMWS_ProtocolCount_d] = {
  {
  .priority = gMWS_BLE_Priority_d,
  .denyCount = 0,
  .denyThreshold = gMWS_BLE_DenyThreshold_d,  
  .abortCount = 0,
  .abortThreshold = gMWS_BLE_AbortThreshold_d,
  .callback = NULL   
  },
  {
  .priority = gMWS_ZIGBEE_Priority_d,
  .denyCount = 0,
  .denyThreshold = gMWS_ZIGBEE_DenyThreshold_d,  
  .abortCount = 0,
  .abortThreshold = gMWS_ZIGBEE_AbortThreshold_d,  
  .callback = NULL,
  }
};

static mwsProtocols_t gMWS_ActiveProtocol = gMWS_None_c;
/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! BLE Section ****************************************************************** */

/*---------------------------------------------------------------------------
* NAME: BLE_RequestAccess
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t BLE_RequestAccess(bool_t requestAccess)
{  
  OSA_EnterCritical(kCriticalDisableInt);
  
  if (requestAccess == true)
  {
    /* No protocol active, gain access */
    if (gMWS_ActiveProtocol == gMWS_None_c)
    {
      gMWS_ActiveProtocol = gMWS_BLE_c;
      maMWSTable[gMWS_ActiveProtocol].denyCount = 0;
    }
    else
    {
      /* Check if current active protocol have higher priority.
       * If not, run callback to abort and gain access */
      if (maMWSTable[gMWS_BLE_c].priority < 
            maMWSTable[gMWS_ActiveProtocol].priority)
      {
        if (maMWSTable[gMWS_ActiveProtocol].callback != NULL)
        {
            maMWSTable[gMWS_ActiveProtocol].callback(gMWS_Abort_c);
        }
        maMWSTable[gMWS_ActiveProtocol].abortCount++;  
        
        if (maMWSTable[gMWS_ActiveProtocol].abortCount++ >=
                  maMWSTable[gMWS_ActiveProtocol].abortThreshold)
        {
          maMWSTable[gMWS_ActiveProtocol].callback(gMWS_AbortThresholdReached_c);
        }        
        
        gMWS_ActiveProtocol = gMWS_BLE_c;
      }
      else
      {
        if (maMWSTable[gMWS_BLE_c].denyCount++ >=
                  maMWSTable[gMWS_BLE_c].denyThreshold)
        {
          maMWSTable[gMWS_BLE_c].callback(gMWS_DenyThresholdReached_c);
          maMWSTable[gMWS_BLE_c].denyCount = 0;
        }
      }
    }
  }
  else
  {
    /* If the current active protocol is BLE, clear access */
    if (gMWS_ActiveProtocol == gMWS_BLE_c)
    {
      maMWSTable[gMWS_ActiveProtocol].denyCount = 0;
      gMWS_ActiveProtocol = gMWS_None_c;
    }
  }
  
  OSA_ExitCritical(kCriticalDisableInt);
  
  return gMWS_ErrorNone_c;
}

/*---------------------------------------------------------------------------
* NAME: BLE_IsInTx
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t BLE_IsInTx(void)
{
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: BLE_RequestScan
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t BLE_RequestScan(void)
{
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: BLE_RequestAdvertise
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t BLE_RequestAdvertise(void)
{
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: BLE_SetPriority
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t BLE_SetPriority(uint8_t priority)
{
  maMWSTable[gMWS_BLE_c].priority = priority;
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: BLE_GetPriority
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
uint8_t BLE_GetPriority(void)
{  
  return maMWSTable[gMWS_BLE_c].priority;
}

/*---------------------------------------------------------------------------
* NAME: BLE_DenySetThreshold
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t BLE_DenySetThreshold(uint8_t threshold)
{
  maMWSTable[gMWS_BLE_c].denyThreshold = threshold;
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: BLE_AbortSetThreshold
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t BLE_AbortSetThreshold(uint8_t threshold)
{
  maMWSTable[gMWS_BLE_c].abortThreshold  = threshold;
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: BLE_RegisterCallback
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t BLE_RegisterCallback(pfMwsCallBack_t callback)
{
  maMWSTable[gMWS_BLE_c].callback = callback;
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: BLE_InactivityDuration
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
uint32_t BLE_InactivityDuration(void)
{
    uint32_t inactivityDuration = 0xFFFFFFFF;
        
    if(SIM_SCGC5 & SIM_SCGC5_BTLL_MASK)
    {
        OSA_EnterCritical(kCriticalDisableInt);

        inactivityDuration = Controller_GetInactivityDuration();   
        
        OSA_ExitCritical(kCriticalDisableInt);
    }

    return inactivityDuration;
}

/*! ZIGBEE Section ****************************************************************** */

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_RequestAccess
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t ZIGBEE_RequestAccess(bool_t requestAccess)
{
  OSA_EnterCritical(kCriticalDisableInt);
  
  if (requestAccess == true)
  {
    /* No protocol active, gain access */
    if (gMWS_ActiveProtocol == gMWS_None_c)
    {
      gMWS_ActiveProtocol = gMWS_ZIGBEE_c;
      maMWSTable[gMWS_ActiveProtocol].denyCount = 0;
    }
    else
    {
      /* Check if current active protocol have higher priority.
      * If not, run callback to abort and gain access */
      if (maMWSTable[gMWS_ZIGBEE_c].priority < 
          maMWSTable[gMWS_ActiveProtocol].priority)
      {
        if (maMWSTable[gMWS_ActiveProtocol].callback != NULL)
        {
          maMWSTable[gMWS_ActiveProtocol].callback(gMWS_Abort_c);
        }
        
        if (maMWSTable[gMWS_ActiveProtocol].abortCount++ >=
                  maMWSTable[gMWS_ActiveProtocol].abortThreshold)
        {
          if (maMWSTable[gMWS_ActiveProtocol].callback != NULL)
          {
            maMWSTable[gMWS_ActiveProtocol].callback(gMWS_AbortThresholdReached_c);
          }
        }                       
        
        gMWS_ActiveProtocol = gMWS_ZIGBEE_c;
      }
      else
      {               
        if (maMWSTable[gMWS_ZIGBEE_c].denyCount++ >=
                  maMWSTable[gMWS_ZIGBEE_c].denyThreshold)
        {
          maMWSTable[gMWS_ZIGBEE_c].callback(gMWS_DenyThresholdReached_c);
          maMWSTable[gMWS_ZIGBEE_c].denyCount = 0;
        }        
      }
    }
  }
  else
  {
    /* If the current active protocol is ZIGBEE, clear access */
    if (gMWS_ActiveProtocol == gMWS_ZIGBEE_c)
    {
      maMWSTable[gMWS_ActiveProtocol].denyCount = 0;
      gMWS_ActiveProtocol = gMWS_None_c;      
    }
  }
  
  OSA_ExitCritical(kCriticalDisableInt);   
  
  return gMWS_ErrorNone_c;     
}

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_IsInTx
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t ZIGBEE_IsInTx(void)
{
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_RequestScan
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t ZIGBEE_RequestScan(void)
{
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_IsInIdle
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t ZIGBEE_IsInIdle(void)
{
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_SetPriority
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t ZIGBEE_SetPriority(uint8_t priority)
{
  maMWSTable[gMWS_ZIGBEE_c].priority = priority;
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_GetPriority
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
uint8_t ZIGBEE_GetPriority(void)
{
  return maMWSTable[gMWS_ZIGBEE_c].priority;
}

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_DenySetThreshold
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t ZIGBEE_DenySetThreshold(uint8_t threshold)
{
  maMWSTable[gMWS_ZIGBEE_c].denyThreshold = threshold;  
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_AbortSetThreshold
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t ZIGBEE_AbortSetThreshold(uint8_t threshold)
{
  maMWSTable[gMWS_ZIGBEE_c].abortThreshold = threshold;
  return gMWS_ErrorNone_c;  
}
 
/*---------------------------------------------------------------------------
* NAME: ZIGBEE_DenyRegisterCallback
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
mwsStatus_t ZIGBEE_RegisterCallback(pfMwsCallBack_t callback)
{
  maMWSTable[gMWS_ZIGBEE_c].callback = callback;
  return gMWS_ErrorNone_c;  
}

/*---------------------------------------------------------------------------
* NAME: ZIGBEE_InactivityDuration
* DESCRIPTION: 
* PARAMETERS:  IN:
* RETURN: 
* NOTES: none
*---------------------------------------------------------------------------*/
uint32_t ZIGBEE_InactivityDuration(void)
{
  return 0;
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************** */
#else /* !gMWS_Enabled_d */

/* Some stubs for linking purposes */

mwsStatus_t ZIGBEE_RegisterCallback(pfMwsCallBack_t callback)
{
  return gMWS_ErrorNone_c;  
}

#endif /* gMWS_Enabled_d */