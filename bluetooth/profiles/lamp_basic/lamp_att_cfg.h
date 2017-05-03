/*! *********************************************************************************
 * \addtogroup Lampster Custom Profile - Config Attribute 
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2016, FiranL.
 * All rights reserved.
 *
 * \file lamp_att_cfg.h
 *
 * This file is the interface file for the Lampster Custom Profile - Config Attribute 
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

#ifndef _LAMP_ATT_CFG_H_
#define _LAMP_ATT_CFG_H_

/* ***********************************************************************************
* Include
*********************************************************************************** */
#include "stdbool.h"
#include "board.h"

/* ***********************************************************************************
* Public constants & macros
*********************************************************************************** */



/* ***********************************************************************************
* Public type definitions
*********************************************************************************** */
enum
{
    gCFG_TID_c = 0,                     /*!< uint8_t, TID tabel id pointer for witch value to be read */ 
    gCFG_TSI_low_c,                     /*!< uint16_t, TSI idle value sensor not pressed  */
    gCFG_TSI_sensitivity_c,             /*!< uint8_t, TSI treshold add value, default 10  */
    gCFG_TSI_min_c,                     /*!< uint16_t, TSI lowest value over time, min value  */   
    gCFG_TSI_max_c,                     /*!< uint16_t, TSI highest value over time, max value  */       
    gCFG_TSI_tmr_c,                     /*!< uint8_t, TSI update time in mili seconds, default 15 ms  */   
    gCFG_TSI_InitHitCnt_c,              /*!< uint8_t, Intermediate state hit cnt  */
    gCFG_TSI_InitIdleCnt_c,             /*!< uint8_t, Intermediate state idle cnt */
    gCFG_TSI_InitHitHitCnt_c,           /*!< uint8_t, Long Press Series hit cnt */
    gCFG_TSI_InitHitIdleCnt_c,          /*!< uint8_t, Intermediate not used state idle cnt  */   
    gCFG_TSI_InitIdleHitCnt_c,          /*!< uint8_t, Intermediate state hit cnt */
    gCFG_TSI_InitIdleIdleCnt_c,         /*!< uint8_t, Idle state idle cnt  */
    gCFG_TSI_InitIdleHitHitCnt_c,       /*!< uint8_t, First Long Press hit cnt */
    gCFG_TSI_InitIdleHitIdleCnt_c,      /*!< uint8_t, Short Press  idle cnt */    
    gCFG_TSI_Recalibrate_low_c,         /*!< uint8_t, Start a low recalibration - TSI not pressed */      
    gCFG_blinkTimeMs_c,                 /*!< uint16_t, On / off time period of a blink in mili seconds, default 300 ms  */
    gCFG_blinkCnt_c,                    /*!< uint8_t, How many blinks are performed, default 5  */
    gCFG_fadeTimeMs_c,                  /*!< uint8_t, Fade refresh time, light increment, default 17 ms */    
    gCFG_fadeTimeCritMs_c,              /*!< uint8_t, Fade refresh time, light increment for temperature shutdown, default 5 ms */
    gCFG_SaveToFlash_c,                 /*!< uint8_t, save all cfg and TSI to flash */    
    gCFG_last                           /* max value in config table list */
};


/* ***********************************************************************************
* Public memory declarations
*********************************************************************************** */

/* ***********************************************************************************
* Public prototypes
*********************************************************************************** */

#ifdef __cplusplus
extern "C" {
#endif

/*!**********************************************************************************
* \brief        Lamp Config set values for config table
*
* \param[in]    serviceHandle   
* \param[in]          pConfig 
*                               
*
* \return       gBleSuccess_c or error.
************************************************************************************/

bleResult_t Las_SetConfig (uint16_t serviceHandle, const uint8_t* pConfig);



#ifdef __cplusplus
}
#endif

#endif /* _LAMP_ATT_CFG_H_ */

/*! **********************************************************************************
 * @}
 ************************************************************************************/
