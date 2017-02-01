/*! *********************************************************************************
 * \addtogroup BLE
 * @{
 ********************************************************************************** */
/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file ble_host_tasks.h
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

#ifndef _BLE_HOST_TASKS_H_
#define _BLE_HOST_TASKS_H_

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "Messaging.h"
#include "fsl_osa_ext.h"
#include "ble_general.h"

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
/*! Message queue for the Host Task - to be defined by the application */
extern msgQueue_t   gHost_TaskQueue;
/*! Message queue for the L2CA Task - to be defined by the application */
extern msgQueue_t   gL2ca_TaskQueue;

/*! Event for the Host Task Queue - to be defined by the application */
extern osaEventId_t gHost_TaskEvent;
/*! Event for the L2CA Task Queue - to be defined by the application */
extern osaEventId_t gL2ca_TaskEvent;

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*! *********************************************************************************
* \brief  Contains the Host Task logic.
*
* \remarks This function must be called exclusively by the Host Task code
* from the application.
*
********************************************************************************** */
void Host_TaskHandler(void * args);

/*! *********************************************************************************
* \brief  Contains the L2CA Task logic.
*
* \remarks This function is designed to be called exclusively by the L2ca Task code
* from the application.
*
********************************************************************************** */
void L2ca_TaskHandler(void * args);

#ifdef __cplusplus
}
#endif 

#endif /* _BLE_HOST_TASKS_H_ */

/*! *********************************************************************************
* @}
********************************************************************************** */
