/*! *********************************************************************************
 * \addtogroup IPSP
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * \file ips_profile.c
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
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "ble_general.h"
#include "l2ca_cb_interface.h"
#include "l2ca_types.h"
#include "ipsp_interface.h"

/************************************************************************************
*************************************************************************************
* Private constants & macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
bleResult_t Ipsp_Connect(ipspConfig_t *pConfig, deviceId_t  peerDeviceId)
{
    return L2ca_ConnectLePsm(gL2ca_Le_Psm_Ipsp_c,
                            peerDeviceId,
                            pConfig->initialCredits);
}

bleResult_t Ipsp_SendData
(
     ipspConfig_t *pConfig,
     deviceId_t  peerDeviceId,
     uint8_t*    pPacket,
     uint16_t    packetLength
)
{
    return L2ca_SendLeCbData(gL2ca_Le_Psm_Ipsp_c,
                      peerDeviceId,
                      pPacket,
                      packetLength);
}

void Ipsp_ControlCallback
(
    ipspConfig_t *pConfig,
    l2capControlMessageType_t  messageType, 
    void* pMessage
)
{
    switch (messageType)
    {
        case gL2ca_LePsmConnectRequest_c:
        {
            l2caLeCbConnectionRequest_t *pConnReq = (l2caLeCbConnectionRequest_t *)pMessage;

            if ((pConfig->role & gIpsp_Router_c) &&
                pConfig->peerDeviceId == pConnReq->deviceId)
            {
                return;
            }
            else
            {
                /* Send Connection Response */
                Ipsp_Connect(pConfig, pConnReq->deviceId);
            }
            break;
        }
        case gL2ca_LePsmConnectionComplete_c:
        {
            l2caLeCbConnectionComplete_t *pConnComplete = (l2caLeCbConnectionComplete_t *)pMessage;

            if (pConnComplete->result == gSuccessful_c)
            {
                pConfig->peerDeviceId = pConnComplete->deviceId;
            }
            break;
        }
        case gL2ca_LePsmDisconnectNotification_c:
        {
            l2caLeCbDisconnection_t *pDisconnect = (l2caLeCbDisconnection_t *)pMessage;

            if (pDisconnect->deviceId == pConfig->peerDeviceId)
            {
                pConfig->peerDeviceId = gInvalidDeviceId_c;
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
