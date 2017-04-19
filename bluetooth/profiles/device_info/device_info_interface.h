/*! *********************************************************************************
* \defgroup Device Information Service
* @{
********************************************************************************** */
/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file device_info_interface.h
* This file is the interface file for the Device Information service
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

#ifndef _DEVICE_INFO_INTERFACE_H_
#define _DEVICE_INFO_INTERFACE_H_

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public constants & macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

typedef struct utf8s_tag
{
    uint8_t    stringLength;
    char        *pUtf8s;
}utf8s_t;

typedef struct systemId_tag
{
    uint8_t    oui[3];
    uint8_t    manufacturerId[5];
}systemId_t;

typedef struct regCertDataList_tag
{
    uint8_t    length;
    void        *pList;
}regCertDataList_t;

typedef struct pnpId_tag
{
    uint8_t     vendorIdSource;
    uint16_t    vendorId;
    uint16_t    productId;
    uint16_t    productVersion;
}pnpId_t;

/* Lamp Serial Number */
typedef union serial_number_tag {
        uint8_t ch[4];       
	struct {
                uint16_t UnitId;       /*!<  LSB unit produce in day of year  */
		uint16_t DOY    : 12; /*!<   LSB   day of year since 01 01 2017   */	
                uint16_t Prod   :  4; /*!<   MSB   Production  parameters         */	
	} param;
} serial_number_t;

/*! Device Info - Configuration */
typedef struct disConfig_tag
{
    uint16_t            serviceHandle;

} disConfig_t;



/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!**********************************************************************************
* \brief        Starts Device Info service functionality
*
* \param[in]    pServiceConfig  Pointer to structure that contains server 
*                               configuration information.
*
* \return       gBleSuccess_c or error.
************************************************************************************/
bleResult_t Dis_Start(disConfig_t *pServiceConfig);

/*!**********************************************************************************
* \brief        Stops Device Info service functionality
*
* \param[in]    pServiceConfig  Pointer to structure that contains server 
*                               configuration information.
*
* \return       gBleSuccess_c or error.
************************************************************************************/
bleResult_t Dis_Stop(disConfig_t *pServiceConfig);

#ifdef __cplusplus
}
#endif 

#endif /* _DEVICE_INFO_INTERFACE_H_ */

/*! *********************************************************************************
* @}
********************************************************************************** */
