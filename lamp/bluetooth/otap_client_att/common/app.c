/*! *********************************************************************************
* \addtogroup BLE OTAP Client ATT
* @{
********************************************************************************** */
/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
* \file app.c
* This file is the source file for the BLE OTAP Client ATT application
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

/* ***********************************************************************************
* Include
*********************************************************************************** */
/* Framework / Drivers */
#include "RNG_interface.h"
#include "Keyboard.h"
#include "Led.h"
#include "TimersManager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"
#include "panic.h"
#if cPWR_UsePowerDownMode
  #include "PWR_Interface.h"
#endif
#include "OtaSupport.h"

/* BLE Host Stack */
#include "gatt_interface.h"
#include "gatt_server_interface.h"
#include "gatt_client_interface.h"
#include "gatt_database.h"
#include "gap_interface.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h"

/* Profile / Services */
#include "device_info_interface.h"
#include "otap_interface.h"
#include "lamp_interface.h"

#include "battery_interface.h"

/* core temperature measurement, voltage reference measurement */
#include "temperature_sensor.h"

/* TPM PWM */
#include "tpm_pwm_led_ctrl.h"

/* TSI Capacitive touch sensor */
#include "tsi_sensor.h"

#include "board.h"
#include "ApplMain.h"
#include "app.h"

/* ***********************************************************************************
* Extern variables
*********************************************************************************** */

/* core temperature T, signed exponent -2 */
/* core voltage reference V, signed exponent -3 */
extern chip_TempVoltage_t g_chip_TV;

/* lamp control light data */
extern lamp_NVdata_t lamp_NVdata;

/************************************************************************************
* Extern functions
************************************************************************************/
extern void ResetMCU(void);


/************************************************************************************
* Private macros
************************************************************************************/


/************************************************************************************
* Private type definitions
************************************************************************************/
typedef struct advState_tag
{
    bool_t      advOn;
} advState_t;

typedef enum otapClientState_tag
{
    mOtapClientStateIdle_c                  = 0x00,
    mOtapClientStateDownloadingImage_c      = 0x01,
    mOtapClientStateImageDownloadComplete_c = 0x02,
} otapClientState_t;

/*! Structure containing the OTAP Client functional data. */
typedef struct otapClientAppData_tag
{
    otapClientState_t           state;
    const uint8_t               currentImgId[gOtap_ImageIdFieldSize_c];         /*!< Id of the currently running image on the OTAP Client */
    const uint8_t               currentImgVer[gOtap_ImageVersionFieldSize_c];   /*!< Version of the currently running image on the OTAP Client */
    deviceId_t                  peerOtapServer;                                 /*!< Device id of the OTAP Server a new image is being downloaded from. */
    uint8_t                     imgId[gOtap_ImageIdFieldSize_c];                /*!< Id of the image being downloaded from the OTAP Server */
    uint8_t                     imgVer[gOtap_ImageVersionFieldSize_c];          /*!< Version of the image being downloaded from the OTAP Server */
    uint32_t                    imgSize;                                        /*!< Size of the image file being downloaded from the OTAP Server */
    uint16_t                    imgComputedCrc;                                 /*!< Computed 16 bit CRC of the image file used in this implementation. */
    uint16_t                    imgReceivedCrc;                                 /*!< Received 16 bit CRC of the image file used in this implementation. */
    uint8_t                     imgSectorBitmap[gBootData_SectorsBitmap_Size_c];    /*!< Flash sector bitmap for the recieved image for the current implementation. */
    uint32_t                    currentPos;                                     /*!< Current position of the file being downloaded. */
    uint16_t                    chunkSize;                                      /*!< Current chunk size for the image file transfer. */
    uint16_t                    chunkSeqNum;                                    /*!< Current chunk sequence number for the block being transferred. */
    uint16_t                    totalBlockChunks;                               /*!< Total number of chunks for the block being transferred. */
    uint32_t                    totalBlockSize;                                 /*!< Total size of the block which was requested. may be smaller than totalBlockChunks * chunkSize. */
    const otapTransferMethod_t  transferMethod;                                 /*!< Currently used transfer method for the OTAP Image File */
    uint16_t                    l2capChannelOrPsm;                              /*!< L2CAP Channel or PSM used for the transfer of the image file: channel 0x0004 for ATT, application specific PSM for CoC. */
    bool_t                      serverWrittenCccd;                              /*!< The OTAP Server has written the CCCD to receive commands from the OTAp Client. */
    otapCmdIdt_t                lastCmdSentToOtapServer;                        /*!< The last command sent to the OTAP Server for which an Indication is expected. */
} otapClientAppData_t;

/************************************************************************************
* Private memory declarations
************************************************************************************/

/* Host Stack Data*/
static bleDeviceAddress_t   maBleDeviceAddress;

/* Adv Parmeters */
static advState_t  mAdvState;

/* Timers */
  /* Temperature voltage timer  */
  static tmrTimerID_t tmrMeasurementTimerId;


static deviceId_t  mPeerDeviceId = gInvalidDeviceId_c;
#if gBondingSupported_d
  static bleDeviceAddress_t   maPeerDeviceAddress;
  static uint8_t mcBondedDevices = 0;
  static bleAddressType_t     mPeerDeviceAddressType;
#endif

static bool_t   mSendDataAfterEncStart = FALSE;

/* Service Data */
static lasConfig_t lasServiceConfig         = {service_lamp};
static disConfig_t disServiceConfig         = {service_device_info};
static otapClientConfig_t otapServiceConfig = {service_otap};

static basConfig_t basServiceConfig = {service_battery, 0};


static uint8_t whiteLightRamp;
enum {
  whiteLightRampUP_c = 0,
  whiteLightRampDN_c,
};

static uint16_t WriteNotifHandles[] =     {value_otap_control_point,
                                           value_otap_data,
                                           value_lamp_Control,
                                           value_lamp_White, 
                                           value_lamp_RGB,
                                           value_lamp_clock,
                                           value_lamp_on_sec,
                                           value_lamp_off_sec};

/* Application Data */

/*! OTAP Client data structure.
 *  Contains current image information and state informations
 *  regarding the image download procedure. */
static otapClientAppData_t     otapClientData = 
{
    .state = mOtapClientStateIdle_c,
    .currentImgId = {0x00, 0x00},     // Current Running Image Id - should be 0x0000
    .currentImgVer = {0x01, 0x00, 0x00,    // Build Version
                      0x41,                // Stack Version
                      0x11, 0x11, 0x11,    // Hardware Id
                      0x01                 // Manufacturer Id
                     },               // Current Image Version
    .peerOtapServer = gInvalidDeviceId_c,
    .imgId = {0x00, 0x00},
    .imgVer = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .imgSize = 0,
    .imgComputedCrc = 0,
    .imgReceivedCrc = 0,
    .imgSectorBitmap = {0x00},
    .currentPos = 0,
    .chunkSize = 0,
    .chunkSeqNum = 0,
    .totalBlockChunks = 0,
    .totalBlockSize = 0,
    .transferMethod = gOtapTransferMethodAtt_c,   // The default transfer method is ATT
    .l2capChannelOrPsm = gL2capCidAtt_c,   // The default L2CAP channel is the ATT Channel
    .lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c,
};


/* ***********************************************************************************
* Private functions prototypes
*********************************************************************************** */

static void BleApp_Config();

/* Gatt and Att callbacks */
static void BleApp_AdvertisingCallback (gapAdvertisingEvent_t* pAdvertisingEvent);
static void BleApp_ConnectionCallback  (deviceId_t peerDeviceId, gapConnectionEvent_t* pConnectionEvent);
static void BleApp_GattServerCallback  (deviceId_t deviceId, gattServerEvent_t* pServerEvent);


static void BleApp_SendAttWriteResponse (deviceId_t deviceId, uint16_t handle, bleResult_t result);
static void BleApp_CccdWritten (deviceId_t deviceId, uint16_t handle, gattCccdFlags_t cccd);
static void BleApp_AttributeWritten (deviceId_t deviceId, uint16_t handle, uint16_t length, uint8_t* pValue);
static void BleApp_AttributeWrittenWithoutResponse (deviceId_t deviceId, uint16_t handle, uint16_t length, uint8_t* pValue);
static void BleApp_HandleValueConfirmation (deviceId_t deviceId);


/* Timers callbacks */
static void MeasurementTimerCallback (void* pParam);

static void BleApp_Advertise (void);


/* OTAP Client functions */
/* Commands received from the OTAP Server */
static void OtapClient_HandleDataChunk (deviceId_t deviceId, uint16_t length, uint8_t* pData);
static void OtapClient_HandleNewImageNotification (deviceId_t deviceId, uint16_t length, uint8_t* pValue);
static void OtapClient_HandleNewImageInfoResponse (deviceId_t deviceId, uint16_t length, uint8_t* pValue);
static void OtapClient_HandleErrorNotification (deviceId_t deviceId, uint16_t length, uint8_t* pValue);
/* Confirmations of commands sent to the OTAP Server */
static void OtapClient_HandleNewImageInfoRequestConfirmation (deviceId_t deviceId);
static void OtapClient_HandleImageBlockRequestConfirmation (deviceId_t deviceId);
static void OtapClient_HandleImageTransferCompleteConfirmation (deviceId_t deviceId);
static void OtapClient_HandleErrorNotificationConfirmation (deviceId_t deviceId);
static void OtapClient_HandleStopImageTransferConfirmation (deviceId_t deviceId);
/* Connection and Disconnection events */
static void OtapClient_HandleConnectionEvent (deviceId_t deviceId);
static void OtapClient_HandleDisconnectionEvent (deviceId_t deviceId);
/* Otap Client operations */
static void OtapClient_ContinueImageDownload (deviceId_t deviceId);
static bool_t OtapClient_IsRemoteImageNewer (uint8_t* pRemoteImgId, uint8_t* pRemoteImgVer);
static otapStatus_t OtapClient_IsImageFileHeaderValid (bleOtaImageFileHeader_t* imgFileHeader);

/************************************************************************************
* Public functions
************************************************************************************/

/*! *********************************************************************************
* \brief    Initializes application specific functionality before the BLE stack init.
*
********************************************************************************** */
void BleApp_Init(void)
{
     tmrErrCode_t tmrerr = gTmrInvalidId_c;
     
    /* Initialize application support for drivers */          
      
    /* Initialize TSI sensor */
    TSI_Init();
  
    /* Init timers */    
    tmrMeasurementTimerId = TMR_AllocateTimer(); /* T+V */
      
   /* start timers */  
    /* Start 5 second measurements voltage and temperature */
    tmrerr = TMR_StartTimer(tmrMeasurementTimerId, gTmrIntervalTimer_c, TmrSeconds(5), MeasurementTimerCallback, NULL);
    


}

/*! *********************************************************************************
* \brief    Starts the BLE application.
*
********************************************************************************** */
void BleApp_Start(void)
{
    
    if (mPeerDeviceId == gInvalidDeviceId_c)
    {
        /* Device is not connected and not advertising*/
        if (!mAdvState.advOn)
        {
            BleApp_Advertise();
        }
    }
    else
    {
        /* Application start code here if the peer device is known. */
    }
}

/*! *********************************************************************************
* \brief        Handles keyboard events.
*
* \param[in]    events    Key event structure.
********************************************************************************** */
void BleApp_HandleKeys(key_event_t events)
{     
  switch (events)
      {
        
          case gKBD_EventPressPB1_c:
          {

          }  break;
           
          case gKBD_EventLongPB1_c:
          {

          }  break;

          default:
          {

          }  break;
      }
    
}


/*! *********************************************************************************
* \brief        Handles TSI events.
*
* \param[in]    pElectrodeFlags    Electrode flags.
********************************************************************************** */
void BleApp_HandleTouch(tsi_event_t* pEvent)
{
  uint8_t control;
  uint8_t event;
  int8_t warmW, coldW;
  
  event = *pEvent;
  switch (event)
      {
        
          case gTSI_EventShortPush_c:
          {
            if(lamp_NVdata.lampControl.bit.OnOff)
            {
              control = lamp_NVdata.lampControl.raw8 & 0x7F;           
              Las_SetLampControl (lasServiceConfig.serviceHandle, control, TRUE); 
            }
            else
            {
              control = lamp_NVdata.lampControl.raw8 | 0x80;
              Las_SetLampControl (lasServiceConfig.serviceHandle, control, TRUE);
            }
            
          }  break;
          
          case gTSI_EventLongPush_c:
          {
             warmW = (int8_t) lamp_NVdata.lampWhite.uint8.warmW;
             coldW = (int8_t) lamp_NVdata.lampWhite.uint8.coldW;
             
            if( whiteLightRamp == whiteLightRampUP_c )
            { 
                /* Ramp up - increment */
                if( (warmW < 100) ) { warmW++; }
                if( (coldW < 100) ) { coldW++; } 
                
                /* check for ramp change */
                if( (warmW >=100) && (coldW >= 100) )
                { 
                  whiteLightRamp = whiteLightRampDN_c; 
                }
                else
                {
                  /* mix is loked */
                  if(lamp_NVdata.lampControl.bit.mix)
                  {
                    if( (warmW >= 100) || (coldW >= 100) )
                    { 
                      whiteLightRamp = whiteLightRampDN_c; 
                    }
                  }
                }

            }
            else
            {
 
                /* Ramp dn - decrement */
                if( (warmW > 0) ) { warmW--; }
                if( (coldW > 0) ) { coldW--; } 
                
                /* check for ramp change */
                if( ( warmW <= 0 ) && ( coldW <= 0 ) )
                { 
                  whiteLightRamp = whiteLightRampUP_c; 
                }
                else
                {
                  /* mix is loked */
                  if(lamp_NVdata.lampControl.bit.mix)
                  {
                    if( (warmW <= 0) || (coldW <= 0) )
                    { 
                      whiteLightRamp = whiteLightRampUP_c; 
                    }
                  }
                }
                
            }
            
            Las_SetLampWhite (lasServiceConfig.serviceHandle, warmW, coldW, TRUE);
            
            //if lamp off turn it on
            if( !((lamp_NVdata.lampControl.raw8 & 0xE0) == 0xC0) )
            {
              control = (lamp_NVdata.lampControl.raw8 & 0xDF) | 0xC0;
              Las_SetLampControl (lasServiceConfig.serviceHandle, control, TRUE);
            }
  
            
          }  break;

          default:
          {

          }  break;
      } 
  
  /* event  treated */
  *pEvent = 0;
}

/*! *********************************************************************************
* \brief        Handles BLE generic callback.
*
* \param[in]    pGenericEvent    Pointer to gapGenericEvent_t.
********************************************************************************** */
void BleApp_GenericCallback (gapGenericEvent_t* pGenericEvent)
{
    switch (pGenericEvent->eventType)
    {
        case gInitializationComplete_c:    
        {
            BleApp_Config();
        }
        break;    
            
        case gPublicAddressRead_c:
        {
            /* Use address read from the controller */
            FLib_MemCpyReverseOrder(maBleDeviceAddress, pGenericEvent->eventData.aAddress, sizeof(bleDeviceAddress_t));
        }
        break;            
            
        case gAdvertisingDataSetupComplete_c:
        {            
            
        }
        break;  
        
        case gAdvertisingParametersSetupComplete_c:
        {
            App_StartAdvertising(BleApp_AdvertisingCallback, BleApp_ConnectionCallback);
        }
        break;         

        case gInternalError_c:
        {
            panic(0,0,0,0);
        }
        break;

        default: 
            break;
    }
}

/************************************************************************************
* Private functions
************************************************************************************/



/*! *********************************************************************************
* \brief        Configures BLE Stack after initialization. Usually used for
*               configuring advertising, scanning, white list, services, et al.
*
********************************************************************************** */
static void BleApp_Config()
{  
    tmrErrCode_t tmrerr = gTmrInvalidId_c;
    uint8_t      handleCount = sizeof(WriteNotifHandles)/sizeof(WriteNotifHandles[0]);
  
    /* Read public address from controller */
    Gap_ReadPublicDeviceAddress();

    /* Register for callbacks*/
    App_RegisterGattServerCallback (BleApp_GattServerCallback);
    GattServer_RegisterHandlesForWriteNotifications (handleCount, WriteNotifHandles);       
    
    /* Register security requirements */
#if gUseServiceSecurity_d
    Gap_RegisterDeviceSecurityRequirements (&deviceSecurityRequirements);
#endif

    /* Set local passkey */
#if gBondingSupported_d
    Gap_SetLocalPasskey(gPasskeyValue_c);
#endif

    /* Setup Advertising and scanning data */
    Gap_SetAdvertisingData(&gAppAdvertisingData, &gAppScanRspData);

    /* Populate White List if bonding is supported */
#if gBondingSupported_d
    bleDeviceAddress_t aBondedDevAdds[gcGapMaximumBondedDevices_d];
    bleResult_t result = Gap_GetBondedStaticAddresses(aBondedDevAdds, gcGapMaximumBondedDevices_d, &mcBondedDevices);

    if (gBleSuccess_c == result && mcBondedDevices > 0)
    {
        for (uint8_t i = 0; i < mcBondedDevices; i++)
        {
            Gap_AddDeviceToWhiteList(gBleAddrTypePublic_c, aBondedDevAdds[i]);
        }
    }
#endif

    mAdvState.advOn = FALSE;

    /* Start services */
    basServiceConfig.batteryLevel = 90;
    Bas_Start(&basServiceConfig);
    
    Dis_Start(&disServiceConfig);  
    OtapCS_Start(&otapServiceConfig);
    Las_Start(&lasServiceConfig);
   
    
    // start advertising
    BleApp_Start();
}

/*! *********************************************************************************
* \brief        Configures GAP Advertise parameters. Advertise will satrt after
*               the parameters are set.
*
********************************************************************************** */
static void BleApp_Advertise(void)
{
    /* Set advertising parameters*/
    Gap_SetAdvertisingParameters(&gAdvParameters);
}

static void BleApp_AdvertisingCallback (gapAdvertisingEvent_t* pAdvertisingEvent)
{
    switch (pAdvertisingEvent->eventType)
    {
        case gAdvertisingStateChanged_c:
        {
            mAdvState.advOn = !mAdvState.advOn;
            if(mAdvState.advOn)
            {

            }
        }
        break;

        case gAdvertisingCommandFailed_c:
        {
            panic(0,0,0,0);
        }
        break;

        default:
            break;
    }
}

static void BleApp_ConnectionCallback (deviceId_t peerDeviceId, gapConnectionEvent_t* pConnectionEvent)
{
    switch (pConnectionEvent->eventType)
    {
        case gConnEvtConnected_c:
        {
            bool_t isBonded = FALSE ;
            
#if gBondingSupported_d    
            /* Copy peer device address information */
            mPeerDeviceAddressType = pConnectionEvent->eventData.connectedEvent.peerAddressType;
            FLib_MemCpy(maPeerDeviceAddress, pConnectionEvent->eventData.connectedEvent.peerAddress, sizeof(bleDeviceAddress_t));
#endif  
            /* Advertising stops when connected */
            mAdvState.advOn = FALSE;
         
            
	    /* Subscribe client*/
            mPeerDeviceId = peerDeviceId;
	    Las_Subscribe(peerDeviceId);		
	    OtapCS_Subscribe(peerDeviceId);
			
            Bas_Subscribe(peerDeviceId);
                    
            mSendDataAfterEncStart = FALSE;
            if (gBleSuccess_c == Gap_CheckIfBonded(peerDeviceId, &isBonded) &&
                TRUE == isBonded) 
            {
                /* Send temperature data after encryption is started */
                mSendDataAfterEncStart = TRUE;
            }
            
	    /* UI */            
            OtapClient_HandleConnectionEvent (peerDeviceId);

         
        }
        break;
        
        case gConnEvtDisconnected_c:
        {
            /* Unsubscribe client */
            mPeerDeviceId = gInvalidDeviceId_c;
            Las_Unsubscribe();
            OtapCS_Unsubscribe();
            
            Bas_Unsubscribe();
            
            /* UI */
            
            /* Restart advertising*/
            BleApp_Start();

            OtapClient_HandleDisconnectionEvent (peerDeviceId);
        }
        break;

#if gBondingSupported_d
        case gConnEvtKeysReceived_c:
        {
            /* Copy peer device address information when IRK is used */
            if (pConnectionEvent->eventData.keysReceivedEvent.pKeys->aIrk != NULL)
            {
                mPeerDeviceAddressType = pConnectionEvent->eventData.keysReceivedEvent.pKeys->addressType;
                FLib_MemCpy(maPeerDeviceAddress, pConnectionEvent->eventData.keysReceivedEvent.pKeys->aAddress, sizeof(bleDeviceAddress_t));
            }
        }
        break;

        case gConnEvtPairingComplete_c:
        {
            if (pConnectionEvent->eventData.pairingCompleteEvent.pairingSuccessful &&
                pConnectionEvent->eventData.pairingCompleteEvent.pairingCompleteData.withBonding)
            {
                /* If a bond is created, write device address in controller’s White List */
                Gap_AddDeviceToWhiteList(mPeerDeviceAddressType, maPeerDeviceAddress);
            }
        }
        break;

        case gConnEvtPairingRequest_c:
        {
            gPairingParameters.centralKeys = pConnectionEvent->eventData.pairingEvent.centralKeys;
            gPairingParameters.peripheralKeys = pConnectionEvent->eventData.pairingEvent.peripheralKeys;
            Gap_AcceptPairingRequest(peerDeviceId, &gPairingParameters);
        }    
        break;
        
        case gConnEvtKeyExchangeRequest_c:
        {
            gapSmpKeys_t sentSmpKeys = gSmpKeys;
            
            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gLtk_c))
            {
                sentSmpKeys.aLtk = NULL;
                /* When the LTK is NULL EDIV and Rand are not sent and will be ignored. */
            }
            
            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gIrk_c))
            {
                sentSmpKeys.aIrk = NULL;
                /* When the IRK is NULL the Address and Address Type are not sent and will be ignored. */
            }
            
            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gCsrk_c))
            {
                sentSmpKeys.aCsrk = NULL;
            }
            
            Gap_SendSmpKeys(peerDeviceId, &sentSmpKeys);
            break;
        }
            
        case gConnEvtLongTermKeyRequest_c:
        {
            if (pConnectionEvent->eventData.longTermKeyRequestEvent.ediv == gSmpKeys.ediv &&
                pConnectionEvent->eventData.longTermKeyRequestEvent.randSize == gSmpKeys.cRandSize)
            {
                /* EDIV and RAND both matched */
                Gap_ProvideLongTermKey(peerDeviceId, gSmpKeys.aLtk, gSmpKeys.cLtkSize);
            }
            else
            {
                /* EDIV or RAND size did not match */
                Gap_DenyLongTermKey(peerDeviceId);
            }
        }
        break;
            
        case gConnEvtEncryptionChanged_c:
        {
            if (mSendDataAfterEncStart)
            {
                /* Application handles encryption changes here. */
            }
        }
        break;
        
#else
        case gConnEvtPairingRequest_c:
        {
            Gap_RejectPairing (peerDeviceId, gPairingNotSupported_c);
        }    
        break;
#endif
        
    default:
        break;
    }
}

static void BleApp_GattServerCallback (deviceId_t deviceId, gattServerEvent_t* pServerEvent)
{
    switch (pServerEvent->eventType)
    {
      case gEvtCharacteristicCccdWritten_c:
          {
              Gap_SaveCccd (deviceId,
                            pServerEvent->eventData.charCccdWrittenEvent.handle,
                            pServerEvent->eventData.charCccdWrittenEvent.newCccd);
              
              BleApp_CccdWritten (deviceId,
                                  pServerEvent->eventData.charCccdWrittenEvent.handle,
                                  pServerEvent->eventData.charCccdWrittenEvent.newCccd) ;
          }
          break;
          
      case gEvtAttributeWritten_c:
              BleApp_AttributeWritten (deviceId,
                                       pServerEvent->eventData.attributeWrittenEvent.handle,
                                       pServerEvent->eventData.attributeWrittenEvent.cValueLength,
                                       pServerEvent->eventData.attributeWrittenEvent.aValue);
          break;
          
      case gEvtAttributeWrittenWithoutResponse_c:
              BleApp_AttributeWrittenWithoutResponse (deviceId,
                                                      pServerEvent->eventData.attributeWrittenEvent.handle,
                                                      pServerEvent->eventData.attributeWrittenEvent.cValueLength,
                                                      pServerEvent->eventData.attributeWrittenEvent.aValue);
          break;
          
      case gEvtHandleValueConfirmation_c:
              BleApp_HandleValueConfirmation (deviceId);
          break;
          
      default:
          break;
    }
}

static void BleApp_CccdWritten (deviceId_t deviceId, uint16_t handle, gattCccdFlags_t cccd)
{
    otapCommand_t otapCommand;
    bleResult_t   bleResult;
    
    /*! Check if the OTAP control point CCCD was written. */
    if (handle == cccd_otap_control_point)
    {
        /*! If the state is Idle try to send a New Image Info Request Command to the OTAP Server. */
        otapCommand.cmdId = gOtapCmdIdNewImageInfoRequest_c;
        FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageId,
                     (uint8_t*)otapClientData.currentImgId,
                     gOtap_ImageIdFieldSize_c);
        FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageVersion,
                     (uint8_t*)otapClientData.currentImgVer,
                     gOtap_ImageVersionFieldSize_c);
        
        bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoRequest_c]);
        if (gBleSuccess_c == bleResult)
        {
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;
            otapClientData.serverWrittenCccd = TRUE;
        }
        else
        {
            /*! A BLE error has occured - Disconnect */
            Gap_Disconnect (deviceId);
        }
    }
    /* lamp cccd */
    else if (handle == cccd_lamp_Control)
    {
     
    }
    else if (handle == cccd_White)
    {
     
    }
    else if (handle == cccd_core_temperature)
    {
      measure_chip_temperature();
      Las_RecordMeasurementTV (lasServiceConfig.serviceHandle);
    }
    else if (handle == cccd_lamp_on_sec)
    {
      Las_GetOnTimer(lasServiceConfig.serviceHandle);
    }
    else if (handle == cccd_lamp_off_sec)
    {
       Las_GetOffTimer(lasServiceConfig.serviceHandle);    
    }     
}

static void BleApp_AttributeWritten(deviceId_t  deviceId,
                                    uint16_t    handle,
                                    uint16_t    length,
                                    uint8_t*    pValue)
{
    bleResult_t bleResult;
    otapCommand_t otapCommand;

    
    /* Only the OTAP Control Point attribute is expected to be written using the
     * ATT Write Command. */
    if (handle == value_otap_control_point)
    {
        /*! Handle all OTAP Server to Client Commands Here. */
        switch(((otapCommand_t*)pValue)->cmdId)
        {
        case gOtapCmdIdNewImageNotification_c:
            bleResult = GattServer_SendAttributeWrittenStatus (deviceId,
                                                               value_otap_control_point,
                                                               gAttErrCodeNoError_c);
            if (gBleSuccess_c == bleResult)
            {
                OtapClient_HandleNewImageNotification (deviceId,
                                                       length,
                                                       pValue);
            }
            else
            {
                /*! A BLE error has occurred - Disconnect */
                Gap_Disconnect (deviceId);
            }
            break;
        case gOtapCmdIdNewImageInfoResponse_c:
            bleResult = GattServer_SendAttributeWrittenStatus (deviceId,
                                                               value_otap_control_point,
                                                               gAttErrCodeNoError_c);

            if (gBleSuccess_c == bleResult)
            {
                OtapClient_HandleNewImageInfoResponse (deviceId,
                                                       length,
                                                       pValue);
            }
            else
            {
                /*! A BLE error has occurred - Disconnect */
                Gap_Disconnect (deviceId);
            }
            break;
        case gOtapCmdIdErrorNotification_c:
            bleResult = GattServer_SendAttributeWrittenStatus (deviceId,
                                                               value_otap_control_point,
                                                               gAttErrCodeNoError_c);

            if (gBleSuccess_c == bleResult)
            {
                OtapClient_HandleErrorNotification (deviceId,
                                                    length,
                                                    pValue);
            }
            else
            {
                /*! A BLE error has occurred - Disconnect */
                Gap_Disconnect (deviceId);
            }
            break;
            
        default:
            otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
            otapCommand.cmd.errNotif.cmdId = pValue[0];
            otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedCommand_c;
    
            bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                        (void*)(&otapCommand),
                                                        cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
            if (gBleSuccess_c == bleResult)
            {
                otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
            }
            else
            {
                /*! A BLE error has occurred - Disconnect */
                Gap_Disconnect (deviceId);
            }
            break;
        };
    }
    /* lamp data BleApp_AttributeWritten  with BT ACK */
    else if (handle == value_lamp_Control)
    {
      if ( (length==1) )
      {
        bleResult = Las_SetLampControl(lasServiceConfig.serviceHandle, pValue[0], FALSE);
        // Report status to client
        BleApp_SendAttWriteResponse (deviceId, handle, bleResult);
      }
    }     
    else if (handle == value_lamp_White)
    {
      if ( (length==2) && (pValue[0] <= PWM_factor_MAX) && (pValue[1] <= PWM_factor_MAX) )
      {
         bleResult = Las_SetLampWhite (lasServiceConfig.serviceHandle, pValue[0], pValue[1], FALSE);
        // Report status to client
        BleApp_SendAttWriteResponse (deviceId, handle, bleResult);
      }
    }     
    else if (handle == value_lamp_RGB)
    {
       if ( (length==3) && (pValue[0] <= PWM_factor_MAX) && (pValue[1] <= PWM_factor_MAX) && (pValue[2] <= PWM_factor_MAX))
      {
        bleResult = Las_SetLampRGB (lasServiceConfig.serviceHandle, pValue[0], pValue[1], pValue[2]);
        // Report status to client
        BleApp_SendAttWriteResponse (deviceId, handle, bleResult);
      }
    }   
    else if (handle == value_lamp_clock)
    {
      if ( (length==7) )
      {

        // Report status to client
        BleApp_SendAttWriteResponse (deviceId, handle, bleResult);
      }
    }
    else if (handle == value_lamp_on_sec)
    {
      if ( (length==4) )
      {
        bleResult = Las_SetOnTimer(lasServiceConfig.serviceHandle, pValue);
        // Report status to client
        BleApp_SendAttWriteResponse (deviceId, handle, bleResult);
      }
    }
    else if (handle == value_lamp_off_sec)
    {
      if ( (length==4) )
      {
        bleResult = Las_SetOffTimer(lasServiceConfig.serviceHandle, pValue);
        // Report status to client
        BleApp_SendAttWriteResponse (deviceId, handle, bleResult);
      }
    }     
    else
    {
        /*! A GATT Server is trying to GATT Write an unknown attribute value.
         *  This should not happen. Disconnect the link. */
        Gap_Disconnect (deviceId);
    }
}

static void BleApp_AttributeWrittenWithoutResponse (deviceId_t deviceId,
                                                    uint16_t handle,
                                                    uint16_t length,
                                                    uint8_t* pValue)
{
    otapCommand_t otapCommand;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    bleResult_t bleResult;
    
    
    /* Only the OTAP Data attribute is expected to be written using the
     * ATT Write Without Response Command. */
    if (handle == value_otap_data)
    {
        if (otapClientData.state == mOtapClientStateDownloadingImage_c)
        {
            if (otapClientData.transferMethod == gOtapTransferMethodAtt_c)
            {
                if (((otapCommand_t*)pValue)->cmdId == gOtapCmdIdImageChunk_c)
                {
                    OtapClient_HandleDataChunk (deviceId,
                                                length,
                                                pValue);
                }
                else
                {
                    /* If the OTAP Client received an unexpected command on the data channel send an error to the OTAP Server. */
                    otapStatus = gOtapStatusUnexpectedCmdOnDataChannel_c;
                }
            }
            else
            {
                /* If the OTAP Client is not expecting image file chunks via ATT send an error to the OTAP Server. */
                otapStatus = gOtapStatusUnexpectedTransferMethod_c;
            }
        }
        else
        {
            /* If the OTAP Client is not expecting image file chunks send an error to the OTAP Server. */
            otapStatus = gOtapStatusImageDataNotExpected_c;
        }
        
        if (otapStatus != gOtapStatusSuccess_c)
        {
            otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
            otapCommand.cmd.errNotif.cmdId = pValue[0];
            otapCommand.cmd.errNotif.errStatus = otapStatus;
    
            bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                        (void*)(&otapCommand),
                                                        cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
            if (gBleSuccess_c == bleResult)
            {
                otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
            }
            else
            {
                /*! A BLE error has occurred - Disconnect */
                Gap_Disconnect (deviceId);
            }
        }
    }
    
   /* lamp data BleApp_AttributeWrittenWithoutResponse  */     
    else if (handle == value_lamp_White)
    {
      if ( (length==2) && (pValue[0] <= PWM_factor_MAX) && (pValue[1] <= PWM_factor_MAX) )
      {
        bleResult = Las_SetLampWhite (lasServiceConfig.serviceHandle, pValue[0], pValue[1], FALSE);
      }
    }     
    else if (handle == value_lamp_RGB)
    {
       if ( (length==3) && (pValue[0] <= PWM_factor_MAX) && (pValue[1] <= PWM_factor_MAX) && (pValue[2] <= PWM_factor_MAX))
      {
        bleResult = Las_SetLampRGB (lasServiceConfig.serviceHandle, pValue[0], pValue[1], pValue[2]);
      }
    }

}

static void BleApp_HandleValueConfirmation (deviceId_t deviceId)
{
    otapCommand_t otapCommand;
    bleResult_t   bleResult;
    
    /*! Check for which command sent to the OTAP Server the confirmation has been received. */
    switch (otapClientData.lastCmdSentToOtapServer)
    {
    case gOtapCmdIdNewImageInfoRequest_c:
        OtapClient_HandleNewImageInfoRequestConfirmation (deviceId);
        break;
        
    case gOtapCmdIdImageBlockRequest_c:
        OtapClient_HandleImageBlockRequestConfirmation (deviceId);
        break;
        
    case gOtapCmdIdImageTransferComplete_c:
        OtapClient_HandleImageTransferCompleteConfirmation (deviceId);
        break;
        
    case gOtapCmdIdErrorNotification_c:
        OtapClient_HandleErrorNotificationConfirmation (deviceId);
        break;
        
    case gOtapCmdIdStopImageTransfer_c:
        OtapClient_HandleStopImageTransferConfirmation (deviceId);
        break;
        
    default:
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNoCommand_c;
        otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedCommand_c;

        bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        if (gBleSuccess_c == bleResult)
        {
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
        }
        else
        {
            /*! A BLE error has occurred - Disconnect */
            Gap_Disconnect (deviceId);
        }
        break;
    };
}

static void OtapClient_HandleDataChunk (deviceId_t deviceId, uint16_t length, uint8_t* pData)
{
    otapCommand_t otapCommand;
    bleResult_t   bleResult;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    
    otapCmdImgChunkCoc_t* pDataChunk = (otapCmdImgChunkCoc_t*)(&((otapCommand_t*)pData)->cmd); //use the CoC Data Chunk type but observe the length
    uint16_t dataLen = length - gOtap_CmdIdFieldSize_c - gOtap_ChunkSeqNumberSize_c; // len
    
    /* Variables for the local image file parsing state machine. */
    static uint32_t currentImgElemRcvdLen = 0; /*!< Contains the number of received bytes for th current image element (header or othe sub element).
                                                         *   This is needed because the */
    static bleOtaImageFileHeader_t imgFileHeader;   /*!< Saved image file header. */
    static uint32_t elementEnd = 0;                 /*!< Current image file element expected end. */
    static subElementHeader_t subElemHdr;
    
    if (deviceId == otapClientData.peerOtapServer)
    {
        /* Check if the command length is as expected. */
        if ((length > (gOtap_CmdIdFieldSize_c + gOtap_ChunkSeqNumberSize_c)) &&
            (((otapClientData.transferMethod == gOtapTransferMethodAtt_c) && (length <= cmdIdToCmdLengthTable[gOtapCmdIdImageChunk_c])) ||
             ((otapClientData.transferMethod == gOtapTransferMethodL2capCoC_c) && (length <= gOtapCmdImageChunkCocLength_c))
            )
           )
        {
            /* Check if the chunk (sequence number) is as expected */
            if ((pDataChunk->seqNumber == otapClientData.chunkSeqNum) &&
                (pDataChunk->seqNumber < otapClientData.totalBlockChunks))
            {
                /*  Check if the data length is as expected. */
                if (((dataLen == otapClientData.chunkSize) && ((pDataChunk->seqNumber < (otapClientData.totalBlockChunks - 1)) || (otapClientData.totalBlockSize % otapClientData.chunkSize == 0))) ||
                    ((dataLen < otapClientData.chunkSize) && (pDataChunk->seqNumber == (otapClientData.totalBlockChunks - 1)) && (dataLen == otapClientData.totalBlockSize % otapClientData.chunkSize))
                   )
                {
                    /* Do more checks here if necessary. */
                }
                else
                {
                    otapStatus = gOtapStatusUnexpectedDataLength_c;
                }
            }
            else
            {
                otapStatus = gOtapStatusUnexpectedSequenceNumber_c;
            }
        }
        else
        {
            otapStatus = gOtapStatusInvalidCommandLength_c;
        }
    }
    else
    {
        otapStatus = gOtapStatusUnexpectedOtapPeer_c;
    }

    /*! If all checks were successful then parse the current data chunk, else send an error notification. */
    if (otapStatus == gOtapStatusSuccess_c)
    {
        pData = (uint8_t*)(&pDataChunk->data);
        
        /* If the Current position is 0 then reset the received length for the current image element
         * and the current image CRC to the initialization value which is 0.
         * The current position should be 0 only at the start of the image file transfer. */
        if (otapClientData.currentPos == 0)
        {
            currentImgElemRcvdLen = 0; 
            otapClientData.imgComputedCrc = 0;
        }
        
        /* Parse all the bytes in the data payload. */
        while (dataLen)
        {
            /* Wait for the header to arrive and check it's contents
             * then handle the elements of the image. */
            if (otapClientData.currentPos < sizeof(bleOtaImageFileHeader_t))
            {
                if ((otapClientData.currentPos + dataLen) >= sizeof(bleOtaImageFileHeader_t))
                {
                    uint16_t residualHeaderLen = sizeof(bleOtaImageFileHeader_t) - otapClientData.currentPos;
                    
                    /* There is enough information in the data payload to complete the header. */
                    FLib_MemCpy ((uint8_t*)(&imgFileHeader) + otapClientData.currentPos, pData, residualHeaderLen);
                    otapClientData.currentPos += residualHeaderLen;
                    pData += residualHeaderLen;
                    dataLen -= residualHeaderLen;
                    
                    /* Check header contents, and if it is not valid return and error and reset the image download position. */
                    otapStatus = OtapClient_IsImageFileHeaderValid (&imgFileHeader);
                    if (otapStatus != gOtapStatusSuccess_c)
                    {
                        otapClientData.currentPos = 0;
                        break;
                    }
                    
                    /* If the header is valid then update the CRC over the header part of the image. */
                    otapClientData.imgComputedCrc = OTA_CrcCompute ((uint8_t*)(&imgFileHeader),
                                                                    sizeof(bleOtaImageFileHeader_t),
                                                                    otapClientData.imgComputedCrc);
                    
                    currentImgElemRcvdLen = 0;
                    
                    /* If the remaining data length is not 0 then the loop will continue with the parsing of the next element. */
                }
                else
                {
                    /* Not enough data to complete the header.
                     * Copy all the data into the temporary header and
                     * increment the current image position. */
                    FLib_MemCpy((uint8_t*)(&imgFileHeader) + otapClientData.currentPos, pData, dataLen);
                    otapClientData.currentPos += dataLen;
                    dataLen = 0;
                }
            }
            else
            {
                /* The parsing has reached the sub-elements portion of the image. 
                 * Wait for each sub-element tag to arrive or parse it if it is known. */
                if (currentImgElemRcvdLen < sizeof(subElementHeader_t))
                {
                    if ((currentImgElemRcvdLen + dataLen) >= sizeof(subElementHeader_t))
                    {
                        uint16_t residualSubElemHdrLen = sizeof(subElementHeader_t) - currentImgElemRcvdLen;
                        
                        /* There is enough information in the data payload to complete the sub-element header. */
                        FLib_MemCpy ((uint8_t*)(&subElemHdr) + currentImgElemRcvdLen, pData, residualSubElemHdrLen);
                        otapClientData.currentPos += residualSubElemHdrLen;
                        currentImgElemRcvdLen += residualSubElemHdrLen;
                        pData += residualSubElemHdrLen;
                        dataLen -= residualSubElemHdrLen;
                        
                        /* Update the CRC over the sub-element header only if it is not the CRC Sub-Element header. */
                        if (subElemHdr.tagId != gBleOtaSubElemTagIdImageFileCrc_c)
                        {
                            otapClientData.imgComputedCrc = OTA_CrcCompute ((uint8_t*)(&subElemHdr),
                                                                            sizeof(subElementHeader_t),
                                                                            otapClientData.imgComputedCrc);
                        }
                        
                        elementEnd = otapClientData.currentPos + subElemHdr.dataLen;
                        
                        /* If the remaining data length is not 0 then the loop will
                        continue with the parsing of the sub-element. */
                    }
                    else
                    {
                        /* Not enough data to complete the sub-element header.
                         * Copy all the data into the temporary sub-element header
                         * and increment the current image position. */
                        FLib_MemCpy ((uint8_t*)(&subElemHdr) + currentImgElemRcvdLen, pData, dataLen);
                        otapClientData.currentPos += dataLen;
                        currentImgElemRcvdLen += dataLen;
                        dataLen = 0;
                    }
                }
                else
                {
                    uint32_t    elementChunkLength = 0;
                    
                    /* Make sure we do not pass the current element boundary. */
                    if ((otapClientData.currentPos + dataLen) >= elementEnd)
                    {
                        elementChunkLength = elementEnd - otapClientData.currentPos;
                    }
                    else
                    {
                        elementChunkLength = dataLen;
                    }
                    
                    /* Handle sub-element payload. */
                    switch (subElemHdr.tagId)
                    {
                    case gBleOtaSubElemTagIdUpgradeImage_c:
                        /* Immediately after receiving the header check if the image sub-element length is valid
                         * by trying to start the image upgrade procedure. */
                        if (currentImgElemRcvdLen == sizeof(subElementHeader_t))
                        {
                            if (gOtaSucess_c != OTA_StartImage(subElemHdr.dataLen))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusImageSizeTooLarge_c;
                                otapClientData.currentPos = 0;
                                break;
                            }
                        }
                        
                        /* Upgrade Image Tag - compute the CRC and try to push the chunk to the storage. */
                        otapClientData.imgComputedCrc = OTA_CrcCompute (pData,
                                                                        elementChunkLength,
                                                                        otapClientData.imgComputedCrc);
                        if (gOtaSucess_c != OTA_PushImageChunk (pData, elementChunkLength, NULL))
                        {
                            otapStatus = gOtapStatusImageStorageError_c;
                            otapClientData.currentPos = 0;
                            OTA_CancelImage();
                            break;
                        }
                        break;
                        
                    case gBleOtaSubElemTagIdSectorBitmap_c:
                        /* Immediately after receiving the header check if the sub-element length is valid. */
                        if (currentImgElemRcvdLen == sizeof(subElementHeader_t))
                        {
                            if (subElemHdr.dataLen != sizeof(otapClientData.imgSectorBitmap))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusInvalidSubElementLength_c;
                                otapClientData.currentPos = 0;
                                OTA_CancelImage();
                                break;
                            }
                        }
                        
                        /* Sector Bitmap Tag - Compute the CRC and copy the received bitmap to the buffer. */
                        otapClientData.imgComputedCrc = OTA_CrcCompute (pData,
                                                                        elementChunkLength,
                                                                        otapClientData.imgComputedCrc);
                        
                        FLib_MemCpy ((uint8_t*)otapClientData.imgSectorBitmap + (currentImgElemRcvdLen - sizeof(subElementHeader_t)),
                                     pData,
                                     elementChunkLength);
                        break;
                        
                    case gBleOtaSubElemTagIdImageFileCrc_c:
                        /* Immediately after receiving the header check if the sub-element length is valid. */
                        if (currentImgElemRcvdLen == sizeof(subElementHeader_t))
                        {
                            if (subElemHdr.dataLen != sizeof(otapClientData.imgReceivedCrc))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusInvalidSubElementLength_c;
                                otapClientData.currentPos = 0;
                                OTA_CancelImage();
                                break;
                            }
                        }
                        
                        /* CRC Tag - Just copy the received CRC to the buffer. */
                        FLib_MemCpy ((uint8_t*)(&otapClientData.imgReceivedCrc) + (currentImgElemRcvdLen - sizeof(subElementHeader_t)),
                                     pData,
                                     elementChunkLength);
                        break;
                        
                    default:
                        /* Unknown sub-element type, just compute the CRC over it. */
                        otapClientData.imgComputedCrc = OTA_CrcCompute (pData,
                                                                        elementChunkLength,
                                                                        otapClientData.imgComputedCrc);
                        break;
                    };
                    
                    if (otapStatus != gOtapStatusSuccess_c)
                    {
                        /* If an error has occurred then break the loop. */
                        break;
                    }
                    
                    otapClientData.currentPos += elementChunkLength;
                    currentImgElemRcvdLen += elementChunkLength;
                    pData += elementChunkLength;
                    dataLen -= elementChunkLength;
                    
                    /* If this element has been completely received then reset the current element
                     * received length to trigger the reception of the next sub-element. */
                    if (otapClientData.currentPos >= elementEnd)
                    {
                        currentImgElemRcvdLen = 0;
                    }
                }
            }
        } /* while (dataLen) */
    }
    
    if (otapStatus == gOtapStatusSuccess_c)
    {
        /* If the chunk has been successfully processed increase the expected sequence number. */
        otapClientData.chunkSeqNum += 1;
        
        /* Check if the block and/or image transfer is complete */
        if (otapClientData.chunkSeqNum >= otapClientData.totalBlockChunks)
        {
            /* If the image transfer is complete check the image CRC then
             * commit the image and set the bootloader flags. */
            if (otapClientData.currentPos >= otapClientData.imgSize)
            {
                if (otapClientData.imgComputedCrc != otapClientData.imgReceivedCrc)
                {
                    otapStatus = gOtapStatusInvalidImageCrc_c;
                    otapClientData.currentPos = 0;
                    OTA_CancelImage();
                }
                else if (gOtaSucess_c != OTA_CommitImage(otapClientData.imgSectorBitmap))
                {
                    otapStatus = gOtapStatusImageStorageError_c;
                    otapClientData.currentPos = 0;
                    OTA_CancelImage();
                }
                else
                {
                    /* The new image was successfully committed, set the bootloader new image flags,
                     * set the image transfer state as downloaded and send an image transfer complete
                     * message to the peer. */
                    OTA_SetNewImageFlag ();
                    otapClientData.state = mOtapClientStateImageDownloadComplete_c;
                    
                    otapCommand.cmdId = gOtapCmdIdImageTransferComplete_c;
                    FLib_MemCpy((uint8_t*)otapCommand.cmd.imgTransComplete.imageId, otapClientData.imgId, sizeof(otapCommand.cmd.imgTransComplete.imageId));
                    otapCommand.cmd.imgTransComplete.status = gOtapStatusSuccess_c;
                    
                    bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                                (void*)(&otapCommand),
                                                                cmdIdToCmdLengthTable[gOtapCmdIdImageTransferComplete_c]);
                    if (gBleSuccess_c == bleResult)
                    {
                        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
                    }
                    else
                    {
                        /*! A BLE error has occurred - Trigger the bootloader and reset now.
                         *  Do not wait for the Image Transfer Complete Confirmation. */
                        Gap_Disconnect (deviceId);
                        OTA_SetNewImageFlag ();
                        ResetMCU ();
                    }
                }
            }
            else
            {
                /* If just the current block is complete ask for another block. */
                OtapClient_ContinueImageDownload (deviceId);
            }
        }
    }
    
    if (otapStatus != gOtapStatusSuccess_c)
    {        
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdImageChunk_c;
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        if (gBleSuccess_c == bleResult)
        {
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
        }
        else
        {
            /*! A BLE error has occurred - Disconnect */
            Gap_Disconnect (deviceId);
        }
    }
}

static void OtapClient_HandleNewImageNotification (deviceId_t deviceId, uint16_t length, uint8_t* pValue)
{
    otapCommand_t otapCommand;
    bleResult_t   bleResult;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    otapCommand_t*  pRemoteCmd = (otapCommand_t*)pValue;
    
    /* Check the command length and parameters. */
    if (length != cmdIdToCmdLengthTable[gOtapCmdIdNewImageNotification_c])
    {
        otapStatus = gOtapStatusInvalidCommandLength_c;
    }
    else if (pRemoteCmd->cmd.newImgNotif.imageFileSize <= (sizeof(bleOtaImageFileHeader_t) + sizeof(subElementHeader_t)))
    {
        otapStatus = gOtapStatusInvalidImageFileSize_c;
    }
    else
    {
        switch (otapClientData.state)
        {
        case mOtapClientStateIdle_c:
            if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgNotif.imageId, pRemoteCmd->cmd.newImgNotif.imageVersion))
            {
                /* If a response for a New Image Info Request is expected from the OTAP Server simply ignore the
                 * New Image Notification. */
                if (otapClientData.lastCmdSentToOtapServer != gOtapCmdIdNewImageInfoRequest_c)
                {
                    /* Set up the Client to receive the image file. */
                    otapClientData.peerOtapServer = deviceId;
                    FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c);
                    FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion, gOtap_ImageVersionFieldSize_c);
                    otapClientData.imgSize = pRemoteCmd->cmd.newImgNotif.imageFileSize;
                    otapClientData.currentPos = 0;
                    otapClientData.chunkSize = 0;
                    otapClientData.chunkSeqNum = 0;
                    otapClientData.totalBlockChunks = 0;
                    otapClientData.totalBlockSize = 0;
                    
                    /* Change the Client state to Downloading and trigger the download. */
                    otapClientData.state = mOtapClientStateDownloadingImage_c;
                    OtapClient_ContinueImageDownload (deviceId);
                }
            }
            /* If the remote image is not newer than the current image simply ignore the New Image Notification */
            break;
     
        case mOtapClientStateDownloadingImage_c:            /* Fallthrough */
        case mOtapClientStateImageDownloadComplete_c:
            /* Simply ignore the message if an image is being downloaded or
             * an image download is complete. */
            break;
            
        default:
            /* Some kind of internal error has occurred. Reset the
             * client state to Idle and act as if the state was Idle. */
            otapClientData.state = mOtapClientStateIdle_c;
            if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgNotif.imageId, pRemoteCmd->cmd.newImgNotif.imageVersion))
            {
                /* If a response for a New Image Info Request is expected from the OTAp Server simply ignore the
                 * New Image Notification. */
                if (otapClientData.lastCmdSentToOtapServer != gOtapCmdIdNewImageInfoRequest_c)
                {
                    /* Set up the Client to receive the image file. */
                    otapClientData.peerOtapServer = deviceId;
                    FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c);
                    FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion, gOtap_ImageVersionFieldSize_c);
                    otapClientData.imgSize = pRemoteCmd->cmd.newImgNotif.imageFileSize;
                    otapClientData.currentPos = 0;
                    otapClientData.chunkSize = 0;
                    otapClientData.chunkSeqNum = 0;
                    otapClientData.totalBlockChunks = 0;
                    otapClientData.totalBlockSize = 0;
                    
                    /* Change the Client state to Downloading and trigger the download. */
                    otapClientData.state = mOtapClientStateDownloadingImage_c;
                    OtapClient_ContinueImageDownload (deviceId);
                }
            }
            /* If the remote image is not newer than the current image simply ignore the New Image Notification */
            break;
        };
    }
        
    if (otapStatus != gOtapStatusSuccess_c)
    {
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = pValue[0];
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        if (gBleSuccess_c == bleResult)
        {
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
        }
        else
        {
            /*! A BLE error has occured - Disconnect */
            Gap_Disconnect (deviceId);
        }
    }
}

static void OtapClient_HandleNewImageInfoResponse (deviceId_t deviceId, uint16_t length, uint8_t* pValue)
{
    otapCommand_t otapCommand;
    bleResult_t   bleResult;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    otapCommand_t*  pRemoteCmd = (otapCommand_t*)pValue;
    
    /* Check the command length and parameters. */
    if (length != cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoResponse_c])
    {
        otapStatus = gOtapStatusInvalidCommandLength_c;
    }
    else if (pRemoteCmd->cmd.newImgInfoRes.imageFileSize <= (sizeof(bleOtaImageFileHeader_t) + sizeof(subElementHeader_t)))
    {
        otapStatus = gOtapStatusInvalidImageFileSize_c;
    }
    else
    {
        switch (otapClientData.state)
        {
        case mOtapClientStateIdle_c:
            if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgInfoRes.imageId, pRemoteCmd->cmd.newImgInfoRes.imageVersion))
            {
                /* Set up the Client to receive the image file. */
                otapClientData.peerOtapServer = deviceId;
                FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgInfoRes.imageId, gOtap_ImageIdFieldSize_c);
                FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgInfoRes.imageVersion, gOtap_ImageVersionFieldSize_c);
                otapClientData.imgSize = pRemoteCmd->cmd.newImgInfoRes.imageFileSize;
                otapClientData.currentPos = 0;
                otapClientData.chunkSize = 0;
                otapClientData.chunkSeqNum = 0;
                otapClientData.totalBlockChunks = 0;
                otapClientData.totalBlockSize = 0;
                
                /* Change the Client state to Downloading and trigger the download. */
                otapClientData.state = mOtapClientStateDownloadingImage_c;
                OtapClient_ContinueImageDownload (deviceId);
            }
            /* If the remote image is not newer than the current image simply ignore the New Image Info Response */
            break;
     
        case mOtapClientStateDownloadingImage_c:            /* Fallthrough */
        case mOtapClientStateImageDownloadComplete_c:
            /* Simply ignore the message if an image is being downloaded or
             * an image download is complete. */
            break;
            
        default:
            /* Some kind of internal error has occurred. Reset the
             * client state to Idle and act as if the state was Idle. */
            otapClientData.state = mOtapClientStateIdle_c;
            if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgInfoRes.imageId, pRemoteCmd->cmd.newImgInfoRes.imageVersion))
            {
                /* Set up the Client to receive the image file. */
                otapClientData.peerOtapServer = deviceId;
                FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgInfoRes.imageId, gOtap_ImageIdFieldSize_c);
                FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgInfoRes.imageVersion, gOtap_ImageVersionFieldSize_c);
                otapClientData.imgSize = pRemoteCmd->cmd.newImgInfoRes.imageFileSize;
                otapClientData.currentPos = 0;
                otapClientData.chunkSize = 0;
                otapClientData.chunkSeqNum = 0;
                otapClientData.totalBlockChunks = 0;
                otapClientData.totalBlockSize = 0;
                
                /* Change the Client state to Downloading and trigger the download. */
                otapClientData.state = mOtapClientStateDownloadingImage_c;
                OtapClient_ContinueImageDownload (deviceId);
            }
            /* If the remote image is not newer than the current image simply ignore the New Image Info Response */
            break;
        };
    }
        
    if (otapStatus != gOtapStatusSuccess_c)
    {
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNewImageInfoResponse_c;
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        if (gBleSuccess_c == bleResult)
        {
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
        }
        else
        {
            /*! A BLE error has occured - Disconnect */
            Gap_Disconnect (deviceId);
        }
    }
}

static void OtapClient_HandleErrorNotification (deviceId_t deviceId, uint16_t length, uint8_t* pValue)
{
    otapCommand_t otapCommand;
    bleResult_t   bleResult;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    otapCommand_t*  pRemoteCmd = (otapCommand_t*)pValue;
    
    /* Check the command length and parameters. */
    if (length == cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c])
    {
        /*! Handle remote error statuses here. */
        if (pRemoteCmd->cmd.errNotif.errStatus < gOtapNumberOfStatuses_c)
        {
            /* Handle all errors in the same way, disconnect to restart the download process. */
            Gap_Disconnect (deviceId);
        }
        else
        {
            otapStatus = gOtapStatusInvalidCommandParameter_c;
        }
    }
    else
    {
        otapStatus = gOtapStatusInvalidCommandLength_c;
    }
        
    if (otapStatus != gOtapStatusSuccess_c)
    {
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNewImageInfoResponse_c;
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        if (gBleSuccess_c == bleResult)
        {
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
        }
        else
        {
            /*! A BLE error has occured - Disconnect */
            Gap_Disconnect (deviceId);
        }
    }
}

static void OtapClient_HandleNewImageInfoRequestConfirmation (deviceId_t deviceId)
{
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;
    
    /* Nothing more to do here. If the New Image Info Request Command has reached
     * the OTAP Server then the OTAP Client expects a New Image Info Response */
}

static void OtapClient_HandleImageBlockRequestConfirmation (deviceId_t deviceId)
{
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;
    
    /* Nothing more to do here. If the Image Block Request Command has reached
     * the OTAP Server then the OTAP Client expects the requested image chunks
     * or an Error Notification. */
}

static void OtapClient_HandleImageTransferCompleteConfirmation (deviceId_t deviceId)
{
    otapCommand_t otapCommand;
    bleResult_t   bleResult;
    
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;
    
    /* If the image transfer was not successful then the image download state should be Idle.
     * If it is, try to trigger a new download.
     * If the Image Transfer Complete Command has reached the OTAP Server and the transfer was succesful 
     * then the OTAP Client will just wait for the restart and the
     * bootloader to flash the new image. */
    if (otapClientData.state == mOtapClientStateIdle_c)
    {
        otapCommand.cmdId = gOtapCmdIdNewImageInfoRequest_c;
        FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageId,
                     (uint8_t*)otapClientData.currentImgId,
                     gOtap_ImageIdFieldSize_c);
        FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageVersion,
                     (uint8_t*)otapClientData.currentImgVer,
                     gOtap_ImageVersionFieldSize_c);
        
        bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoRequest_c]);
        if (gBleSuccess_c == bleResult)
        {
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;
        }
        else
        {
            /*! A BLE error has occured - Disconnect */
            Gap_Disconnect (deviceId);
        }
    }
    else if (otapClientData.state == mOtapClientStateImageDownloadComplete_c)
    {
        /* If the image transfer is complete trigger the bootloader and reset the device. */
        Gap_Disconnect (deviceId);
        OTA_SetNewImageFlag ();
        ResetMCU ();
    }
}

static void OtapClient_HandleErrorNotificationConfirmation (deviceId_t deviceId)
{
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;
    
    /* Reset block download parameters to safe values. */
    otapClientData.chunkSize = 0;
    otapClientData.chunkSeqNum = 0;
    otapClientData.totalBlockChunks = 0;
    otapClientData.totalBlockSize = 0;
    
    /* If an error has occured try to continue the image download from a "safe" point. */
    OtapClient_ContinueImageDownload (deviceId);
}

static void OtapClient_HandleStopImageTransferConfirmation (deviceId_t deviceId)
{
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;
    
    /* Reset block download parameters to safe values. */
    otapClientData.chunkSize = 0;
    otapClientData.chunkSeqNum = 0;
    otapClientData.totalBlockChunks = 0;
    otapClientData.totalBlockSize = 0;
    
    /* If an error has occured try to continue the image download from a "safe" point. */
    OtapClient_ContinueImageDownload (deviceId);
}

static void OtapClient_HandleConnectionEvent (deviceId_t deviceId)
{
    switch (otapClientData.state)
    {
    case mOtapClientStateIdle_c:
        /*! If the OTAP Server has written the CCCD to receive commands fromt he OTAp Client then send a
         *  new image info request. */
        if (otapClientData.serverWrittenCccd == TRUE)
        {
            otapCommand_t otapCommand;
            bleResult_t   bleResult;
    
            otapCommand.cmdId = gOtapCmdIdNewImageInfoRequest_c;
            FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageId,
                         (uint8_t*)otapClientData.currentImgId,
                         gOtap_ImageIdFieldSize_c);
            FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageVersion,
                         (uint8_t*)otapClientData.currentImgVer,
                         gOtap_ImageVersionFieldSize_c);
            
            bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                        (void*)(&otapCommand),
                                                        cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoRequest_c]);
            if (gBleSuccess_c == bleResult)
            {
                otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;
                otapClientData.serverWrittenCccd = TRUE;
            }
            else
            {
                /*! A BLE error has occured - Disconnect */
                Gap_Disconnect (deviceId);
            }
        }
        break;
    case  mOtapClientStateDownloadingImage_c:
        /*! If the state is Downloading try to continue the download from where it was left off.
         *  Check if the appropriate server is connected first. */
        if (otapClientData.peerOtapServer == deviceId)
        {
            /* Reset block download parameters to safe values. */
            otapClientData.chunkSize = 0;
            otapClientData.chunkSeqNum = 0;
            otapClientData.totalBlockChunks = 0;
            otapClientData.totalBlockSize = 0;
            
            OtapClient_ContinueImageDownload (deviceId);
        }
        break;
    case mOtapClientStateImageDownloadComplete_c:
        /*! If the image download is complete try to set the new image flag
         *  and reset the MCU for the bootloader ot kick in. */
        Gap_Disconnect (deviceId);
        OTA_SetNewImageFlag ();
        ResetMCU ();
        break;
    default:
        /* Some kind of internal error has occurred. Reset the
         * client state to Idle and act as if the state was Idle. */
        otapClientData.state = mOtapClientStateIdle_c;
        break;
    };
}

static void OtapClient_HandleDisconnectionEvent (deviceId_t deviceId)
{
    /* Check if the peer OTAP server was disconnected and if so reset block download
     * parameters to safe values. */
    if (otapClientData.peerOtapServer == deviceId)
    {
        otapClientData.chunkSize = 0;
        otapClientData.chunkSeqNum = 0;
        otapClientData.totalBlockChunks = 0;
        otapClientData.totalBlockSize = 0;
    }
}

static void OtapClient_ContinueImageDownload (deviceId_t deviceId)
{
    otapCommand_t   otapCommand;
    bleResult_t     bleResult;
    uint32_t        bytesToDownload;
    uint32_t        maxBlockSize;
    
    switch (otapClientData.state)
    {
    case mOtapClientStateIdle_c:
        /* If the state is Idle do nothing. No need to continue the transfer of an image. */
        break;
    case mOtapClientStateDownloadingImage_c:
        /* If the last received chunk sequence number is equal to the total block
         * chunks or they are both zero then ask for a new block from the server. */
        if (otapClientData.chunkSeqNum == otapClientData.totalBlockChunks)
        {
            /* Ask for another block only if the image transfer was not completed. */
            if (otapClientData.currentPos < otapClientData.imgSize)
            {
                bytesToDownload = otapClientData.imgSize - otapClientData.currentPos;
                
                if (otapClientData.transferMethod == gOtapTransferMethodAtt_c)
                {
                    maxBlockSize = gOtap_ImageChunkDataSizeAtt_c * gOtap_MaxChunksPerBlock_c;
                    otapClientData.l2capChannelOrPsm = gL2capCidAtt_c;
                    otapClientData.chunkSize = gOtap_ImageChunkDataSizeAtt_c;
                }
                else if (otapClientData.transferMethod == gOtapTransferMethodL2capCoC_c)
                {
                    if (otapClientData.l2capChannelOrPsm == gOtap_L2capLePsm_c)
                    {
                        maxBlockSize = gOtap_ImageChunkDataSizeL2capCoc_c * gOtap_MaxChunksPerBlock_c;
                        otapClientData.chunkSize = gOtap_ImageChunkDataSizeL2capCoc_c;
                    }
                    else
                    {
                        /* If the L2CAP CoC is not valid then some kind of error or missconfiguration has
                         * occurred. Send a proper error notification to the peer and
                         * reset the download state machine to Idle. */
                        otapClientData.state = mOtapClientStateIdle_c;
                        
                        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
                        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNoCommand_c;
                        otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedL2capChannelOrPsm_c;

                        bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                                    (void*)(&otapCommand),
                                                                    cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
                        if (gBleSuccess_c == bleResult)
                        {
                            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
                        }
                        else
                        {
                            /*! A BLE error has occured - Disconnect */
                            Gap_Disconnect (deviceId);
                        }
                        
                        return;
                    }
                }
                else
                {
                    /* If the transfer method is not recognized then this image has been missconfigured
                     * or a critical error has occurred. Send a proper error notification to the peer and
                     * reset the download state machien to Idle. */
                    otapClientData.state = mOtapClientStateIdle_c;
                    
                    otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
                    otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNoCommand_c;
                    otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedTransferMethod_c;

                    bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                                (void*)(&otapCommand),
                                                                cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
                    if (gBleSuccess_c == bleResult)
                    {
                        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
                    }
                    else
                    {
                        /*! A BLE error has occured - Disconnect */
                        Gap_Disconnect (deviceId);
                    }
                    
                    return;
                }
                
                if (bytesToDownload >= maxBlockSize)
                {
                    /* If there are more bytes to download than the maximum block size then
                     * ask a full block from the server on the selected tansfer method and set up
                     * the client to recieve the chunks.*/
                    otapClientData.chunkSeqNum = 0;
                    otapClientData.totalBlockChunks = gOtap_MaxChunksPerBlock_c;
                    otapClientData.totalBlockSize = maxBlockSize;
                }
                else
                {
                    /* If there are fewer bytes to download than the maximum block size then compute the
                    *  number of chunks expected and set the expected block size to the number of 
                    *  bytes to download. */
                    otapClientData.chunkSeqNum = 0;
                    /* Compute number of full chunks. Integer division. */
                    otapClientData.totalBlockChunks = bytesToDownload / otapClientData.chunkSize;
                    /* Add an extra chunk if the chunk size is not a divisor of the number of bytes to download. */
                    otapClientData.totalBlockChunks += (bytesToDownload % otapClientData.chunkSize) ? 1 : 0;
                    otapClientData.totalBlockSize = bytesToDownload;
                }

                /* Send the Block request Command with teh determined parameters. */
                otapCommand.cmdId = gOtapCmdIdImageBlockRequest_c;
                
                FLib_MemCpy(otapCommand.cmd.imgBlockReq.imageId, otapClientData.imgId, gOtap_ImageIdFieldSize_c);
                otapCommand.cmd.imgBlockReq.startPosition = otapClientData.currentPos;
                otapCommand.cmd.imgBlockReq.blockSize = otapClientData.totalBlockSize;
                otapCommand.cmd.imgBlockReq.chunkSize = otapClientData.chunkSize;
                otapCommand.cmd.imgBlockReq.transferMethod = otapClientData.transferMethod;
                otapCommand.cmd.imgBlockReq.l2capChannelOrPsm = otapClientData.l2capChannelOrPsm;
                
                bleResult = OtapCS_SendCommandToOtapServer (service_otap,
                                                            (void*)(&otapCommand),
                                                            cmdIdToCmdLengthTable[gOtapCmdIdImageBlockRequest_c]);
                if (gBleSuccess_c == bleResult)
                {
                    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdImageBlockRequest_c;
                }
                else
                {
                    /*! A BLE error has occured - Disconnect */
                    Gap_Disconnect (deviceId);
                }
            }
        }
        break;
    case mOtapClientStateImageDownloadComplete_c:
        /*! If the image download is complete try to set the new image flag
         *  and reset the MCU for the bootloader ot kick in. */
        Gap_Disconnect (deviceId);
        OTA_SetNewImageFlag ();
        ResetMCU ();
        break;
    default:
        /* This code should never be reached in normal running conditions.
        Do nothing here, no need to continue the transfer of an image. */
        break;
    };
}

static bool_t OtapClient_IsRemoteImageNewer (uint8_t* pRemoteImgId, uint8_t* pRemoteImgVer)
{
    uint32_t    remoteBuildVer;
    uint32_t    localeBuildVer;
    /* Ignore the Image Id for the moment. */
    /* Check the Manufacturer Id */
    if (pRemoteImgVer[7] != otapClientData.currentImgVer[7])
    {
        return FALSE;
    }
    
    /* Check Hardware Id */
    if (!FLib_MemCmp((void*)(&(pRemoteImgVer[4])), (void*)(&(otapClientData.currentImgVer[4])), 3))
    {
        return FALSE;
    }
    
    /* Check Stack Version */
    if (pRemoteImgVer[3] < otapClientData.currentImgVer[3])
    {
        return FALSE;
    }
    
    /* Check Build Version */
    remoteBuildVer = (uint32_t)pRemoteImgVer[0] + ((uint32_t)(pRemoteImgVer[1]) << 8) + ((uint32_t)(pRemoteImgVer[2]) << 16);
    localeBuildVer = (uint32_t)otapClientData.currentImgVer[0] + ((uint32_t)(otapClientData.currentImgVer[1]) << 8) + ((uint32_t)(otapClientData.currentImgVer[2]) << 16);
    if (remoteBuildVer <= localeBuildVer)
    {
        return FALSE;
    }
    
    return TRUE;
}

static otapStatus_t OtapClient_IsImageFileHeaderValid (bleOtaImageFileHeader_t* imgFileHeader)
{
    if (imgFileHeader->fileIdentifier != gBleOtaFileHeaderIdentifier_c)
    {
        return gOtapStatusUnknownFileIdentifier_c;
    }
    
    if (imgFileHeader->headerVersion != gbleOtapHeaderVersion0100_c)
    {
        return gOtapStatusUnknownHeaderVersion_c;
    }
    
    if (imgFileHeader->headerLength != sizeof(bleOtaImageFileHeader_t))
    {
        return gOtapStatusUnexpectedHeaderLength_c;
    }
    
    if (imgFileHeader->fieldControl != gBleOtaFileHeaderDefaultFieldControl_c)
    {
        return gOtapStatusUnexpectedHeaderFieldControl_c;
    }
    
    if (imgFileHeader->companyId != gBleOtaCompanyIdentifier_c)
    {
        return gOtapStatusUnknownCompanyId_c;
    }
    
    if (FALSE == FLib_MemCmp (imgFileHeader->imageId, otapClientData.imgId, sizeof(imgFileHeader->imageId)))
    {
        return gOtapStatusUnexpectedImageId_c;
    }
    
    if (FALSE == FLib_MemCmp (imgFileHeader->imageVersion, otapClientData.imgVer, sizeof(imgFileHeader->imageVersion)))
    {
        return gOtapStatusUnexpectedImageVersion_c;
    }
    
    if (imgFileHeader->totalImageFileSize != otapClientData.imgSize)
    {
        return gOtapStatusUnexpectedImageFileSize_c;
    }
    
    return gOtapStatusSuccess_c;
}









/*! *********************************************************************************
* \brief        Measurement Timer for Temperature and voltage on core.
*
* \param[in]    pParam        Callback parameters.
********************************************************************************** */
static void MeasurementTimerCallback(void * pParam)
{
    measure_chip_temperature();

    // if active BLE conection 
    if(mPeerDeviceId != gInvalidDeviceId_c)
    {
      Las_RecordMeasurementTV (lasServiceConfig.serviceHandle);
    
      basServiceConfig.batteryLevel = 90;
      Bas_RecordBatteryMeasurement(basServiceConfig.serviceHandle, basServiceConfig.batteryLevel);
    }

}


/*! *********************************************************************************
* \brief        Sends GATT response to the client
********************************************************************************** */
static void BleApp_SendAttWriteResponse (deviceId_t deviceId, uint16_t handle, bleResult_t result)
{
  attErrorCode_t attErrorCode;
  
  // Determine response to send (OK or Error)
  if(result == gBleSuccess_c)
    attErrorCode = gAttErrCodeNoError_c;
  else{
    attErrorCode = (attErrorCode_t)(result & 0x00FF);
  }
  // Send response to client  
  GattServer_SendAttributeWrittenStatus(deviceId, handle, attErrorCode);
}

/*! *********************************************************************************
* @}
********************************************************************************** */
