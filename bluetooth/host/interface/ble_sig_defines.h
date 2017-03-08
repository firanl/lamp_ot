/*! *********************************************************************************
 * \addtogroup BLE
 * @{
 ********************************************************************************** */
/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file ble_sig_defines.h
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

#ifndef _BLE_SIG_DEFINES_H_
#define _BLE_SIG_DEFINES_H_

/************************************************************************************
*************************************************************************************
* Public constants & macros
*************************************************************************************
************************************************************************************/
 
/*! Bluetooth SIG UUID constants for GATT declarations */

/*! Primary Service declaration UUID */
#define gBleSig_PrimaryService_d                0x2800
/*! Secondary Service declaration UUID */
#define gBleSig_SecondaryService_d              0x2801
/*! Include declaration UUID */
#define gBleSig_Include_d                       0x2802
/*! Characteristic declaration UUID */
#define gBleSig_Characteristic_d                0x2803

/*! Client Characteristic Configuration Descriptor declaration UUID */
#define gBleSig_CCCD_d                          0x2902
/*! Server Characteristic Configuration Descriptor declaration UUID */
#define gBleSig_SCCD_d                          0x2903
/*! Characteristic Presentation Format declaration UUID */
#define gBleSig_CharPresFormatDescriptor_d      0x2904
/*! Valid Range Descriptor declaration UUID */
#define gBleSig_ValidRangeDescriptor_d          0x2906


/*! GAP Service UUID */
#define gBleSig_GenericAccessProfile_d          0x1800
/*! GATT Service UUID */
#define gBleSig_GenericAttributeProfile_d       0x1801
/*! Immediate Alert Service UUID */
#define gBleSig_ImmediateAlertService_d         0x1802
/*! Link Loss Service UUID */
#define gBleSig_LinkLossService_d               0x1803
/*! Tx Power Service UUID */
#define gBleSig_TxPowerService_d                0x1804
/*! Glucose Service UUID */
#define gBleSig_GlucoseService_d                0x1808
/*! Health Thermometer Service UUID */
#define gBleSig_HealthThermometerService_d      0x1809
/*! Device Information Service UUID */
#define gBleSig_DeviceInformationService_d      0x180A
/*! Heart Rate Service UUID */
#define gBleSig_HeartRateService_d              0x180D
/*! Battery Service UUID */
#define gBleSig_BatteryService_d                0x180F
/*! Blood Pressure Service UUID */
#define gBleSig_BloodPressureService_d          0x1810
/*! Alert Notification Service UUID */
#define gBleSig_AlertNotificationService_d      0x1811
/*! HID Service UUID */
#define gBleSig_HidService_d                    0x1812
/*! Running Speed And Cadence Service UUID */
#define gBleSig_RunningSpeedAndCadenceService_d 0x1814
/*! Cycling Speed And Cadence Service UUID */
#define gBleSig_CyclingSpeedAndCadenceService_d 0x1816
/*! Cycling Power Service UUID */
#define gBleSig_CyclingPowerService_d           0x1818
/*! Internet Protocol Support Service UUID */
#define gBleSig_IpsService_d                    0x1820

/* GATT Characteristics */
/* https://www.bluetooth.com/specifications/gatt/characteristics */
  /*! GAP Device Name Characteristic UUID */
  #define gBleSig_GapDeviceName_d                 0x2A00
  /*! GAP Appearance Characteristic UUID */
  /* https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.gap.appearance.xml */
  #define gBleSig_GapAppearance_d                 0x2A01
  /*! GAP Peripheral Preferred Connection Parameters Characteristic UUID */
  /* https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.gap.peripheral_preferred_connection_parameters.xml */
  #define gBleSig_GapPpcp_d                       0x2A04
  /*! GATT Service Changed Characteristic UUID */
  /* https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.gatt.service_changed.xml */
  #define gBleSig_GattServiceChanged_d            0x2A05
  /*! Alert Level Characteristic UUID */
  #define gBleSig_AlertLevel_d                    0x2A06
  /*! TX Power Characteristic UUID */
  #define gBleSig_TxPower_d                       0x2A07
  /*! Date Time   */
  /* https://www.bluetooth.com/ko-kr/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.date_time.xml */
  #define gBleSig_Date_Time_d                    0x2A08
  /*! Glucose Measurement Characteristic UUID */
  #define gBleSig_GlucoseMeasurement_d            0x2A18
  /*! Battery Level Characteristic UUID */
  #define gBleSig_BatteryLevel_d                  0x2A19
  /*! Temperature Measurement Characteristic UUID */
  #define gBleSig_TemperatureMeasurement_d        0x2A1C
  /*! Temperature Type Characteristic UUID */
  #define gBleSig_TemperatureType_d               0x2A1D
  /*! Intermediate Temperature Characteristic UUID */
  #define gBleSig_IntermediateTemperature_d       0x2A1E
  /*! Measurement Interval Characteristic UUID */
  #define gBleSig_MeasurementInterval_d           0x2A21
  /*! System ID Characteristic UUID */
  #define gBleSig_SystemId_d                      0x2A23
  /*! Model Number String Characteristic UUID */
  #define gBleSig_ModelNumberString_d             0x2A24
  /*! Serial Number String Characteristic UUID */
  #define gBleSig_SerialNumberString_d            0x2A25
  /*! Firmware Revision String Characteristic UUID */
  #define gBleSig_FirmwareRevisionString_d        0x2A26
  /*! Hardware Revision String Characteristic UUID */
  #define gBleSig_HardwareRevisionString_d        0x2A27
  /*! Software Revision String Characteristic UUID */
  #define gBleSig_SoftwareRevisionString_d        0x2A28
  /*! Manufacturer Name String Characteristic UUID */
  #define gBleSig_ManufacturerNameString_d        0x2A29
  /*! IEEE 11073-20601 Regulatory Certification Data List Characteristic UUID */
  #define gBleSig_IeeeRcdl_d                      0x2A2A
  /*! Boot Keyboard Input Report UUID */
  #define gBleSig_BootKeyboardInputReport_d       0x2A22
  /*! Boot Keyboard output Report UUID */
  #define gBleSig_BootKeyboardOutputReport_d      0x2A32
  /*! Boot Mouse Input Report UUID */
  #define gBleSig_BootMouseInputReport_d          0x2A33
  /*! Glucose Measurement Context Characteristic UUID */
  #define gBleSig_GlucoseMeasurementContext_d     0x2A34
  /*! Blood Pressure Measurement UUID */
  #define gBleSig_BpMeasurement_d                 0x2A35
  /*! Intermediate Cuff Pressure UUID */
  #define gBleSig_IntermediateCuffPressure_d      0x2A36
  /*! Heart Rate Measurement UUID */
  #define gBleSig_HrMeasurement_d                 0x2A37
  /*! Body Sensor Location UUID */
  #define gBleSig_BodySensorLocation_d            0x2A38
  /*! Heart Rate Control Point UUID */
  #define gBleSig_HrControlPoint_d                0x2A39
  /*! Alert Notif Control Point UUID */
  #define gBleSig_AlertNotifControlPoint_d        0x2A44
  /*! Unread Alert Status UUID */
  #define gBleSig_UnreadAlertStatus_d             0x2A45
  /*! New Alert UUID */
  #define gBleSig_NewAlert_d                      0x2A46
  /*! Supported New Alert Category UUID */
  #define gBleSig_SupportedNewAlertCategory_d     0x2A47
  /*! Supported Unread Alert Category UUID */
  #define gBleSig_SupportedUnreadAlertCategory_d  0x2A48
  /*! Blood Pressure Feature UUID */
  #define gBleSig_BloodPressureFeature_d          0x2A49
  /*! HID Information UUID */
  #define gBleSig_HidInformation_d                0x2A4A
  /*! HID Control Point UUID */
  #define gBleSig_HidCtrlPoint_d                  0x2A4C
  /*! Report UUID */
  #define gBleSig_Report_d                        0x2A4D
  /*! Protocol Mode UUID */
  #define gBleSig_ProtocolMode_d                  0x2A4E
  /*! Scan Interval Window UUID */
  #define gBleSig_ScanIntervalWindow_d            0x2A4F
  /*! PnP Id UUID */
  #define gBleSig_PnpId_d                         0x2A50
  /*! Glucose Feature Characteristic UUID */
  #define gBleSig_GlucoseFeature_d                0x2A51
  /*! Record Access Ctrl Point Characteristic UUID */
  #define gBleSig_RaCtrlPoint_d                   0x2A52
  /*! RSC Measurement UUID */
  #define gBleSig_RscMeasurement_d                0x2A53
  /*! RSC Feature UUID */
  #define gBleSig_RscFeature_d                    0x2A54
  /*! SC Control Point UUID */
  #define gBleSig_ScControlPoint_d                0x2A55
  /*! CSC Measurement Characteristic UUID */
  #define gBleSig_CscMeasurement_d                0x2A5B
  /*! CSC Feature Characteristic UUID */
  #define gBleSig_CscFeature_d                    0x2A5C
  /*! Sensor Location Characteristic UUID */
  #define gBleSig_SensorLocation_d                0x2A5D
  /*! CP Measurement Characteristic UUID */
  #define gBleSig_CpMeasurement_d                 0x2A63
  /*! CP Measurement Vector UUID */
  #define gBleSig_CpVector_d                      0x2A64
  /*! CP Feature CharacteristicUUID */
  #define gBleSig_CpFeature_d                     0x2A65
  /*! CP Control Point UUID */
  #define gBleSig_CpControlPoint_d                0x2A66
  /*! Temperature Characteristic UUID */
  #define gBleSig_Temperature_d                   0x2A6E


  /*! GAP Appearance Characteristic Value Fields */
  #define gBleSig_GapAppearance_Bytes_d         2


  /*! GAP Peripheral Preferred Connection Parameters Characteristic Value Fields */
    #define gBleSig_GapPpcp_Bytes_d         8
    /* Minimum Connection Interval  uint16 	6 	3200   connInterval_min = Minimum Connection Interval * 1.25 ms */
    #define gBleSig_GapPpcp_mci_d   0x0A, 0x00
    /* Maximum Connection Interval uint16 	6 	3200 */
    #define gBleSig_GapPpcp_Mci_d   0x10, 0x00
    /* Slave Latency   uint16 	0 	1000  */
    #define gBleSig_GapPpcp_SL_d    0x64, 0x00
    /* Connection Supervision Timeout Multiplier  uint16 	10 	3200 */
    #define gBleSig_GapPpcp_cstm_d    0xE2, 0x04

  /*! GATT Service Changed Characteristic Value Fields  */
    #define gBleSig_GattServiceChanged_Bytes_d  4
    /* Start of Affected Attribute Handle Range uint16 	1 	65535  */
    #define gBleSig_GattServiceChanged_SAAHR_d  0x00, 0x00
    /* End of Affected Attribute Handle Range  uint16 	1 	65535 */
    #define gBleSig_GattServiceChanged_EAAHR_d  0x00, 0x00


/* DESCRIPTOR(.., gBleSig_CharPresFormatDescriptor_d, .., 7, Format, Exponent, Unit, Namespace, Description) 
* https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.characteristic_presentation_format.xml 
* The actual value = Characteristic Value * 10^Exponent. */
#define gBleSig_CharPresFormatDescriptorBytes_d         7
  /* Format Mandatory  8bit 	range 0x00 	0x1B */
  /* https://www.bluetooth.com/specifications/assigned-numbers/format-types */
    /* 4 	unsigned 8-bit integer */
    #define gBleSig_unsigned_8_bit_integer_d    0x04
    /* 6	unsigned 16-bit integer	*/
    #define gBleSig_unsigned_16_bit_integer_d   0x06
    /* 14 	signed 16-bit integer */
    #define gBleSig_signed_16_bit_integer_d     0x0E

  /* Exponent Mandatory sint8 	-128 0 127  */
    /* 10^Exponent: 0 =1 */
    #define gBleSig_Exponent_0_d    0x00 
    /* 10^Exponent: -2 */
    #define gBleSig_Exponent_neg2_d 0xFE 
    /* 10^Exponent: -3 */
    #define gBleSig_Exponent_neg3_d 0xFD 

  /* Unit The Unit is a UUID. Mandatory uint16 	N/A 	N/A  */
  /* https://www.bluetooth.com/specifications/assigned-numbers/units */
    /* 0x2700	unitless	org.bluetooth.unit.unitless */
    #define gBleSig_unitless_d                       0x00, 0x27
    /* 0x272F	Celsius temperature (degree Celsius)	org.bluetooth.unit.thermodynamic_temperature.degree_celsius */
    #define gBleSig_Celsius_temperature_d            0x2F, 0x27
    /* 0x2728	electric potential difference (volt)	org.bluetooth.unit.electric_potential_difference.volt */
    #define gBleSig_electric_potential_difference_d  0x28, 0x27
    /* 0x27AD	percentage	org.bluetooth.unit.percentage */
    #define gBleSig_percentage_d  0xAD, 0x27

  /* Namespace (organization) Mandatory  8bit 	0 	1 */
    /* 0 	No Namespace  */
    #define gBleSig_No_Namespaces_d                   0x00
    /* 1 	Bluetooth SIG Assigned Numbers */
    #define gBleSig_Bluetooth_SIG_Assigned_Numbers_d  0x01

  /* Description  Name Space field. Mandatory 16bit 	N/A 	N/A 	*/
  /* https://www.bluetooth.com/specifications/assigned-numbers/gatt-namespace-descriptors */
    /* unknown	0x0000 */
    #define gBleSig_unknown_d  0x00, 0x00


/*! Macro that returns whether or not an input 16-bit UUID is a grouping type. */
#define BleSig_IsGroupingAttributeUuid16(uuid16) \
 ((uuid16) == gBleSig_PrimaryService_d || \
  (uuid16) == gBleSig_SecondaryService_d || \
  (uuid16) == gBleSig_Include_d || \
  (uuid16) == gBleSig_Characteristic_d)
     
/*! Macro that returns whether or not an input 16-bit UUID is a Service declaration. */
#define BleSig_IsServiceDeclarationUuid16(uuid16) \
 ((uuid16) == gBleSig_PrimaryService_d || \
  (uuid16) == gBleSig_SecondaryService_d)

/*! Macro that declares a 16 bit UUID in a bleUuid_t union. */
#define Uuid16(uuid)   {.uuid16 = (uuid)} 
   
/*! Macro that declares a 32 bit UUID in a bleUuid_t union. */
#define Uuid32(uuid)   {.uuid32 = (uuid)} 

#define UuidArray(value)  (value & 0xFF), (value >> 8)
#endif /* _BLE_SIG_DEFINES_H_ */                

/*! *********************************************************************************
* @}
********************************************************************************** */
