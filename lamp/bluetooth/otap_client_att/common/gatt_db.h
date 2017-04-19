PRIMARY_SERVICE(service_gatt, gBleSig_GenericAttributeProfile_d)
    CHARACTERISTIC(char_service_changed, gBleSig_GattServiceChanged_d, ( gGattCharPropNotify_c) )
        VALUE(value_service_changed, gBleSig_GattServiceChanged_d, (gPermissionNone_c), gBleSig_GattServiceChanged_Bytes_d, gBleSig_GattServiceChanged_SAAHR_d, gBleSig_GattServiceChanged_EAAHR_d)
        CCCD(cccd_service_changed)

PRIMARY_SERVICE(service_gap, gBleSig_GenericAccessProfile_d)
    CHARACTERISTIC(char_device_name, gBleSig_GapDeviceName_d, (gGattCharPropRead_c) )
        VALUE(value_device_name, gBleSig_GapDeviceName_d, (gPermissionFlagReadable_c), sizeof(LA_LAMP_GapDeviceName), LA_LAMP_GapDeviceName)
    CHARACTERISTIC(char_appearance, gBleSig_GapAppearance_d, (gGattCharPropRead_c) )
            VALUE(value_appearance, gBleSig_GapAppearance_d, (gPermissionFlagReadable_c), gBleSig_GapAppearance_Bytes_d, UuidArray(gGenericDisplay_c))
    CHARACTERISTIC(char_ppcp, gBleSig_GapPpcp_d, (gGattCharPropRead_c) )
        VALUE(value_ppcp, gBleSig_GapPpcp_d, (gPermissionFlagReadable_c), gBleSig_GapPpcp_Bytes_d ,gBleSig_GapPpcp_mci_d, gBleSig_GapPpcp_Mci_d, gBleSig_GapPpcp_SL_d, gBleSig_GapPpcp_cstm_d)



PRIMARY_SERVICE(service_device_info, gBleSig_DeviceInformationService_d)
    CHARACTERISTIC(char_manuf_name, gBleSig_ManufacturerNameString_d, (gGattCharPropRead_c) )
        VALUE(value_manuf_name, gBleSig_ManufacturerNameString_d, (gPermissionFlagReadable_c), sizeof(DI_ManufacturerNameString), DI_ManufacturerNameString)
    CHARACTERISTIC(char_model_no, gBleSig_ModelNumberString_d, (gGattCharPropRead_c) )
        VALUE(value_model_no, gBleSig_ModelNumberString_d, (gPermissionFlagReadable_c),   sizeof(DI_ModelNumberString), DI_ModelNumberString)
    CHARACTERISTIC(char_serial_no, gBleSig_SerialNumberString_d, (gGattCharPropRead_c) )
        VALUE(value_serial_no, gBleSig_SerialNumberString_d, (gPermissionFlagReadable_c), sizeof(DI_SerialNumberString), DI_SerialNumberString)
    CHARACTERISTIC(char_hw_rev, gBleSig_HardwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_hw_rev, gBleSig_HardwareRevisionString_d, (gPermissionFlagReadable_c), 4, DI_HardwareRevisionString)
    CHARACTERISTIC(char_fw_rev, gBleSig_FirmwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_fw_rev, gBleSig_FirmwareRevisionString_d, (gPermissionFlagReadable_c), 2, DI_FirmwareRevision )
    CHARACTERISTIC(char_sw_rev, gBleSig_SoftwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_sw_rev, gBleSig_SoftwareRevisionString_d, (gPermissionFlagReadable_c), sizeof(DI_SoftwareRevisionString), DI_SoftwareRevisionString)
          
          
PRIMARY_SERVICE_UUID128(service_lamp, uuid_service_lamp)
    CHARACTERISTIC_UUID128(char_lamp_Control, uuid_char_lamp_Control, (gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c ) )
        VALUE_UUID128(value_lamp_Control, uuid_char_lamp_Control, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 1, LA_LAMP_CONTROL)
        CCCD(cccd_lamp_Control)

    CHARACTERISTIC_UUID128(char_lamp_White, uuid_char_lamp_White, (gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c) )
        VALUE_UUID128(value_lamp_White, uuid_char_lamp_White, (gPermissionFlagReadable_c | gPermissionFlagWritable_c),  2, LA_LAMP_WARM_WHITE, LA_LAMP_COLD_WHITE)
        DESCRIPTOR(desc_lamp_White, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), gBleSig_CharPresFormatDescriptorBytes_d, gBleSig_unsigned_16_bit_integer_d, gBleSig_Exponent_0_d, gBleSig_percentage_d, gBleSig_No_Namespaces_d, gBleSig_unknown_d)
        CCCD(cccd_White)

    CHARACTERISTIC_UUID128(char_lamp_RGB, uuid_char_lamp_RGB, (gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c) )
        VALUE_UUID128(value_lamp_RGB, uuid_char_lamp_RGB, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 3, LA_LAMP_R, LA_LAMP_G, LA_LAMP_B)
        CCCD(cccd_RGB)
 
    CHARACTERISTIC(char_core_temperature, gBleSig_Temperature_d, (gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c))
        VALUE(value_core_temperature, gBleSig_Temperature_d, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 2, LA_LAMP_TEMP)
        DESCRIPTOR(desc_core_temperature, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), gBleSig_CharPresFormatDescriptorBytes_d, gBleSig_signed_16_bit_integer_d, gBleSig_Exponent_neg2_d, gBleSig_Celsius_temperature_d, gBleSig_Bluetooth_SIG_Assigned_Numbers_d , gBleSig_unknown_d)
        CCCD(cccd_core_temperature)          

    CHARACTERISTIC_UUID128(char_core_voltage, uuid_char_core_voltage, (gGattCharPropNotify_c | gGattCharPropRead_c) )
        VALUE_UUID128(value_core_voltage, uuid_char_core_voltage, (gPermissionFlagReadable_c ),  2, LA_LAMP_VCC)
        DESCRIPTOR(desc_core_voltage, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), gBleSig_CharPresFormatDescriptorBytes_d, gBleSig_signed_16_bit_integer_d, gBleSig_Exponent_neg3_d, gBleSig_electric_potential_difference_d, gBleSig_No_Namespaces_d, gBleSig_unknown_d)
        CCCD(cccd_core_voltage)
 
    CHARACTERISTIC_UUID128(char_lamp_on_sec, uuid_char_lamp_on_sec, ( gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c ) )
        VALUE_UUID128(value_lamp_on_sec, uuid_char_lamp_on_sec, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 4, 0x00, 0x00, 0x00, 0x00)
        CCCD(cccd_lamp_on_sec) 
 
    CHARACTERISTIC_UUID128(char_lamp_off_sec, uuid_char_lamp_off_sec, ( gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c ) )
        VALUE_UUID128(value_lamp_off_sec, uuid_char_lamp_off_sec, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 4, 0x00, 0x00, 0x00, 0x00)   
        CCCD(cccd_lamp_off_sec)     
          
     CHARACTERISTIC_UUID128(char_lamp_config, uuid_char_lamp_config, ( gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c ) )  
        VALUE_UUID128_VARLEN(value_lamp_config, uuid_char_lamp_config, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 5, 1, 0x00, 0x00, 0x00, 0x00, 0x00)
        CCCD(cccd_lamp_TSI)          

          
PRIMARY_SERVICE_UUID128(service_otap, uuid_service_otap)
    CHARACTERISTIC_UUID128(char_otap_control_point, uuid_char_otap_control_point, (gGattCharPropWrite_c | gGattCharPropIndicate_c))
        VALUE_UUID128_VARLEN(value_otap_control_point, uuid_char_otap_control_point, (gPermissionFlagWritable_c), 16, 16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
        CCCD(cccd_otap_control_point)
    CHARACTERISTIC_UUID128(char_otap_data, uuid_char_otap_data, (gGattCharPropWriteWithoutRsp_c))
        VALUE_UUID128_VARLEN(value_otap_data, uuid_char_otap_data, (gPermissionFlagWritable_c), 20, 20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
 
       