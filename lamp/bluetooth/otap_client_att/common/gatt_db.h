PRIMARY_SERVICE(service_gatt, gBleSig_GenericAttributeProfile_d)
    CHARACTERISTIC(char_service_changed, gBleSig_GattServiceChanged_d, (gGattCharPropRead_c | gGattCharPropNotify_c) )
        VALUE(value_service_changed, gBleSig_GattServiceChanged_d, (gPermissionNone_c), 4, 0x00, 0x00, 0x00, 0x00)
        CCCD(cccd_service_changed)

PRIMARY_SERVICE(service_gap, gBleSig_GenericAccessProfile_d)
    CHARACTERISTIC(char_device_name, gBleSig_GapDeviceName_d, (gGattCharPropRead_c) )
        VALUE(value_device_name, gBleSig_GapDeviceName_d, (gPermissionFlagReadable_c), 19, "FSL_BLE_OTAP_CLIENT")
    CHARACTERISTIC(char_appearance, gBleSig_GapAppearance_d, (gGattCharPropRead_c) )
            VALUE(value_appearance, gBleSig_GapAppearance_d, (gPermissionFlagReadable_c), 2, UuidArray(gUnknown_c))
    CHARACTERISTIC(char_ppcp, gBleSig_GapPpcp_d, (gGattCharPropRead_c) )
        VALUE(value_ppcp, gBleSig_GapPpcp_d, (gPermissionFlagReadable_c), 8, 0x0A, 0x00, 0x10, 0x00, 0x64, 0x00, 0xE2, 0x04)



PRIMARY_SERVICE(service_device_info, gBleSig_DeviceInformationService_d)
    CHARACTERISTIC(char_manuf_name, gBleSig_ManufacturerNameString_d, (gGattCharPropRead_c) )
        VALUE(value_manuf_name, gBleSig_ManufacturerNameString_d, (gPermissionFlagReadable_c), 9, "Freescale")
    CHARACTERISTIC(char_model_no, gBleSig_ModelNumberString_d, (gGattCharPropRead_c) )
        VALUE(value_model_no, gBleSig_ModelNumberString_d, (gPermissionFlagReadable_c), 9, "OTAP Demo")
    CHARACTERISTIC(char_serial_no, gBleSig_SerialNumberString_d, (gGattCharPropRead_c) )
        VALUE(value_serial_no, gBleSig_SerialNumberString_d, (gPermissionFlagReadable_c), 7, "BLESN01")
    CHARACTERISTIC(char_hw_rev, gBleSig_HardwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_hw_rev, gBleSig_HardwareRevisionString_d, (gPermissionFlagReadable_c), sizeof(BOARD_NAME), BOARD_NAME)
    CHARACTERISTIC(char_fw_rev, gBleSig_FirmwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_fw_rev, gBleSig_FirmwareRevisionString_d, (gPermissionFlagReadable_c), 5, "1.1.1")
    CHARACTERISTIC(char_sw_rev, gBleSig_SoftwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_sw_rev, gBleSig_SoftwareRevisionString_d, (gPermissionFlagReadable_c), 5, "1.1.4")
          
PRIMARY_SERVICE_UUID128(service_lamp, uuid_service_lamp)
    CHARACTERISTIC_UUID128(char_lamp_Control, uuid_char_lamp_Control, (gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c) )
        VALUE_UUID128(value_lamp_Control, uuid_char_lamp_Control, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 1, LA_LAMP_CONTROL)
        CCCD(cccd_lamp_Control)

    CHARACTERISTIC_UUID128(char_lamp_White, uuid_char_lamp_White, (gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c) )
        VALUE_UUID128(value_lamp_White, uuid_char_lamp_White, (gPermissionFlagReadable_c | gPermissionFlagWritable_c),  2, LA_LAMP_WHITE)
        DESCRIPTOR(desc_lamp_White, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), gBleSig_CharPresFormatDescriptorBytes_d, gBleSig_unsigned_16_bit_integer_d, gBleSig_Exponent_0_d, gBleSig_percentage_d, gBleSig_No_Namespaces_d, gBleSig_unknown_d)
        CCCD(cccd_White)

    CHARACTERISTIC_UUID128(char_lamp_RGB, uuid_char_lamp_RGB, (gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c) )
        VALUE_UUID128(value_lamp_RGB, uuid_char_lamp_RGB, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 3, LA_LAMP_RGB)
 
    CHARACTERISTIC(char_core_temperature, gBleSig_Temperature_d, (gGattCharPropRead_c | gGattCharPropNotify_c))
        VALUE(value_core_temperature, gBleSig_Temperature_d, (gPermissionFlagReadable_c), 2, 0xD1, 0x07)
        DESCRIPTOR(desc_core_temperature, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), gBleSig_CharPresFormatDescriptorBytes_d, gBleSig_signed_16_bit_integer_d, gBleSig_Exponent_neg2_d, gBleSig_Celsius_temperature_d, gBleSig_No_Namespaces_d, gBleSig_unknown_d)
        CCCD(cccd_core_temperature)          

    CHARACTERISTIC_UUID128(char_core_voltage, uuid_char_core_voltage, (gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c) )
        VALUE_UUID128(value_core_voltage, uuid_char_core_voltage, (gPermissionFlagReadable_c ),  2, 0x0D, 0x02)
        DESCRIPTOR(desc_core_voltage, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), gBleSig_CharPresFormatDescriptorBytes_d, gBleSig_signed_16_bit_integer_d, gBleSig_Exponent_neg3_d, gBleSig_electric_potential_difference_d, gBleSig_No_Namespaces_d, gBleSig_unknown_d)
        CCCD(cccd_core_voltage)
             
    CHARACTERISTIC(char_lamp_clock, gBleSig_Date_Time_d, (gGattCharPropNotify_c | gGattCharPropRead_c | gGattCharPropWrite_c) )
        VALUE(value_lamp_clock,  gBleSig_Date_Time_d, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 7, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00 )
        CCCD(cccd_lamp_clock) 
 
    CHARACTERISTIC_UUID128(char_lamp_onHHMM, uuid_char_lamp_onHHMM, (gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c) )
        VALUE_UUID128(value_lamp_onHHMM, uuid_char_lamp_onHHMM, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 2, 0x00, 0x00)
 
    CHARACTERISTIC_UUID128(char_lamp_offHHMM, uuid_char_lamp_offHHMM, (gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c) )
        VALUE_UUID128(value_lamp_offHHMM, uuid_char_lamp_offHHMM, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 2, 0x00, 0x00)          

PRIMARY_SERVICE_UUID128(service_otap, uuid_service_otap)
    CHARACTERISTIC_UUID128(char_otap_control_point, uuid_char_otap_control_point, (gGattCharPropWrite_c | gGattCharPropIndicate_c))
        VALUE_UUID128_VARLEN(value_otap_control_point, uuid_char_otap_control_point, (gPermissionFlagWritable_c), 16, 16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
        CCCD(cccd_otap_control_point)
    CHARACTERISTIC_UUID128(char_otap_data, uuid_char_otap_data, (gGattCharPropWriteWithoutRsp_c))
        VALUE_UUID128_VARLEN(value_otap_data, uuid_char_otap_data, (gPermissionFlagWritable_c), 20, 20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
 
 PRIMARY_SERVICE(service_battery, gBleSig_BatteryService_d)
    CHARACTERISTIC(char_battery_level, gBleSig_BatteryLevel_d, (gGattCharPropNotify_c | gGattCharPropRead_c))
        VALUE(value_battery_level, gBleSig_BatteryLevel_d, (gPermissionFlagReadable_c), 1, 0x5A)
        DESCRIPTOR(desc_bat_level, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), 7, 0x04, 0x00, 0xAD, 0x27, 0x01, 0x01, 0x00)
        CCCD(cccd_battery_level)         