#define MANUFACTURER_ID 57005
#define DEVICE_ID 1
#define REVISION 0

#define COMOBJ_Moisture 0    //ok
#define COMOBJ_LowerAlarm 1  //ok
#define COMOBJ_UpperAlarm 2  //ok
#define COMOBJ_Resistance 3  //ok
#define COMOBJ_Tension 4
#define COMOBJ_Change 5
#define PARAM_startup_delay 0
#define PARAM_sensor_delay 1
#define PARAM_Samples 2
#define PARAM_cyclic 3
#define PARAM_cycle 4
#define PARAM_SendOnChange 5
#define PARAM_DeltaMoisture 6
#define PARAM_LowerAlarm 7
#define PARAM_SendOnLowerAlarm 8
#define PARAM_RepeatLowerAlarm 9
#define PARAM_UpperAlarm 10
#define PARAM_SendOnUpperAlarm 11
#define PARAM_RepeatUpperAlarm 12

KnxComObject KnxDevice::_comObjectsList[] = {
    /* Index 0 - moisture */ KnxComObject(KNX_DPT_9_006, 0x34),
    /* Index 1 - LowerAlarm */ KnxComObject(KNX_DPT_1_001, 0x34),
    /* Index 2 - UpperAlarm */ KnxComObject(KNX_DPT_1_001, 0x34),
    /* Index 3 - Resistance */ KnxComObject(KNX_DPT_7_001, 0x34),
    /* Index 4 - Tension */ KnxComObject(KNX_DPT_7_001, 0x34),
    /* Index 5 - Change */ KnxComObject(KNX_DPT_1_001, 0x34)
};
const byte KnxDevice::_numberOfComObjects = sizeof (_comObjectsList) / sizeof (KnxComObject); // do not change this code

byte KonnektingDevice::_paramSizeList[] = {
    /* Index 0 - startup_delay */ PARAM_UINT8,
    /* Index 1 - sensor_delay */ PARAM_UINT8,
    /* Index 2 - Samples */ PARAM_UINT16,
    /* Index 3 - cyclic */ PARAM_UINT8,
    /* Index 4 - cycle */ PARAM_UINT32,
    /* Index 5 - SendOnChange */ PARAM_UINT8,
    /* Index 6 - DeltaMoisture */ PARAM_UINT8,
    /* Index 7 - LowerAlarm */ PARAM_UINT8,
    /* Index 8 - SendOnLowerAlarm */ PARAM_UINT8,
    /* Index 9 - RepeatLowerAlarm */ PARAM_UINT8,
    /* Index 10 - UpperAlarm */ PARAM_UINT8,
    /* Index 11 - SendOnUpperAlarm */ PARAM_UINT8,
    /* Index 12 - RepeatUpperAlarm */ PARAM_UINT8

};
const int KonnektingDevice::_numberOfParams = sizeof (_paramSizeList); // do not change this code
