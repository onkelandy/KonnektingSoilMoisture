#define MANUFACTURER_ID 57005
#define DEVICE_ID 1
#define REVISION 0

#define COMOBJ_moisture 0    //ok
#define COMOBJ_LowerAlarm 1  //ok
#define COMOBJ_UpperAlarm 2  //ok
#define COMOBJ_Resistance 3  //ok
#define PARAM_Geraeteanlaufzeit 0
#define PARAM_cyclic 1
#define PARAM_cycle 2
#define PARAM_DeltaMoisture 3
#define PARAM_SendOnChange 4
#define PARAM_LowerAlarm 5       //ok
#define PARAM_SendOnLowerAlarm 6 //ok
#define PARAM_UpperAlarm 7       //ok
#define PARAM_SendOnUpperAlarm 8 //ok
#define PARAM_Samples 9
        
KnxComObject KnxDevice::_comObjectsList[] = {
    /* Index 0 - moisture */ KnxComObject(KNX_DPT_9_006, 0x34),
    /* Index 1 - LowerAlarm */ KnxComObject(KNX_DPT_1_001, 0x34),
    /* Index 2 - UpperAlarm */ KnxComObject(KNX_DPT_1_001, 0x34),
    /* Index 3 - Resistance */ KnxComObject(KNX_DPT_7_001, 0x34)
};
const byte KnxDevice::_numberOfComObjects = sizeof (_comObjectsList) / sizeof (KnxComObject); // do not change this code
       
byte KonnektingDevice::_paramSizeList[] = {
    /* Index 0 - Geraeteanlaufzeit */ PARAM_UINT8,
    /* Index 1 - cyclic */ PARAM_UINT8,
    /* Index 2 - cycle */ PARAM_UINT32,
    /* Index 3 - DeltaMoisture */ PARAM_UINT8,
    /* Index 4 - SendOnChange */ PARAM_UINT8,
    /* Index 5 - LowerAlarm */ PARAM_INT16,
    /* Index 6 - SendOnLowerAlarm */ PARAM_UINT8,
    /* Index 7 - UpperAlarm */ PARAM_INT16,
    /* Index 8 - SendOnUpperAlarm */ PARAM_UINT8,
    /* Index 9 - Samples */ PARAM_UINT16
};
const int KonnektingDevice::_numberOfParams = sizeof (_paramSizeList); // do not change this code
