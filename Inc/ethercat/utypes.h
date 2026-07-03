#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

typedef struct
{
    struct
    {
        uint16_t ControlWord;
        uint16_t StatusWord;
        int8_t OperationMode;
        int8_t OperationModeDisplay;
        int32_t PositionActualValue;
        uint16_t Lan9252Gpi;
        uint16_t Lan9252Gpo;
        uint8_t McuLed;
    } Parameters;
} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
