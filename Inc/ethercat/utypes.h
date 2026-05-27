#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

typedef struct
{
    struct
    {
        uint8_t Button1;
    } Buttons;

    struct
    {
        uint8_t LED0;
        uint8_t LED1;
    } LEDs;

    struct
    {
        uint32_t Multiplier;
    } Parameters;
} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
