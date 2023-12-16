#ifndef _PROTOBUF_UTILITIES_
#define _PROTOBUF_UTILITIES_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "Sabertooth.h"
#include "generated/commands.h"

inline USARTClass& getSabertoothSerial(SabertoothSerialLine serial)
{
    switch (serial)
    {
        case SabertoothSerialLine::Serial1:
        default:
            return Serial1;
        case SabertoothSerialLine::Serial2:
            return Serial2;
        case SabertoothSerialLine::Serial3:
            return Serial3;
    }
}

#endif