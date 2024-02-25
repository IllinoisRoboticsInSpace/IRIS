#ifndef _PROTOBUF_UTILITIES_
#define _PROTOBUF_UTILITIES_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "Sabertooth.h"
#include "RotaryEncoderOperator.h"
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

inline RotaryEncoder::LatchMode getLatchMode(LatchMode latchmode)
{
    switch (latchmode)
    {
        case LatchMode::FOUR3:
        default:
            return RotaryEncoder::LatchMode::FOUR3;
        case LatchMode::FOUR0:
            return RotaryEncoder::LatchMode::FOUR0;
        case LatchMode::TWO03:
            return RotaryEncoder::LatchMode::TWO03;
    }
}

// // Computes the maximum Protobuf message size in bytes
// uint32_t maxReceiveProtoMessageBytesSize()
// {
//     Serial_Message_To_Arduino message;

//     //TODO: figure out how to compute this size
//     // https://stackoverflow.com/questions/30915704

//     return message.serialized_size();
// }

#endif