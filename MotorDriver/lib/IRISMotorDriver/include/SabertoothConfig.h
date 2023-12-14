#ifndef _SABERTOOTH_CONFIG_
#define _SABERTOOTH_CONFIG_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "Sabertooth.h"

#define DEFAULT_SABERTOOTH_BAUD_RATE 9600

class SabertoothConfig
{
  public:
    SabertoothConfig();
    SabertoothConfig(const SabertoothConfig& other);
    /*!
    Assigns current object to other object
    \param other SabertoothConfig
    */
    SabertoothConfig& operator=(const SabertoothConfig& other);

  private:
    // General
    bool inverted;
    bool enabled; // Throw error if false

    // Sabertooth specific
    UARTClass* serialLine;
    unsigned int address;
    unsigned int baudrate;
    Sabertooth sabertooth;
};

#endif