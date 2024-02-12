#ifndef _SABERTOOTH_CONFIG_
#define _SABERTOOTH_CONFIG_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "Sabertooth.h"
#include "generated/commands.h"
#include "MotorOperator.h"

#define DEFAULT_SABERTOOTH_BAUD_RATE 9600
#define DEFAULT_SABERTOOTH_ADDRESS 130
#define DEFAULT_SABERTOOTH_SERIAL_LINE Serial1
#define DEFAULT_SABERTOOTH_MOTOR_NUM 1
#define SABERTOOTH_MAX_OUTPUT 126

class SabertoothOperator : public MotorOperator
{
  public:
    SabertoothOperator(byte address, unsigned int baudrate, byte motornum, USARTClass& serial, bool inverted, bool enabled);
    SabertoothOperator(byte address, unsigned int baudrate, byte motornum, USARTClass& serial);
    SabertoothOperator();
    SabertoothOperator(const SabertoothOperator& other);
    /*!
    Assigns current object to other object
    \param other SabertoothOperator
    */
    SabertoothOperator& operator=(const SabertoothOperator& other);

    //Operation functions
    bool init() override;
    void setOutput(float percentOutput);
    bool applyConfigUpdate(const Sabertooth_Config_Data& update);

  private:
    // Sabertooth specific
    USARTClass& serialLine;
    unsigned int baudrate;
    byte motornum;
    Sabertooth sabertooth;
};

#endif