#ifndef _MOTOR_DRIVER_
#define _MOTOR_DRIVER_

#include <Arduino.h>

#include "Sabertooth.h" //? DESIGN: is sabertooth necessary for motor driver to have?

#define NUM_ARDUINO_PINS 4 // placeholder
#define SERIAL_DEFAULT_BAUD_RATE 112500 // was there already (previous comment said "for printing")

/**
 * The MotorDriver class is responsible for keeping track of all in use
 * encoders, motors, PID loops, and other devices/services that would be
 * used with the arduino. It is responsible for configuring these resources
 * by processing the serial commands that are recieved from the host device.
 * The MotorDriver class is written to handle message errors such as corrupted
 * data and invalid commands.
 * 
 * @note All functionality in the Arduino Motor Driver must be verified and
 * synced up with development on the Python host driver.
 * 
 * 
 * Version 0.0.1:
 * For the first version of the MotorDriver, all update loop processing
 * will occur sequentially as it is easy to maintain. This version will
 * focus on supporting motor actuation commands. For simplicity this version
 * does not need to have CRC checks in the message definition.
 * 
 * Desired Features:
 * - Add configuration, handling, and communication of encoder data.
 * Support both timer interrupts, encoder interrupts, and polling in that order.
 * 
 * - Design a robust finite state machine to represent the communication
 * protocol. For instance, choosing which commands should send back an
 * acknowledge message (ACK) and which shouldn't.
 * 
 * - Add PID controller configuration, execution, and connecting a PID
 * controller to an encoder and motor.
 * 
 * - Debugging mode/support such as print statements back to host, or echoing
 * back of all recieved commands.
 * 
 * - Create interfaces to abstract the implementation of encoders and motors.
 * 
 * Advanced Features:
 * - Multithread the execution of PID controller loops to enable more even
 * and consistent update rates.
 * 
 * - Multithread and queue the updating of encoder data. Basically, if
 * the serial line is in the process of sending a message then, an encoder
 * message will need to wait till the first message is done sending. Therefore
 * all encoder updates that can not be serviced as a given moment will be queued
 * up for sending instead of being processed.
 */
class MotorDriver
{
  public:
    MotorDriver();
    MotorDriver(Sabertooth *st, unsigned int baudRate);

    bool init_motor_driver();

    void update();
    void setBaudRate(unsigned int baudRate);
    void setMode(bool mode_auto);
    void Read();
    void Parse();
    void Execute();
    void ReadEncoder();

    // placeholder, not representative of motion
    enum MotorDirection {
      up, down, left, right
    };

    struct MotorDriverConfig {
      unsigned int motorID;
      unsigned int baudRate;
      int serialAddress;
      int serialPins[NUM_ARDUINO_PINS];
      MotorDirection direction;
      bool mode_auto;
    };

  private:
    bool initialized;
    MotorDriverConfig config;
    Sabertooth *st; //? DESIGN: see above (line 6)

};
#endif