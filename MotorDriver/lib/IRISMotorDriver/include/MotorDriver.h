#ifndef _MOTOR_DRIVER_
#define _MOTOR_DRIVER_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "Sabertooth.h"
#include "SabertoothOperator.h"
#include "RotaryEncoderOperator.h"
#include "WriteBufferFixedSize.h"
#include "ReadBufferFixedSize.h"

#define MAX_MOTOR_ID 15 // Maximum number of motor ids 0 indexed
#define MAX_MOTOR_CONFIGS (MAX_MOTOR_ID + 1)

#define MAX_ENCODER_ID 14 // Maximum number of encoders ids 0 indexed
#define MAX_ENCODER_CONFIGS (MAX_ENCODER_ID + 1)
#define DEFAULT_HOST_SERIAL_BAUD_RATE 112500 // Baud rate of serial communication with host

//TODO: Write unit test to always check that this is valid
#define FIXED_RECEIVED_MESSAGE_LENGTH 24 // The number of bytes of a message received from host
#define RECEIVED_COMMAND_BUFFER_SIZE (FIXED_RECEIVED_MESSAGE_LENGTH * 2) // Size of commands buffer, data comes from host

#define FIXED_SEND_MESSAGE_LENGTH 24 // The number of bytes of a message to send to host
#define SEND_COMMAND_BUFFER_SIZE (FIXED_SEND_MESSAGE_LENGTH) // Size of buffer for data that goes to host

// Number of times deserialization can occur before robot motors are turned off
#define MAX_DESERIALIZATION_ERRORS 8

// Debug functionality message defines
#define MAX_DEBUG_STRING_SIZE_BYTES 6

// TODO: Figure out how to alias the Serial_Message_To_Jetson type to decrease clutter.
// https://learn.microsoft.com/en-us/cpp/cpp/aliases-and-typedefs-cpp?view=msvc-170
// using Serial_Message_To_Jetson = Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>;

/**
 * The MotorDriver class is responsible for keeping track of all in use
 * encoders, motors, PID loops, and other devices/services that would be
 * used with the arduino. It is responsible for configuring these resources
 * by processing the serial commands that are received from the host device.
 * The MotorDriver class is written to handle message errors such as corrupted
 * data and invalid commands.
 * 
 * @attention Must call initMotorDriver() function in arduino setup() function
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
 * back of all received commands.
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
    MotorDriver(unsigned int serialTransferBaudRate, std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> motor_configs, std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS> encoder_configs);
    MotorDriver(unsigned int serialTransferBaudRate, std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> motor_configs);
    MotorDriver(unsigned int serialTransferBaudRate);
    MotorDriver();
 
    // An init function allows user to update internal state of motor driver before it connects to attached devices.
    bool initMotorDriver(); 
    SabertoothOperator getConfig(unsigned int motorID);
    void setConfig(unsigned int motorID, SabertoothOperator config);
    void resetConfigs();
    void setDebugMode(bool enabled);

    void update();

    // helpers for update
    // These are public because otherwise they can't be unit tested
    // A more proper solution is to use Unity CMock in unit tests and move these methods to private
    unsigned int read();
    EmbeddedProto::Error parse(Serial_Message_To_Arduino& deserialized_message, EmbeddedProto::ReadBufferFixedSize<RECEIVED_COMMAND_BUFFER_SIZE>& buffer);
    void execute(Serial_Message_To_Arduino& deserialized_message);

    // helpers for sending messages from Arduino to the Jetson
    static bool send_message(const Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>& message_to_jetson);

  private:
    unsigned int serialTransferBaudRate;
    std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> motor_configs; // contains configs of sabertooths
    std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS> encoder_configs;
    EmbeddedProto::ReadBufferFixedSize<RECEIVED_COMMAND_BUFFER_SIZE> receive_command_buffer; //Operates on uint8
    bool debug_mode_enabled;
    unsigned int num_deserialization_errors;

    // Preallocated memory
    static Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES> proto_send_message;

    // Sending message from Arduino to the Jetson
    static EmbeddedProto::WriteBufferFixedSize<SEND_COMMAND_BUFFER_SIZE> send_command_buffer;
};
#endif