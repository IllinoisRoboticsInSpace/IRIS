#ifndef _MOTOR_DRIVER_
#define _MOTOR_DRIVER_

#include <Arduino.h>
#undef min
#undef max
#include <array>

// Include both operator types and other dependencies.
#include "Sabertooth.h"
#include "SabertoothOperator.h"
#include "CytronOperator.h"    
#include "RotaryEncoderOperator.h"
#include "WriteBufferFixedSize.h"
#include "ReadBufferFixedSize.h"

#define MAX_MOTOR_ID 15 // Maximum number of motor ids 0 indexed
#define MAX_MOTOR_CONFIGS (MAX_MOTOR_ID + 1)

#define MAX_ENCODER_ID 14 // Maximum number of encoders ids 0 indexed
#define MAX_ENCODER_CONFIGS (MAX_ENCODER_ID + 1)
#define DEFAULT_HOST_SERIAL_BAUD_RATE 112500 // Baud rate of serial communication with host

// Predefined lengths for messaging
#define FIXED_RECEIVED_MESSAGE_LENGTH 16 // The number of bytes of a message received from host
#define RECEIVED_COMMAND_BUFFER_SIZE (FIXED_RECEIVED_MESSAGE_LENGTH * 2)
#define FIXED_SEND_MESSAGE_LENGTH 16 
#define SEND_COMMAND_BUFFER_SIZE (FIXED_SEND_MESSAGE_LENGTH)

// Debug functionality defines
#define MAX_DEBUG_STRING_SIZE_BYTES 6

/**
 * MotorDriver is responsible for managing motor and encoder resources.
 * It now supports both Sabertooth and Cytron motor driver types.
 */
class MotorDriver
{
  public:
    // Constructors – note that motor_configs now refer to Sabertooth configurations.
    MotorDriver(unsigned int serialTransferBaudRate,
                std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> configs);
    MotorDriver(unsigned int serialTransferBaudRate);
    MotorDriver();
 
    // Initialize communication lines and all configured devices.
    bool initMotorDriver(); 

    // Get and set Sabertooth motor config (if needed).
    SabertoothOperator getConfig(unsigned int motorID);
    void setConfig(unsigned int motorID, SabertoothOperator config);
    CytronOperator getCytronConfig(unsigned int motorID);
    void setCytronConfig(unsigned int motorID, CytronOperator config);
    void resetConfigs();
    void setDebugMode(bool enabled);

    // Main update loop.
    void update();

    // Helpers for reading, parsing, and executing commands.
    unsigned int read();
    EmbeddedProto::Error parse(Serial_Message_To_Arduino& deserialized_message,
                               EmbeddedProto::ReadBufferFixedSize<RECEIVED_COMMAND_BUFFER_SIZE>& buffer);
    void execute(Serial_Message_To_Arduino& deserialized_message);

    // Helper for sending messages back to the host.
    static bool send_message(const Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>& message_to_jetson);

  private:
    unsigned int serialTransferBaudRate;
    // Existing Sabertooth configs.
    std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> motor_configs;
    std::array<CytronOperator, MAX_MOTOR_CONFIGS> cytron_configs;
    
    std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS> encoder_configs;
    EmbeddedProto::ReadBufferFixedSize<RECEIVED_COMMAND_BUFFER_SIZE> receive_command_buffer;
    bool debug_mode_enabled;

    // Preallocated message buffer.
    Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES> proto_send_message;
    static EmbeddedProto::WriteBufferFixedSize<SEND_COMMAND_BUFFER_SIZE> send_command_buffer;
};

#endif
