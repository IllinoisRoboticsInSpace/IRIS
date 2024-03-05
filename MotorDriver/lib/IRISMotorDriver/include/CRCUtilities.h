#include <CRC8.h>
#include "generated/commands.h"
#include "DebugTools.h"
#include "MotorDriver.h"


class ChecksumUtil{
    public:
    ChecksumUtil(uint8_t polynome, uint8_t XORstart, uint8_t XORend, bool reverseIn, bool reverseOut){}

    void convertValues(Serial_Message_To_Arduino& deserialized_message, EmbeddedProto::ReadBufferFixedSize<COMMAND_BUFFER_SIZE>& buffer){}
    void setLen(uint16_t len){this->length = len;}
    uint8_t Checksum(Serial_Message_To_Arduino& message){}
    uint8_t getLen(){return this->length;}


    private:
    CRC8 CRC;

    static uint8_t length;
    static uint8_t messageConv[];
    int bytes; 
};

