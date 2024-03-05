#include "CRCUtilities.h"

ChecksumUtil::ChecksumUtil(uint8_t polynome, uint8_t XORstart, uint8_t XORend, bool reverseIn, bool reverseOut){
    CRC.reset();
    ChecksumUtil::CRC = CRC8(polynome, XORstart, XORend, reverseIn, reverseOut);

    ChecksumUtil::setLen(CRC.getEndXOR() - CRC.getStartXOR());
}

void ChecksumUtil::convertValues(Serial_Message_To_Arduino& deserialized_message, EmbeddedProto::ReadBufferFixedSize<COMMAND_BUFFER_SIZE>& buffer){ //grab uint8_t converted values from message
    ChecksumUtil::bytes = Serial.readBytes(ChecksumUtil::messageConv, ChecksumUtil::getLen());
}

uint8_t ChecksumUtil::Checksum(Serial_Message_To_Arduino& message){
    for(int i = 0; i < ChecksumUtil::getLen(); i++){
        CRC.add(ChecksumUtil::messageConv[i]);
    }
    return CRC.getCRC();
}