#include "CRCUtilities.h"

ChecksumUtil::ChecksumUtil(uint8_t polynome, uint8_t XORstart, uint8_t XORend, bool reverseIn, bool reverseOut){
    CRC.reset();
    ChecksumUtil::CRC = CRC8(polynome, XORstart, XORend, reverseIn, reverseOut);

    ChecksumUtil::setLen(CRC.getEndXOR() - CRC.getStartXOR());
}

void ChecksumUtil::convertValues(){ //grab uint8_t converted values from Serial
    Serial.readBytes(ChecksumUtil::bytes, FIXED_RECEIVED_MESSAGE_LENGTH); //only read the message length
    int n = 0;
    for(int i = FIXED_RECEIVED_MESSAGE_LENGTH - ChecksumUtil::getLen(); i < FIXED_RECEIVED_MESSAGE_LENGTH; i++){
        messageConv[n] = bytes[i];
        n++;
    }
}

uint8_t ChecksumUtil::Checksum(){
    for(int i = 0; i < ChecksumUtil::getLen(); i++){
        CRC.add(ChecksumUtil::messageConv[i]);
    }
    return CRC.getCRC();
}