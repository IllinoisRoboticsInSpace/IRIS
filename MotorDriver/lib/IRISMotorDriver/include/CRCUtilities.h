#include <CRC8.h>
#include <CRC.h>
#include "MotorDriver.h"


class ChecksumUtil{
    public:
    ChecksumUtil(uint8_t polynome, uint8_t XORstart, uint8_t XORend, bool reverseIn, bool reverseOut);
    ChecksumUtil(uint8_t XORstart, uint8_t XORend);

    void convertValues();
    void setLen(uint16_t len){this->length = len;}
    uint8_t Checksum();
    uint8_t getLen(){return this->length;}


    private:
    CRC8 CRC;

    uint8_t length;
    static uint8_t messageConv[];
    static uint8_t bytes[]; 
};

