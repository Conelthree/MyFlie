#ifndef __JY61P__
#define __JY61P__

#include "stm32f7xx.h"

#define ATKP_MAX_DATA_SIZE 6

typedef struct
{
    uint8_t startByte1;
    uint8_t startByte2;
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[ATKP_MAX_DATA_SIZE];
    uint8_t checkSum;
} IMU901_t;


#endif
