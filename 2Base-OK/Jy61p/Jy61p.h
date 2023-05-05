#ifndef __JY61P__
#define __JY61P__

#include <stdbool.h>
#include "usart.h"
#include "Motor_PID.h"

#define ATKP_MAX_DATA_SIZE 9

typedef struct
{
	float roll;
    float pitch;
    float yaw;
	float target_angle;
    float refer_angle;
    bool  status;
} ATTITUDE_t;


typedef struct
{
    uint8_t msgID;
    uint8_t data[ATKP_MAX_DATA_SIZE];
    uint8_t checkSum;
	ATTITUDE_t angle;
} JY61PData_t;

extern JY61PData_t Jy61pData;

void Jy61p_Init(void);
void receive_imu901_IRQ(void);
void Jy61p_Enable(void);
void Jy61p_Disable(void);
void imu_calibration(void);
int16_t Jy61p_PIDout(void);

#endif
