#ifndef __MOTOR_POWER_H__
#define __MOTOR_POWER_H__

#include "tim.h"
#include "math.h"
#include "stdio.h"

//定时器占空比最大值
#define MOTOR_SPEED_MAX 10000
//电机参数结构体
typedef struct
{
    TIM_HandleTypeDef *htim_pwm;		//驱动电机的TIM
    uint32_t channel_A;					//PWM输出A相
    uint32_t channel_B;					//PWM输出B相
    int32_t speed;						//定时器占空比(0~10000)

    TIM_HandleTypeDef *htim_ic;			//输入捕获的TIM
    uint32_t ic_channel;				//输入捕获通道(编码器A相)
    GPIO_TypeDef *IC_GPIO_Port_v;		//读取电机方向引脚对应port
	uint16_t IC_Pin_v;					//读取电机方向引脚(编码器B相)
	
    uint16_t updata;					//输入捕获计TIM中断计数
	int32_t freq;						//输出频率
} motor_t;

extern motor_t motor1;
extern motor_t motor2;
extern motor_t motor3;
extern motor_t motor4;


static void motor1_Enable(motor_t *motor);
static void motor2_Enable(motor_t *motor);
static void motor3_Enable(motor_t *motor);
static void motor4_Enable(motor_t *motor);

void Motor_Init(void);

static void set_motor_speed(motor_t *motor, int32_t speed);
void Motor_Power(void);
void Set_Chassis_Speed(float x,float y,float z);


static void TIM3CaptureChannel1Callback(void);
static void TIM3CaptureChannel3Callback(void);
static void TIM5CaptureChannel1Callback(void);
static void TIM5CaptureChannel3Callback(void);

static void TIM3_IcOverflowCntCallback(void);
static void TIM5_IcOverflowCntCallback(void);

int32_t read_freq(motor_t *motor);

extern int distence;

#endif
