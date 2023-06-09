#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include "tim.h"

/* 电机信息结构体数据 */
typedef struct
{
    TIM_HandleTypeDef *htim_pwm;          //提供PWM定时器
    uint32_t channel_A;                   //PWM_A通道
    uint32_t channel_B;					          //PWM_B通道
    int32_t speed;                        //PWM波值

    TIM_HandleTypeDef *htim_ic;           //输入捕获定时器
    uint32_t ic_channel;				          //输入捕获通道
    GPIO_TypeDef *IC_GPIO_Port_v;         //检测电平变化GPIO口
    uint16_t IC_Pin_v;

    uint16_t updata;                      //定时器计数溢出次数统计
    int32_t freq;                         //转速
} motor_t;

extern motor_t motor1;
extern motor_t motor2;
extern motor_t motor3;
extern motor_t motor4;

#define MOTOR3_ENCODER_Pin GPIO_PIN_1
#define MOTOR3_ENCODER_GPIO_Port GPIOA
#define MOTOR4_ENCODER_Pin GPIO_PIN_3
#define MOTOR4_ENCODER_GPIO_Port GPIOA
#define MOTOR1_ENCODER_Pin GPIO_PIN_7
#define MOTOR1_ENCODER_GPIO_Port GPIOA
#define MOTOR2_ENCODER_Pin GPIO_PIN_1
#define MOTOR2_ENCODER_GPIO_Port GPIOB

/* 电机初始化与使能 */
void motor1_Enable(motor_t *motor);
void motor2_Enable(motor_t *motor);
void motor3_Enable(motor_t *motor);
void motor4_Enable(motor_t *motor);
void set_motor1val(uint16_t val);
void set_motor(uint16_t speed);

void car_go(void);
void car_left(void);
void car_rightback(void);
void car_turn(void);

#endif
