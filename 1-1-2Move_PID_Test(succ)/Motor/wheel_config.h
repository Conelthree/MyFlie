#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include "tim.h"

/* �����Ϣ�ṹ������ */
typedef struct
{
    TIM_HandleTypeDef *htim_pwm;          //�ṩPWM��ʱ��
    uint32_t channel_A;                   //PWM_Aͨ��
    uint32_t channel_B;					          //PWM_Bͨ��
    int32_t speed;                        //PWM��ֵ

    TIM_HandleTypeDef *htim_ic;           //���벶��ʱ��
    uint32_t ic_channel;				          //���벶��ͨ��
    GPIO_TypeDef *IC_GPIO_Port_v;         //����ƽ�仯GPIO��
    uint16_t IC_Pin_v;

    uint16_t updata;                      //��ʱ�������������ͳ��
    int32_t freq;                         //ת��
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

/* �����ʼ����ʹ�� */
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
