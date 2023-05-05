#ifndef __MOTOR_PID_H__
#define __MOTOR_PID_H__

#include "stm32f7xx.h"
#include <stdio.h>
typedef struct {
	//期望
    volatile float expect;
	//频率反馈
    float feedback;
	//积分值
    float integrate;
	
	float err;
	
	float last_err;
	
	float last2_err;
	//增量式增量
	float delta;
    //两次误差的增量
    float dis_err;
	//限幅后输出值
    float control_output;
	//
//    Testime pid_controller_dt;
	//
    uint8_t short_circuit_flag;
 } pid_data_t;


typedef struct {

    float integrate_max;

    float kp;

    float ki;

    float kd;

    float control_output_limit;
} pid_paramer_t;

extern pid_data_t motor1_pid_data;
extern pid_data_t motor2_pid_data;
extern pid_data_t motor3_pid_data;
extern pid_data_t motor4_pid_data;

extern pid_paramer_t motor1_pid_paramer;
extern pid_paramer_t motor2_pid_paramer;
extern pid_paramer_t motor3_pid_paramer;
extern pid_paramer_t motor4_pid_paramer;

static void motor_pid_data_init(pid_data_t *motor_pid_data);

void Motor_PID_Init(void);
float pid_incremental(pid_data_t *data, pid_paramer_t *para);
float pid_positional(pid_data_t *data, pid_paramer_t *para);

static void set_motor1_pid(uint16_t num,float p,float i,float d);
static void pid_change(uint16_t num,uint16_t p,uint16_t i,uint16_t d);

void pid_changep(uint16_t p_in);
void pid_changei(uint16_t i_in);
void pid_changed(uint16_t d_in);

#endif
