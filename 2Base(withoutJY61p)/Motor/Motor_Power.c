#include "Motor_Power.h"
#include "Motor_PID.h"

#define CHASSIS_RADIUS 21.0         //车体中心到轮子轴心距离
#define WHEEL_ANGLE 45            //全向轮小轮毂与车体坐标系x轴夹角
#define RADIAN 57.3                 //单位弧度对应的角度
#define MAX_SPEED 250               //限制电机运转最大速度
#define TIME_PARAM 2               //等待时间

#define ENCODE_THRESHOLD 20         //编码器计数路径阈值
#define ENCODER_FACTOR 4            //编码器计数P环放大因子

motor_t motor1;
motor_t motor2;
motor_t motor3;
motor_t motor4;


/**********************************************************************
 * @Name    motor1_Enable
 * @declaration : 初始化motor1结构体   并使能其对应时钟与中断
 * @param   motor  电机结构体  
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void motor1_Enable(motor_t *motor)
{
    motor->htim_pwm = &htim1;
    motor->channel_A = TIM_CHANNEL_2;
    motor->channel_B = TIM_CHANNEL_1;
    motor->speed = 0;

    motor->htim_ic = &htim5;
    motor->ic_channel = TIM_CHANNEL_1;
    motor->IC_GPIO_Port_v = GPIOA;
    motor->IC_Pin_v = GPIO_PIN_1;

    motor->updata = 0;
    motor->freq = 0 ;

    HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel_A);
    HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel_B);
    HAL_TIM_IC_Start_IT(motor->htim_ic, motor->ic_channel);
}

/**********************************************************************
 * @Name    motor2_Enable
 * @declaration : 初始化motor2结构体   并使能其对应时钟与中断
 * @param   motor  电机结构体  
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void motor2_Enable(motor_t *motor)
{
    motor->htim_pwm = &htim1;
    motor->channel_A = TIM_CHANNEL_3;
    motor->channel_B = TIM_CHANNEL_4;
    motor->speed = 0;

    motor->htim_ic = &htim5;
    motor->ic_channel = TIM_CHANNEL_3;
    motor->IC_GPIO_Port_v = GPIOA;
    motor->IC_Pin_v = GPIO_PIN_3;

    motor->updata = 0;
    motor->freq = 0;

    HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel_A);
    HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel_B);
    HAL_TIM_IC_Start_IT(motor->htim_ic, motor->ic_channel);
}

/**********************************************************************
 * @Name    motor3_Enable
 * @declaration : 初始化motor3结构体   并使能其对应时钟与中断
 * @param   motor  电机结构体  
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void motor3_Enable(motor_t *motor)
{
    motor->htim_pwm = &htim2;
    motor->channel_A = TIM_CHANNEL_3;
    motor->channel_B = TIM_CHANNEL_4;
    motor->speed = 0;

    motor->htim_ic = &htim3;
    motor->ic_channel = TIM_CHANNEL_1;
    motor->IC_GPIO_Port_v = GPIOA;
    motor->IC_Pin_v = GPIO_PIN_7;

    motor->updata = 0;
    motor->freq = 0;

    HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel_A);
    HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel_B);
    HAL_TIM_IC_Start_IT(motor->htim_ic, motor->ic_channel);
}

/**********************************************************************
 * @Name    motor4_Enable
 * @declaration : 初始化motor4结构体   并使能其对应时钟与中断
 * @param   motor  电机结构体  
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void motor4_Enable(motor_t *motor)
{
    motor->htim_pwm = &htim2;
    motor->channel_A = TIM_CHANNEL_2;
    motor->channel_B = TIM_CHANNEL_1;
    motor->speed = 0;

    motor->htim_ic = &htim3;
    motor->ic_channel = TIM_CHANNEL_3;
    motor->IC_GPIO_Port_v = GPIOB;
    motor->IC_Pin_v = GPIO_PIN_1;

    motor->updata = 0;
    motor->freq = 0;

    HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel_A);
    HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel_B);
    HAL_TIM_IC_Start_IT(motor->htim_ic, motor->ic_channel);
}

void Motor_Init(void)
{
	motor1_Enable(&motor1);
	motor2_Enable(&motor2);
	motor3_Enable(&motor3);
	motor4_Enable(&motor4);
	
	
}

/**********************************************************************
 * @Name    set_motor_speed
 * @declaration : 将pwm_pluse赋值给motor结构体中的speed  并向其通道注入动力
 * @param   motor  motor结构体的地址 speed 动力
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void set_motor_speed(motor_t *motor, int32_t speed)
{
    int32_t ccr = 0;

    if (speed >= 0)
    {
        if (speed > MOTOR_SPEED_MAX)
            ccr = MOTOR_SPEED_MAX;
        else
            ccr = speed;

        motor->speed = ccr;

        __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel_A, motor->speed);
        __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel_B, 0);
    }

    else if (speed < 0)
    {
        if (speed < -MOTOR_SPEED_MAX)
            ccr = MOTOR_SPEED_MAX;
        else
            ccr = -speed;

        motor->speed = ccr;

        __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel_B, motor->speed);
        __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel_A, 0);
    }
}


static float x_speed = 0;
static float y_speed = 0;
static float z_speed = 0;


double motor_target[5];              //存储四个电机速度的目标值

void Motor_Power(void)
{
	double x_temp,y_temp,z_temp;
	x_temp = x_speed;
	y_temp = y_speed;
	z_temp = z_speed;
	
	motor_target[1] = -cos(WHEEL_ANGLE/RADIAN) * x_temp + sin(WHEEL_ANGLE/RADIAN) * y_temp - z_temp;
    motor_target[2] = cos(WHEEL_ANGLE/RADIAN) * x_temp + sin(WHEEL_ANGLE/RADIAN) * y_temp + z_temp;
    motor_target[3] = cos(WHEEL_ANGLE/RADIAN) * x_temp + sin(WHEEL_ANGLE/RADIAN) * y_temp - z_temp;
    motor_target[4] = -cos(WHEEL_ANGLE/RADIAN) * x_temp + sin(WHEEL_ANGLE/RADIAN) * y_temp + z_temp;
	
	motor1_pid_data.expect = motor_target[1];
	motor2_pid_data.expect = motor_target[2];
	motor3_pid_data.expect = motor_target[3];
	motor4_pid_data.expect = motor_target[4];
	
    motor1_pid_data.feedback = -read_freq(&motor1);	
    motor2_pid_data.feedback = read_freq(&motor2);	
    motor3_pid_data.feedback = read_freq(&motor3);	
    motor4_pid_data.feedback = -read_freq(&motor4);	
	
	set_motor_speed(&motor1, (int32_t)pid_incremental(&motor1_pid_data,&motor1_pid_paramer));
	set_motor_speed(&motor2, (int32_t)pid_incremental(&motor2_pid_data,&motor2_pid_paramer));
	set_motor_speed(&motor3, (int32_t)pid_incremental(&motor3_pid_data,&motor3_pid_paramer));
	set_motor_speed(&motor4, (int32_t)pid_incremental(&motor4_pid_data,&motor4_pid_paramer));

}

void Set_Chassis_Speed(float x,float y,float z)
{
	x_speed = x;
	y_speed = y;
	z_speed = z;
}

void Move_Direct()
{
	
}

