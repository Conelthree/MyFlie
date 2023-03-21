#include "wheel.h"
#include "wheel_encode.h"
#include "math.h"
#include "wheel_task.h"
//#include "track_task.h"
//#include "imu_task.h"
//#include "time_cnt.h"
#include "stdio.h"

#define CHASSIS_RADIUS 21.0         //车体中心到轮子轴心距离
#define WHEEL_ANGLE 45            //全向轮小轮毂与车体坐标系x轴夹角
#define RADIAN 57.3                 //单位弧度对应的角度
#define MAX_SPEED 250               //限制电机运转最大速度
#define TIME_PARAM 2               //等待时间

#define ENCODE_THRESHOLD 20         //编码器计数路径阈值
#define ENCODER_FACTOR 4            //编码器计数P环放大因子

uint32_t time = 0;                  //获取的系统运行时间
unsigned short int  time_count=0;   //计数
chassis_t chassis_param;            //定义底盘参数

extern motor_t motor1;
extern motor_t motor2;
extern motor_t motor3;
extern motor_t motor4;
//extern ATTITUDE_t  attitude;

double motor_target[5];              //存储四个电机速度的目标值
int val_track_row = 0;				//保存循迹pid的输出值
int val_track_vertical = 0; 
int val_imu = 0;                //保存陀螺仪pid的输出值

//extern uint8_t red_mode;
//extern uint8_t blue_mode;

/**********************************************************************
 * @Name    set_chassis_speed
 * @declaration : 直接对目标速度值进行修改
 * @param   x y w 对应方向的值
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void set_chassis_speed (float x, float y, float w)
{
    chassis_param.x_speed = x;
    chassis_param.y_speed = y;
    chassis_param.w_speed = w;
}

///************************************************************
//*函 数 名:change_chassis_status
//*功能说明:底盘使能开关
//*形    参:状态，bool值
//*返 回 值:无
//**************************************************************/
//void change_chassis_status(bool status)
//{
//    chassis_param.status = status;
//}

/**********************************************************************
 * @Name    chassis_synthetic_control
 * @declaration : 底盘的综合控制函数，包含多种控制 内层与外层PID
 * @param   None
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
 void chassis_synthetic_control(void)
{
		static long long int cnt=0;
		cnt++;
//    int i;
    double x, y, w; 
//   double max_val, factor;
//    if(chassis_param.status == false) return;//如果底盘不被使能，则没有后续操作

//    max_val = 0;        //对最大值数据进行初始化
//    factor = 1;         //倍率因子初始化

    time_count++;
    if(time_count == TIME_PARAM) {
        time_count = 0;
//			  val_imu=imu_pid(&attitude);
//				val_track_row = track_pid(&row_bar,row_weight);
//				val_track_vertical = track_pid(&vertical_bar,vertical_weight);
				}
		
    x = chassis_param.x_speed;
    y = chassis_param.y_speed;
    w = chassis_param.w_speed + val_imu;
        
    /***************************************
     * motor1  前方右侧
     * motor2  后方右侧
     * motor3  前方左侧
     * motor4  后方左侧
    ****************************************/
//    motor_target[1] = -cos(WHEEL_ANGLE/RADIAN) * x + sin(WHEEL_ANGLE/RADIAN) * y + CHASSIS_RADIUS * w;
//    motor_target[2] = cos(WHEEL_ANGLE/RADIAN) * x + sin(WHEEL_ANGLE/RADIAN) * y + CHASSIS_RADIUS * w;
//    motor_target[3] = -cos(WHEEL_ANGLE/RADIAN) * x - sin(WHEEL_ANGLE/RADIAN) * y + CHASSIS_RADIUS * w;
//    motor_target[4] = cos(WHEEL_ANGLE/RADIAN) * x - sin(WHEEL_ANGLE/RADIAN) * y + CHASSIS_RADIUS * w;

    motor_target[1] = cos(WHEEL_ANGLE/RADIAN) * x + sin(WHEEL_ANGLE/RADIAN) * y - w;
    motor_target[2] = -cos(WHEEL_ANGLE/RADIAN) * x + sin(WHEEL_ANGLE/RADIAN) * y + w;
    motor_target[3] = cos(WHEEL_ANGLE/RADIAN) * x - sin(WHEEL_ANGLE/RADIAN) * y + w;
    motor_target[4] = -cos(WHEEL_ANGLE/RADIAN) * x - sin(WHEEL_ANGLE/RADIAN) * y - w;
//			motor_target[1] = x +  w;
//			motor_target[2] = y +  w;
//			motor_target[3] = x -  w;
//			motor_target[4] = y -  w;
//    
   
    
//    /* 再来一个限幅操作，避免单边速度过高导致控制效果不理想 */
//    for(i = 1; i <= 4; ++i) {                                       //找出最大值
//        if(motor_target[i] > max_val) max_val = motor_target[i];
//    }
//    
//    /*最大值是否超限制，进行操作，确保最大值仍在范围内且转速比例不变*/
//    if(max_val > MAX_SPEED) {             
//        factor = MAX_SPEED / max_val;
//        for(i = 1; i < 4; ++ i) {
//            motor_target[i] *= factor;
//        }
//    }
    
   /*最外层电机pid操作*/
   /*计算电机最终目标值*/
   motor1_pid_data.expect = motor_target[1]; //+ val_track_row;// + val_track_back;
   motor2_pid_data.expect = motor_target[2]; //+ val_track_vertical;// + val_track_back;
   motor3_pid_data.expect = motor_target[3];// + val_track_row;// + val_track_back;
   motor4_pid_data.expect = motor_target[4];// + val_track_vertical;// + val_track_back;
  
   /*读取当前电机转速*/
   motor1_pid_data.feedback = -read_freq(&motor1);
   motor2_pid_data.feedback = read_freq(&motor2);
   motor3_pid_data.feedback = -read_freq(&motor3);
   motor4_pid_data.feedback = read_freq(&motor4);
  
   /*进行pid运算 并将输出值注入电机通道*/
//   float why =0;
	 set_motor_speed(&motor1, pid_incremental(&motor1_pid_data,&motor1_pid_paramer));
   set_motor_speed(&motor2, pid_incremental(&motor2_pid_data,&motor2_pid_paramer));
   set_motor_speed(&motor3, pid_incremental(&motor3_pid_data,&motor3_pid_paramer));
   set_motor_speed(&motor4, pid_incremental(&motor4_pid_data,&motor4_pid_paramer));
//	 why=pid_incremental(&motor3_pid_data,&motor3_pid_paramer);
//	 printf("%f\r\n",why);

//   set_motor_speed(1, pid_incremental(&motor1_pid_data,&motor1_pid_paramer));
//   set_motor_speed(2, pid_incremental(&motor2_pid_data,&motor2_pid_paramer));
//   set_motor_speed(3, pid_incremental(&motor3_pid_data,&motor3_pid_paramer));
//   set_motor_speed(4, pid_incremental(&motor4_pid_data,&motor4_pid_paramer));
   //printf("%f  %f  %f  %f\r\n",motor1_pid_data.feedback ,motor2_pid_data.feedback ,motor3_pid_data.feedback ,motor4_pid_data.feedback);
  // printf("%f,%f\n",motor3_pid_data.feedback,motor3_pid_data.expect);
   //motor_pid();
}


/**********************************************************************
 * @Name    move_by_encoder
 * @declaration : 通过编码器规定路程行走
 * @param   direct 方向 val 路程
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void move_by_encoder(int  direct, int val)
{
    time = TIME_ISR_CNT;            //获取系统时间
    double bias = 0, variation;     //变量声明
    encoder = 0;                    //将编码器累加值置0
    if(direct == 1)
    {
        if(val < 0)//向右
        {
            while(__fabs(val) > encoder )
            {
                if( (TIME_ISR_CNT - time > 100) && ( __fabs( (val - encoder) ) < ENCODE_THRESHOLD)) goto EXIT_FUNC;  //超时处理，避免卡死
                bias =  -(__fabs(val) - encoder);                       //得到差值
                variation = bias * ENCODER_FACTOR;                              //计算得出输出值。P环
                variation = variation < - MAX_SPEED ? -MAX_SPEED : variation;   //限幅
                set_chassis_speed(variation, 0, 0);                             //分配速度
            }
        }
        else
        {

            while(val > encoder)//向左
            {
                if( (TIME_ISR_CNT - time > 100) && ( (val - encoder ) < ENCODE_THRESHOLD) ) goto EXIT_FUNC;
                bias = val - encoder;
                variation = bias * ENCODER_FACTOR;
                variation = variation > MAX_SPEED ? MAX_SPEED : variation;
                set_chassis_speed(variation, 0, 0);
            }
        }
    }
    else if(direct == 2)
    {	
				if(val < 0)//向后
        {
            while(__fabs(val) > encoder )
            {
                if( (TIME_ISR_CNT - time > 100) && ( __fabs( (val - encoder) ) < ENCODE_THRESHOLD)) goto EXIT_FUNC;  //超时处理，避免卡死
                bias =  -(__fabs(val) - encoder);                       //得到差值
                variation = bias * ENCODER_FACTOR;                              //计算得出输出值。P环
                variation = variation < - MAX_SPEED ? -MAX_SPEED : variation;   //限幅
                set_chassis_speed(0, variation, 0);                             //分配速度
            }
        }
        else  //向前
        {

            while(val > encoder)
            {
                if( (TIME_ISR_CNT - time > 100) && ( (val - encoder ) < ENCODE_THRESHOLD) ) goto EXIT_FUNC;
                bias = val - encoder;
                variation = bias * ENCODER_FACTOR;
                variation = variation > MAX_SPEED ? MAX_SPEED : variation;
                set_chassis_speed(0, variation, 0);
            }
        }
				
    }
    
    EXIT_FUNC:
        set_chassis_speed(0, 0, 0);//停车
}








