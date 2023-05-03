#include "Motor_PID.h"

pid_data_t motor1_pid_data;
pid_data_t motor2_pid_data;
pid_data_t motor3_pid_data;
pid_data_t motor4_pid_data;

/* 电机1最外层pid控制参数 */
pid_paramer_t motor1_pid_paramer = {
    .integrate_max = 7000,
    .kp = 77.400002,
    .ki = 9,
    .kd = 2.5,
    .control_output_limit = 9000
};

/* 电机2最外层pid控制参数 */
pid_paramer_t motor2_pid_paramer = {
    .integrate_max = 7000,
    .kp = 77.400002,
    .ki = 9,
    .kd = 2.5,
    .control_output_limit = 9000
};

/* 电机3最外层pid控制参数 */
pid_paramer_t motor3_pid_paramer = {
    .integrate_max = 7000,
    .kp = 77.400002,
    .ki = 9,
    .kd = 2.5,
    .control_output_limit = 9000
};

/* 电机4最外层pid控制参数 */
pid_paramer_t motor4_pid_paramer = {
    .integrate_max = 7000,
    .kp = 77.400002,
    .ki = 9,
    .kd = 2.5,
    .control_output_limit = 9000
};

static uint16_t p_raw = 774;
static uint16_t i_raw = 90;
static uint16_t d_raw = 25;
static uint16_t num_raw = 5;

void pid_changenum(uint16_t num_in)
{
	num_raw = num_in;
	pid_change(num_raw,p_raw,i_raw,d_raw);
}

void pid_changed(uint16_t d_in)
{
	d_raw = d_in;
	pid_change(num_raw,p_raw,i_raw,d_raw);
}


void pid_changei(uint16_t i_in)
{
	i_raw = i_in;
	pid_change(num_raw,p_raw,i_raw,d_raw);
}

void pid_changep(uint16_t p_in)
{
	p_raw = p_in;
	pid_change(num_raw,p_raw,i_raw,d_raw);
}


void pid_change(uint16_t num,uint16_t p,uint16_t i,uint16_t d)
{
	set_motor1_pid(num,(float) p / 10,(float) i / 10,(float) d / 10);
	printf("%d,%f,%f,%f\n",num,(float) p / 10,(float) i / 10,(float) d / 10);
}

void set_motor1_pid(uint16_t num,float p,float i,float d)
{
	pid_paramer_t* motor_pid_paramer;
	if(num == 1)
			motor_pid_paramer = &motor1_pid_paramer;
		else if(num == 2)
			motor_pid_paramer = &motor2_pid_paramer;
		else if(num == 3)
			motor_pid_paramer = &motor3_pid_paramer;
		else  if(num == 4)
			motor_pid_paramer = &motor4_pid_paramer;
		else{
		motor_pid_paramer = &motor1_pid_paramer;
		motor_pid_paramer->kp = p;
		motor_pid_paramer->ki = i;
		motor_pid_paramer->kd = d;
	
		motor_pid_paramer = &motor2_pid_paramer;
		motor_pid_paramer->kp = p;
		motor_pid_paramer->ki = i;
		motor_pid_paramer->kd = d;
		
		motor_pid_paramer = &motor3_pid_paramer;
		motor_pid_paramer->kp = p;
		motor_pid_paramer->ki = i;
		motor_pid_paramer->kd = d;		
		motor_pid_paramer = &motor4_pid_paramer;
		motor_pid_paramer->kp = p;
		motor_pid_paramer->ki = i;
		motor_pid_paramer->kd = d;
		}
		motor_pid_paramer->kp = p;
		motor_pid_paramer->ki = i;
		motor_pid_paramer->kd = d;
	
}

/**********************************************************************
 * @Name    motor_pid_data_init
 * @declaration : 电机pid初始化
 * @param   motor_pid_data 电机pid结构体
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void motor_pid_data_init(pid_data_t *motor_pid_data)
{
    motor_pid_data->expect = 0;
    motor_pid_data->feedback = 0;

    motor_pid_data->err = 0;
    motor_pid_data->last_err = 0;
    motor_pid_data->last2_err=0;
    motor_pid_data->integrate = 0;
    motor_pid_data->delta = 0;
    motor_pid_data->dis_err = 0;

    motor_pid_data->control_output = 0;

    motor_pid_data->short_circuit_flag = 0;

}


void Motor_PID_Init(void)
{
	motor_pid_data_init(&motor1_pid_data);
	motor_pid_data_init(&motor2_pid_data);
	motor_pid_data_init(&motor3_pid_data);
	motor_pid_data_init(&motor4_pid_data);
}

/**********************************************************************
 * @Name    pid_positional
 * @declaration : pid控制器计算
 * @param   date 进行计算的数据及结构体  para  pid参数
 * @retval   : data->control_output pid计算结果
 * @author  hoson_stars
 ***********************************************************************/
float pid_positional(pid_data_t *data, pid_paramer_t *para)
{
	float controller_dt;
    //短路直接输出期待值
	if (data->short_circuit_flag) {
		data->control_output = data->expect;
		return data->control_output;
	}
	//获取dt
//	Get_Time_Period(&data->pid_controller_dt);
//	controller_dt = data->pid_controller_dt.Time_Delta / 1000000.0;
	//第一次计算间隔时间将出现间隔时间很大的情况
	if (controller_dt < 0.001f)
		return 0;
	//保存上次偏差
	data->last_err = data->err;
	//期望减去反馈得到偏差			  
	data->err = data->expect - data->feedback;
	//计算偏差微分
	data->dis_err = data->err - data->last_err;
	//自定义偏差微分处理
//	if (data->err_callback)
//		data->err_callback(data, para);
	//积分限幅
	if (para->integrate_max) {
		if (data->integrate >= para->integrate_max)
			data->integrate = para->integrate_max;
		if (data->integrate <= -para->integrate_max)
			data->integrate = -para->integrate_max;
	}
    data->integrate += para->ki * data->err * controller_dt;
	//总输出计算
	data->control_output = para->kp * data->err
		+ data->integrate
		+ para->kd * data->dis_err;
	//总输出限幅
	if (para->control_output_limit) {
		if (data->control_output >= para->control_output_limit)
			data->control_output = para->control_output_limit;
		if (data->control_output <= -para->control_output_limit)
			data->control_output = -para->control_output_limit;
	}
	//返回总输出
	return data->control_output;
}


float  Iout = 0;
float  Pout = 0;
float  Dout = 0;

/**********************************************************************
 * @Name    pid_positional
 * @declaration : pid控制器计算(增量式  用于电机行走最外层pid)
 * @param   date 进行计算的数据及结构体  para  pid参数
 * @retval   : data->control_output pid计算结果
 * @author  hoson_stars
 ***********************************************************************/
float pid_incremental(pid_data_t *data, pid_paramer_t *para)
{
    float controller_dt;
    //短路直接输出期待值
    if (data->short_circuit_flag)
    {
        data->control_output = data->expect;
        return data->control_output;
    }
    //获取dt
//   Get_Time_Period(&data->pid_controller_dt);
//   controller_dt = data->pid_controller_dt.Time_Delta / 1000000.0;
    //第一次计算间隔时间将出现间隔时间很大的情况
//    if (controller_dt < 0.001f)
//        return 0;
    //开始进行增量式计算
    data->last2_err = data->last_err;
    data->last_err = data->err;
    data->err =  data->expect - data->feedback;

    Pout = para->kp * (data->err - data->last_err);
    Iout = para->ki * data->err;
    Dout = para->kd * (data->err - 2.0f * data->last_err + data->last2_err);

    data->delta = Pout + Iout + Dout;
    data->control_output += data->delta;

    if (para->control_output_limit)
    {
        if (data->control_output >= para->control_output_limit)
            data->control_output = para->control_output_limit;
        if (data->control_output <= -para->control_output_limit)
            data->control_output = -para->control_output_limit;
    }
    
    if(data->expect==0 &&data->err==0) {
        data->control_output = 0;
    } 
    //返回总输出
    return data->control_output;
}

