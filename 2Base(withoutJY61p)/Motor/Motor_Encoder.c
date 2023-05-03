#include "Motor_Power.h"

#define TIM_PRESCALER 108-1  //分频
#define MAX_TIM_CNT 65535    //定时器最大计数值
#define TIM_IC_FREQ 1000000  //频率
#define SPEED_PARAM TIM_IC_FREQ/20    //速度因子
#define FILTER 4            //均值滤波 采样次数    
 
#define IC_RISE_EDGE 0     //上升沿
#define IC_FALL_EDGE 1     //下降沿
#define FORWARD    1         //前进方向
#define BACKWARD  -1        //后退方向

#define POLE 2               //MG540电机旋转磁场的极对数



/**********************************************************************
 * @Name    TIM5CaptureChannel1Callback
 * @declaration : 通过输入捕获功能计算电机1转速与频率  
 * @param   None    
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void TIM5CaptureChannel1Callback()
{
    static uint8_t ic_edge = IC_RISE_EDGE;
    static uint32_t cap_val_1 = 0;             //第一次上升沿CCR寄存器内值
    static uint32_t cap_val_2 = 0;             //第二次上升沿CCR寄存器内值
    static int32_t sum_speed = 0;              //均值滤波采样速度值总和
    static int32_t single_speed = 0;           //单个速度值
    static int32_t average_speed = 0;          //滤波处理后的平均速度
    static int32_t last_average_speed = 0;     //存储上一次滤波得出的平均速度
    static uint32_t cap_cnt = 0;               //记录采样次数
    static int8_t direct = 0;                  //方向
    static uint8_t flag_first = 0;              //标志是否是第一次滤波

    if(ic_edge == IC_RISE_EDGE)                //当检测到上升沿
    {
        motor1.updata = 0;                     //更新中断事件记录
        cap_val_1 = HAL_TIM_ReadCapturedValue(motor1.htim_ic, motor1.ic_channel);   //读取TIM5 CCR1的值
        ic_edge = IC_FALL_EDGE;                                                     //等待下降沿
        
        /* 检测另一端GPIO口的电平变化来判断方向*/
        if(HAL_GPIO_ReadPin(motor1.IC_GPIO_Port_v, motor1.IC_Pin_v) == GPIO_PIN_RESET)
        {
            direct = -BACKWARD;
        }
        else
        {
            direct = -FORWARD;
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(motor1.htim_ic, motor1.ic_channel, TIM_ICPOLARITY_FALLING);     //转换输入捕获极性为下降沿
    }
    
    else                                                                                              //检测到下降沿
    {
        cap_val_2 = HAL_TIM_ReadCapturedValue(motor1.htim_ic, motor1.ic_channel);                     //再次读取TIM5 CCR1的值
        if(motor1.updata == 0)                                                                        //若两次捕获之间计数器未溢出
        {
            single_speed = SPEED_PARAM / (cap_val_2 - cap_val_1) * direct;
        }
        else if(motor1.updata == 1)                                                                   //若两次捕获之间溢出一个计数周期
        {
            single_speed = SPEED_PARAM / ((MAX_TIM_CNT - cap_val_1) + cap_val_2) * direct;
        }
        else                                                                                          //若两次捕获之间溢出N个计数周期
        {
            single_speed = SPEED_PARAM / ((MAX_TIM_CNT - cap_val_1) + MAX_TIM_CNT * (motor1.updata - 1) + cap_val_2) * direct;
        }
        
        sum_speed += single_speed;                              //采样速度累加
        cap_cnt++;                                              //累积采样次数
        
        if(cap_cnt == FILTER) {                                 //采样完成                                   
           cap_cnt = 0;                                     
           last_average_speed = average_speed;                  //保存上一次滤波后的速度值  
           if(flag_first == 0) {
               average_speed = (sum_speed+last_average_speed) / FILTER; //第一次滤波时 last_average_speed=0
               flag_first = 1;
           }
           else {
               average_speed = (sum_speed-single_speed+last_average_speed) / FILTER;    //往后的滤波都舍弃最后一个采集的速度 替换为上一次滤波后的速度
           }
           
           if(!(__fabs(average_speed + last_average_speed) < 5)) {              //若上一次与这一次差值过大并且相加后接近0 说明方向值发生跳变
                motor1.freq =  average_speed ;
           } 
           cap_cnt = 0;             //采样处理完后  将统计采样次数的值清0 等待下一次采样处理
           sum_speed = 0;           //采样处理完后  将采样总和的值清0 等待下一次采样处理
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(motor1.htim_ic, motor1.ic_channel, TIM_ICPOLARITY_RISING);      //转换输入捕获极性为上升沿，等待下一次采样
        ic_edge = IC_RISE_EDGE;     //等待下一个周期的第一个上升沿
    }
}

/**********************************************************************
 * @Name    TIM5CaptureChannel3Callback
 * @declaration : 通过输入捕获功能计算电机2转速与频率  
 * @param   None    
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void TIM5CaptureChannel3Callback()
{
    static uint8_t ic_edge = IC_RISE_EDGE;
    static uint32_t cap_val_1 = 0;             //第一次上升沿CCR寄存器内值
    static uint32_t cap_val_2 = 0;             //第二次上升沿CCR寄存器内值
    static int32_t sum_speed = 0;              //均值滤波采样速度值总和
    static int32_t single_speed = 0;           //单个速度值
    static int32_t average_speed = 0;          //滤波处理后的平均速度
    static int32_t last_average_speed = 0;     //存储上一次滤波得出的平均速度
    static uint32_t cap_cnt = 0;               //记录采样次数
    static int8_t direct = 0;                  //方向
    static uint8_t flag_first = 0;             //标志是否是第一次滤波

    if(ic_edge == IC_RISE_EDGE)                //当检测到上升沿
    {
        motor2.updata = 0;                     //更新中断事件记录
        cap_val_1 = HAL_TIM_ReadCapturedValue(motor2.htim_ic, motor2.ic_channel);   //读取TIM5 CCR3的值
        ic_edge = IC_FALL_EDGE;                                                     //等待下降沿

        /* 检测另一端GPIO口的电平变化来判断方向*/
        if(HAL_GPIO_ReadPin(motor2.IC_GPIO_Port_v, motor2.IC_Pin_v) == GPIO_PIN_RESET)
        {
            direct = -BACKWARD;
        }
        else
        {
            direct = -FORWARD;
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(motor2.htim_ic, motor2.ic_channel, TIM_ICPOLARITY_FALLING);     //转换输入捕获极性为下降沿
    }
    
    else                                                                                              //检测到下降沿
    {
        cap_val_2 = HAL_TIM_ReadCapturedValue(motor2.htim_ic, motor2.ic_channel);                     //再次读取TIM5 CCR3的值
        if(motor2.updata == 0)                                                                        //若两次捕获之间计数器未溢出
        {
            single_speed = SPEED_PARAM / (cap_val_2 - cap_val_1) * direct;
        }
        else if(motor2.updata == 1)                                                                   //若两次捕获之间溢出一个计数周期
        {
            single_speed = SPEED_PARAM / ((MAX_TIM_CNT - cap_val_1) + cap_val_2) * direct;
        }
        else                                                                                          //若两次捕获之间溢出N个计数周期
        {
            single_speed = SPEED_PARAM / ((MAX_TIM_CNT - cap_val_1) + MAX_TIM_CNT * (motor2.updata - 1) + cap_val_2) * direct;
        }
        
        sum_speed += single_speed;                              //采样周期累加
        cap_cnt++;                                              //累积采样次数
        
        if(cap_cnt == FILTER) {                                 //采样完成                                   
           cap_cnt = 0;                                     
           last_average_speed = average_speed;                  //保存上一次滤波后的速度值
           if(flag_first == 0) {
               average_speed = (sum_speed+last_average_speed) / FILTER; //第一次滤波时 last_average_speed=0
               flag_first = 1;
           }
           else {
               average_speed = (sum_speed-single_speed+last_average_speed) / FILTER;    //往后的滤波都舍弃最后一个采集的速度 替换为上一次滤波后的速度
           }
           
           if(!(__fabs(average_speed + last_average_speed) < 5)) {              //若上一次与这一次差值过大并且相加后接近0 说明方向值发生跳变
                motor2.freq =  average_speed ;
             
           } 
           cap_cnt = 0;             //采样处理完后  将统计采样次数的值清0 等待下一次采样处理
           sum_speed = 0;           //采样处理完后  将采样总和的值清0 等待下一次采样处理
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(motor2.htim_ic, motor2.ic_channel, TIM_ICPOLARITY_RISING);      //转换输入捕获极性为上升沿，等待下一次采样
        ic_edge = IC_RISE_EDGE;     //等待下一个周期的第一个上升沿
        
    }
}

/**********************************************************************
 * @Name    TIM3CaptureChannel1Callback
 * @declaration : 通过输入捕获功能计算电机3转速与频率  
 * @param   None    
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void TIM3CaptureChannel1Callback()
{
   static uint8_t ic_edge = IC_RISE_EDGE;
    static uint32_t cap_val_1 = 0;             //第一次上升沿CCR寄存器内值
    static uint32_t cap_val_2 = 0;             //第二次上升沿CCR寄存器内值
    static int32_t sum_speed = 0;              //均值滤波采样速度值总和
    static int32_t single_speed = 0;           //单个速度值
    static int32_t average_speed = 0;          //滤波处理后的平均速度
    static int32_t last_average_speed = 0;     //存储上一次滤波得出的平均速度
    static uint32_t cap_cnt = 0;               //记录采样次数
    static int8_t direct = 0;                  //方向
    static uint8_t flag_first = 0;             //标志是否是第一次滤波

    if(ic_edge == IC_RISE_EDGE)                //当检测到上升沿
    {
        motor3.updata = 0;                     //更新中断事件记录
        cap_val_1 = HAL_TIM_ReadCapturedValue(motor3.htim_ic, motor3.ic_channel);   //读取TIM3 CCR1的值
        ic_edge = IC_FALL_EDGE;                                                     //等待下降沿

        /* 检测另一端GPIO口的电平变化来判断方向*/
        if(HAL_GPIO_ReadPin(motor3.IC_GPIO_Port_v, motor3.IC_Pin_v) == GPIO_PIN_RESET)
        {
            direct = BACKWARD;
        }
        else
        {
            direct = FORWARD;
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(motor3.htim_ic, motor3.ic_channel, TIM_ICPOLARITY_FALLING);     //转换输入捕获极性为下降沿
    }
    
    else                                                                                              //检测到下降沿
    {
        cap_val_2 = HAL_TIM_ReadCapturedValue(motor3.htim_ic, motor3.ic_channel);                     //再次读取TIM3 CCR1的值
        if(motor3.updata == 0)                                                                        //若两次捕获之间计数器未溢出
        {
            single_speed = SPEED_PARAM / (cap_val_2 - cap_val_1) * direct;
        }
        else if(motor3.updata == 1)                                                                   //若两次捕获之间溢出一个计数周期
        {
            single_speed = SPEED_PARAM / ((MAX_TIM_CNT - cap_val_1) + cap_val_2) * direct;
        }
        else                                                                                          //若两次捕获之间溢出N个计数周期
        {
            single_speed = SPEED_PARAM / ((MAX_TIM_CNT - cap_val_1) + MAX_TIM_CNT * (motor3.updata - 1) + cap_val_2) * direct;
        }
        
        sum_speed += single_speed;                              //采样周期累加
        cap_cnt++;                                              //累积采样次数
        
        if(cap_cnt == FILTER) {                                 //采样完成                                   
           cap_cnt = 0;                                     
           last_average_speed = average_speed;                  //保存上一次滤波后的速度值
           if(flag_first == 0) {
               average_speed = (sum_speed+last_average_speed) / FILTER; //第一次滤波时 last_average_speed=0
               flag_first = 1;
           }
           else {
               average_speed = (sum_speed-single_speed+last_average_speed) / FILTER;    //往后的滤波都舍弃最后一个采集的速度 替换为上一次滤波后的速度
           }
           
           if(!(__fabs(average_speed + last_average_speed) < 5)) {              //若上一次与这一次差值过大并且相加后接近0 说明方向值发生跳变
                motor3.freq =  average_speed ;
           } 
           cap_cnt = 0;             //采样处理完后  将统计采样次数的值清0 等待下一次采样处理
           sum_speed = 0;           //采样处理完后  将采样总和的值清0 等待下一次采样处理
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(motor3.htim_ic, motor3.ic_channel, TIM_ICPOLARITY_RISING);      //转换输入捕获极性为上升沿，等待下一次采样
        ic_edge = IC_RISE_EDGE;     //等待下一个周期的第一个上升沿
        
    }
}

/**********************************************************************
 * @Name    TIM3CaptureChannel3Callback
 * @declaration : 通过输入捕获功能计算电机4转速与频率  
 * @param   None    
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void TIM3CaptureChannel3Callback()
{
   static uint8_t ic_edge = IC_RISE_EDGE;
    static uint32_t cap_val_1 = 0;             //第一次上升沿CCR寄存器内值
    static uint32_t cap_val_2 = 0;             //第二次上升沿CCR寄存器内值
    static int32_t sum_speed = 0;              //均值滤波采样速度值总和
    static int32_t single_speed = 0;           //单个速度值
    static int32_t average_speed = 0;          //滤波处理后的平均速度
    static int32_t last_average_speed = 0;     //存储上一次滤波得出的平均速度
    static uint32_t cap_cnt = 0;               //记录采样次数
    static int8_t direct = 0;                  //方向
    static uint8_t flag_first = 0;             //标志是否是第一次滤波

    if(ic_edge == IC_RISE_EDGE)                //当检测到上升沿
    {
        motor4.updata = 0;                     //更新中断事件记录
        cap_val_1 = HAL_TIM_ReadCapturedValue(motor4.htim_ic, motor4.ic_channel);   //读取TIM3 CCR3的值
        ic_edge = IC_FALL_EDGE;                                                     //等待下降沿

        /* 检测另一端GPIO口的电平变化来判断方向*/
        if(HAL_GPIO_ReadPin(motor4.IC_GPIO_Port_v, motor4.IC_Pin_v) == GPIO_PIN_RESET)
        {
            direct = BACKWARD;
        }
        else
        {
            direct = FORWARD;
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(motor4.htim_ic, motor4.ic_channel, TIM_ICPOLARITY_FALLING);     //转换输入捕获极性为下降沿
    }
    
    else                                                                                              //检测到下降沿
    {
        cap_val_2 = HAL_TIM_ReadCapturedValue(motor4.htim_ic, motor4.ic_channel);                     //再次读取TIM3 CCR1的值
        if(motor4.updata == 0)                                                                        //若两次捕获之间计数器未溢出
        {
            single_speed = SPEED_PARAM / (cap_val_2 - cap_val_1) * direct;
        }
        else if(motor4.updata == 1)                                                                   //若两次捕获之间溢出一个计数周期
        {
            single_speed = SPEED_PARAM / ((MAX_TIM_CNT - cap_val_1) + cap_val_2) * direct;
        }
        else                                                                                          //若两次捕获之间溢出N个计数周期
        {
            single_speed = SPEED_PARAM / ((MAX_TIM_CNT - cap_val_1) + MAX_TIM_CNT * (motor4.updata - 1) + cap_val_2) * direct;
        }
        
        sum_speed += single_speed;                              //采样周期累加
        cap_cnt++;                                              //累积采样次数
        
        if(cap_cnt == FILTER) {                                 //采样完成                                   
           cap_cnt = 0;                                     
           last_average_speed = average_speed;                  //保存上一次滤波后的速度值
           if(flag_first == 0) {
               average_speed = (sum_speed+last_average_speed) / FILTER; //第一次滤波时 last_average_speed=0
               flag_first = 1;
           }
           else {
               average_speed = (sum_speed-single_speed+last_average_speed) / FILTER;    //往后的滤波都舍弃最后一个采集的速度 替换为上一次滤波后的速度
           }
           
           if(!(__fabs(average_speed + last_average_speed) < 5)) {              //若上一次与这一次差值过大并且相加后接近0 说明方向值发生跳变
                motor4.freq =  average_speed ;
           } 
           cap_cnt = 0;             //采样处理完后  将统计采样次数的值清0 等待下一次采样处理
           sum_speed = 0;           //采样处理完后  将采样总和的值清0 等待下一次采样处理
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(motor4.htim_ic, motor4.ic_channel, TIM_ICPOLARITY_RISING);      //转换输入捕获极性为上升沿，等待下一次采样
        ic_edge = IC_RISE_EDGE;     //等待下一个周期的第一个上升沿
        
    }
}


int32_t read_freq(motor_t *motor)
{   double temp = motor->freq;
    motor->freq = 0;
	return temp;
}



/**********************************************************************
 * @Name    TIM5_IcOverflowCntCallback
 * @declaration : TIM5定时器计数器溢出   更新中断事件次数
 * @param   None    
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void TIM5_IcOverflowCntCallback(void)
{
    if(motor1.updata < MAX_TIM_CNT)
    {
        motor1.updata++;
    }
    else
    {
        motor1.updata = 0;
        motor1.freq = 0;
    }

    if(motor2.updata < MAX_TIM_CNT)
    {
        motor2.updata++;
    }
    else
    {
        motor2.updata = 0;
        motor2.freq = 0;
    }
}

/**********************************************************************
 * @Name    TIM3_IcOverflowCntCallback
 * @declaration : TIM3定时器计数器溢出   更新中断事件次数
 * @param   None    
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void TIM3_IcOverflowCntCallback()
{
    if(motor3.updata < MAX_TIM_CNT)
    {
        motor3.updata++;
    }
    else
    {
        motor3.updata = 0;
        motor3.freq = 0;
    }

    if(motor4.updata < MAX_TIM_CNT)
    {
        motor4.updata++;
    }
    else
    {
        motor4.updata = 0;
        motor4.freq = 0;
    }
}


/**********************************************************************
 * @Name    HAL_TIM_PeriodElapsedCallback
 * @declaration : 溢出中断回调函数    
 * @param   None    
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM3)
    {
        TIM3_IcOverflowCntCallback();
    }

    if(htim->Instance == TIM5)
    {
        TIM5_IcOverflowCntCallback();
    }
}

int distence = 0;
/**********************************************************************
 * @Name    HAL_TIM_IC_CaptureCallback
 * @declaration : 输入捕获事件回调函数    
 * @param   None    
 * @retval   : 无
 * @author  hoson_stars
 ***********************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static long cnt_encoder=0;
	cnt_encoder++;
	  static int count = 0;
    if(htim->Instance == TIM5)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            TIM5CaptureChannel1Callback();           //电机1输入捕获
        }
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {  
            TIM5CaptureChannel3Callback();           //电机2输入捕获
        }
    }

    else if(htim->Instance == TIM3)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            TIM3CaptureChannel1Callback();           //电机3输入捕获
        }
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            TIM3CaptureChannel3Callback();           //电机4输入捕获
        }
    }
    
    count++;
    
    if(count == 400) {
        count = 0;
        distence += (__fabs(motor1.freq)+__fabs(motor2.freq)+__fabs(motor3.freq)+__fabs(motor4.freq))/4.0;   //编码器计算路程
    }
}



