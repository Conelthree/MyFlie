#include "Jy61p.h"

uint8_t Jy61p_Rx_buffer;

uint8_t Jy61p_Rx_Pkt[11];			//接收数据包
uint8_t Rx_Cnt = 0; 			//接收字节计数
uint8_t ck_sum = 0;				//接收字节和

JY61PData_t Jy61pData;
pid_data_t Jy61p_PidData;
pid_paramer_t Jy61p_PidParm = {
	.integrate_max = 500,
    .kp = 5,
    .ki = 0,
    .kd = 0,
    .control_output_limit = 1000
};

void Jy61p_Init(void)
{
	Jy61p_PidData.expect = 0;
    Jy61p_PidData.feedback = 0;

    Jy61p_PidData.err = 0;
    Jy61p_PidData.last_err = 0;
    Jy61p_PidData.last2_err=0;
    Jy61p_PidData.integrate = 0;
    Jy61p_PidData.delta = 0;
    Jy61p_PidData.dis_err = 0;

    Jy61p_PidData.control_output = 0;

    Jy61p_PidData.short_circuit_flag = 0;
	
    HAL_UART_Receive_IT(&huart5,&Jy61p_Rx_buffer,1);
	
}

void receive_imu901_IRQ(void)
{
		int i = 0;
		Jy61p_Rx_Pkt[Rx_Cnt++] = Jy61p_Rx_buffer;
		ck_sum += Jy61p_Rx_buffer;
		if(Rx_Cnt == 2 && (Jy61p_Rx_Pkt[0] != 0x55 ||Jy61p_Rx_Pkt[1] != 0x53))	//找数据包头
		{
			Jy61p_Rx_Pkt[0] = 0;
			Jy61p_Rx_Pkt[1] = 0;
			Rx_Cnt = 0;
			ck_sum = 0;
			HAL_UART_Receive_IT(&huart5,&Jy61p_Rx_buffer, 1);
			return;
		}
		if(Rx_Cnt == 11)														//收到一个完整数据包
		{
			if(Jy61p_Rx_Pkt[10] != (ck_sum - Jy61p_Rx_Pkt[10]))					//校验和错误
			{
				for(i = 0;i < 11;i++)
				{
					Jy61p_Rx_Pkt[i] = 0;
				}
				Rx_Cnt = 0;
				ck_sum = 0;
				HAL_UART_Receive_IT(&huart5,&Jy61p_Rx_buffer, 1);
				return;
			}
			else
			{
				
				for(i = 1;i < 10;i++)
				{
					Jy61pData.data[i-2] = Jy61p_Rx_Pkt[i];
					Jy61p_Rx_Pkt[i] = 0;
				}
				Jy61pData.msgID = Jy61p_Rx_Pkt[1];
				
				Jy61p_Rx_Pkt[0] = 0;
				Jy61p_Rx_Pkt[1] = 0;
				Jy61p_Rx_Pkt[10] = 0;
				Rx_Cnt = 0;
				ck_sum = 0;
				int16_t data = (int16_t)((int16_t)Jy61pData.data[5] << 8 | Jy61pData.data[4]);
				Jy61pData.angle.yaw = (float) data / 32768 * 180;
//				printf("%f\n",Jy61pData.angle.yaw);
			}
		}
		HAL_UART_Receive_IT(&huart5,&Jy61p_Rx_buffer,1);
}

void Jy61p_Enable(void)
{
	Jy61pData.angle.status = true;
}

void Jy61p_Disable(void)
{
	Jy61pData.angle.status = false;
}

void imu_calibration(void)
{
    float current_yaw = 0, last_yaw = 0;
    uint8_t init_times = 0;
    while(init_times <= 100) {
        current_yaw = Jy61pData.angle.yaw;
        if (__fabs(current_yaw - last_yaw) < 5) 
		{
            HAL_Delay(10);
            init_times++;
        }
		else
		{
            init_times = 0;
        }
        last_yaw = current_yaw;
    }
    Jy61pData.angle.refer_angle = current_yaw;
}


int16_t Jy61p_PIDout(void)
{
	if(Jy61pData.angle.status == true) 
	{
        Jy61p_PidData.expect = Jy61pData.angle.target_angle;
        if ((Jy61pData.angle.yaw - Jy61pData.angle.refer_angle) > 180)
            Jy61pData.angle.yaw -= 360;
        else if ((Jy61pData.angle.yaw - Jy61pData.angle.refer_angle) < -180)
            Jy61pData.angle.yaw += 360;

        Jy61p_PidData.feedback = Jy61pData.angle.yaw - Jy61pData.angle.refer_angle;
		//printf("yaw=%f\n,refer=%f\n",attitude->yaw,attitude->refer_angle);
				
        if(__fabs(Jy61p_PidData.feedback - Jy61p_PidData.expect) <= 1.5) return 0;   //3度范围内认为已达到目标值
        else return pid_positional(&Jy61p_PidData, &Jy61p_PidParm);
    }
    else return 0;
}

