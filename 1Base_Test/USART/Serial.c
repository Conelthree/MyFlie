#include "Serial.h"
#include "imu.h"
//extern usart_TypeDef usart3;
//extern usart_TypeDef usart6;
uint8_t USART_RX_BUF[USART_REC_LEN];

uint16_t USART_RX_STA=0;

uint8_t aRxBuffer[RXBUFFERSIZE];

//---------------------Serial_OUT------------------------------------




//void Serial_SendString(char *String)
//{
//	uint8_t i;
//	for (i=0; String[i] != '\0';i++)
//	{
//		Serial_SendByte (String [i]);
//	
//	}
//}

//printf函数输出重定向
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit (&huart1 ,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

//-------------------------------------------------------------------

//---------------------Serial_IN-------------------------------------

//接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
//	if(huart == &huart3)
//	{
////		USART3_RxCpltCallback();
//	}
//	else if(huart == &huart6)
//	{
////		USART6_RxCpltCallback();
//	}
	if(huart->Instance == USART1)
	{
//		printf("in!\n");
		if((USART_RX_STA&0x8000)==0)//鎺ユ敹鏈畬鎴�
		{
			if(USART_RX_STA&0x4000)//鎺ユ敹鍒颁簡0x0d
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//鎺ユ敹閿欒,閲嶆柊寮€濮�
				else USART_RX_STA|=0x8000;	//鎺ユ敹瀹屾垚浜� 
			}
			else //杩樻病鏀跺埌0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//鎺ユ敹鏁版嵁閿欒,閲嶆柊寮€濮嬫帴鏀�	  
				}		 
			}
		}
	}
	if(huart == &huart5)
	{
		receive_imu901_IRQ(&attitude);
	}
}


//uint8_t Serial_GetReciveData(void)
//{
////	uint8_t Temp = 0;
//	ReciveFlag = 0;

//	return Receivedata;
//}	

void USART1_myIRQHandler(void)                	
{ 
	uint32_t timeout=0;
    uint32_t maxDelay=0x1FFFF;

	
	timeout=0;
    while (HAL_UART_GetState(&huart1)!=HAL_UART_STATE_READY)//绛夊緟灏辩华
	{
        timeout++;////瓒呮椂澶勭悊
        if(timeout>maxDelay) break;		
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart1,(uint8_t *)aRxBuffer, RXBUFFERSIZE)!=HAL_OK)//涓€娆″鐞嗗畬鎴愪箣鍚庯紝閲嶆柊寮€鍚腑鏂苟璁剧疆RxXferCount涓�1
	{
        timeout++; //瓒呮椂澶勭悊
        if(timeout>maxDelay) break;	
	}

}
//scanf函数输入重定向
//int fgetc(FILE *f)
//{
//	int ch;
//	HAL_UART_Receive (&huart3 ,(uint8_t *)&ch,1,0xFFFF);
//	return (ch);
//}


