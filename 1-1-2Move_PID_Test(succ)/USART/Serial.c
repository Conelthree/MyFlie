#include "Serial.h"

//extern usart_TypeDef usart3;
//extern usart_TypeDef usart6;


void Serial_Init(usart_TypeDef *usart)
{
	HAL_UART_Receive_IT(usart->huart,usart->Receivedata,1);
}



//---------------------Serial_OUT------------------------------------

//
void Serial_SendByte(usart_TypeDef *usart)
{
	HAL_UART_Transmit (usart->huart,usart->Senddata,1,0xFFFF);	
}


//void USART3_RxCpltCallback(void)
//{
//	HAL_UART_Receive (usart3.huart ,usart3.Receivedata,1,0xFFFF);
//	usart3.ReceiveFlag = 1;
//	HAL_UART_Receive_IT(usart3.huart,usart3.Receivedata,1);
//}



//void USART6_RxCpltCallback(void)
//{
//	HAL_UART_Receive (usart6.huart ,usart6.Receivedata,1,0xFFFF);
//	usart6.ReceiveFlag = 1;
//	if(usart6.ReceiveFlag == 0)
//	{
//		HAL_UART_Receive_IT(usart6.huart,usart6.Receivedata,1);
//	}
//}
//
//void Serial_SendArray(uint8_t *Array,uint16_t Length)
//{
//	uint16_t i;
//	for (i = 0;i <Length ;i++)
//	{
//		Serial_SendByte (Array[i]);
//	}
//	
//}

//

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
	HAL_UART_Transmit (&huart3 ,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

//-------------------------------------------------------------------

//---------------------Serial_IN-------------------------------------

//接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart == &huart3)
	{
//		USART3_RxCpltCallback();
	}
	else if(huart == &huart6)
	{
//		USART6_RxCpltCallback();
	}
}


//uint8_t Serial_GetReciveData(void)
//{
////	uint8_t Temp = 0;
//	ReciveFlag = 0;

//	return Receivedata;
//}	


//scanf函数输入重定向
int fgetc(FILE *f)
{
	int ch;
	HAL_UART_Receive (&huart3 ,(uint8_t *)&ch,1,0xFFFF);
	return (ch);
}


