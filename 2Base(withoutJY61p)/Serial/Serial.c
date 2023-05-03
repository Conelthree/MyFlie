#include "Serial.h"

#if EN_USART1_RX

uint8_t USART_RX_BUF[USART_REC_LEN];

uint16_t USART_RX_STA=0;

uint8_t aRxBuffer[RXBUFFERSIZE];


void Serial_Init(void)
{
	HAL_UART_Receive_IT(&huart1,aRxBuffer,1);
}


int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,100);
	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance==USART1)
	{
		if((USART_RX_STA&0x8000)==0)
		{
			if(USART_RX_STA&0x4000)
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;
				else USART_RX_STA|=0x8000;
			}
			else
			{	
				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;  
				}		 
			}
		}

	}

}

void USART1_myIRQHandler(void)
{ 
	uint32_t timeout=0;
    uint32_t maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS
	OSIntEnter();    
#endif
	
	timeout=0;
    while (HAL_UART_GetState(&huart1)!=HAL_UART_STATE_READY)
	{
        timeout++;
        if(timeout>maxDelay) break;		
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart1,(uint8_t *)aRxBuffer, RXBUFFERSIZE)!=HAL_OK)
	{
        timeout++;
        if(timeout>maxDelay) break;	
	}
#if SYSTEM_SUPPORT_OS
	OSIntExit();  											 
#endif
} 
#endif

