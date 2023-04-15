
#ifndef __SERIAL_H_
#define __SERIAL_H_

#include "stm32f7xx.h"
#include "usart.h"
#include <stdio.h>

#define USART_REC_LEN  			200
#define EN_USART1_RX 			1	  	

#define RXBUFFERSIZE   1
extern uint8_t aRxBuffer[RXBUFFERSIZE];
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大为USART_REC_LEN个字节，末位为换行符
extern uint16_t USART_RX_STA; //接收状态标记

void USART1_myIRQHandler(void);
#endif
