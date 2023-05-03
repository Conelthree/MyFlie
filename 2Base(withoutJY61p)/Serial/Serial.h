#ifndef __SERIAL__
#define __SERIAL__

#include "stm32f7xx.h"
#include <stdio.h>
#include "usart.h"

#define USART_REC_LEN  			200
#define EN_USART1_RX 			1  	
extern uint8_t  USART_RX_BUF[USART_REC_LEN];
extern uint16_t USART_RX_STA;

#define RXBUFFERSIZE   1
extern uint8_t aRxBuffer[RXBUFFERSIZE];


void USART1_myIRQHandler(void);
void Serial_Init(void);

#endif
