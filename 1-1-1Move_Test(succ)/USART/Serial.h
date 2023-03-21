#ifndef __SERIAL_H_
#define __SERIAL_H_

#include "stm32f7xx.h"
#include "usart.h"
#include <stdio.h>

typedef struct usart
{
	UART_HandleTypeDef *huart;
	uint8_t *Receivedata;
	uint8_t *Senddata;
	uint8_t ReceiveFlag;

} usart_TypeDef;


void Serial_Init(usart_TypeDef *usart);
void Serial_SendByte(usart_TypeDef *usart);
void USART3_RxCpltCallback(void);
void USART6_RxCpltCallback(void);
//void Serial_SendArray(uint8_t *Array,uint16_t Length);

//void Serial_SendString(char *String);



#endif
