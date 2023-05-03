#ifndef __GLOBALCLOCK__
#define __GLOBALCLOCK__

#include "tim.h"
#include "Motor_Power.h"

extern volatile uint32_t TIME_ISR_CNT;

void TIM6_IRQ(void);
void GlobalClock_Init(void);

static void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);

#endif
