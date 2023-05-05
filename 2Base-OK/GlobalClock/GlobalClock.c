#include "GlobalClock.h"

volatile uint32_t TIME_ISR_CNT;//全局时钟计数

void TIM6_IRQ(void)
{
	TIME_ISR_CNT++;
//	printf("inturpt_enter!,%d\r\n",TIME_ISR_CNT);
	Motor_Power();
	__HAL_TIM_CLEAR_IT(&htim6,TIM_FLAG_UPDATE);
}


void GlobalClock_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim6);
	delay_init();
}

// Created by luohx on 2020/9/25.
//

static uint8_t fac_us = 0;          //us计时
static uint32_t fac_ms = 0;         //ms计时

/**********************************************************************************************************
*函 数 名: delay_init
*功能说明: 初始化自定义delay功能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void delay_init(void) {
  fac_us = SystemCoreClock / 1000000 / 2;
  fac_ms = SystemCoreClock / 1000;
}

/**********************************************************************************************************
*函 数 名: delay_us
*功能说明: 延时us单位
*形    参: 需要延时的us
*返 回 值: 无
**********************************************************************************************************/
void delay_us(uint16_t nus) {
  uint32_t ticks = 0;
  uint32_t told = 0;
  uint32_t tnow = 0;
  uint32_t tcnt = 0;
  uint32_t reload = 0;
  reload = SysTick->LOAD;
  ticks = nus * fac_us;
  told = SysTick->VAL;
  while (1) {
    tnow = SysTick->VAL;
    if (tnow != told) {
      if (tnow < told) {
        tcnt += told - tnow;
      } else {
        tcnt += reload - tnow + told;
      }
      told = tnow;
      if (tcnt >= ticks) {
        break;
      }
    }
  }
}

/**********************************************************************************************************
*函 数 名: delay_ms
*功能说明: 延时ms单位
*形    参: 需要延时的ms
*返 回 值: 无
**********************************************************************************************************/
void delay_ms(uint16_t nms) {
  uint32_t ticks = 0;
  uint32_t told = 0;
  uint32_t tnow = 0;
  uint32_t tcnt = 0;
  uint32_t reload = 0;
  reload = SysTick->LOAD;
  ticks = nms * fac_ms;
  told = SysTick->VAL;
  while (1) {
    tnow = SysTick->VAL;
    if (tnow != told) {
      if (tnow < told) {
        tcnt += told - tnow;
      } else {
        tcnt += reload - tnow + told;
      }
      told = tnow;
      if (tcnt >= ticks) {
        break;
      }
    }
  }
}



