#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "AllHeader.h"

// 提供一个兼容HAL库的获取系统毫秒级时钟的函数
uint32_t HAL_GetTick(void);

void TIM6_Init(void);
void TIM7_Init(void);
void TIM3_Init(void);

void power_decect(void);
void cotrol_led(void);

void delay_time(u16 time);
void my_delay(u16 s);
void System_Tick_Increment(void); // 系统tick计数函数


#endif
