#ifndef __JAMIEXU_DELAY_H__
#define __JAMIEXU_DELAY_H__

#include "stm32f10x.h"                 // Device header


void SysTick_Init(void);

void SysTick_Delay_Ms(uint32_t ms);

void SysTick_Delay_Us(uint32_t us);

#endif
