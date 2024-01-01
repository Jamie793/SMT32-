#ifndef __USART_JAMIEXU_H_
#define __USART_JAMIEXU_H_
#include "stm32f10x.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "pid.h"

typedef struct 
{
    u16 ref_x;
    u16 ref_y;
    u16 tar_x;
    u16 tar_y;
}DataFrame;


void USART1_Init(void);

void USART2_Init(void);

void USART2_SendBytes(u8 *buffer, u32 len);

#endif
