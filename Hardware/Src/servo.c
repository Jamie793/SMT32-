#include "servo.h"

void Servor_Init(void)
{
    
    GPIO_InitTypeDef GPIO_InitStr;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStr;
    TIM_OCInitTypeDef TIM_OCInitStr;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStr.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStr.GPIO_Pin = SERVO_HOR_PIN;
    GPIO_InitStr.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_PORT, &GPIO_InitStr);

    GPIO_InitStr.GPIO_Pin = SERVO_VER_PIN;
    GPIO_Init(SERVO_PORT, &GPIO_InitStr);


    TIM_TimeBaseInitStr.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStr.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStr.TIM_Period = TIM2_ARR;
    TIM_TimeBaseInitStr.TIM_Prescaler = TIM2_PSC;
    TIM_TimeBaseInitStr.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStr);
    TIM_Cmd(TIM2, ENABLE);

    TIM_OCInitStr.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStr.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStr.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStr.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM2, &TIM_OCInitStr);
    TIM_OC2Init(TIM2, &TIM_OCInitStr);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
}



void Servo_Hor_Angle(u8 angle)
{
    // TIM_Cmd(TIM2, ENABLE);
    TIM2->CCR1 = 500 + (angle / 180.0) * 2000;
    // SysTick_Delay_Ms(1000);
    // TIM_Cmd(TIM2, DISABLE);
}

void Servo_Hor_PWM(u16 pwm){
    TIM2->CCR1 = pwm;
}

void Servo_Ver_Angle(u8 angle)
{
    // TIM_Cmd(TIM2, ENABLE);
    TIM2->CCR2 = 500 + (angle / 180.0) * 2000;
    // SysTick_Delay_Ms(1000);
    // TIM_Cmd(TIM2, DISABLE);
}

void Servo_Ver_PWM(u16 pwm){
    TIM2->CCR2 = pwm;
}