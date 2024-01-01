#ifndef __SERVO_JAMIEXU_H_
#define __SERVO_JAMIEXU_H_
#include "stm32f10x.h"
#include "delay.h"


#define SERVO_PORT GPIOA

#define SERVO_HOR_PIN GPIO_Pin_0
#define SERVO_VER_PIN GPIO_Pin_1


#define TIM2_ARR (20000 - 1)
#define TIM2_PSC (72 - 1)

void Servor_Init(void);

void Servo_Hor_Angle(u8 angle);

void Servo_Ver_Angle(u8 angle);

void Servo_Hor_PWM(u16 pwm);

void Servo_Ver_PWM(u16 pwm);
#endif
