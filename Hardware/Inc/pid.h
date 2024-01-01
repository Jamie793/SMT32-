#ifndef __PID_JAMIEXU_H_
#define __PID_JAMIEXU_H_
#include "stm32f10x.h"
#include "usart.h"
#include "servo.h"

#define PIDDEBUG_PACKAGET_HEADER 0x59485A53 

#define PID_PSC (72000 - 1)
#define PID_ARR (50 - 1)

//0.08 0.07 0.05  0.5
#define SERVO_HOR_KP 0.3
//0.03 0.06 0.07 0.2
#define SERVO_HOR_KI 0.2
//0 0 0.02 0
#define SERVO_HOR_KD 0

#define SERVO_VER_KP 0.2
#define SERVO_VER_KI 0.4
#define SERVO_VER_KD 0.2



typedef struct{
    float Kp, Ki, Kd;
    float actual_val;
    float target_val;
    float err;
    float err1;
    float err2;
    float uk;
}_PID;

void PID_Init(void);

void PID_Set_Actual(_PID *pid, float actual);

void PID_Set_Target(_PID *pid, float target);


void PID_Parameter_Init(_PID *pid, float p, float i, float d);

uint16_t PID_Calc(_PID *pid);

u8 PID_Debug_Calc(u8 *buffer, u32 size);

u8 PID_Debug_Check(u8 *buffer, u32 size);

void PID_Debug_Set_Actual(u8 channel, u32 value);

void PID_Debug_Set_Target(u8 channel, u32 value);

void PID_Debug_Proc(u8 *buffer);


#endif
