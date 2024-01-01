#include "stm32f10x.h"
#include "servo.h"
#include "delay.h"
#include "usart.h"
#include "pid.h"


int main(void)
{
    SysTick_Init();
    USART1_Init();
    USART2_Init();
    Servor_Init();
    PID_Init();
    printf("Init done\n");
    
    Servo_Hor_Angle(90);
    Servo_Ver_Angle(45);

    while (1)
    {
        /* code */
    }
}
