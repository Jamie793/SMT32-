#include "delay.h"

void SysTick_Init(void)
{
    NVIC_SetPriority(SysTick_IRQn, NVIC_PriorityGroup_0);
}

void SysTick_Delay_Ms(uint32_t ms)
{
    uint32_t i;
    SysTick_Config(SystemCoreClock / 1000);
    for (i = 0; i < ms; i++)
    {
        while (!((SysTick->CTRL) & (1 << 16)))
            ;
    }
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Delay_Us(uint32_t us)
{
    uint32_t i;
    SysTick_Config(SystemCoreClock / 1000000);
    for (i = 0; i < us; i++)
    {
        while (!((SysTick->CTRL) & (1 << 16)))
            ;
    }
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
