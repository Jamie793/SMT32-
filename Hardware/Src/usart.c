#include "usart.h"
u8 usart1_rec_data[255];
u8 usart1_data_pos = 0;
u8 usart1_rec_bit = 0;

u8 usart2_rec_data[255];
u8 usart2_data_pos = 0;
u8 usart2_rec_bit = 0;

DataFrame *mDataFrame;
void USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStr;
    USART_InitTypeDef USART_InitStr;
    NVIC_InitTypeDef NVIC_InitStr;

    mDataFrame = (DataFrame *)malloc(sizeof(DataFrame));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStr.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStr.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStr.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStr);

    GPIO_InitStr.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStr.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStr);

    USART_InitStr.USART_BaudRate = 115200;
    USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStr.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStr.USART_Parity = USART_Parity_No;
    USART_InitStr.USART_StopBits = USART_StopBits_1;
    USART_InitStr.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStr);
    USART_Cmd(USART1, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStr.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStr.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStr);
}

void USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStr;
    USART_InitTypeDef USART_InitStr;
    NVIC_InitTypeDef NVIC_InitStr;

    mDataFrame = (DataFrame *)malloc(sizeof(DataFrame));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStr.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStr.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStr.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStr);

    GPIO_InitStr.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStr.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStr);

    USART_InitStr.USART_BaudRate = 115200;
    USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStr.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStr.USART_Parity = USART_Parity_No;
    USART_InitStr.USART_StopBits = USART_StopBits_1;
    USART_InitStr.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStr);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStr.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStr.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStr);
}

int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);

    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        ;

    return (ch);
}

void USART2_SendBytes(u8 *buffer, u32 len)
{
    for (size_t i = 0; i < len; i++)
    {
        USART_SendData(USART2, buffer[i]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
            ;
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        u8 data = USART_ReceiveData(USART1);
        if (usart1_rec_data[usart1_data_pos - 1] == 0xEA && usart1_rec_data[usart1_data_pos - 2] == 0xAE)
            usart1_data_pos = 2;
        usart1_rec_data[usart1_data_pos++] = data;
        if (usart1_rec_data[0] == 0xAE && usart1_rec_data[1] == 0xEA && usart1_rec_data[usart1_data_pos - 1] == 0xAE && usart1_rec_data[usart1_data_pos - 2] == 0xEA)
        {
            memcpy(mDataFrame, usart1_rec_data + 2, 8);
            usart1_rec_bit = 1;
        }
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);

        u16 data = USART_ReceiveData(USART2);
        usart2_rec_data[usart2_data_pos++] = data;
        if (data == '\n'){
            PID_Debug_Proc(usart2_rec_data);
            memset(usart2_rec_data, 0, 255);
            usart2_data_pos = 0;
        }
        // if (usart2_rec_data[usart2_data_pos - 4] == 0x53 && usart2_rec_data[usart2_data_pos - 3] == 0x5A && usart2_rec_data[usart2_data_pos - 2] == 0x48 && usart2_rec_data[usart2_data_pos - 1] == 0x59){
        //     usart2_data_pos = 4;
        // }


        // if (usart2_rec_data[0] == 0x53 && usart2_rec_data[1] == 0x5A && usart2_rec_data[2] == 0x48 && usart2_rec_data[3] == 0x59 && usart2_data_pos >= 0x17)
        // {
        //     PID_Debug_Proc(usart2_rec_data);
        //     usart2_data_pos = 0;
        // }
    }
}