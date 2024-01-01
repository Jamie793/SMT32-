#include "pid.h"
u8 buffer[100];
_PID pid_hor;
_PID pid_ver;
extern DataFrame *mDataFrame;
extern u8 usart1_rec_bit;

void PID_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStr;
    USART_InitTypeDef USART_InitStr;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStr;

    PID_Parameter_Init(&pid_hor, SERVO_HOR_KP, SERVO_HOR_KI, SERVO_HOR_KD);
    PID_Parameter_Init(&pid_ver, SERVO_VER_KP, SERVO_VER_KI, SERVO_VER_KD);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitStr.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStr.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStr.TIM_Period = PID_PSC;
    TIM_TimeBaseInitStr.TIM_Prescaler = PID_ARR;
    TIM_TimeBaseInitStr.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStr);
    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitStr.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStr.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStr);
}

void PID_Parameter_Init(_PID *pid, float p, float i, float d)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->uk = 0;
    pid->err = 0;
    pid->err1 = 0;
    pid->err2 = 0;
    pid->actual_val = 0;
    pid->target_val = 0;
}

uint16_t PID_Calc(_PID *pid)
{
    pid->err = pid->target_val - pid->actual_val;
    float duk = ((pid->Kp * (pid->err - pid->err1 + 1e-8)) + (pid->Ki * pid->err) +
                 (pid->Kd * (pid->err - (2 * pid->err1) + pid->err2)));
    pid->uk += duk;
    pid->err2 = pid->err1;
    pid->err1 = pid->err;
    return pid->uk;
}

void PID_Set_Actual(_PID *pid, float actual)
{
    pid->actual_val = actual;
}

void PID_Set_Target(_PID *pid, float target)
{
    pid->target_val = target;
}

int abs_value(int num)
{
    if (num < 0)
        return -num;
    return num;
}

void PID_Debug_Set_Actual(u8 channel, u32 value)
{
    *(u32 *)(buffer) = PIDDEBUG_PACKAGET_HEADER;
    buffer[4] = channel;
    *(u32 *)(buffer + 5) = 0x0F;
    buffer[9] = 0x02;
    *(u32 *)(buffer + 10) = value;
    buffer[14] = PID_Debug_Calc(buffer, 0x0F - 1);
    USART2_SendBytes(buffer, 0x0F);
}

void PID_Debug_Set_Target(u8 channel, u32 value)
{
    *(u32 *)(buffer) = PIDDEBUG_PACKAGET_HEADER;
    buffer[4] = channel;
    *(u32 *)(buffer + 5) = 0x0F;
    buffer[9] = 0x01;
    *(u32 *)(buffer + 10) = value;
    buffer[14] = PID_Debug_Calc(buffer, 0x0F - 1);
    USART2_SendBytes(buffer, 0x0F);
}

void PID_Debug_Proc(u8 *buffer)
{
    // u8 channel = buffer[4];
    // u32 size = *(u32 *)(buffer + 4);
    // u8 cmd = buffer[9];
    // float p = *(float *)(buffer + 10);
    // float i = *(float *)(buffer + 14);
    // float d = *(float *)(buffer + 18);
    // pid_hor.Kp = p;
    // pid_hor.Ki = i;
    // pid_hor.Kd = d;
    // if (buffer[0] == 'P')
    //     pid_hor.Kp = atof(buffer + 1);
    // else if (buffer[0] == 'I')
    //     pid_hor.Ki = atof(buffer + 1);
    // else if (buffer[0] == 'D')
    //     pid_hor.Kd = atof(buffer + 1);
    // else if (buffer[0] == 'R')
    //     PID_Parameter_Init(&pid_hor, SERVO_HOR_KP, SERVO_HOR_KI, SERVO_HOR_KD);

    if (buffer[0] == 'P')
        pid_ver.Kp = atof(buffer + 1);
    else if (buffer[0] == 'I')
        pid_ver.Ki = atof(buffer + 1);
    else if (buffer[0] == 'D')
        pid_ver.Kd = atof(buffer + 1);
    else if (buffer[0] == 'R')
        PID_Parameter_Init(&pid_ver, SERVO_VER_KP, SERVO_VER_KI, SERVO_VER_KD);

    printf("Rec Pid, P: %.2f, %.2f, %.2f\n", pid_ver.Kp, pid_ver.Ki, pid_ver.Kd);
}

u8 PID_Debug_Calc(u8 *buffer, u32 size)
{
    u8 v = 0;
    while (size--)
    {
        v += *buffer;
        buffer++;
    }
    return v;
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (usart1_rec_bit)
        {
            usart1_rec_bit = 0;
            PID_Set_Actual(&pid_hor, mDataFrame->tar_x);
            PID_Set_Target(&pid_hor, mDataFrame->ref_x);
            if (abs_value(mDataFrame->ref_x - mDataFrame->tar_x) > 70)
            {

                uint16_t pwm_x = PID_Calc(&pid_hor);
                if (pwm_x < 500)
                    pwm_x = 500;
                if (pwm_x > 2500)
                    pwm_x = 2500;
                Servo_Hor_PWM(pwm_x);
            }

            PID_Set_Actual(&pid_ver, mDataFrame->tar_y);
            PID_Set_Target(&pid_ver, mDataFrame->ref_y);
            if (abs_value(mDataFrame->ref_y - mDataFrame->tar_y) > 70)
            {
                uint16_t pwm_y = PID_Calc(&pid_ver);
                if (pwm_y < 1000)
                    pwm_y = 1000;
                if (pwm_y > 1500)
                    pwm_y = 1500;

                Servo_Ver_PWM(pwm_y);
            }
            // printf("pid: %.2f,%.2f,%.2f,%.2f,%.2f\n", pid_hor.actual_val, pid_hor.target_val, pid_hor.Kp, pid_hor.Ki, pid_hor.Kd);
            printf("pid: %.2f,%.2f,%.2f,%.2f,%.2f\n", pid_ver.actual_val, pid_ver.target_val, pid_ver.Kp, pid_ver.Ki, pid_ver.Kd);
        }
    }
}