/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>

/*
UART
已经集成在开发板里，接入typeC即可用串口，波特率115200
GPIO
GPIO-LED : PORTB PB2 PULL-DOWN
GPIO-KEY : PORTA PA18 PULL-DOWN
GPIO-INF : PORTA  R2:PA23  L2:PA24  L1:PA25  R1:PA26  PULL-UP
SysTick:Enabled 320000->1ms
PWM
pwm-motor: TIMA0
PWM-SERVO: TIMG12
*/


//functions declaration
void delay_ms(unsigned int ms);
void UART0_sendString(const char* str);
void UART0_sendChar(const char ch);
void PWM_setDuty(float duty, GPTIMER_Regs* PWM_Inst, DL_TIMER_CC_INDEX PWM_Channel);
void PWM_setFrequency(uint32_t freq, GPTIMER_Regs* PWM_Inst);
int INF_getGPIOstate(GPIO_Regs* gpio, uint32_t pin);
void Motor_Set(int speed, uint32_t in1_pin, uint32_t in2_pin, GPTIMER_Regs* pwm_inst, DL_TIMER_CC_INDEX pwm_ch);
void MOTOR_setSpeed(int speedA, int speedB, int speedC, int speedD);



//definitions
#define L1      INF_getGPIOstate(INF_INPUT_PORT, INF_INPUT_L1_PIN)
#define L2      INF_getGPIOstate(INF_INPUT_PORT, INF_INPUT_L2_PIN)
#define R1      INF_getGPIOstate(INF_INPUT_PORT, INF_INPUT_R1_PIN)
#define R2      INF_getGPIOstate(INF_INPUT_PORT, INF_INPUT_R2_PIN)


#define MOTOR_A_PWM_CH  GPIO_PWM_MOTOR_C0_IDX
#define MOTOR_B_PWM_CH  GPIO_PWM_MOTOR_C1_IDX
#define MOTOR_C_PWM_CH  GPIO_PWM_MOTOR_C2_IDX
#define MOTOR_D_PWM_CH  GPIO_PWM_MOTOR_C3_IDX

#define SERVO_MIDDLE    0
#define KI              0
#define KP              0
#define KD              0
#define BLACK           1
#define WHITE           0

volatile unsigned int delay_times = 0;
volatile unsigned int period = 32000;
char buff[64];


int main(void)
{
    SYSCFG_DL_init();

    while (1) {
        //循迹部分
        //将数据打包通过串口发送
        sprintf(buff, "L1:%d, L2:%d, R1:%d, R2:%d\r\n", 
        L1, L2, R1, R2);
        UART0_sendString(buff);
        //使用逻辑判断实现循迹
        if (L2 == BLACK && R2 == WHITE)             //线在左外侧
            MOTOR_setSpeed(0, 100, 0, 100);
        else if (L2 == WHITE && R2 == BLACK)        //线在右外侧
            MOTOR_setSpeed(100, 0, 100, 0);
        else if (L2 == WHITE && R2 == WHITE)
        {
            if (L1 == BLACK && R1 == WHITE)         //线在左内侧 
                MOTOR_setSpeed(50, 100, 50, 100);
            else if (L1 == WHITE && R1 == BLACK)    //线在右内侧
                MOTOR_setSpeed(100, 50, 100, 50);
            else if (L1 == BLACK && R1 == BLACK)    //线在中央
                MOTOR_setSpeed(100, 100, 100, 100);
        }
        else if (L2 == BLACK && L1 == BLACK && R1 == BLACK && R2 == BLACK)
            MOTOR_setSpeed(0, 0, 0, 0);
    }
}

void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while (delay_times != 0)
        ;
}

void UART0_sendChar(const char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

void UART0_sendString(const char* str)
{
    while (*str != 0 && str != 0)
    {
        UART0_sendChar(*str++);
    }
}

void PWM_setDuty(float duty, GPTIMER_Regs* PWM_Inst, DL_TIMER_CC_INDEX PWM_Channel)
{
    uint32_t CompareValue;
    CompareValue = period - period * duty;
    DL_Timer_setCaptureCompareValue(PWM_Inst, CompareValue, PWM_Channel);
}

void PWM_setFrequency(uint32_t freq, GPTIMER_Regs* PWM_Inst)
{
    period = PWM_MOTOR_INST_CLK_FREQ / freq;
    DL_Timer_setLoadValue(PWM_Inst, period);
}

int INF_getGPIOstate(GPIO_Regs* gpio, uint32_t pin)
{
    return (DL_GPIO_readPins(gpio, pin) != 0)? 1:0;
}

void Motor_Set(int speed, uint32_t in1_pin, uint32_t in2_pin, GPTIMER_Regs* PWM_Inst, DL_TIMER_CC_INDEX PWM_Channel)
{
    if(speed > 0)
    {
        DL_GPIO_writePins(MOTORS_PORT, in1_pin);
        DL_GPIO_clearPins(MOTORS_PORT, in2_pin);
        PWM_setDuty((float)speed/255.0f, PWM_Inst, PWM_Channel);
    }
    else if (speed<0)
    {
        DL_GPIO_clearPins(MOTORS_PORT, in1_pin);
        DL_GPIO_writePins(MOTORS_PORT, in2_pin);
        PWM_setDuty((float)(-speed)/255.0f, PWM_Inst, PWM_Channel);
    }
    else
    {
        DL_GPIO_clearPins(MOTORS_PORT, in1_pin);
        DL_GPIO_clearPins(MOTORS_PORT, in2_pin);
        PWM_setDuty(0, PWM_Inst, PWM_Channel);   
    }
}

void MOTOR_setSpeed(int speedA, int speedB, int speedC, int speedD)
{
    Motor_Set(speedA, MOTORS_AIN1_PIN, MOTORS_AIN2_PIN, PWM_MOTOR_INST, GPIO_PWM_MOTOR_C0_IDX);
    Motor_Set(speedB, MOTORS_BIN1_PIN, MOTORS_BIN2_PIN, PWM_MOTOR_INST, GPIO_PWM_MOTOR_C1_IDX);
    Motor_Set(speedC, MOTORS_CIN1_PIN, MOTORS_CIN2_PIN, PWM_MOTOR_INST, GPIO_PWM_MOTOR_C2_IDX);
    Motor_Set(speedD, MOTORS_DIN1_PIN, MOTORS_DIN2_PIN, PWM_MOTOR_INST, GPIO_PWM_MOTOR_C3_IDX);
}
//NVIC
void SysTick_Handler(void)
{
    if (delay_times != 0)
    {
        delay_times--;
    }
}
