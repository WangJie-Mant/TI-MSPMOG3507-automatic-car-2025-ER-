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
double limit_abs(double x, double maxx);
double MOTOR_getSpeed();



//definitions
#define L1      INF_getGPIOstate(INF_INPUT_PORT, INF_INPUT_L1_PIN)
#define L2      INF_getGPIOstate(INF_INPUT_PORT, INF_INPUT_L2_PIN)
#define R1      INF_getGPIOstate(INF_INPUT_PORT, INF_INPUT_R1_PIN)
#define R2      INF_getGPIOstate(INF_INPUT_PORT, INF_INPUT_R2_PIN)


#define MOTOR_A_PWM_CH  GPIO_PWM_MOTOR_C0_IDX
#define MOTOR_B_PWM_CH  GPIO_PWM_MOTOR_C1_IDX
#define MOTOR_C_PWM_CH  GPIO_PWM_MOTOR_C2_IDX
#define MOTOR_D_PWM_CH  GPIO_PWM_MOTOR_C3_IDX

#define SERVO_MIDDLE                0
#define MOTOR_BASIC_SPEED           100
//motor pid paras
#define MOTOR_PID_KP                0.1
#define MOTOR_PID_KI                0
#define MOTOR_PID_KD                0.1
#define MOTOR_PID_OUT_MAX           0
#define MOTOR_PID_POUT_MAX          0
#define MOTOR_PID_IOUT_MAX          0
#define MOTOR_PID_DOUT_MAX          0
//inf pid paras
#define INF_PID_KP                  0.1
#define INF_PID_KI                  0
#define INF_PID_KD                  0.1
#define INF_PID_OUT_MAX             0
#define INF_PID_POUT_MAX            0
#define INF_PID_IOUT_MAX            0
#define INF_PID_DOUT_MAX            0
#define BLACK                       1
#define WHITE                       0


volatile unsigned int delay_times = 0;
volatile unsigned int period = 32000;
volatile uint32_t Encoder_Gpios[4][2] = {ENCODER_INPUT_E1A_PIN, ENCODER_INPUT_E1B_PIN, 
                                        ENCODER_INPUT_E2A_PIN, ENCODER_INPUT_E2B_PIN, 
                                        ENCODER_INPUT_E3A_PIN, ENCODER_INPUT_E3B_PIN,
                                        ENCODER_INPUT_E4A_PIN, ENCODER_INPUT_E4B_PIN};
char buff[64];
double weights[4] = {-2, -1, 1, 2};
double INF_Data_Sum = 0;
double Wheel_Distance = 0;
typedef struct
{
    int32_t count;
    int8_t direction;
    uint8_t lastA;
    uint8_t lastB;
    uint32_t encoderA_gpio;
    uint32_t encoderB_gpio;
} Encoder_t;
typedef struct
{
    double kp, ki, kd;
    double target, current;
    double error, lasterror;
    double out, pout, iout, dout;
    double pout_max, iout_max, dout_max, out_max;
} pid_t;
Encoder_t Motors_Encoder[4];

//PID Part
void pid_init(pid_t *p, double kp, double ki, double kd, 
double out_max, double pout_max, double iout_max, double dout_max)
{
    p->kp=kp;
    p->ki=ki;
    p->kd=kd;
    p->target=0;
    p->current=0;
    p->error=0;
    p->lasterror=0;
    p->out=0;
    p->pout=0;
    p->iout=0;
    p->dout=0;
    p->out_max=out_max;
    p->iout_max=iout_max;
    p->pout_max = pout_max;
    p->dout_max = dout_max;
}

double pid_calc(pid_t *p, double target, double current)
{
    p->target=target;
    p->current=current;
    p->lasterror=p->error;
    p->error=p->target-p->current;

    p->pout = limit_abs(p->kp*p->error, p->pout_max);
    p->iout+=p->ki * p->error;
    p->iout = limit_abs(p->iout, p->iout_max);
    p->dout = limit_abs(p->kd*(p->error-p->lasterror), p->dout_max);

    p->out=p->pout+p->iout+p->dout;
    p->out=limit_abs(p->out,p->out_max);
    return p->out;
}

double INF_pid_calc(pid_t *p)
{
    p->pout = limit_abs(p->kp*p->error, p->pout_max);
    p->iout += p->ki*p->error;
    p->iout = limit_abs(p->iout, p->iout_max);
    p->dout = limit_abs(p->kd*(p->error - p->lasterror), p->dout_max);

    p->out = limit_abs(p->pout + p->iout + p->dout, p->out_max);
    return p->out;
}

//Encoder Part
void Encoder_init(Encoder_t *p, uint32_t encoderA_gpio, uint32_t encoderB_gpio)
{
    p->count = 0;
    p->direction = 0;
    p->lastA = 0;
    p->lastB = 0;
    p->encoderA_gpio = encoderA_gpio;
    p->encoderB_gpio = encoderB_gpio;
}

void EncoderA_Update(Encoder_t *p)
{
    bool A_state = (DL_GPIO_readPins(ENCODER_INPUT_PORT, p->encoderA_gpio) != 0)?1:0;
    bool B_state = (DL_GPIO_readPins(ENCODER_INPUT_PORT, p->encoderB_gpio) != 0)?1:0;

    if (A_state)
    {
        if (B_state)                        //B高电平,反转
        {
            p->direction = -1;
            p->count--;
        }
        else
        {
            p->direction = 1;
            p->count++;
        }
        
    }
    else                                    //A下降沿 
    {
        if (B_state)                        //B高电平，正转
        {
            p->direction = 1;
            p->count++;
        }
        else
        {
            p->direction = -1;
            p->count--;
        }
    }
    p->lastA = A_state;
    p->lastB = B_state;
}

void EncoderB_Update(Encoder_t *p)
{
    bool A_State = (DL_GPIO_readPins(ENCODER_INPUT_PORT, p->encoderA_gpio) != 0)?1:0;
    bool B_State = (DL_GPIO_readPins(ENCODER_INPUT_PORT, p->encoderB_gpio) != 0)?1:0;
    if (B_State)                            
    {
        if (A_State)                        //A高电平，正转                        
        {
            p->direction = 1;
            p->count--;
        }
        else
        {
            p->direction = -1;
            p->count++;
        }
        
    }
    else                                    
    {
        if (A_State)                        
        {
            p->direction = -1;
            p->count--;
        }
        else
        {
            p->direction = 1;
            p->count++;
        }
    }
    p->lastA = A_State;
    p->lastB = B_State;
}

int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(ENCODER_INPUT_INT_IRQN);

    //declarations
    pid_t Motors_Pid[4], INF_Pid;

    for (int i = 0; i<4; i++)
        pid_init(&Motors_Pid[i], MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_OUT_MAX, MOTOR_PID_POUT_MAX, MOTOR_PID_IOUT_MAX, MOTOR_PID_DOUT_MAX);
    pid_init(&INF_Pid, INF_PID_KP, INF_PID_KI, INF_PID_KD,INF_PID_OUT_MAX, INF_PID_IOUT_MAX, INF_PID_DOUT_MAX, INF_PID_OUT_MAX);

    for (int i = 0; i<4; i++)
    {
        Encoder_init(&Motors_Encoder[i], Encoder_Gpios[i][0], Encoder_Gpios[i][1]);
    }

    while (1) {
        //循迹部分
        int INF_Data[4] = {L2, L1, R1, R2};
        //将数据打包通过串口发送
        sprintf(buff, "L1:%d, L2:%d, R1:%d, R2:%d\r\n", 
        L1, L2, R1, R2);
        UART0_sendString(buff);
        //根据权重计算偏移，若黑线在正中央L1,R1,则误差相抵没有输出，
        //若黑线偏左，则error为负数；若黑线偏右，则error为正数
        INF_Pid.lasterror = INF_Pid.error;
        //计算当前误差
        for (int i = 0; i<4; i++)
        {
            INF_Pid.error += weights[i] * INF_Data[i];
            INF_Data_Sum+=INF_Data[i];
        }
        //计算平均误差
        if (INF_Data_Sum > 0)
            INF_Pid.error /= INF_Data_Sum;
        else                                        //若丢失，则停车
        {    
            MOTOR_setSpeed(0, 0, 0, 0);
            continue;
        }
        //计算循迹pid
        double omega_d = INF_pid_calc(&INF_Pid);
        //VL = V + L/2*omega
        //VR = V + L/2*omega
        //左侧轮子的整体轮速
        double VL_Target = MOTOR_BASIC_SPEED - (Wheel_Distance)/2.0 * omega_d;
        double VR_Target = MOTOR_BASIC_SPEED + (Wheel_Distance)/2.0 * omega_d;

        //将轮速写入电机pid
        for (int i = 0; i<4; i+=2)
        {
            Motors_Pid[i].target = VL_Target;
        }
        for (int i = 1; i<4; i+=2)
        {
            Motors_Pid[i].target = VR_Target;
        }
        //电机pid部分
        //获取编码器轮速和转动方向
        
        

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

double limit_abs(double x, double maxx)
{
    if(x>=maxx) return maxx;
    else if(x<=-maxx) return -maxx;
    else return x;
}


//NVIC
void SysTick_Handler(void)
{
    if (delay_times != 0)
    {
        delay_times--;
    }
}

void GROUP1_IRQHandler(void)                        //GROUP1->GPIOA/B
{
    switch(DL_Interrupt_getPendingGroup(DL_Interrupt_GROUP_1))
    {
        case ENCODER_INPUT_E1A_IIDX:                //E1A被触发
        {
            EncoderA_Update(&Motors_Encoder[0]);
            break;
        }
        case ENCODER_INPUT_E1B_IIDX:                //E1B被触发
        {
            EncoderB_Update(&Motors_Encoder[0]);
            break;
        }
        case ENCODER_INPUT_E2A_IIDX:
        {
            EncoderA_Update(&Motors_Encoder[1]);
            break;
        }
        case ENCODER_INPUT_E2B_IIDX:
        {
            EncoderB_Update(&Motors_Encoder[1]);
            break;
        }
        case ENCODER_INPUT_E3A_IIDX:
        {
            EncoderA_Update(&Motors_Encoder[2]);
            break;
        }
        case ENCODER_INPUT_E3B_IIDX:
        {
            EncoderB_Update(&Motors_Encoder[2]);
            break;
        }
        case ENCODER_INPUT_E4A_IIDX:
        {
            EncoderA_Update(&Motors_Encoder[3]);
            break;
        }
        case ENCODER_INPUT_E4B_IIDX:
        {
            EncoderB_Update(&Motors_Encoder[3]);
            break;
        }
        default: break;
    }
}

