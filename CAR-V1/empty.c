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
#include "motor.h"


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
void Motor_Set(int speed, motor_t *p);
void MOTOR_setSpeed(int speedA, int speedB, int speedC, int speedD);
double limit_abs(double x, double maxx);
//double MOTOR_getSpeed();
void Motor_Set_Position(motor_t *motor);
void Motor_Set_Speed(motor_t *motor);
void Motor_Stop(motor_t *motor);
void Motor_Set_Duty(float duty, motor_t *motor);



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
char buff[64];
double weights[4] = {-2, -1, 1, 2};
motor_t motors[4];
double INF_Data_Sum = 0;
double Wheel_Distance = 0;

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
    p->dead_zone = 0;
}

double pid_calc(pid_t *p, double target, double current)
{
    p->target=target;
    p->current=current;
    
    p->error=p->target-p->current;

    if (p->error < p->dead_zone && p->error > -p->dead_zone) p->error = 0;

    p->pout = limit_abs(p->kp*p->error, p->pout_max);
    p->iout+=p->ki * p->error;
    p->iout = limit_abs(p->iout, p->iout_max);
    p->dout = limit_abs(p->kd*(p->error-p->lasterror), p->dout_max);

    p->out=p->pout+p->iout+p->dout;
    p->out=limit_abs(p->out,p->out_max);
    p->lasterror=p->error;
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

void motors_init()
{
    motors[0] = motorA;
    motors[1] = motorB;
    motors[2] = motorC;
    motors[3] = motorD;
}

int main(void){
    SYSCFG_DL_init();

    //declarations

    motor_Init();
    motors_init();
    pid_t INF_Pid;

    pid_init(&INF_Pid, INF_PID_KP, INF_PID_KI, INF_PID_KD, INF_PID_OUT_MAX, INF_PID_POUT_MAX, INF_PID_IOUT_MAX, INF_PID_DOUT_MAX);
    for (int i = 0 ; i <4; i++)
        pid_init(&motors[i].speed_pid, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_OUT_MAX, MOTOR_PID_POUT_MAX, MOTOR_PID_IOUT_MAX,MOTOR_PID_DOUT_MAX);
    for (int i = 0; i<4; i++)
        pid_init(&motors[i].pos_pid, MOTOR_POS_KP, MOTOR_POS_KI, MOTOR_POS_KD, MOTOR_POS_OUT_MAX, MOTOR_POS_POUT_MAX, MOTOR_POS_IOUT_MAX, MOTOR_POS_DOUT_MAX);


    for (int i = 0; i<4; i++)
    {
        pid_init(&motors[i].pos_pid, MOTOR_POS_KP, MOTOR_POS_KI, MOTOR_POS_KD, MOTOR_POS_OUT_MAX, MOTOR_POS_POUT_MAX, MOTOR_POS_IOUT_MAX, MOTOR_POS_DOUT_MAX);
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
        //电机pid部分
        //获取编码器轮速和转动方向，已经在motor.c中的中断实现
        //计算速度pid控制量：
        double pwm_left = limit_abs(pid_calc(&motorA.speed_pid, VL_Target, motorA.encoder.speed),1.0f);
        double pwm_right = limit_abs(pid_calc(&motorB.speed_pid, VR_Target, motorB.encoder.speed), 1.0f);

        Motor_Set_Duty(pwm_left, &motorA);
        Motor_Set_Duty(pwm_right, &motorB);
        Motor_Set_Duty(pwm_left, &motorC);
        Motor_Set_Duty(pwm_right, &motorD);
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


void Motor_Set(int speed, motor_t *p)
{
    //-255 <= speed <= 255 
    if (speed > 0)
    {
        PWM_setDuty((float)speed/255.0f, p->PWM_Inst, p->PWM_Channel);
        DL_GPIO_writePins(p->forward_GPIO_PORT, p->forward_GPIO_PIN);
        DL_GPIO_clearPins(p->reverse_GPIO_PORT, p->reverse_GPIO_PIN);
    }
    else if (speed < 0)
    {
        PWM_setDuty((float)(-speed) / 255.0f, p->PWM_Inst, p->PWM_Channel);
        DL_GPIO_writePins(p->reverse_GPIO_PORT, p->reverse_GPIO_PIN);
        DL_GPIO_clearPins(p->forward_GPIO_PORT, p->forward_GPIO_PIN);
    }
    else {
        PWM_setDuty(0, p->PWM_Inst, p->PWM_Channel);
        DL_GPIO_clearPins(p->forward_GPIO_PORT, p->forward_GPIO_PIN);
        DL_GPIO_clearPins(p->reverse_GPIO_PORT, p->reverse_GPIO_PIN);
    }
}


void Motor_Set_Duty(float duty, motor_t *p)
{
    if (duty > 0)
    {
        PWM_setDuty(duty, p->PWM_Inst, p->PWM_Channel);
        DL_GPIO_writePins(p->forward_GPIO_PORT, p->forward_GPIO_PIN);
        DL_GPIO_clearPins(p->reverse_GPIO_PORT, p->reverse_GPIO_PIN);
    }
    else if (duty < 0)
    {
        PWM_setDuty(-duty, p->PWM_Inst, p->PWM_Channel);
        DL_GPIO_writePins(p->reverse_GPIO_PORT, p->reverse_GPIO_PIN);
        DL_GPIO_clearPins(p->forward_GPIO_PORT, p->forward_GPIO_PIN);
    }
    else {
        PWM_setDuty(0, p->PWM_Inst, p->PWM_Channel);
        DL_GPIO_clearPins(p->forward_GPIO_PORT, p->forward_GPIO_PIN);
        DL_GPIO_clearPins(p->reverse_GPIO_PORT, p->reverse_GPIO_PIN);
    }
}

void MOTOR_setSpeed(int speedA, int speedB, int speedC, int speedD)
{
    Motor_Set(speedA, &motors[0]);
    Motor_Set(speedB, &motors[1]);
    Motor_Set(speedC, &motors[2]);
    Motor_Set(speedD, &motors[3]);
}

double limit_abs(double x, double maxx)
{
    if(x>=maxx) return maxx;
    else if(x<=-maxx) return -maxx;
    else return x;
}

void Motor_Set_Speed(motor_t *motor)
{
    motor->duty = pid_calc(&motor->speed_pid, motor->speed, motor->encoder.speed);
    PWM_setDuty(motor->duty, motor->PWM_Inst, motor->PWM_Channel);
}

void Motor_Set_Position(motor_t *motor)
{
    motor->pos_pulse = (int)((motor->position*PULSE_PER_CYCLE)/(LINE_SPEED_C));
    motor->speed = pid_calc(&motor->pos_pid, motor->pos_pulse, motor->encoder.count);
    Motor_Set_Speed(motor);
}

void Motor_Stop(motor_t *motor)
{
    motor->speed = 0;
    Motor_Set_Speed(motor);
    if (motor->encoder.speed == 0)
    {
        motor->encoder.count = 0;
        motor->encoder.lastcount = 0;
        motor->speed_pid.iout = 0;
    }

}

//NVIC
void SysTick_Handler(void)
{
    if (delay_times != 0)
    {
        delay_times--;
    }
}



