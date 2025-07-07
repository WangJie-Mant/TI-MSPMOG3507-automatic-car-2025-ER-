#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stdlib.h"
#include "ti_msp_dl_config.h"
// #include "ti/devices/msp/peripherals/hw_gptimer.h"

#define MOTOR_SPEED_RERATIO 20
#define PULSE_PER_ROUND 13
#define ENCODER_MULTIPLE 1.0

#define PULSE_PER_CYCLE     (MOTOR_SPEED_RERATIO*PULSE_PER_ROUND*ENCODER_MULTIPLE)
#define RADIUS_OF_TIRE      3.3
#define LINE_SPEED_C        RADIUS_OF_TIRE*2*3.14
#define SPEED_RECORD_NUM    20


#define MOTOR_BASIC_SPEED           100
//motor pid paras
#define MOTOR_PID_KP                0.1
#define MOTOR_PID_KI                1
#define MOTOR_PID_KD                0.1
#define MOTOR_PID_OUT_MAX           0
#define MOTOR_PID_POUT_MAX          0
#define MOTOR_PID_IOUT_MAX          0
#define MOTOR_PID_DOUT_MAX          0

#define MOTOR_POS_KP                0.1
#define MOTOR_POS_KI                1
#define MOTOR_POS_KD                0.1
#define MOTOR_POS_OUT_MAX           100
#define MOTOR_POS_POUT_MAX          30
#define MOTOR_POS_IOUT_MAX          0
#define MOTOR_POS_DOUT_MAX          30

typedef struct
{
    uint8_t direct;
    uint32_t count;
    uint32_t lastcount;
    float speed;
    float speed_Record[SPEED_RECORD_NUM];
} encoder_t;

typedef struct
{
    double kp, ki, kd;
    double target, current;
    double error, lasterror;
    double out, pout, iout, dout;
    double pout_max, iout_max, dout_max, out_max;
    double dead_zone;
} pid_t;

typedef struct
{
    uint8_t direction;
    uint8_t timer_index;
    int32_t pos_pulse;
    int32_t forward_GPIO_PIN;
    int32_t reverse_GPIO_PIN;
    float duty;
    float speed;
    float position;
    encoder_t encoder;
    pid_t pos_pid;
    pid_t speed_pid;
    GPIO_Regs *forward_GPIO_PORT;
    GPIO_Regs *reverse_GPIO_PORT;
    DL_TIMER_CC_INDEX PWM_Channel;
    GPTIMER_Regs* PWM_Inst;
} motor_t;

motor_t motorA, motorB, motorC, motorD;
void motor_Init();
float Speed_Low_Filter(float newSpeed, encoder_t* encoder);

#endif