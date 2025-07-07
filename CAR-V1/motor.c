#include "motor.h"
#include "ti_msp_dl_config.h"

float Speed_Low_Filter(float newSpeed, encoder_t* encoder);

void motor_Init()
{
    NVIC_EnableIRQ(ENCODER1_A_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER2_A_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER3_A_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER4_A_INST_INT_IRQN);

    NVIC_EnableIRQ(CLOCK_INST_INT_IRQN);

    motorA.PWM_Inst = PWM_MOTOR_INST;
    motorA.duty = 0;
    motorA.speed = 0;
    motorA.timer_index = DL_TIMER_CC_0_INDEX;
    motorA.forward_GPIO_PORT = MOTORS_PORT;
    motorA.reverse_GPIO_PORT = MOTORS_PORT;
    motorA.forward_GPIO_PIN = MOTORS_AIN1_PIN;
    motorA.reverse_GPIO_PIN = MOTORS_AIN2_PIN;
    motorA.PWM_Channel = GPIO_PWM_MOTOR_C0_IDX;

    motorB.PWM_Inst = PWM_MOTOR_INST;
    motorB.duty = 0;
    motorB.speed = 0;
    motorB.timer_index = DL_TIMER_CC_0_INDEX;
    motorB.forward_GPIO_PORT = MOTORS_PORT;
    motorB.reverse_GPIO_PORT = MOTORS_PORT;
    motorB.forward_GPIO_PIN = MOTORS_BIN1_PIN;
    motorB.reverse_GPIO_PIN = MOTORS_BIN2_PIN;
    motorB.PWM_Channel = GPIO_PWM_MOTOR_C1_IDX;

    motorC.PWM_Inst = PWM_MOTOR_INST;
    motorC.duty = 0;
    motorC.speed = 0;
    motorC.timer_index = DL_TIMER_CC_0_INDEX;
    motorC.forward_GPIO_PORT = MOTORS_PORT;
    motorC.reverse_GPIO_PORT = MOTORS_PORT;
    motorC.forward_GPIO_PIN = MOTORS_CIN1_PIN;
    motorC.reverse_GPIO_PIN = MOTORS_CIN2_PIN;
    motorC.PWM_Channel = GPIO_PWM_MOTOR_C2_IDX;

    motorD.PWM_Inst = PWM_MOTOR_INST;
    motorD.duty = 0;
    motorD.speed = 0;
    motorD.timer_index = DL_TIMER_CC_0_INDEX;
    motorD.forward_GPIO_PORT = MOTORS_PORT;
    motorD.reverse_GPIO_PORT = MOTORS_PORT;
    motorD.forward_GPIO_PIN = MOTORS_DIN1_PIN;
    motorD.reverse_GPIO_PIN = MOTORS_DIN2_PIN;
    motorD.PWM_Channel = GPIO_PWM_MOTOR_C3_IDX;

}

float Speed_Low_Filter(float newSpeed, encoder_t* encoder)
{
    float sum = 0.0f;
    uint32_t test_Speed = newSpeed;
    for (uint8_t i = SPEED_RECORD_NUM - 1; i>0; i--)
    {
        encoder->speed_Record[i] = encoder->speed_Record[i-1];
        sum += encoder->speed_Record[i-1];
    }
    encoder->speed_Record[0] = newSpeed;
    sum += newSpeed;
    test_Speed = sum/SPEED_RECORD_NUM;
    return sum/SPEED_RECORD_NUM;
}

void ENCODER1_A_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(ENCODER1_A_INST))
    {
        case DL_TIMERG_IIDX_CC0_DN:
        {
            motorA.encoder.direct = (DL_GPIO_readPins(GPIO_ENCODER_ENCODER1_B_PORT, GPIO_ENCODER_ENCODER1_B_PIN) != 0)? 1:0;
            motorA.encoder.count = 
                motorA.encoder.direct? (motorA.encoder.count + 1): (motorA.encoder.count -1);
            break;
        }
        default:
            break;
    }
}

void ENCODER2_A_INST_IRQHandler(void)
{
    switch(DL_TimerG_getPendingInterrupt(ENCODER2_A_INST))
    {
        case DL_TIMERG_IIDX_CC0_DN:
        {
            motorB.encoder.direct = DL_GPIO_readPins(GPIO_ENCODER_ENCODER2_B_PORT, GPIO_ENCODER_ENCODER2_B_PIN);
            motorB.encoder.count = 
                motorB.encoder.direct? (motorB.encoder.count + 1): (motorB.encoder.count -1);
            break;
        }
        default:
            break;
    }
}

void ENCODER3_A_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(ENCODER3_A_INST))
    {
        case DL_TIMERG_IIDX_CC0_DN:
        {
            motorC.encoder.direct = DL_GPIO_readPins(GPIO_ENCODER_ENCODER3_B_PORT, GPIO_ENCODER_ENCODER3_B_PIN);
            motorC.encoder.count = 
                motorC.encoder.direct? (motorC.encoder.count + 1) : (motorC.encoder.count - 1);
            break;
        }
        default:
            break;
    }
}

void ENCODER4_A_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(ENCODER4_A_INST))
    {
        case DL_TIMERG_IIDX_CC0_DN:
        {
            motorD.encoder.direct = DL_GPIO_readPins(GPIO_ENCODER_ENCODER4_B_PORT, GPIO_ENCODER_ENCODER4_B_PIN);
            motorD.encoder.count = 
                motorD.encoder.direct? (motorD.encoder.count + 1) : (motorD.encoder.count - 1);
            break;
        }
        default: break;
    }
}

/*PRTIOD 10ms / 100Hz */
void CLOCK_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(CLOCK_INST))
    {
        case DL_TIMER_IIDX_ZERO:
        {
            DL_TimerA_clearInterruptStatus(CLOCK_INST, DL_TIMER_IIDX_ZERO);
            motorA.encoder.speed = (float)(motorA.encoder.count - motorA.encoder.lastcount)*6000/PULSE_PER_CYCLE;
            motorB.encoder.speed = (float)(motorB.encoder.count - motorB.encoder.lastcount)*6000/PULSE_PER_CYCLE;
            motorC.encoder.speed = (float)(motorC.encoder.count - motorC.encoder.lastcount)*6000/PULSE_PER_CYCLE;
            motorD.encoder.speed = (float)(motorD.encoder.count - motorD.encoder.lastcount)*6000/PULSE_PER_CYCLE;

            motorA.encoder.speed = Speed_Low_Filter(motorA.encoder.speed, &motorA.encoder);
            motorB.encoder.speed = Speed_Low_Filter(motorB.encoder.speed, &motorB.encoder);
            motorC.encoder.speed = Speed_Low_Filter(motorC.encoder.speed, &motorC.encoder);
            motorD.encoder.speed = Speed_Low_Filter(motorD.encoder.speed, &motorD.encoder);

            motorA.encoder.lastcount = motorA.encoder.count;
            motorB.encoder.lastcount = motorB.encoder.count;
            motorC.encoder.lastcount = motorC.encoder.count;
            motorD.encoder.lastcount = motorD.encoder.count;

            break;
        }
        default: break;
    }
}