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

uint32_t period = 32000;
volatile unsigned int delay_time;

void delay_ms(unsigned int ms)
{
    delay_time = ms;
    while (delay_time != 0)
        ;
}

void set_Duty(float duty, uint32_t channel)
{
    uint32_t CompareValue;
    CompareValue = period - period * duty;
    if (channel == 0)
        DL_Timer_setCaptureCompareValue(PWM_0_INST, CompareValue, DL_TIMER_CC_0_INDEX);
    else
        DL_Timer_setCaptureCompareValue(PWM_0_INST, CompareValue, DL_TIMER_CC_1_INDEX);
}

void set_Freq(uint32_t freq, uint32_t channel)
{
    period = PWM_0_INST_CLK_FREQ / freq;
    DL_Timer_setLoadValue(PWM_0_INST, period);
}

int main(void)
{
    SYSCFG_DL_init();
    DL_TimerG_startCounter(PWM_0_INST);

    float duty = 0.0f;
    float step = 0.01f;
    int direction = 1;

    while (1)
    {
        set_Duty(duty, 0);

        duty += step * direction;
        if (duty >= 0.99f)
        {
            duty = 0.99f;
            direction = -1;
        }
        if (duty <= 0.01f)
        {
            duty = 0.01f;
            direction = 1;
        }

        delay_ms(100);
    }
}

void SysTick_Handler(void)
{
    if (delay_time != 0)
    {
        delay_time--;
    }
}
