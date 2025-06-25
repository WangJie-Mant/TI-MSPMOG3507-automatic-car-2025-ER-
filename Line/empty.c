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

volatile unsigned int delay_times = 0;
volatile unsigned char uart_data = 0;
volatile unsigned int inf_data[4] = {0};
volatile uint32_t gpios[4] = {INF_INPUT_L1_PIN, INF_INPUT_L2_PIN, INF_INPUT_R1_PIN, INF_INPUT_R2_PIN};
unsigned int states[4] = {0};
char buf[64];

void Uart0_sendChar(char ch);
void Uart0_sendString(char* str);

void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while (delay_times != 0);
}

int main(void)
{
    SYSCFG_DL_init();

    while (1) {
        Uart0_sendString("Next Round...\r\n");
        delay_ms(5000);//test
        for (int i = 0; i < 4; i++)
        {
            states[i] = (DL_GPIO_readPins(INF_INPUT_PORT, gpios[i]) != 0)? 1: 0;
        }
        sprintf(buf, "L1:%d, L2:%d, L3:%d, L4:%d\r\n", states[0],states[1],states[2],states[3]);
        Uart0_sendString(buf);
    }
}

void Uart0_sendChar(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

void Uart0_sendString(char* str)
{
    while (*str != 0 && str != 0)
    {
        Uart0_sendChar(*str++);
    }
}

void INF_getGPIOstate(GPIO_Regs* gpio, uint32_t pin[], int times, uint8_t states[])
{
    for (int i = 0; i<times; i++)
    {
        states[i] = (DL_GPIO_readPins(gpio, pin[i]) != 0)? 1:0;
    }
}

void SysTick_Handler(void)
{
    if (delay_times != 0) delay_times--;
}
