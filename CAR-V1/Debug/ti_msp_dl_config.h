/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMA0
#define PWM_MOTOR_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                      DL_GPIO_PIN_0
#define GPIO_PWM_MOTOR_C0_IOMUX                                   (IOMUX_PINCM1)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                  IOMUX_PINCM1_PF_TIMA0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                      DL_GPIO_PIN_1
#define GPIO_PWM_MOTOR_C1_IOMUX                                   (IOMUX_PINCM2)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                  IOMUX_PINCM2_PF_TIMA0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_PWM_MOTOR_C2_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C2_PIN                                     DL_GPIO_PIN_15
#define GPIO_PWM_MOTOR_C2_IOMUX                                  (IOMUX_PINCM37)
#define GPIO_PWM_MOTOR_C2_IOMUX_FUNC                 IOMUX_PINCM37_PF_TIMA0_CCP2
#define GPIO_PWM_MOTOR_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_MOTOR_C3_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C3_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_MOTOR_C3_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_MOTOR_C3_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMA0_CCP3
#define GPIO_PWM_MOTOR_C3_IDX                                DL_TIMER_CC_3_INDEX

/* Defines for PWM_SERVO */
#define PWM_SERVO_INST                                                    TIMG12
#define PWM_SERVO_INST_IRQHandler                              TIMG12_IRQHandler
#define PWM_SERVO_INST_INT_IRQN                                (TIMG12_INT_IRQn)
#define PWM_SERVO_INST_CLK_FREQ                                          4000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_SERVO_C0_PORT                                             GPIOA
#define GPIO_PWM_SERVO_C0_PIN                                     DL_GPIO_PIN_14
#define GPIO_PWM_SERVO_C0_IOMUX                                  (IOMUX_PINCM36)
#define GPIO_PWM_SERVO_C0_IOMUX_FUNC                IOMUX_PINCM36_PF_TIMG12_CCP0
#define GPIO_PWM_SERVO_C0_IDX                                DL_TIMER_CC_0_INDEX



/* Defines for ENCODER1_A */
#define ENCODER1_A_INST                                                  (TIMG8)
#define ENCODER1_A_INST_IRQHandler                              TIMG8_IRQHandler
#define ENCODER1_A_INST_INT_IRQN                                (TIMG8_INT_IRQn)
#define ENCODER1_A_INST_LOAD_VALUE                                      (31999U)
/* GPIO defines for channel 0 */
#define GPIO_ENCODER1_A_C0_PORT                                            GPIOA
#define GPIO_ENCODER1_A_C0_PIN                                    DL_GPIO_PIN_29
#define GPIO_ENCODER1_A_C0_IOMUX                                  (IOMUX_PINCM4)
#define GPIO_ENCODER1_A_C0_IOMUX_FUNC                 IOMUX_PINCM4_PF_TIMG8_CCP0

/* Defines for ENCODER2_A */
#define ENCODER2_A_INST                                                  (TIMG7)
#define ENCODER2_A_INST_IRQHandler                              TIMG7_IRQHandler
#define ENCODER2_A_INST_INT_IRQN                                (TIMG7_INT_IRQn)
#define ENCODER2_A_INST_LOAD_VALUE                                      (31999U)
/* GPIO defines for channel 0 */
#define GPIO_ENCODER2_A_C0_PORT                                            GPIOA
#define GPIO_ENCODER2_A_C0_PIN                                    DL_GPIO_PIN_28
#define GPIO_ENCODER2_A_C0_IOMUX                                  (IOMUX_PINCM3)
#define GPIO_ENCODER2_A_C0_IOMUX_FUNC                 IOMUX_PINCM3_PF_TIMG7_CCP0

/* Defines for ENCODER3_A */
#define ENCODER3_A_INST                                                  (TIMG6)
#define ENCODER3_A_INST_IRQHandler                              TIMG6_IRQHandler
#define ENCODER3_A_INST_INT_IRQN                                (TIMG6_INT_IRQn)
#define ENCODER3_A_INST_LOAD_VALUE                                      (31999U)
/* GPIO defines for channel 0 */
#define GPIO_ENCODER3_A_C0_PORT                                            GPIOA
#define GPIO_ENCODER3_A_C0_PIN                                    DL_GPIO_PIN_21
#define GPIO_ENCODER3_A_C0_IOMUX                                 (IOMUX_PINCM46)
#define GPIO_ENCODER3_A_C0_IOMUX_FUNC                IOMUX_PINCM46_PF_TIMG6_CCP0

/* Defines for ENCODER4_A */
#define ENCODER4_A_INST                                                  (TIMG0)
#define ENCODER4_A_INST_IRQHandler                              TIMG0_IRQHandler
#define ENCODER4_A_INST_INT_IRQN                                (TIMG0_INT_IRQn)
#define ENCODER4_A_INST_LOAD_VALUE                                      (31999U)
/* GPIO defines for channel 0 */
#define GPIO_ENCODER4_A_C0_PORT                                            GPIOA
#define GPIO_ENCODER4_A_C0_PIN                                     DL_GPIO_PIN_5
#define GPIO_ENCODER4_A_C0_IOMUX                                 (IOMUX_PINCM10)
#define GPIO_ENCODER4_A_C0_IOMUX_FUNC                IOMUX_PINCM10_PF_TIMG0_CCP0





/* Defines for CLOCK */
#define CLOCK_INST                                                       (TIMA1)
#define CLOCK_INST_IRQHandler                                   TIMA1_IRQHandler
#define CLOCK_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define CLOCK_INST_LOAD_VALUE                                           (63999U)




/* Defines for I2C_SCR */
#define I2C_SCR_INST                                                        I2C1
#define I2C_SCR_INST_IRQHandler                                  I2C1_IRQHandler
#define I2C_SCR_INST_INT_IRQN                                      I2C1_INT_IRQn
#define GPIO_I2C_SCR_SDA_PORT                                              GPIOB
#define GPIO_I2C_SCR_SDA_PIN                                       DL_GPIO_PIN_3
#define GPIO_I2C_SCR_IOMUX_SDA                                   (IOMUX_PINCM16)
#define GPIO_I2C_SCR_IOMUX_SDA_FUNC                    IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_SCR_SCL_PORT                                              GPIOA
#define GPIO_I2C_SCR_SCL_PIN                                      DL_GPIO_PIN_17
#define GPIO_I2C_SCR_IOMUX_SCL                                   (IOMUX_PINCM39)
#define GPIO_I2C_SCR_IOMUX_SCL_FUNC                    IOMUX_PINCM39_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for LED1: GPIOB.2 with pinCMx 15 on package pin 50 */
#define LED_LED1_PIN                                             (DL_GPIO_PIN_2)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM15)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOA)

/* Defines for KEY1: GPIOA.18 with pinCMx 40 on package pin 11 */
#define KEY_KEY1_PIN                                            (DL_GPIO_PIN_18)
#define KEY_KEY1_IOMUX                                           (IOMUX_PINCM40)
/* Port definition for Pin Group INF_INPUT */
#define INF_INPUT_PORT                                                   (GPIOA)

/* Defines for L1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define INF_INPUT_L1_PIN                                        (DL_GPIO_PIN_25)
#define INF_INPUT_L1_IOMUX                                       (IOMUX_PINCM55)
/* Defines for L2: GPIOA.24 with pinCMx 54 on package pin 25 */
#define INF_INPUT_L2_PIN                                        (DL_GPIO_PIN_24)
#define INF_INPUT_L2_IOMUX                                       (IOMUX_PINCM54)
/* Defines for R2: GPIOA.23 with pinCMx 53 on package pin 24 */
#define INF_INPUT_R2_PIN                                        (DL_GPIO_PIN_23)
#define INF_INPUT_R2_IOMUX                                       (IOMUX_PINCM53)
/* Defines for R1: GPIOA.26 with pinCMx 59 on package pin 30 */
#define INF_INPUT_R1_PIN                                        (DL_GPIO_PIN_26)
#define INF_INPUT_R1_IOMUX                                       (IOMUX_PINCM59)
/* Port definition for Pin Group MOTORS */
#define MOTORS_PORT                                                      (GPIOB)

/* Defines for AIN1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define MOTORS_AIN1_PIN                                          (DL_GPIO_PIN_6)
#define MOTORS_AIN1_IOMUX                                        (IOMUX_PINCM23)
/* Defines for AIN2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define MOTORS_AIN2_PIN                                          (DL_GPIO_PIN_7)
#define MOTORS_AIN2_IOMUX                                        (IOMUX_PINCM24)
/* Defines for BIN1: GPIOB.8 with pinCMx 25 on package pin 60 */
#define MOTORS_BIN1_PIN                                          (DL_GPIO_PIN_8)
#define MOTORS_BIN1_IOMUX                                        (IOMUX_PINCM25)
/* Defines for BIN2: GPIOB.9 with pinCMx 26 on package pin 61 */
#define MOTORS_BIN2_PIN                                          (DL_GPIO_PIN_9)
#define MOTORS_BIN2_IOMUX                                        (IOMUX_PINCM26)
/* Defines for CIN1: GPIOB.16 with pinCMx 33 on package pin 4 */
#define MOTORS_CIN1_PIN                                         (DL_GPIO_PIN_16)
#define MOTORS_CIN1_IOMUX                                        (IOMUX_PINCM33)
/* Defines for CIN2: GPIOB.15 with pinCMx 32 on package pin 3 */
#define MOTORS_CIN2_PIN                                         (DL_GPIO_PIN_15)
#define MOTORS_CIN2_IOMUX                                        (IOMUX_PINCM32)
/* Defines for DIN1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define MOTORS_DIN1_PIN                                         (DL_GPIO_PIN_14)
#define MOTORS_DIN1_IOMUX                                        (IOMUX_PINCM31)
/* Defines for DIN2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define MOTORS_DIN2_PIN                                         (DL_GPIO_PIN_13)
#define MOTORS_DIN2_IOMUX                                        (IOMUX_PINCM30)
/* Defines for ENCODER1_B: GPIOA.7 with pinCMx 14 on package pin 49 */
#define GPIO_ENCODER_ENCODER1_B_PORT                                     (GPIOA)
#define GPIO_ENCODER_ENCODER1_B_PIN                              (DL_GPIO_PIN_7)
#define GPIO_ENCODER_ENCODER1_B_IOMUX                            (IOMUX_PINCM14)
/* Defines for ENCODER2_B: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_ENCODER_ENCODER2_B_PORT                                     (GPIOB)
#define GPIO_ENCODER_ENCODER2_B_PIN                              (DL_GPIO_PIN_0)
#define GPIO_ENCODER_ENCODER2_B_IOMUX                            (IOMUX_PINCM12)
/* Defines for ENCODER3_B: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_ENCODER_ENCODER3_B_PORT                                     (GPIOB)
#define GPIO_ENCODER_ENCODER3_B_PIN                             (DL_GPIO_PIN_18)
#define GPIO_ENCODER_ENCODER3_B_IOMUX                            (IOMUX_PINCM44)
/* Defines for ENCODER4_B: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_ENCODER_ENCODER4_B_PORT                                     (GPIOA)
#define GPIO_ENCODER_ENCODER4_B_PIN                             (DL_GPIO_PIN_27)
#define GPIO_ENCODER_ENCODER4_B_IOMUX                            (IOMUX_PINCM60)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_PWM_SERVO_init(void);
void SYSCFG_DL_ENCODER1_A_init(void);
void SYSCFG_DL_ENCODER2_A_init(void);
void SYSCFG_DL_ENCODER3_A_init(void);
void SYSCFG_DL_ENCODER4_A_init(void);
void SYSCFG_DL_CLOCK_init(void);
void SYSCFG_DL_I2C_SCR_init(void);
void SYSCFG_DL_UART_0_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
