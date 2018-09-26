/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : system_init.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : Functions and macros relating to system initialisation
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is included only once
#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

// Includes
#include "common_type.h"

//
// General
//

// Hardware specification
#if defined(FORCE_CM730)
#define IS_CM730 1
#define IS_CM740 0
#elif defined(FORCE_CM740)
#define IS_CM730 0
#define IS_CM740 1
#else
#define IS_CM730 1
#define IS_CM740 0
#endif
#if (IS_CM730 + IS_CM740) != 1
#error "Invalid hardware specification => Check system_init.h!"
#endif

// Servo hardware specification
#if defined(FORCE_MX_SERVOS)
#define MX_SERVOS 1
#define X_SERVOS 0
#elif defined(FORCE_X_SERVOS)
#define MX_SERVOS 0
#define X_SERVOS 1
#else
#define MX_SERVOS 1
#define X_SERVOS 0
#endif
#if (MX_SERVOS + X_SERVOS) != 1
#error "Invalid servo specification => Check system_init.h!"
#endif

// Interrupt macros
#define DISABLE_USART_INTERRUPTS()     GLOBAL_INTERRUPT_DISABLE()  // Disables   all USART, handler and timer interrupts
#define REENABLE_USART_INTERRUPTS()    GLOBAL_INTERRUPT_ENABLE()   // Re-enables all USART, handler and timer interrupts
#define DISABLE_HANDLER_INTERRUPTS()   DISABLE_INTERRUPTS_PRI1()   // Disables   the handler and timer interrupts
#define REENABLE_HANDLER_INTERRUPTS()  REENABLE_INTERRUPTS_PRI()   // Re-enables the handler and timer interrupts
#define DISABLE_TIMER_INTERRUPTS()     DISABLE_INTERRUPTS_PRI2()   // Disables   the timer interrupts
#define REENABLE_TIMER_INTERRUPTS()    REENABLE_INTERRUPTS_PRI()   // Re-enables the timer interrupts

// Configuration defines
#define DEBUG_USE_JTAG      0     // [CONFIG] Non-zero => Enable JTAG debugging (the Zigbee enable pin is hijacked, and all buttons except for the reset button are disabled and *should not* be pressed!)

// Defines
#define TIM2_PRESCALER      722   // TIM2_PERIOD = (TIM2_PRESCALER + 1)/SYS_CLOCK = 723/72MHz = 10.042us
#define TIM2_CCR1_VAL       778   // Period   778 * 10.042 = 7.812ms
#define TIM2_CCR2_VAL       100   // Period   100 * 10.042 = 1.004ms
#define TIM2_CCR3_VAL       12    // Period    12 * 10.042 = 120.5us
#define TIM2_CCR4_VAL       4     // Period     4 * 10.042 = 40.17us

//
// Port A
//

// Pin definitions
#define PIN_ADC4            GPIO_Pin_0
#define PIN_ADC5            GPIO_Pin_1
#define PIN_ADC6            GPIO_Pin_2
#define PIN_ADC7            GPIO_Pin_3
#define PIN_ADC8            GPIO_Pin_4
#define PIN_ADC9            GPIO_Pin_5
#define PIN_ADC10           GPIO_Pin_6
#define PIN_ADC11           GPIO_Pin_7
#define PIN_LED6_R          GPIO_Pin_8
#define PIN_CPU_TXD         GPIO_Pin_9
#define PIN_CPU_RXD         GPIO_Pin_10
#define PIN_LED6_G          GPIO_Pin_11
#define PIN_LED6_B          GPIO_Pin_12
#define PIN_USB_SLEEP       GPIO_Pin_13
#define PIN_SW_MODE         GPIO_Pin_14
#define PIN_SW_START        GPIO_Pin_15

// Port definitions
#define PORT_ADC4           GPIOA
#define PORT_ADC5           GPIOA
#define PORT_ADC6           GPIOA
#define PORT_ADC7           GPIOA
#define PORT_ADC8           GPIOA
#define PORT_ADC9           GPIOA
#define PORT_ADC10          GPIOA
#define PORT_ADC11          GPIOA
#define PORT_LED6_R         GPIOA
#define PORT_CPU_TXD        GPIOA
#define PORT_CPU_RXD        GPIOA
#define PORT_LED6_G         GPIOA
#define PORT_LED6_B         GPIOA
#define PORT_USB_SLEEP      GPIOA
#define PORT_SW_MODE        GPIOA
#define PORT_SW_START       GPIOA

//
// Port B
//

// Pin definitions
#define PIN_ADC14           GPIO_Pin_0
#define PIN_ADC15           GPIO_Pin_1
#define PIN_BOOT1           GPIO_Pin_2
#define PIN_ENABLE_ZIGBEE   GPIO_Pin_3
#define PIN_ENABLE_TX       GPIO_Pin_4
#define PIN_ENABLE_RX       GPIO_Pin_5
#define PIN_DXL_TXD         GPIO_Pin_6
#define PIN_DXL_RXD         GPIO_Pin_7
#define PIN_ENABLE_DXLPWR   GPIO_Pin_8
#define PIN_SIG_BUZZER      GPIO_Pin_9
#define PIN_PC_TXD          GPIO_Pin_10
#define PIN_PC_RXD          GPIO_Pin_11
#define PIN_LED3            GPIO_Pin_12
#define PIN_SIG_SCK         GPIO_Pin_13
#define PIN_SIG_MISO        GPIO_Pin_14
#define PIN_SIG_MOSI        GPIO_Pin_15

// Port definitions
#define PORT_ADC14          GPIOB
#define PORT_ADC15          GPIOB
#define PORT_BOOT1          GPIOB
#define PORT_ENABLE_ZIGBEE  GPIOB
#define PORT_ENABLE_TX      GPIOB
#define PORT_ENABLE_RX      GPIOB
#define PORT_DXL_TXD        GPIOB
#define PORT_DXL_RXD        GPIOB
#define PORT_ENABLE_DXLPWR  GPIOB
#define PORT_SIG_BUZZER     GPIOB
#define PORT_PC_TXD         GPIOB
#define PORT_PC_RXD         GPIOB
#define PORT_LED3           GPIOB
#define PORT_SIG_SCK        GPIOB
#define PORT_SIG_MISO       GPIOB
#define PORT_SIG_MOSI       GPIOB

//
// Port C
//

// Pin definitions
#define PIN_ADC0            GPIO_Pin_0
#define PIN_ADC1            GPIO_Pin_1
#define PIN_ADC2            GPIO_Pin_2
#define PIN_ADC3            GPIO_Pin_3
#define PIN_ADC12           GPIO_Pin_4
#define PIN_ADC13           GPIO_Pin_5
#define PIN_LED4            GPIO_Pin_6
#define PIN_LED5_R          GPIO_Pin_7
#define PIN_LED5_G          GPIO_Pin_8
#define PIN_LED5_B          GPIO_Pin_9
#define PIN_SIG_ACC_CS      GPIO_Pin_10
#define PIN_SIG_GYRO_CS     GPIO_Pin_11
#define PIN_ZIGBEE_TXD      GPIO_Pin_12
#define PIN_LED_TX          GPIO_Pin_13
#define PIN_LED_RX          GPIO_Pin_14
#define PIN_LED2            GPIO_Pin_15

// Port definitions
#define PORT_ADC0           GPIOC
#define PORT_ADC1           GPIOC
#define PORT_ADC2           GPIOC
#define PORT_ADC3           GPIOC
#define PORT_ADC12          GPIOC
#define PORT_ADC13          GPIOC
#define PORT_LED4           GPIOC
#define PORT_LED5_R         GPIOC
#define PORT_LED5_G         GPIOC
#define PORT_LED5_B         GPIOC
#define PORT_SIG_ACC_CS     GPIOC
#define PORT_SIG_GYRO_CS    GPIOC
#define PORT_ZIGBEE_TXD     GPIOC
#define PORT_LED_TX         GPIOC
#define PORT_LED_RX         GPIOC
#define PORT_LED2           GPIOC

//
// Port D
//

// Pin definitions
#define PIN_ZIGBEE_RXD      GPIO_Pin_2

// Port definitions
#define PORT_ZIGBEE_RXD     GPIOD

//
// LED definitions
//

// Pin definitions
#define PIN_LED_MANAGE      PIN_LED4
#define PIN_LED_EDIT        PIN_LED3
#define PIN_LED_PLAY        PIN_LED2

// Port definitions
#define PORT_LED_MANAGE     PORT_LED4
#define PORT_LED_EDIT       PORT_LED3
#define PORT_LED_PLAY       PORT_LED2

//
// Compass definitions
//

// Pin definitions
#define PIN_COMPASS_SDA_CM730   PIN_ADC2
#define PIN_COMPASS_SCL_CM730   PIN_ADC3
#define PIN_COMPASS_SDA_CM740   PIN_ZIGBEE_RXD
#define PIN_COMPASS_SCL_CM740   PIN_ZIGBEE_TXD

// Port definitions
#define PORT_COMPASS_SDA_CM730  PORT_ADC2
#define PORT_COMPASS_SCL_CM730  PORT_ADC3
#define PORT_COMPASS_SDA_CM740  PORT_ZIGBEE_RXD
#define PORT_COMPASS_SCL_CM740  PORT_ZIGBEE_TXD

// Select the required definitions
#if IS_CM730
#define PIN_COMPASS_SDA         PIN_COMPASS_SDA_CM730
#define PIN_COMPASS_SCL         PIN_COMPASS_SCL_CM730
#define PORT_COMPASS_SDA        PORT_COMPASS_SDA_CM730
#define PORT_COMPASS_SCL        PORT_COMPASS_SCL_CM730
#endif
#if IS_CM740
#define PIN_COMPASS_SDA         PIN_COMPASS_SDA_CM740
#define PIN_COMPASS_SCL         PIN_COMPASS_SCL_CM740
#define PORT_COMPASS_SDA        PORT_COMPASS_SDA_CM740
#define PORT_COMPASS_SCL        PORT_COMPASS_SCL_CM740
#endif

//
// Functions
//

// Configuration functions
void System_Configuration(void);
void USART_Configuration(u8 PORT, u32 baudrate);
u32  USART_GetBaudrate(u8 PORT);

#endif /* SYSTEM_INIT_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
