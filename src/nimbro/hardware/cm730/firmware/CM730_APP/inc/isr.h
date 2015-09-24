/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : isr.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/26
* Description        : Contains the interrupt service routines for the firmware
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is only included once
#ifndef ISR_H
#define ISR_H

// Includes
#include "stm32f10x_type.h"

// Set whether a 3 cell (11.1V) or 4 cell (14.8V) LiPo battery is being used
#define USING_4CELL_BATTERY         1 // [CONFIG] Non-zero => Set battery warning levels for a 4 cell battery, Zero => Set battery warning levels for a 3 cell battery
#define USING_3CELL_BATTERY         (!USING_4CELL_BATTERY)

// Define the battery voltage levels
#if USING_4CELL_BATTERY
#define VOLTAGE_LEVEL_6             172  // 17.2V Overcharged
#define VOLTAGE_LEVEL_5             168  // 16.8V Fully charged
#define VOLTAGE_LEVEL_4             159  // 15.9V      ...
#define VOLTAGE_LEVEL_3             155  // 15.5V      ...
#define VOLTAGE_LEVEL_2             151  // 15.1V      ...
#define VOLTAGE_LEVEL_1             147  // 14.7V Warning level
#define VOLTAGE_LEVEL_0             141  // 14.1V Empty
#else
#define VOLTAGE_LEVEL_6             130  // 13.0V Overcharged
#define VOLTAGE_LEVEL_5             126  // 12.6V Fully charged
#define VOLTAGE_LEVEL_4             119  // 11.9V      ...
#define VOLTAGE_LEVEL_3             116  // 11.6V      ...
#define VOLTAGE_LEVEL_2             113  // 11.3V      ...
#define VOLTAGE_LEVEL_1             110  // 11.0V Warning level
#define VOLTAGE_LEVEL_0             106  // 10.6V Empty
#endif
#define LOW_BATTERY_LIMIT           VOLTAGE_LEVEL_1
#define HIGH_BATTERY_LIMIT          VOLTAGE_LEVEL_6

// Interrupt service routines
void ISR_TIMER2(void);
void ISR_BATTERY_CHECK(void);
void ISR_1MS_TIMER(void);
void ISR_GYRO_ACC_SPI(void);
void ISR_COMPASS_I2C(void);
void ISR_BUZZER(void);
void ISR_BUTTON(void);
void ISR_RX_TX_LED(void);
void ISR_PC_USART(void);
void ISR_PC_RXHANDLER(void);
void ISR_PC_TXHANDLER(void);
void ISR_DXL_USART(void);
void ISR_DXL_RXHANDLER(void);
void ISR_DXL_TXHANDLER(void);
void ISR_ZIG_USART(void);
void ISR_LED_RGB_TIMER(void);
void ISR_DELAY(void);
void ISR_ADC(void);

// Externed variables
extern vu32 gbMillisec;

#endif /* ISR_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
