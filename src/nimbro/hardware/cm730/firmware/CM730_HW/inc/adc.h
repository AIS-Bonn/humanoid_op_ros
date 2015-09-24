/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : adc.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/09/08
* Description        : Contains the functions and defines for ADC conversions
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is only included once
#ifndef ADC_H
#define ADC_H

// Includes
#include "stm32f10x_type.h"

// ADC port definitions
#define ADC_PORT0      0
#define ADC_PORT1      1
#define ADC_PORT2      2
#define ADC_PORT3      3
#define ADC_PORT4      4
#define ADC_PORT5      5
#define ADC_PORT6      6
#define ADC_PORT7      7
#define ADC_PORT8      8
#define ADC_PORT9      9
#define ADC_PORT10     10
#define ADC_PORT11     11
#define ADC_PORT12     12
#define ADC_PORT13     13
#define ADC_PORT14     14
#define ADC_PORT15     15
#define ADC_NUMBER     16

// ADC port aliases
#define ADC_VOLTAGE    ADC_PORT0

// Functions
void initADCISR(void);
u16  getADC(u8 ADCPort);
u8   getBatteryVoltage(void);

// Interrupt service routine to manage ADC conversions
void __ISR_ADC(void);

#endif /* ADC_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
