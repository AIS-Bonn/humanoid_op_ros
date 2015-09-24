/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : isr.c
* Author             : danceww
* Version            : V0.1
* Date               : 2010/08/23
* Description        : Contains the interrupt service routines for the firmware
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "isr.h"
#include "stm32f10x_lib.h"
#include "system_init.h"
#include "system_func.h"
#include "CM_DXL_COM.h"
#include "gyro_acc.h"
#include "compass.h"
#include "button.h"
#include "sound.h"
#include "usart.h"
#include "adc.h"
#include "led.h"

// Defines
#define LOW_BATTERY_COUNT         20   // Number of consecutive battery voltage measurements required below the nominal threshold before a tone is played
#define SOUND_LOW_VOLTAGE_LENGTH  255  // Low voltage sound length
#define SOUND_LOW_VOLTAGE_DATA    22   // Low voltage sound data

// Global variables
vu8  gbCounter0 = 0;
vu16 gwCounter1 = 0;
vu32 gbMillisec = 0;

// Handle a timer 2 interrupt
void ISR_TIMER2(void)
{
	// Rate calculation:
	// System clock frequency = 72MHz, TIM2_PRESCALER = 722
	// TIM2_PERIOD = (TIM2_PRESCALER + 1) / SYS_CLOCK = 10.041667us
	// TIM2_CCR4_VAL = 4 => CC4 Elapse Period = 4 * TIM2_PERIOD = 40.1667us
	// The condition ((gwCounter1 & (2^n)-1) == X) for X < 2^n is true exactly when the lowest n bits of gwCounter1 are X.
	// The frequency at which this occurs is the counting frequency of gwCounter1, downscaled by a factor of 2^n,
	// independent of what X actually is. As such, the values of X can be chosen so that no two such conditions are
	// true at the same time, minimising the number of instructions that a call to this ISR requires.

	// Check whether the timer has elapsed with respect to CC4 and perform the required actions if so
	if(TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET) // CC4 Period 40.1667us / Freq 24896Hz
	{
		// Clear the timer elapsed bit and reset the timer counter
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		TIM_SetCounter(TIM2, 0);
		TIM_SetCompare4(TIM2, TIM2_CCR4_VAL);

		// Period 40.1667us / Freq 24896Hz
		ISR_COMPASS_I2C();

		// Update the base downscale counter (scales 24896Hz down to 8299Hz)
		if(++gbCounter0 == 3)
			gbCounter0 = 0;

		// Execute the downscaled ISRs (this IF must be entered at 8299Hz)
		if(gbCounter0 == 0)
		{
			// Period 120.5us / Freq 8299Hz
			ISR_ADC();

			// Period 482.0us / Freq 2075Hz
			if((gwCounter1 & 3) == 0)
			{
				ISR_LED_RGB_TIMER();
			}

			// Period 964.0us / Freq 1037Hz
			if((gwCounter1 & 7) == 1)
			{
				ISR_1MS_TIMER();
			}

			// Period 3856us / Freq 259.3Hz
			if((gwCounter1 & 31) == 2)
			{
				ISR_GYRO_ACC_SPI();
				ISR_BUZZER();
				ISR_BUTTON();
			}

			// Period 123.4ms / Freq 8.104Hz
			if((gwCounter1 & 1023) == 3)
			{
				ISR_RX_TX_LED();
			}

			// Period 987.1ms / Freq 1.013Hz
			if((gwCounter1 & 8191) == 5)
			{
				ISR_BATTERY_CHECK();
			}

			// Increment the downscale counter
			gwCounter1++;
		}
	}
}

// Check the current battery voltage
void ISR_BATTERY_CHECK(void)
{
	// Declare variables
	static u8 bLowBatteryCount;

	// Retrieve the battery voltage and write it to the control table
	GB_BATTERY_VOLTAGE = getBatteryVoltage();

	// Count how many consecutive voltage measurements have violated the low battery limit
	if(GB_BATTERY_VOLTAGE <= LOW_BATTERY_LIMIT)
	{
		if(bLowBatteryCount < LOW_BATTERY_COUNT)
			bLowBatteryCount++;
	}
	else bLowBatteryCount = 0;

	// Play the low battery sound if our voltage has consistently been observed to be too low
	if(bLowBatteryCount == LOW_BATTERY_COUNT)
	{
		setBuzzerPlayLength(SOUND_LOW_VOLTAGE_LENGTH);
		setBuzzerData(SOUND_LOW_VOLTAGE_DATA);
		if(!getBuzzerState())
			PlayBuzzer();
	}

	// Set the voltage error bit if necessary
	if((GB_BATTERY_VOLTAGE <= VOLTAGE_LEVEL_0) || (GB_BATTERY_VOLTAGE > VOLTAGE_LEVEL_6))
		gbAlarmState |=  VOLTAGE_ERROR_BIT;
	else
		gbAlarmState &= ~VOLTAGE_ERROR_BIT;
}

// Handle elapse of the ~1ms timer (it's actually 0.964ms)
void ISR_1MS_TIMER(void)
{
	// Increment the millisecond counter (overflows every 49.7 days!)
	gbMillisec++;
}

// Handle gyroscope and accelerometer chip communications
void ISR_GYRO_ACC_SPI(void)
{
	__ISR_GYRO_ACC_SPI();
}

// Handle compass chip communications
void ISR_COMPASS_I2C(void)
{
	__ISR_COMPASS_I2C();
}

// Manage the buzzer
void ISR_BUZZER(void)
{
	__ISR_BUZZER_MANAGE();
}

// Retrieve the current button states
void ISR_BUTTON(void)
{
	// Read the button states
	GB_BUTTON = ReadButton();
}

// Handle the RX and TX LED states
void ISR_RX_TX_LED(void)
{
	// Turn off the RX and TX LEDs
	LED_SetState(LED_RX, OFF);
	LED_SetState(LED_TX, OFF);
}

// Handle the PC USART
void ISR_PC_USART(void)
{
	__ISR_PC_USART();
}

// Handle the PC Rx communications
void ISR_PC_RXHANDLER(void)
{
	__ISR_PC_RXHANDLER();
}

// Handle the PC Tx communications
void ISR_PC_TXHANDLER(void)
{
	__ISR_PC_TXHANDLER();
}

// Handle the DXL USART
void ISR_DXL_USART(void)
{
	__ISR_DXL_USART();
}

// Handle the DXL Rx communications
void ISR_DXL_RXHANDLER(void)
{
	__ISR_DXL_RXHANDLER();
}

// Handle the DXL Tx communications
void ISR_DXL_TXHANDLER(void)
{
	__ISR_DXL_TXHANDLER();
}

// Handle the ZIG USART
void ISR_ZIG_USART(void)
{
	__ISR_ZIG_USART();
}

// Handle the RGB LEDs
void ISR_LED_RGB_TIMER(void)
{
	__ISR_LED_RGB_TIMER();
}

// Handle the SysTick elapse event for timing delays (refer to mDelay())
void ISR_DELAY(void)
{
	__ISR_DELAY();
}

// Handle ADC interrupts
void ISR_ADC(void)
{
	__ISR_ADC();
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
