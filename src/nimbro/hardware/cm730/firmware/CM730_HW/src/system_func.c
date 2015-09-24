/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : system_func.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : Contains system-related functions and defines
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
 *******************************************************************************/

// Includes
#include "system_func.h"
#include "stm32f10x_lib.h"
#include "system_init.h"

// Global variables
vu16 gwTimingDelay; // Use to count the milliseconds in mDelay()
u32 tmpdly;         // Used to make sure that the compiler doesn't optimise away the calculations in the uDelay() function
PowerState gbDxlPwr = OFF;

//
// Delay functions
//

// Delay for a given number of microseconds
void uDelay(u32 usec)
{
	// Declare variables
	u32 max;

	// Wait for the desired amount of time
	tmpdly = 1;
	for(max = 1; max <= usec; max++) tmpdly *= max;
	for(max = 1; max <= usec; max++) tmpdly *= max;
	for(max = 1; max <= usec; max++) tmpdly *= max;
	for(max = 1; max <= usec; max++) tmpdly *= max;
	for(max = 1; max <= usec; max++) tmpdly *= max;
	for(max = 1; max <= usec; max++) tmpdly *= max;
	for(max = 1; max <= usec; max++) tmpdly *= 729;
	for(max = 1; max <= usec; max++) tmpdly *= 411;
}

// Delay for a given number of milliseconds
void mDelay(u16 msec)
{
	// Enable the SysTick counter
	SysTick_CounterCmd(SysTick_Counter_Enable);

	// Wait for the __ISR_DELAY() function to decrement the timing delay variable to zero (one decrement per ms)
	gwTimingDelay = msec;
	while(gwTimingDelay != 0);

	// Disable and clear the SysTick counter
	SysTick_CounterCmd(SysTick_Counter_Disable);
	SysTick_CounterCmd(SysTick_Counter_Clear);
}

// Interrupt service routine for the SysTick timer elapse (period 1ms)
void __ISR_DELAY(void)
{
	// Decrement the timing delay variable to zero
	if(gwTimingDelay != 0)
		gwTimingDelay--;
}

//
// Utility functions
//

// Determine the cause of the last reset
u8 getResetSource(void)
{
	// Declare variables
	u8 retval = NO_RESET;

	// Check the RCC reset flags to determine the cause of the last reset
	     if (RCC_GetFlagStatus(RCC_FLAG_PORRST ) == SET) retval = POWER_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_PINRST ) == SET) retval = PIN_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_SFTRST ) == SET) retval = SOFT_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) retval = IWDG_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) == SET) retval = WWDG_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST) == SET) retval = LOW_POWER_RESET;

	// Clear the RCC reset flags
	RCC_ClearFlag();

	// Return the required value
	return retval;
}

// Turn on/off the MOSFET that controls the power to the NimbRo-OP arms
void DXLSetPower(PowerState state)
{
	// Toggle the dynamixel power output pin as required
	if(state == ON) GPIO_SetBits  (PORT_ENABLE_DXLPWR, PIN_ENABLE_DXLPWR);
	else            GPIO_ResetBits(PORT_ENABLE_DXLPWR, PIN_ENABLE_DXLPWR);
	gbDxlPwr = state;
}

//
// EEPROM functions
//

// Read a word from the EEPROM memory
u16 EEPROM_Read(u32 Offset)
{
	// Read the required word
	u16* Adr;
	Adr = (u16*) (EEPROM_START_ADDRESS + (Offset << 1));
	return *Adr;
}

// Write a word to the EEPROM memory
void EEPROM_Write(u32 Offset, u16 Data)
{
	// Declare variables
	u16 Buffer[512];
	u16 cnt;
	u32 Adr;

	// Calculate the starting address
	Adr = EEPROM_START_ADDRESS + (Offset << 1);

	// Check that the value we wish to write is actually new, and that our address offset is in range
	if((Data != EEPROM_Read(Offset)) && (Offset < 512))
	{
		// Read out the entire EEPROM memory and insert our new data
		for(cnt = 0; cnt < 512; cnt++)
			Buffer[cnt] = EEPROM_Read(cnt);
		Buffer[Offset] = Data;

		// Unlock the Flash and clear all pending flags
		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

		// Write the data to Flash
		if((Data == 0) || (EEPROM_Read(Offset) == 0xFFFF))
		{
			// Write just our new data into the EEPROM
			FLASH_ProgramHalfWord(Adr, Data);
		}
		else
		{
			// Erase the entire Flash
			FLASH_ErasePage(EEPROM_START_ADDRESS);

			// Write the entire Flash contents anew
			Adr = EEPROM_START_ADDRESS;
			for(cnt = 0; cnt < 512; cnt++)
			{
				if(Buffer[cnt] != 0xFFFF)
					FLASH_ProgramHalfWord(Adr, Buffer[cnt]);
				Adr += 2;
			}
		}

		// Lock the Flash
		FLASH_Lock();
	}
}

// Clear the EEPROM memory
void EEPROM_Clear(void)
{
	// Erase the EEPROM memory
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(EEPROM_START_ADDRESS);
	FLASH_Lock();
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
