/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : system_func.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : Contains system-related functions and defines
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is only included once
#ifndef SYSTEM_FUNC_H
#define SYSTEM_FUNC_H

//Includes
#include "common_type.h"

// Chip reset types
#define NO_RESET              0
#define PIN_RESET             1
#define POWER_RESET           2
#define SOFT_RESET            3
#define IWDG_RESET            4 // Independent watchdog reset
#define WWDG_RESET            5 // Window watchdog reset
#define LOW_POWER_RESET       6

// Defines
#define EEPROM_START_ADDRESS  ((u32) 0x0807F000) // EEPROM emulation start address => After 64KB of Flash memory

// Global variables
extern PowerState gbDxlPwr;

// Delay functions
void uDelay(u32 usec);
void mDelay(u16 msec);
void __ISR_DELAY(void);

// Utility functions
u8   getResetSource(void);
void DXLSetPower(PowerState state);

// EEPROM functions
u16  EEPROM_Read (u32 Offset);
void EEPROM_Write(u32 Offset, u16 Data);
void EEPROM_Clear(void);

#endif /* SYSTEM_FUNC_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
