/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : button.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : Contains the functions and defines for the buttons
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "button.h"
#include "stm32f10x_lib.h"
#include "system_init.h"

// Read which buttons are currently pressed (returns OR-ed button flags for the actuated buttons)
u8 ReadButton(void)
{
	// Declare variables
	u8 retval = 0;

	// Check which buttons are currently being pressed
	if(GPIO_ReadInputDataBit(PORT_SW_START, PIN_SW_START) != SET ) retval |= BUTTON_START;
	if(GPIO_ReadInputDataBit(PORT_SW_MODE , PIN_SW_MODE ) != SET ) retval |= BUTTON_MODE;

	// Ignore all buttons (except for the reset button) if JTAG debugging is enabled
#if DEBUG_USE_JTAG
	retval = 0;
#endif

	// Return the required OR-ed value
	return retval;
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
