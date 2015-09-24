/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : button.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : Contains the functions and defines for the buttons
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is only included once
#ifndef BUTTON_H
#define BUTTON_H

// Includes
#include "stm32f10x_type.h"

// Button flags
#define BUTTON_MODE   0x1
#define BUTTON_START  0x2

// Functions
u8 ReadButton(void);

#endif /* BUTTON_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
