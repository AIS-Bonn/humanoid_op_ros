/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : led.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/24
* Description        : Contains the functions and defines for the LEDs
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is only included once
#ifndef LED_H
#define LED_H

// Includes
#include "common_type.h"

// LED port flags
#define LED_MANAGE  0x01
#define LED_EDIT    0x02
#define LED_PLAY    0x04
#define LED_RX      0x08
#define LED_TX      0x10
#define LED_ALL     0x5F

// RGB LED port flags
#define RGBLED5     5
#define RGBLED6     6

// RGB LED colour flags
#define LED_R       0x01
#define LED_G       0x02
#define LED_B       0x04

// Interrupt service routine to control the RGB LEDs
void __ISR_LED_RGB_TIMER(void);

// Functions
PowerState LED_GetState(u8 LED_PORT);
void       LED_SetState(u8 LED_PORT, PowerState NewState);
u8   LED5_GetState();       // Uses the flags LED_R, LED_G and LED_B
void LED5_SetState(u8 RGB); // Uses the flags LED_R, LED_G and LED_B
void RGBLED_SetColour(u8 RGBLED_PORT, u8 R, u8 G, u8 B);

#endif /* LED_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
