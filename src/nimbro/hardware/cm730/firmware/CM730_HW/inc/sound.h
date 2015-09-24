/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : sound.h
* Author             : Robotis
* Version            : V0.1
* Date               : 2011/02/17
* Description        : Functions to control sound output and the buzzer
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is only included once
#ifndef SOUND_H
#define SOUND_H

// Includes
#include "stm32f10x_type.h"

// Interrupt service routine for buzzer
void __ISR_BUZZER_MANAGE(void);

// Play functions
void PlayBuzzer(void);

// Buzzer functions
void setBuzzer(u16 periodUs);
u8   getBuzzerState(void);
void setBuzzerPlayLength(u8 length);
u8   getBuzzerPlayLength(void);
void setBuzzerData(u8 data);
u8   getBuzzerData(void);
void setBuzzerOff(void);

#endif /* SOUND_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
