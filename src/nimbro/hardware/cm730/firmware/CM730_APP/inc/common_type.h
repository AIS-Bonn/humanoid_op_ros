/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : common_type.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/24
* Description        : Contains the common data types and defines for the firmware
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is included only once
#ifndef COMMON_TYPE_H
#define COMMON_TYPE_H

// Includes
#include "cortexm3_macro.h"

// Preprocessor typedefs
#define byte                        u8
#define word                        u16

// Key codes
#define ESC_KEY                     0x1B
#define TAB_KEY                     0x09
#define ENTER_KEY                   0x0D

// ASCII codes
#define CTRL_A                      1
#define BEEP                        7
#define BS_ASCII                    8 
#define LF_ASCII                    10
#define CLEAR_SCREEN                12
#define CR_ASCII                    13

// Interrupt macros
#define GLOBAL_INTERRUPT_DISABLE()  __SETPRIMASK()            // Globally disable all interrupts using PRIMASK
#define GLOBAL_INTERRUPT_ENABLE()   __RESETPRIMASK()          // Globally enable all interrupts using PRIMASK
#define DISABLE_INTERRUPTS_PRI1()   __BASEPRICONFIG(0x1 << 6) // Use BASEPRI interrupt masking to disable all interrupts with priority number greater-equal 0100[0000] (disabling the most)
#define DISABLE_INTERRUPTS_PRI2()   __BASEPRICONFIG(0x2 << 6) // Use BASEPRI interrupt masking to disable all interrupts with priority number greater-equal 1000[0000] (..................)
#define DISABLE_INTERRUPTS_PRI3()   __BASEPRICONFIG(0x3 << 6) // Use BASEPRI interrupt masking to disable all interrupts with priority number greater-equal 1100[0000] (disabling the least)
#define REENABLE_INTERRUPTS_PRI()   __BASEPRICONFIG(0x0 << 6) // Disable selective masking of interrupts via BASEPRI

// Byte packing helper macros
#define WORD_CAST(AA)               (*(u16 *)(&(AA))) // Assumes a little endian packing of the two bytes in memory (i.e. that the least significant byte is at the lower address = &AA)
#define LOW_BYTE(AA)                ((u8)((AA)&0xFF)) // Assumes AA is 16-bit
#define HIGH_BYTE(AA)               ((u8)((AA)>>8))   // Assumes AA is 16-bit

// Enumerations
typedef enum {OFF = 0, ON = !OFF} PowerState;

#endif /* COMMON_TYPE_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
