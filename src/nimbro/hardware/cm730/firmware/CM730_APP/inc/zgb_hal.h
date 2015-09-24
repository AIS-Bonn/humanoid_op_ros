/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : zgb_hal.h
* Author             : Robotis
* Version            : -
* Date               : -
* Description        : Functions relating to the Zigbee hardware abstraction layer
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is included only once
#ifndef ZGB_HAL_H
#define ZGB_HAL_H

// Includes
#include "stm32f10x_type.h"

// Functions
u8   zgb_hal_open(u8 devIndex, u32 baudrate);
void zgb_hal_close(void);
u16  zgb_hal_tx(u8 *pPacket, u16 numPacket);
u16  zgb_hal_rx(u8 *pPacket, u16 numPacket);

#endif /* ZGB_HAL_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
