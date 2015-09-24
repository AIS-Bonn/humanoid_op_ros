/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : zigbee.h
* Author             : Robotis
* Version            : -
* Date               : -
* Description        : Functions relating to Zigbee communications
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is only included once
#ifndef ZIGBEE_H
#define ZIGBEE_H

// Includes
#include "common_type.h"

// Device control functions
u8   zgb_initialize(u8 devIndex);
void zgb_terminate(void);

// Communication functions
u8   zgb_tx_data(u16 data);
u8   zgb_rx_check(void);
u16  zgb_rx_data(void);
u16  zgb_scan_id(void);
void Zigbee_SetState(PowerState state);

// RC-100 button key values
#define RC100_BTN_U  0x0001
#define RC100_BTN_D  0x0002
#define RC100_BTN_L  0x0004
#define RC100_BTN_R  0x0008
#define RC100_BTN_1  0x0010
#define RC100_BTN_2  0x0020
#define RC100_BTN_3  0x0040
#define RC100_BTN_4  0x0080
#define RC100_BTN_5  0x0100
#define RC100_BTN_6  0x0200

#endif /* ZIGBEE_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
