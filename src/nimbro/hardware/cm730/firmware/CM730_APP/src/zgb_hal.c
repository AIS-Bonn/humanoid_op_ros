/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : zgb_hal.c
* Author             : Robotis
* Version            : -
* Date               : -
* Description        : Functions relating to the Zigbee hardware abstraction layer
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "zgb_hal.h"
#include "system_init.h"
#include "usart.h"

// Open the Zigbee device
u8 zgb_hal_open(u8 devIndex, u32 baudrate)
{
	// Configure the baudrate of the Zigbee USART port
	USART_Configuration(USART_ZIG, 57600); // Note: We actually fix the baudrate instead of looking at what was passed!

	// Return success
	return 1;
}

// Close the Zigbee device
void zgb_hal_close(void)
{
	// Note: No actions required to close the Zigbee device
}

// Transmit a packet to the Zigbee device
u16 zgb_hal_tx(u8 *pPacket, u16 numPacket)
{
	// Declare variables
	u16 i;

	// Transmit the packet byte for byte
	for(i = 0; i < numPacket; i++)
		TxDData(USART_ZIG, pPacket[i]);

	// Return success
	return numPacket;
}

// Wait to receive a given number of bytes from the Zigbee device (there must be at least one byte of data already available)
u16 zgb_hal_rx(u8 *pPacket, u16 numPacket)
{
	// Declare variables
	u16 cnt = 0;

	// If no data is available yet then return immediately
	if(!RxDDataAvailable(USART_ZIG)) return ((u16) -1);

	// Receive the required number of bytes one by one
	while(1)
	{
		if(RxDDataAvailable(USART_ZIG))
		{
			pPacket[cnt++] = RxDData(USART_ZIG);
			if(cnt >= numPacket) break;
		}
		else break;
	}

	// Return the number of bytes that were read
	return cnt;
}
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
