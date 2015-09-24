/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : zigbee.c
* Author             : Robotis
* Version            : -
* Date               : -
* Description        : Functions relating to Zigbee communications
* Comment            : This file has been modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "zigbee.h"
#include "stm32f10x_lib.h"
#include "system_init.h"
#include "system_func.h"
#include "zgb_hal.h"
#include "usart.h"

// Global variables
u8  gbRcvPacket[6] = {0};
u16 gwRcvPacketNum = 0;
u16 gwRcvData = 0;
u8  gbRcvFlag = 0;
u16 gwMyZigbeeID = 0;

// Initialise the Zigbee device
u8 zgb_initialize(u8 devIndex)
{
	// Open the Zigbee device with a fixed baudrate
	if(zgb_hal_open(devIndex, 57600) == 0) return 0;

	// Initialise the global variables
	gbRcvFlag = 0;
	gwRcvData = 0;
	gwRcvPacketNum = 0;

	// Return success
	return 1;
}

// Terminate the Zigbee connection
void zgb_terminate(void)
{
	// Close the Zigbee device
	zgb_hal_close();
}

// Transmit data over the Zigbee connection
u8 zgb_tx_data(u16 data)
{
	// Declare variables
	u8 SendPacket[6];
	u8 lowbyte, highbyte;

	// Split the data word into its respective bytes
	lowbyte = LOW_BYTE(data);
	highbyte = HIGH_BYTE(data);

	// Construct the required packet
	SendPacket[0] = 0xFF;
	SendPacket[1] = 0x55;
	SendPacket[2] = lowbyte;
	SendPacket[3] = ~lowbyte;
	SendPacket[4] = highbyte;
	SendPacket[5] = ~highbyte;

	// Transmit the packet
	if(zgb_hal_tx(SendPacket, 6) != 6) return 0;

	// Return success
	return 1;
}

// Check whether there is a Zigbee Rx packet available, and if so extract its data (Returns 0 => No data available, 1 => Data available, access with zgb_rx_data())
u8 zgb_rx_check(void)
{
	// Declare variables
	u16 RcvNum, i, j;

	// If we already have extracted data available then the answer is clearly yes
	if(gbRcvFlag == 1) return gbRcvFlag;

	// Fill the packet buffer with received Zigbee data
	if(gwRcvPacketNum < 6)
	{
		RcvNum = zgb_hal_rx(&gbRcvPacket[gwRcvPacketNum], 6 - gwRcvPacketNum);
		if(RcvNum != ((u16) -1))
			gwRcvPacketNum += RcvNum;
	}

	// Find a packet header in the received data
	if(gwRcvPacketNum >= 2)
	{
		// Find an 0xFF 0x55 packet header in the received data
		for(i = 0; i < gwRcvPacketNum; i++)
		{
			if(gbRcvPacket[i] == 0xFF)
			{
				if(i <= gwRcvPacketNum - 2)
				{
					if(gbRcvPacket[i+1] == 0x55)
						break;
				}
			}
		}

		// If the packet header doesn't already coincide with the start of our packet buffer, then ensure it does
		if(i > 0)
		{
			// If no 0xFF 0x55 pair was found in the received data then see whether the last byte at least could be the start of such a pair
			if((i == gwRcvPacketNum) && (gbRcvPacket[i-1] == 0xFF)) i--;

			// Remove all the bytes preceding the packet header
			for(j = i; j < gwRcvPacketNum; j++)
				gbRcvPacket[j-i] = gbRcvPacket[j];
			gwRcvPacketNum -= i;
		}
	}

	// Verify the receipt of a packet if we have six bytes starting with a packet header
	if(gwRcvPacketNum == 6)
	{
		if((gbRcvPacket[0] == 0xFF) && (gbRcvPacket[1] == 0x55) && (gbRcvPacket[2] == ~gbRcvPacket[3]) && (gbRcvPacket[4] == ~gbRcvPacket[5]))
		{
			gwRcvData = (((u16) gbRcvPacket[4]) << 8) | ((u16) gbRcvPacket[2]);
			gbRcvFlag = 1; // A packet has just been received and correctly parsed!
		}
		gbRcvPacket[0] = 0x00;
		gwRcvPacketNum = 0;
	}

	// Return whether a word of data is available in gwRcvData
	return gbRcvFlag;
}

// Retrieve the current parsed Zigbee received data
u16 zgb_rx_data(void)
{
	// Clear the data available flag
	gbRcvFlag = 0;

	// Return the required data
	return gwRcvData;
}

// Scan for the Zigbee ID
u16 zgb_scan_id(void)
{
	// Declare variables
	u8 checkcount = 2;

	// Reset the current Zigbee ID
	gwMyZigbeeID = 0;

	// Try to ascertain the required ID
	while((checkcount > 0) && (gwMyZigbeeID == 0))
	{
		// Cycle the power on the Zigbee device and clear the local Rx buffer for the Zigbee
		Zigbee_SetState(OFF);
		mDelay(2);
		USARTClearBuffers(USART_ZIG, USART_CLEAR_RX);
		Zigbee_SetState(ON);
		mDelay(10);

		// Send a '!!' token
		TxDData(USART_ZIG,'!');
		TxDData(USART_ZIG,'!');

		// Wait for a certain amount of time to see whether there is a response
		mDelay(305);

		// Look for a response '!' token as a sign of life
		while(RxDDataAvailable(USART_ZIG))
		{
			if(RxDData(USART_ZIG) == '!')
			{
				gwMyZigbeeID = 1;
				break;
			}
		}

		// Decrement the number of times still to check
		checkcount--;
	}

	// Cycle the power on the Zigbee device
	mDelay(50);
	Zigbee_SetState(OFF);
	mDelay(2);
	Zigbee_SetState(ON);

	// Return the resulting ID
	return gwMyZigbeeID;
}

// Set the power state of the Zigbee
void Zigbee_SetState(PowerState state)
{
	// Enable/disable the Zigbee by setting/resetting the corresponding pin
	if(state == ON) GPIO_ResetBits(PORT_ENABLE_ZIGBEE, PIN_ENABLE_ZIGBEE);
	else            GPIO_SetBits  (PORT_ENABLE_ZIGBEE, PIN_ENABLE_ZIGBEE);
}
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
