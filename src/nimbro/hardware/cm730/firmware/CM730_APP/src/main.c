/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/23
* Description        : Main program body
* Comment            : This file has been heavily modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Notes:
// Certain parameters and options in this firmware are intended to be configurable.
// Search for '[CONFIG]' in the entire firmware folder in order to locate all such
// parameters. Do not modify anything else unless you really know what you're doing!
//
// In summary:
//
//  - USING_4CELL_BATTERY (isr.h)
//       Non-zero => Set battery warning levels for a 4 cell battery
//       Zero     => Set battery warning levels for a 3 cell battery
//    If you use a 4 cell battery, but set up 3 cell battery voltage level warnings,
//    then you run the risk of over-discharging your LiPo battery, possibly ruining
//    the battery, and in the worst case, possibly even causing an explosive event!
//
//  - DEBUG_USE_JTAG (system_init.h)
//       Non-zero => Enable JTAG debugging
//       Zero     => Disable JTAG debugging
//    If JTAG debugging is enabled then the Zigbee enable pin is hijacked, and all
//    buttons except for the reset button are disabled and *should not* be pressed!
//
//  - FWD_PC_BYTES_DIRECTLY (usart.h)
//       Non-zero => Forward bytes received from the PC directly to the DXLs without
//                   waiting for a complete packet to arrive.
//       Zero     => Only forward complete and valid packets from the PC to the DXLs.
//    Direct PC --> DXL byte forwarding makes the communications faster, but
//    significantly LESS ROBUST! Use with caution, and only if timing is so critical
//    that packet forwarding is experimentally found to be too slow. Also note that
//    the DXLTx packet count ignores all packets that originate from the PC when this
//    option is set, so the DXLTX_PACKET_CNT will be essentially constant/invalid.
//
//  - ALLOW_ZIGBEE (usart.h)
//       Non-zero => Zigbee communications are permitted/enabled
//       Zero     => Zigbee communications are not permitted/disabled
//    Disabling the Zigbee communications is in the form of disallowing Rx/Tx bytes
//    to be sent/received over the corresponding UART peripheral, and disabling the
//    REMOCON functionality in the control register table.

// Includes
#include "stm32f10x_type.h"
#include "system_init.h"
#include "system_func.h"
#include "CM_DXL_COM.h"
#include "led.h"

// Main function
int main(void)
{
	// Declare variables
	u8 bCount;

	// Configure and initialise the system
	System_Configuration();

	// Wait 200ms while flashing LEDs
	for(bCount = 0; bCount < 2; bCount++)
	{
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, ON);
		mDelay(50);
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, OFF);
		mDelay(50);
	}

	// Turn on the power to the arms (for the call to DXLServoTorqueOff() and DXLServoConfig())
	GB_DYNAMIXEL_POWER = 1;
	DXLSetPower(ON); // Note: This raw instruction is used instead of OnControlTableWrite(P_DYNAMIXEL_POWER) as we do not wish to enable DXL forwarding just yet...

	// Wait 100ms while flashing LEDs
	for(bCount = 0; bCount < 1; bCount++)
	{
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, ON);
		mDelay(50);
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, OFF);
		mDelay(50);
	}

	// Command all servos on the dynamixel bus to disable their torque
	DXLServoTorqueOff();

	// Configure the less critical settings of the servo in peace
	DXLServoConfig();

	// Wait 1000ms while flashing LEDs
	for(bCount = 0; bCount < 10; bCount++)
	{
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, ON);
		mDelay(50);
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, OFF);
		mDelay(50);
	}

	// Turn off the power to the arms
	GB_DYNAMIXEL_POWER = 0;
	OnControlTableWrite(P_DYNAMIXEL_POWER);

	// Main loop
	Process();

	// Enter an infinite loop
	while(1);
}
// EOF