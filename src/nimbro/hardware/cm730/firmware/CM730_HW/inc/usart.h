/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : usart.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/24
* Description        : Functions and macros relating to USART
* Comment            : This file has been completely rewritten by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Ensure header is included only once
#ifndef USART_H
#define USART_H

// Includes
#include "stm32f10x_type.h"
#include "system_init.h"

// Defines - USART buffer clear flags
#define USART_CLEAR_NONE      0x0
#define USART_CLEAR_RX        0x1
#define USART_CLEAR_TX        0x2
#define USART_CLEAR_ALL       (USART_CLEAR_RX|USART_CLEAR_TX)

// Defines - USART ports
#define USART_DXL  0
#define USART_ZIG  1
#define USART_PC   2

// Defines - Interrupts
#define IRQ_DXL_RXHANDLER      EXTI1_IRQChannel
#define IRQ_PC_RXHANDLER       EXTI2_IRQChannel
#define IRQ_DXL_TXHANDLER      EXTI3_IRQChannel
#define IRQ_PC_TXHANDLER       EXTI4_IRQChannel

// Packet sizes
#define MAX_PACKET_PARAMS      240  // Maximum number of parameters that is accepted in a packet
#define MIN_PACKET_BYTES       6    // Minimum number of bytes possible in a valid packet

// Configuration defines
#define FWD_PC_BYTES_DIRECTLY  1    // [CONFIG] Non-zero => Forward bytes received from the PC directly to the DXLs without waiting for a complete packet to arrive. This makes the communications faster, but LESS ROBUST!
#define ALLOW_ZIGBEE           0    // [CONFIG] Non-zero => Zigbee communications are permitted/enabled

// Force disable zigbee if on CM740
#if IS_CM740
#undef ALLOW_ZIGBEE
#define ALLOW_ZIGBEE 0
#endif

// Structs
struct DxlPacket
{
	u8 ID;                       // Dynamixel device ID
	u8 Instruction;              // Instruction (instruction packet) or error (status packet) byte of the packet
	u8 NumParams;                // Number of parameters (stored in Param[]) of the packet
	u8 Param[MAX_PACKET_PARAMS]; // Parameter array
};

// Initialisation functions
void USARTInit(u8 PORT);
void USARTClearBuffers(u8 PORT, u8 flag);
void USARTResetCounters(u8 PORT, u8 flag);
void USARTUpdateControlTable();

// Communication functions
u8   RxDDataAvailable(u8 PORT);                      // Use for all ports
u8   RxDData(u8 PORT);                               // Use for USART_ZIG only
void RxDDataDP(u8 PORT, struct DxlPacket *DP);       // Use for USART_DXL and USART_PC only
u8   TxDDataTransmitting(u8 PORT);                   // Use for all ports
void TxDData(u8 PORT, u8 data);                      // Use for USART_ZIG only
void TxDDataDP(u8 PORT, const struct DxlPacket *DP); // Use for USART_DXL and USART_PC only
void WaitForTxDData(u8 PORT);                        // Use for all ports

// Interrupt service routines to handle the USART communications
void __ISR_PC_USART(void);
void __ISR_PC_RXHANDLER(void);
void __ISR_PC_TXHANDLER(void);
void __ISR_DXL_USART(void);
void __ISR_DXL_RXHANDLER(void);
void __ISR_DXL_TXHANDLER(void);
void __ISR_ZIG_USART(void);

// Buffering and dynamixel forwarding
void enableDXLBuffering(void);
void disableDXLBuffering(void);
void enableDXLForwarding(void);
void disableDXLForwarding(void);

#endif /* USART_H */
/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
