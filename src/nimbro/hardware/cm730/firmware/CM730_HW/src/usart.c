/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : usart.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : Functions and macros relating to USART
* Comment            : This file has been completely rewritten by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "usart.h"
#include "stm32f10x_lib.h"
#include "system_init.h"
#include "CM_DXL_COM.h"
#include "dxl_defs.h"
#include "isr.h"
#include "led.h"

// Includes - Library
#include <string.h> // For memcpy() - When we use memcpy() we always cast the pointers to [const] char * first so as to be absolutely sure to remove any pointer alignment assumptions by the compiler (which can cause hard faults!)

// Defines - Buffers
#define USART_RX_BUFMASK       0x3FF                        // Use this as an AND-mask to wrap the buffer pointers
#define USART_RX_BUFSIZE       (USART_RX_BUFMASK+1)         // The size of the Rx buffers
#define USART_TX_BUFMASK       0x3FF                        // Use this as an AND-mask to wrap the buffer pointers
#define USART_TX_BUFSIZE       (USART_TX_BUFMASK+1)         // The size of the Tx buffers
#define USART_ZIG_RX_BUFMASK   0x3FF                        // Use this as an AND-mask to wrap the buffer pointers
#define USART_ZIG_RX_BUFSIZE   (USART_ZIG_RX_BUFMASK+1)     // The size of the ZIG Rx buffer

// Defines - Mutex
#define MUTEX_NONE             0x00                         // Mutex value if none of the flags are set
#define MUTEX_LOCK             0x01                         // Mutex locked flag
#define MUTEX_INT_WAS_BLOCKED  0x02                         // Execution of interrupt was blocked by locked mutex flag
#define MUTEX_INT_EXECUTED     0x04                         // Interrupt has successfully executed

// Defines - Protocol
#define DXL_BYTE_TIMEOUT       25                           // Maximum allowed time between consecutive bytes in a dynamixel packet (in ms) [Note: In the Dynamixel protocol this maximum is specified as 100ms, but here we run a tighter ship!]
#define INIT_TIME_LAST_BYTE    (-((u32)DXL_BYTE_TIMEOUT)-1) // Value that ensures that the very first received byte is seen as a timeout (=> start of new packet)

// Defines - Misc
#define FALSE                  0
#define TRUE                   1

// Enumerations
enum ParseState
{
	SEEN_NOTHING,
	SEEN_1xFF,
	SEEN_2xFF,
	SEEN_ID,
	SEEN_LENGTH,
	SEEN_INSTRUCTION,
	SEEN_PARAMS,
	SEEN_ALL
};

// Structs
struct RxParseInfo
{
	volatile enum ParseState State; // Current state of the packet parsing state machine (see RxProcessByte())
	vu8 ID;                         // +
	vu8 Length;                     // |
	vu8 Instruction;                // |
	vu8 NumParams;                  // | Variables used for saving parsed packet data
	vu8 ParamCount;                 // |
	vu8 Param[MAX_PACKET_PARAMS];   // |
	vu8 CheckSum;                   // +
};
struct RxInfo
{
	vu8  RawBuf[USART_RX_BUFSIZE];  // Circular buffer used to store raw received data bytes
	vu16 RawBufReadPtr;             // The index in the circular buffer RawBuf that will be next read
	vu16 RawBufWritePtr;            // The index in the circular buffer RawBuf that will be next written to
	vu8  Buf[USART_RX_BUFSIZE];     // Circular buffer that should exclusively contain valid packets, back to back
	vu16 BufReadPtr;                // The index in the circular buffer Buf that will be next read
	vu16 BufWritePtr;               // The index in the circular buffer Buf that will be next written to
	vu16 BufPacketCount;            // The number of packets received via the Rx buffer
	vu16 BufOverflowCount;          // The number of packets lost due to buffer overflow
	vu16 BufErrorCount;             // The number of buffer errors detected (various possible causes, there is a critical bug if this is ever non-zero)
	vu16 CheckSumErrorCount;        // Number of potential packets discarded due to checksum errors
	vu16 OREErrorCount;             // Number of USART ORE errors that are encountered
	struct RxParseInfo ParseInfo;   // Storage struct for information parsed from the Rx data bytes
	vu32 TimeLastByte;              // The time in ms of the last received Rx data byte
	vu8 Mutex;                      // Mutex used to safely handle the Rx buffer being modified in interrupts
	vu8 Port;                       // The identifier of the USART port that the RxInfo object belongs to (i.e. USART_DXL, USART_PC)
};
struct TxInfo
{
	vu8  RawBuf[USART_TX_BUFSIZE];  // Circular buffer used to store raw data bytes to be transmitted
	vu16 RawBufReadPtr;             // The index in the circular buffer RawBuf that will be next read
	vu16 RawBufWritePtr;            // The index in the circular buffer RawBuf that will be next written to
	vu8  Buf[USART_TX_BUFSIZE];     // Circular buffer that should exclusively contain valid packets, back to back
	vu16 BufReadPtr;                // The index in the circular buffer Buf that will be next read
	vu16 BufWritePtr;               // The index in the circular buffer Buf that will be next written to
	vu16 BufReadDebt;               // The minimum number of bytes that BufReadPtr still needs to be advanced in order to return to a valid circular buffer state
	vu8  BufMoreData;               // Flag that indicates whether more data is available in the packet buffer that couldn't be transcribed into the raw buffer due to space restrictions
	vu16 BufPacketCount;            // The number of packets sent via the Tx buffer
	vu16 BufOverflowCount;          // The number of packets lost due to buffer overflow
	vu16 BufErrorCount;             // The number of buffer errors detected (various possible causes, there is a critical bug if this is ever non-zero)
	vu8 Transmitting;               // Flag that indicates whether Buf is currently being transmitted/flushed or not
	vu8 Mutex;                      // Mutex used to safely handle the Rx buffer being modified in interrupts
	vu8 Port;                       // The identifier of the USART port that the RxInfo object belongs to (i.e. USART_DXL, USART_PC)
};

// Interrupt functions
inline void SetInterruptPending(u8 IRQChannel);

// Reset functions
void ResetRxParseInfo(struct RxParseInfo *RPI);
void ResetRxInfo(struct RxInfo *RI, u8 PORT);
void ResetTxInfo(struct TxInfo *TI, u8 PORT);

// Protocol functions
inline void RxProcessByte(struct RxInfo *RI, u8 data);
inline void RxUpdateControlTable(void);
inline void TxUpdateControlTable(void);

// Zigbee circular buffer functions
inline u8 ZIGRxBufReadByte(void);

// Rx circular buffer functions
inline void RxBufLockMutex(struct RxInfo *RI);                 // Used to lock out the Rx handler from executing
inline void RxBufUnlockMutex(struct RxInfo *RI);               // Used to lock out the Rx handler from executing
inline u8 RxBufPacketAvailable(const struct RxInfo *RI);       // For use by all code
void RxBufReadPacket(struct RxInfo *RI, struct DxlPacket *DP); // For use by non-interrupt code
void RxBufAppendPacket(struct RxInfo *RI);                     // For use by the Rx handler interrupts

// Tx circular buffer functions
inline void TxBufLockMutex(struct TxInfo *TI);                         // Used to lock out the Tx handler from executing
inline void TxBufUnlockMutex(struct TxInfo *TI);                       // Used to lock out the Tx handler from executing
inline u8 TxBufPacketAvailable(const struct TxInfo *TI);               // For use by all code
void TxBufAppendPacket(struct TxInfo *TI, const struct DxlPacket *DP); // For use by non-interrupt code
void TxBufInjectPacket(struct TxInfo *TI, const struct RxInfo *RI);    // For use by the Rx handler interrupts
void TxBufFlushPackets(struct TxInfo *TI);                             // For use by the Tx handler interrupts

// USART functions
inline void PCTxSendByte(u8 data);  // Wraps the USART_SendData() function for PCTx
inline void DXLTxSendByte(u8 data); // Wraps the USART_SendData() function for DXLTx

// Rx and Tx info structs
struct RxInfo PCRx  = {{0}, 0, 0, {0}, 0, 0, 0, 0, 0, 0, 0, {SEEN_NOTHING, INVALID_ID, 0, INST_NONE, 0, 0, {0}, 0}, INIT_TIME_LAST_BYTE, MUTEX_NONE, USART_PC};
struct RxInfo DXLRx = {{0}, 0, 0, {0}, 0, 0, 0, 0, 0, 0, 0, {SEEN_NOTHING, INVALID_ID, 0, INST_NONE, 0, 0, {0}, 0}, INIT_TIME_LAST_BYTE, MUTEX_NONE, USART_DXL};
struct TxInfo PCTx  = {{0}, 0, 0, {0}, 0, 0, 0, FALSE, 0, 0, 0, FALSE, MUTEX_NONE, USART_PC};
struct TxInfo DXLTx = {{0}, 0, 0, {0}, 0, 0, 0, FALSE, 0, 0, 0, FALSE, MUTEX_NONE, USART_DXL};

// Buffering and Dynamixel forwarding flags
vu8 gbDXLForwarding = FALSE; // True => Automatically forward packets PC --> DXL
vu8 gbDXLBuffering  = TRUE;  // True => Store packets received from the DXLs in the local circular buffer (in addition to automatically forwarding them to the PC)

// Circular buffer for Zigbee
vu8  ZIGRxBuf[USART_ZIG_RX_BUFSIZE] = {0}; // Circular buffer for the Zigbee Rx data
vu16 ZIGRxBufReadPtr                =  0;  // The index in the circular buffer that will be next read
vu16 ZIGRxBufWritePtr               =  0;  // The index in the circular buffer that will be next written to
vu16 ZIGRxBufOverflowCount          =  0;  // Number of bytes lost due to buffer overflows

// Miscellaneous
vu8 TmpVar = 0;

//
// Interrupt functions
//

// Trigger a given interrupt
inline void SetInterruptPending(u8 IRQChannel)
{
	NVIC_SetIRQChannelPendingBit(IRQChannel);
	__asm("DSB;ISB");
}

//
// Reset functions
//

// Reset a given RxParseInfo struct
void ResetRxParseInfo(struct RxParseInfo *RPI)
{
	// Reset the struct members
	RPI->State = SEEN_NOTHING;
	RPI->ID = INVALID_ID;
	RPI->Length = 0;
	RPI->Instruction = INST_NONE;
	RPI->NumParams = 0;
	RPI->ParamCount = 0;
	memset((char *) &RPI->Param[0], 0, MAX_PACKET_PARAMS);
	RPI->CheckSum = 0;
}

// Reset a given RxInfo struct
void ResetRxInfo(struct RxInfo *RI, u8 PORT)
{
	// Reset the struct members
	memset((char *) &RI->RawBuf[0], 0, USART_RX_BUFSIZE);
	RI->RawBufReadPtr = 0;
	RI->RawBufWritePtr = 0;
	memset((char *) &RI->Buf[0], 0, USART_RX_BUFSIZE);
	RI->BufReadPtr = 0;
	RI->BufWritePtr = 0;
	RI->BufPacketCount = 0;
	RI->BufOverflowCount = 0;
	RI->BufErrorCount = 0;
	RI->CheckSumErrorCount = 0;
	RI->OREErrorCount = 0;
	ResetRxParseInfo(&RI->ParseInfo);
	RI->TimeLastByte = INIT_TIME_LAST_BYTE;
	RI->Mutex = MUTEX_NONE;
	RI->Port = PORT;
}

// Reset a given TxInfo struct
void ResetTxInfo(struct TxInfo *TI, u8 PORT)
{
	// Reset the struct members
	memset((char *) &TI->RawBuf[0], 0, USART_TX_BUFSIZE);
	TI->RawBufReadPtr = 0;
	TI->RawBufWritePtr = 0;
	memset((char *) &TI->Buf[0], 0, USART_TX_BUFSIZE);
	TI->BufReadPtr = 0;
	TI->BufWritePtr = 0;
	TI->BufReadDebt = 0;
	TI->BufMoreData = FALSE;
	TI->BufPacketCount = 0;
	TI->BufOverflowCount = 0;
	TI->BufErrorCount = 0;
	TI->Transmitting = FALSE;
	TI->Mutex = MUTEX_NONE;
	TI->Port = PORT;
}

//
// Protocol functions
//

// Process one received byte using a state machine to detect packets
inline void RxProcessByte(struct RxInfo *RI, u8 data)
{
	// Note: The required packet format is [0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION/ERROR] [[PARAMETERS]] [CHECKSUM]
	//       ID is the Dynamixel device ID that the packet is intended for
	//       LENGTH is the length of the packet in bytes (starting at INSTRUCTION and including CHECKSUM)
	//       INSTRUCTION/ERROR is the command for the target device (instruction packet) or the current error status of the device (status packet)
	//       PARAMETERS is the data (can be multiple bytes) that is required to complete the given command or is being returned from the device
	//       CHECKSUM is a checksum for the packet, and should be the 1's complement of the sum of all the non-0xFF bytes
	//       See: http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
	//            http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm

	// Retrieve a local pointer to the internal RxParseInfo struct
	struct RxParseInfo* RPI = &RI->ParseInfo;

	// If more than a certain amount of time has passed since the last byte then it can't possibly be from the same packet
	if(gbMillisec - RI->TimeLastByte > DXL_BYTE_TIMEOUT) RPI->State = SEEN_NOTHING;
	RI->TimeLastByte = gbMillisec;

	// State machine for the packet parsing
	switch(RPI->State)
	{
		// If seen nothing yet (waiting for start of a packet)
		default: // If in some indeterminate state, assume we are waiting for a packet to start
		case SEEN_NOTHING:
			if(data == 0xFF) RPI->State = SEEN_1xFF;
			else             RPI->State = SEEN_NOTHING;
			break;

		// If seen one 0xFF start byte so far (waiting for second 0xFF start byte)
		case SEEN_1xFF:
			if(data == 0xFF) RPI->State = SEEN_2xFF;
			else             RPI->State = SEEN_NOTHING;
			break;

		// If seen two consecutive start bytes so far (waiting for ID)
		case SEEN_2xFF:
			RPI->ID = data;
			if(data == 0xFF) RPI->State = SEEN_2xFF; // Wait for the first non-0xFF before proceeding
			else             RPI->State = SEEN_ID;
			break;

		// If seen the ID byte (waiting for LENGTH)
		case SEEN_ID:
			RPI->Length = data;
			RPI->NumParams = RPI->Length - 2; // The number of bytes in PARAMETERS is the packet length minus the INSTRUCTION and CHECKSUM bytes
			if((RPI->Length >= 2) && (RPI->NumParams <= MAX_PACKET_PARAMS)) RPI->State = SEEN_LENGTH;
			else if(data == 0xFF)                                           RPI->State = SEEN_1xFF;    // Bad packet: Start listening for a new one
			else                                                            RPI->State = SEEN_NOTHING; // Bad packet: Start listening for a new one
			break;

		// If seen the packet length byte (waiting for INSTRUCTION/ERROR)
		case SEEN_LENGTH:
			RPI->Instruction = data;
			RPI->CheckSum = RPI->ID + RPI->Length + RPI->Instruction; // Note: The checksum isn't complete yet at this point...
			RPI->ParamCount = 0;
			if(RPI->ParamCount == RPI->NumParams) RPI->State = SEEN_PARAMS;      // Skip the parameters as we don't expect any
			else                                  RPI->State = SEEN_INSTRUCTION; // Wait for some parameters
			break;

		// If seen the instruction byte, and potentially some incomplete number of parameter bytes too (waiting for PARAMETERS)
		case SEEN_INSTRUCTION:
			RPI->Param[RPI->ParamCount++] = data;
			RPI->CheckSum += data;
			if(RPI->ParamCount == RPI->NumParams) RPI->State = SEEN_PARAMS;      // We just received our last parameter
			else                                  RPI->State = SEEN_INSTRUCTION; // Wait for some more parameters
			break;

		// If seen all the params as expected based on the packet length (waiting for CHECKSUM)
		case SEEN_PARAMS:
			RPI->CheckSum ^= 0xFF; // 1's complement, equivalent to ~ operator
			if(data == RPI->CheckSum)
			{
				// We have just received a complete packet, so increment our packet counter
				RI->BufPacketCount++;

				// Inject the received packet into our circular buffers as appropriate
				if(RI->Port == USART_PC)
				{
					// Transcribe the received packet into our Rx buffer
					RxBufAppendPacket(&PCRx);

					// Automatically forward packets PC --> DXL
#if !FWD_PC_BYTES_DIRECTLY
					if((PCRx.ParseInfo.ID != GB_ID) && (gbDXLForwarding == TRUE))
						TxBufInjectPacket(&DXLTx, &PCRx);
#endif

					// Turn on the RX LED to signal that we have received a packet from the PC (this LED is turned off every 123.4ms by the Timer2 interrupt)
					LED_SetState(LED_RX, ON);
				}
				else if(RI->Port == USART_DXL)
				{
					// Transcribe the received packet into our Rx buffer
					if(gbDXLBuffering == TRUE)
						RxBufAppendPacket(&DXLRx);

					// Automatically forward packets DXL --> PC
					TxBufInjectPacket(&PCTx, &DXLRx);

					// Turn on the TX LED to signal that we have received a packet from the DXL (this LED is turned off every 123.4ms by the Timer2 interrupt)
					LED_SetState(LED_TX, ON);
				}

				// Update the Rx control table values
				RxUpdateControlTable();

				// Reset the packet parsing state
				RPI->State = SEEN_NOTHING;
			}
			else // Incorrect checksum...
			{
				RI->CheckSumErrorCount++;
				RxUpdateControlTable();
				if(data == 0xFF) RPI->State = SEEN_1xFF;    // Bad packet: Start listening for a new one
				else             RPI->State = SEEN_NOTHING; // Bad packet: Start listening for a new one
			}
			break;
	}
}

// Update the control table with the latest Rx counter values
inline void RxUpdateControlTable(void)
{
	// DXL Rx buffer counters
	GW_DXLRX_PACKET_CNT   = DXLRx.BufPacketCount;
	GW_DXLRX_OVERFLOW_CNT = DXLRx.BufOverflowCount;
	GW_DXLRX_BUFERROR_CNT = DXLRx.BufErrorCount;
	GW_DXLRX_CHKERROR_CNT = DXLRx.CheckSumErrorCount;
	GW_DXLRX_ORE_CNT      = DXLRx.OREErrorCount;

	// PC Rx buffer counters
	GW_PCRX_PACKET_CNT   = PCRx.BufPacketCount;
	GW_PCRX_OVERFLOW_CNT = PCRx.BufOverflowCount;
	GW_PCRX_BUFERROR_CNT = PCRx.BufErrorCount;
	GW_PCRX_CHKERROR_CNT = PCRx.CheckSumErrorCount;
	GW_PCRX_ORE_CNT      = PCRx.OREErrorCount;
}

// Update the control table with the latest Tx counter values
inline void TxUpdateControlTable(void)
{
	// DXL Tx buffer counters
	GW_DXLTX_PACKET_CNT   = DXLTx.BufPacketCount;
	GW_DXLTX_OVERFLOW_CNT = DXLTx.BufOverflowCount;
	GW_DXLTX_BUFERROR_CNT = DXLTx.BufErrorCount;

	// PC Tx buffer counters
	GW_PCTX_PACKET_CNT   = PCTx.BufPacketCount;
	GW_PCTX_OVERFLOW_CNT = PCTx.BufOverflowCount;
	GW_PCTX_BUFERROR_CNT = PCTx.BufErrorCount;
}

//
// Zigbee circular buffer functions
//

// Read a data byte from the ZIG Rx circular buffer
inline u8 ZIGRxBufReadByte(void)
{
#if ALLOW_ZIGBEE
	// Declare variables
	u8 data = 0;

	// Read a byte from the ZIG Rx circular buffer
	DISABLE_USART_INTERRUPTS();
	if(ZIGRxBufReadPtr != ZIGRxBufWritePtr)
	{
		data = ZIGRxBuf[ZIGRxBufReadPtr++];
		ZIGRxBufReadPtr &= USART_ZIG_RX_BUFMASK;
	}
	REENABLE_USART_INTERRUPTS();

	// Return the read byte
	return data;
#else
	return 0;
#endif
}

//
// Rx circular buffer functions
//

// Lock the mutex of the Rx buffer
inline void RxBufLockMutex(struct RxInfo *RI)
{
	// Safely lock the mutex
	DISABLE_HANDLER_INTERRUPTS();
	RI->Mutex |= MUTEX_LOCK;
	REENABLE_HANDLER_INTERRUPTS();
}

// Unlock the mutex of the Rx buffer
inline void RxBufUnlockMutex(struct RxInfo *RI)
{
	// Safely unlock the mutex
	DISABLE_HANDLER_INTERRUPTS();
	u8 blocked = RI->Mutex & MUTEX_INT_WAS_BLOCKED;
	RI->Mutex = MUTEX_NONE;
	REENABLE_HANDLER_INTERRUPTS();

	// If an Rx handler interrupt tried to execute while the mutex was locked, then execute it now
	if(blocked)
	{
		if(RI->Port == USART_DXL)
			SetInterruptPending(IRQ_DXL_RXHANDLER);
		else if(RI->Port == USART_PC)
			SetInterruptPending(IRQ_PC_RXHANDLER);
		while(!(RI->Mutex & MUTEX_INT_EXECUTED));
	}
}

// Check whether a packet is available in the given Rx packet circular buffer
inline u8 RxBufPacketAvailable(const struct RxInfo *RI)
{
	// Return whether any data is available in the packet buffer
	DISABLE_HANDLER_INTERRUPTS();
	u8 dataAvailable = (RI->BufWritePtr != RI->BufReadPtr);
	REENABLE_HANDLER_INTERRUPTS();
	return dataAvailable;
}

// Read a packet out of the given Rx packet circular buffer
// This function is for use by non-interrupt code and is designed to be safely interruptible by the Rx handler interrupt (which calls RxBufAppendPacket())
void RxBufReadPacket(struct RxInfo *RI, struct DxlPacket *DP)
{
	// Verify that a packet is actually available
	if(RxBufPacketAvailable(RI))
	{
		// Lock the buffer mutex
		RxBufLockMutex(RI);

		// Declare variables
		u8 error = FALSE;

		// Store a local copy of the buffer read pointer
		u16 readPtr = RI->BufReadPtr;
		u16 writePtr = RI->BufWritePtr;

		// Check the assumption that BufReadPtr points to the head (first 0xFF) of a valid packet in the buffer
		if((RI->Buf[readPtr] != 0xFF) || (RI->Buf[(readPtr + 1) & USART_RX_BUFMASK] != 0xFF))
		{
			// Set a flag that a buffer error occurred
			error = TRUE;
		}
		else
		{
			// Extract the first chunk of packet information
			DP->ID          = RI->Buf[(readPtr + 2) & USART_RX_BUFMASK];
			DP->NumParams   = RI->Buf[(readPtr + 3) & USART_RX_BUFMASK] - 2; // NumParams = LENGTH - 2
			DP->Instruction = RI->Buf[(readPtr + 4) & USART_RX_BUFMASK];

			// Check that we're not about to overtake our BufWritePtr
			u16 bytesAvailable = (writePtr - readPtr) & USART_RX_BUFMASK;
			if(DP->NumParams + 6 > bytesAvailable)
			{
				// Set a flag that a buffer error occurred
				error = TRUE;
			}
		}

		// Keep going only if we haven't had an error so far
		if(error == FALSE)
		{
			// Increment the read pointer to the first parameter byte
			readPtr += 5;
			readPtr &= USART_RX_BUFMASK;

			// Extract the packet parameters
			if(DP->NumParams == 1)
			{
				DP->Param[0] = RI->Buf[readPtr];
			}
			else if(DP->NumParams == 2)
			{
				DP->Param[0] = RI->Buf[readPtr];
				DP->Param[1] = RI->Buf[(readPtr + 1) & USART_RX_BUFMASK];
			}
			else if(DP->NumParams > 2)
			{
				u16 firstParam = readPtr;
				u16 lastParam = (firstParam + DP->NumParams - 1) & USART_RX_BUFMASK;
				if(lastParam < firstParam) // The packet wraps around the highest index of the circular buffer
				{
					u16 numWrap = lastParam + 1;
					u16 numFit = DP->NumParams - numWrap;
					memcpy((char *) &DP->Param[0], (const char *) &RI->Buf[firstParam], numFit);
					memcpy((char *) &DP->Param[numFit], (const char *) &RI->Buf[0], numWrap);
				}
				else
				{
					memcpy((char *) &DP->Param[0], (const char *) &RI->Buf[firstParam], DP->NumParams);
				}
			}

			// Increment the read pointer to the first byte after the checksum
			readPtr += DP->NumParams + 1;
			readPtr &= USART_RX_BUFMASK;

			// Update the buffer read pointer
			RI->BufReadPtr = readPtr;
		}
		else
		{
			// Return an invalid packet
			DP->ID = INVALID_ID;
			DP->Instruction = INST_NONE;
			DP->NumParams = 0;

			// The buffer was corrupt in some unknown way so just clear it
			RI->BufReadPtr = RI->BufWritePtr;
			RI->BufErrorCount++;
			RxUpdateControlTable();
		}

		// Unlock the buffer mutex
		RxBufUnlockMutex(RI);
	}
	else
	{
		// Return an invalid packet
		DP->ID = INVALID_ID;
		DP->Instruction = INST_NONE;
		DP->NumParams = 0;
	}
}

// Write a received packet to the given Rx packet circular buffer (the packet to append is passed as RxInfo::ParseInfo)
// This function is for use by the Rx handler interrupt
void RxBufAppendPacket(struct RxInfo *RI)
{
	// Declare variables
	const struct RxParseInfo* RPI = &RI->ParseInfo;

	// Sanity check the packet size
	if(RPI->NumParams > MAX_PACKET_PARAMS)
	{
		RI->BufOverflowCount++;
		RxUpdateControlTable();
		return;
	}

	// Retrieve the number of bytes in the packet
	u16 Bytes = RPI->NumParams + 6;

	// Work out how much read debt we have (bytes of overflow)
	u16 bytesFree = (RI->BufReadPtr - RI->BufWritePtr - 1) & USART_RX_BUFMASK;
	u16 readDebt = (bytesFree < Bytes ? Bytes - bytesFree : 0);

	// Check whether we are going to hit our current BufReadPtr, in which case discard the required number of unread packets from the buffer :(
	u16 incCount = 0;
	u8 inc, fixed = FALSE;
	while(readDebt > incCount)
	{
		// Signal that at least one overflow occurred
		fixed = TRUE;

		// Increment the buffer overflow counter
		RI->BufOverflowCount++;

		// Check the assumption that BufReadPtr points to the head (first 0xFF) of a valid packet in the buffer.
		if((RI->Buf[RI->BufReadPtr] != 0xFF) || (RI->Buf[(RI->BufReadPtr + 1) & USART_RX_BUFMASK] != 0xFF))
		{
			RI->BufReadPtr = RI->BufWritePtr; // Buffer is corrupt in some unknown way so just clear it
			RI->BufErrorCount++;
			break;
		}

		// Jump forward to the LENGTH byte of the packet
		incCount += 3;
		RI->BufReadPtr += 3;
		RI->BufReadPtr &= USART_RX_BUFMASK;

		// Jump forward by the number of bytes specified in the packet length
		inc = RI->Buf[RI->BufReadPtr] + 1;
		incCount += inc;
		RI->BufReadPtr += inc;
		RI->BufReadPtr &= USART_RX_BUFMASK;
	}

	// Update the control table if required
	if(fixed == TRUE) RxUpdateControlTable();

	// Write the first few fields of the packet into the circular buffer
	u16 ptr = RI->BufWritePtr;
	RI->Buf[ptr++ & USART_RX_BUFMASK] = 0xFF;
	RI->Buf[ptr++ & USART_RX_BUFMASK] = 0xFF;
	RI->Buf[ptr++ & USART_RX_BUFMASK] = RPI->ID;
	RI->Buf[ptr++ & USART_RX_BUFMASK] = RPI->NumParams + 2;
	RI->Buf[ptr++ & USART_RX_BUFMASK] = RPI->Instruction;

	// Write the packet parameters into the circular buffer
	if(RPI->NumParams == 1)
	{
		RI->Buf[ptr++ & USART_RX_BUFMASK] = RPI->Param[0];
	}
	else if(RPI->NumParams == 2)
	{
		RI->Buf[ptr++ & USART_RX_BUFMASK] = RPI->Param[0];
		RI->Buf[ptr++ & USART_RX_BUFMASK] = RPI->Param[1];
	}
	else if(RPI->NumParams > 2)
	{
		u16 firstParam = ptr & USART_RX_BUFMASK;
		u16 lastParam = (firstParam + RPI->NumParams - 1) & USART_RX_BUFMASK;
		if(lastParam < firstParam) // The parameters need to wrap around the highest index of the circular buffer
		{
			u16 numWrap = lastParam + 1;
			u16 numFit = RPI->NumParams - numWrap;
			memcpy((char *) &RI->Buf[firstParam], (const char *) &RPI->Param[0], numFit);
			memcpy((char *) &RI->Buf[0], (const char *) &RPI->Param[numFit], numWrap);
		}
		else
		{
			memcpy((char *) &RI->Buf[firstParam], (const char *) &RPI->Param[0], RPI->NumParams);
		}
		ptr += RPI->NumParams;
	}

	// Seal off the packet with the previously calculated checksum
	RI->Buf[ptr++ & USART_RX_BUFMASK] = RPI->CheckSum;
	RI->BufWritePtr = ptr & USART_RX_BUFMASK; // Equivalent to: (RI->BufWritePtr + Bytes) & USART_RX_BUFMASK
}

//
// Tx circular buffer functions
//

// Lock the mutex of the Tx buffer
inline void TxBufLockMutex(struct TxInfo *TI)
{
	// Safely lock the mutex
	DISABLE_HANDLER_INTERRUPTS();
	TI->Mutex |= MUTEX_LOCK;
	REENABLE_HANDLER_INTERRUPTS();
}

// Unlock the mutex of the Tx buffer
inline void TxBufUnlockMutex(struct TxInfo *TI)
{
	// Safely unlock the mutex
	DISABLE_HANDLER_INTERRUPTS();
	u8 blocked = TI->Mutex & MUTEX_INT_WAS_BLOCKED;
	TI->Mutex = MUTEX_NONE;
	REENABLE_HANDLER_INTERRUPTS();

	// If a Tx handler interrupt tried to execute while the mutex was locked, then execute it now
	if(blocked)
	{
		if(TI->Port == USART_DXL)
			SetInterruptPending(IRQ_DXL_TXHANDLER);
		else if(TI->Port == USART_PC)
			SetInterruptPending(IRQ_PC_TXHANDLER);
		while(!(TI->Mutex & MUTEX_INT_EXECUTED));
	}
}

// Check whether a packet is available in the given Tx packet circular buffer
inline u8 TxBufPacketAvailable(const struct TxInfo *TI)
{
	// Return whether any data is available in the packet buffer
	DISABLE_HANDLER_INTERRUPTS();
	u8 dataAvailable = (TI->BufWritePtr != TI->BufReadPtr);
	REENABLE_HANDLER_INTERRUPTS();
	return dataAvailable;
}

// Write a packet into the given Tx circular buffer
// This function is for use by non-interrupt code and is designed to be safely interruptible by both the Rx and Tx handlers
void TxBufAppendPacket(struct TxInfo *TI, const struct DxlPacket *DP)
{
	// Sanity check the packet size
	if(DP->NumParams > MAX_PACKET_PARAMS)
	{
		TI->BufOverflowCount++;
		TxUpdateControlTable();
		return;
	}

	// Lock the buffer mutex
	TxBufLockMutex(TI);

	// Retrieve the number of bytes in the packet
	u16 Bytes = DP->NumParams + 6;

	// Safely reserve the required number of bytes in the buffer
	u16 readPtr, writePtr, bytesFree, readDebt;
	DISABLE_HANDLER_INTERRUPTS();
	readPtr = TI->BufReadPtr;
	writePtr = TI->BufWritePtr;
	bytesFree = (readPtr - writePtr - 1) & USART_TX_BUFMASK;
	readDebt = (bytesFree < Bytes ? Bytes - bytesFree : 0);
	TI->BufReadDebt = readDebt; // Store that the read pointer still needs to be incremented by at least this number of bytes in order to return to a valid buffer state
	TI->BufWritePtr += Bytes;   // By advancing the write pointer we are protecting the bytes in the buffer where our packet is about to go (this logic only fails if so many packets are injected *before* this function completes that the write pointer wraps around the whole circular buffer and past where it was just now)
	TI->BufWritePtr &= USART_TX_BUFMASK;
	REENABLE_HANDLER_INTERRUPTS();

	// Check for buffer overflows (increment the local copy of the read pointer as far as is possible while only accessing the protected bytes of the buffer)
	u16 incCount = 0;
	u8 inc, fixed = FALSE, error = FALSE;
	while(readDebt >= incCount + 5) // Note: We need at least 5 bytes of debt remaining in order for us to be able to safely read a packet length out of the buffer
	{
		// Signal that at least one overflow occurred
		fixed = TRUE;

		// Increment the buffer overflow counter
		TI->BufOverflowCount++;

		// Check the assumption that readPtr points to the head (first 0xFF) of a valid packet in the buffer.
		if((TI->Buf[readPtr] != 0xFF) || (TI->Buf[(readPtr + 1) & USART_TX_BUFMASK] != 0xFF))
		{
			error = TRUE;
			break;
		}

		// Jump forward to the LENGTH byte of the packet
		incCount += 3;
		readPtr  += 3;
		readPtr  &= USART_TX_BUFMASK;

		// Jump forward by the number of bytes specified in the packet length
		inc = TI->Buf[readPtr] + 1;
		incCount += inc;
		readPtr  += inc;
		readPtr  &= USART_TX_BUFMASK;
	}

	// Resolve the pending read debt (it must either be cleared up by an associated interrupt, or manually here)
	DISABLE_HANDLER_INTERRUPTS();
	if(error == TRUE)
	{
		TI->BufReadPtr = writePtr; // Clears the packet buffer of all but the packet we're about to write into it, and any packets that were injected via interrupts into the buffer since the beginning of this function
		TI->BufErrorCount++;
	}
	else if(TI->BufReadDebt == readDebt) // This is true if no packet has been injected into the buffer by an interrupt since the beginning of this function => We need to clear up our own read debt!
	{
		// If we have any remaining debt (maximum 4 bytes left) then discard the last remaining packet as well
		if(readDebt > incCount)
		{
			// Signal that at least one overflow occurred
			fixed = TRUE;

			// Increment the buffer overflow counter
			TI->BufOverflowCount++;

			// Check the assumption that readPtr points to the head (first 0xFF) of a valid packet in the buffer.
			if((TI->Buf[readPtr] != 0xFF) || (TI->Buf[(readPtr + 1) & USART_TX_BUFMASK] != 0xFF))
			{
				TI->BufReadPtr = writePtr; // Clears the packet buffer of all but the packet we're about to write into it
				TI->BufErrorCount++;
			}
			else
			{
				// Jump forward to the LENGTH byte of the packet
				readPtr += 3;
				readPtr &= USART_TX_BUFMASK;

				// Jump forward by the number of bytes specified in the packet length
				readPtr += TI->Buf[readPtr] + 1;
				readPtr &= USART_TX_BUFMASK;

				// Update the buffer read pointer
				TI->BufReadPtr = readPtr;
			}
		}
		else // This is true if we have already cleared up all the read debt that we had, and no interrupt interfered...
		{
			TI->BufReadPtr = readPtr;
		}
	}
	TI->BufReadDebt = 0;
	REENABLE_HANDLER_INTERRUPTS();

	// Update the control table if required
	if(fixed == TRUE) TxUpdateControlTable();

	// Write the first few fields of the packet into the circular buffer
	TI->Buf[writePtr++ & USART_TX_BUFMASK] = 0xFF;
	TI->Buf[writePtr++ & USART_TX_BUFMASK] = 0xFF;
	TI->Buf[writePtr++ & USART_TX_BUFMASK] = DP->ID;
	TI->Buf[writePtr++ & USART_TX_BUFMASK] = DP->NumParams + 2;
	TI->Buf[writePtr++ & USART_TX_BUFMASK] = DP->Instruction;

	// Copy out the parameters into the buffer while calculating the checksum
	u8 i, CheckSum = DP->ID + (DP->NumParams + 2) + DP->Instruction;
	for(i = 0; i < DP->NumParams; i++)
	{
		TI->Buf[writePtr++ & USART_TX_BUFMASK] = DP->Param[i];
		CheckSum += DP->Param[i];
	}

	// Seal off the packet with the previously calculated checksum
	TI->Buf[writePtr & USART_TX_BUFMASK] = ~CheckSum;

	// Unlock the buffer mutex and trigger the Tx handler interrupt
	TI->Mutex = MUTEX_NONE;
	if(TI->Port == USART_DXL)
		SetInterruptPending(IRQ_DXL_TXHANDLER);
	else if(TI->Port == USART_PC)
		SetInterruptPending(IRQ_PC_TXHANDLER);
}

// Write a packet received from an Rx port into the given Tx packet circular buffer
// This function is for use by the Rx handler interrupt and can safely interrupt the TxBufAppendPacket() function
void TxBufInjectPacket(struct TxInfo *TI, const struct RxInfo *RI)
{
	// Declare variables
	const struct RxParseInfo* RPI = &RI->ParseInfo;

	// Sanity check the packet size
	if(RPI->NumParams > MAX_PACKET_PARAMS)
	{
		TI->BufOverflowCount++;
		TxUpdateControlTable();
		return;
	}

	// Retrieve the number of bytes in the packet
	u16 Bytes = RPI->NumParams + 6;

	// Resolve how much total read debt we currently have
	u16 bytesFree, readDebt;
	bytesFree = (TI->BufReadPtr - TI->BufWritePtr - 1) & USART_TX_BUFMASK;
	if(((TI->BufReadDebt + bytesFree) & USART_TX_BUFMASK) == 0) // Note: Equivalent to TI->BufReadPtr + TI->BufReadDebt == TI->BufWritePtr + 1 (mod USART_TX_BUFMASK)
		readDebt = TI->BufReadDebt + Bytes;
	else
		readDebt = (bytesFree < Bytes ? Bytes - bytesFree : 0);
	TI->BufReadDebt = 0;

	// Check for buffer overflows
	u16 incCount = 0;
	u8 inc, fixed = FALSE;
	while(readDebt > incCount)
	{
		// Signal that at least one overflow occurred
		fixed = TRUE;

		// Increment the buffer overflow counter
		TI->BufOverflowCount++;

		// Check the assumption that BufReadPtr points to the head (first 0xFF) of a valid packet in the buffer.
		if((TI->Buf[TI->BufReadPtr] != 0xFF) || (TI->Buf[(TI->BufReadPtr + 1) & USART_TX_BUFMASK] != 0xFF))
		{
			TI->BufReadPtr = TI->BufWritePtr; // Buffer is corrupt in some unknown way so just clear it
			TI->BufErrorCount++;
			break;
		}

		// Jump forward to the LENGTH byte of the packet
		incCount += 3;
		TI->BufReadPtr += 3;
		TI->BufReadPtr &= USART_TX_BUFMASK;

		// Jump forward by the number of bytes specified in the packet length
		inc = TI->Buf[TI->BufReadPtr] + 1;
		incCount += inc;
		TI->BufReadPtr += inc;
		TI->BufReadPtr &= USART_TX_BUFMASK;
	}

	// Update the control table if required
	if(fixed == TRUE) TxUpdateControlTable();

	// Write the first few fields of the packet into the circular buffer
	u16 ptr = TI->BufWritePtr;
	TI->Buf[ptr++ & USART_TX_BUFMASK] = 0xFF;
	TI->Buf[ptr++ & USART_TX_BUFMASK] = 0xFF;
	TI->Buf[ptr++ & USART_TX_BUFMASK] = RPI->ID;
	TI->Buf[ptr++ & USART_TX_BUFMASK] = RPI->NumParams + 2;
	TI->Buf[ptr++ & USART_TX_BUFMASK] = RPI->Instruction;

	// Write the packet parameters into the circular buffer
	if(RPI->NumParams == 1)
	{
		TI->Buf[ptr++ & USART_TX_BUFMASK] = RPI->Param[0];
	}
	else if(RPI->NumParams == 2)
	{
		TI->Buf[ptr++ & USART_TX_BUFMASK] = RPI->Param[0];
		TI->Buf[ptr++ & USART_TX_BUFMASK] = RPI->Param[1];
	}
	else if(RPI->NumParams > 2)
	{
		u16 firstParam = ptr & USART_TX_BUFMASK;
		u16 lastParam = (firstParam + RPI->NumParams - 1) & USART_TX_BUFMASK;
		if(lastParam < firstParam) // The parameters need to wrap around the highest index of the circular buffer
		{
			u16 numWrap = lastParam + 1;
			u16 numFit = RPI->NumParams - numWrap;
			memcpy((char *) &TI->Buf[firstParam], (const char *) &RPI->Param[0], numFit);
			memcpy((char *) &TI->Buf[0], (const char *) &RPI->Param[numFit], numWrap);
		}
		else
		{
			memcpy((char *) &TI->Buf[firstParam], (const char *) &RPI->Param[0], RPI->NumParams);
		}
		ptr += RPI->NumParams;
	}

	// Seal off the packet with the previously calculated checksum
	TI->Buf[ptr++ & USART_TX_BUFMASK] = RPI->CheckSum;
	TI->BufWritePtr = ptr & USART_TX_BUFMASK; // Equivalent to: (TI->BufWritePtr + Bytes) & USART_TX_BUFMASK

	// Trigger the Tx handler interrupt
	if(TI->Port == USART_DXL)
		SetInterruptPending(IRQ_DXL_TXHANDLER);
	else if(TI->Port == USART_PC)
		SetInterruptPending(IRQ_PC_TXHANDLER);
}

// Flush packets from a given Tx packet buffer into the corresponding raw Tx buffer
void TxBufFlushPackets(struct TxInfo *TI)
{
	// Declare variables
	u16 packetBytes, bytesAvailable, bytesFree, i;
	u8 update = FALSE;

	// If data is available in the packet buffer, copy it out packet for packet into the raw Tx buffer
	TI->BufMoreData = FALSE;
	while(TxBufPacketAvailable(TI))
	{
		// Check the assumption that BufReadPtr points to the head (first 0xFF) of a valid packet in the buffer
		if((TI->Buf[TI->BufReadPtr] != 0xFF) || (TI->Buf[(TI->BufReadPtr + 1) & USART_TX_BUFMASK] != 0xFF))
		{
			TI->BufReadPtr = TI->BufWritePtr; // The buffer is corrupt in some unknown way so just clear it
			TI->BufErrorCount++;
			update = TRUE;
			break;
		}

		// Retrieve the number of bytes in the packet
		packetBytes = TI->Buf[(TI->BufReadPtr + 3) & USART_TX_BUFMASK] + 4; // Bytes in packet = LENGTH + 4

		// Check that we are not about to overtake our BufWritePtr
		bytesAvailable = (TI->BufWritePtr - TI->BufReadPtr) & USART_TX_BUFMASK;
		if(packetBytes > bytesAvailable)
		{
			TI->BufReadPtr = TI->BufWritePtr; // The buffer is corrupt in some unknown way so just clear it
			TI->BufErrorCount++;
			update = TRUE;
			break;
		}

		// Check how many bytes are free in the raw Tx buffer
		DISABLE_USART_INTERRUPTS();
		bytesFree = (TI->RawBufReadPtr - TI->RawBufWritePtr - 1) & USART_TX_BUFMASK;
		REENABLE_USART_INTERRUPTS();

		// If the packet will fit into the raw Tx buffer then transcribe it
		if(packetBytes <= bytesFree) // Note: Even if the USART interrupt fires, it doesn't modify the RawBufWritePtr and doesn't *reduce* the number of free buffer bytes, so we're ok!
		{
			// Transcribe the packet
			for(i = 0; i < packetBytes; i++)
			{
				DISABLE_USART_INTERRUPTS();
				TI->RawBuf[TI->RawBufWritePtr++] = TI->Buf[TI->BufReadPtr++];
				TI->RawBufWritePtr &= USART_TX_BUFMASK;
				REENABLE_USART_INTERRUPTS();
				TI->BufReadPtr &= USART_TX_BUFMASK;
			}

			// Increment the packet counter
			TI->BufPacketCount++;
			update = TRUE;

// 			// Turn on the TX LED to signal that we are transmitting a packet to the PC (this LED is turned off every 123.4ms by the Timer2 interrupt)
// 			if(TI->Port == USART_PC)
// 				LED_SetState(LED_TX, ON);

// 			// Turn on the TX LED to signal that we are transmitting a packet to the DXL (this LED is turned off every 123.4ms by the Timer2 interrupt)
// 			if(TI->Port == USART_DXL)
// 				LED_SetState(LED_TX, ON);
		}
		else
		{
			TI->BufMoreData = TRUE;
			break;
		}
	}

	// Update the control table
	if(update == TRUE) TxUpdateControlTable();
}

//
// USART functions
//

// Send a data byte over USART3 to the PC
inline void PCTxSendByte(u8 data)
{
	// Note: This function may contain STM32F103xx-specific code!

	// Communications logging (can be enabled in CM_DXL_COM.h for debugging purposes)
#if COMMS_LOGA == COMMS_LOG_PCTX
	COMMS_LOGA_WRITE(data);
#endif
#if COMMS_LOGB == COMMS_LOG_PCTX
	COMMS_LOGB_WRITE(data);
#endif

	// Send the data byte
	DISABLE_USART_INTERRUPTS();    // Disable the USART interrupts
	USART3->SR = ~USART_SR_TC;     // Explicitly clear the TC flag
	USART3->DR = (u16) data;       // Write the data to the USART3 TDR register
	USART3->CR1 |= USART_CR1_TCIE; // Enable the TC interrupt for USART3 (PC)
	REENABLE_USART_INTERRUPTS();   // Re-enable the USART interrupts
}

// Send a data byte over USART1 to the DXLs
inline void DXLTxSendByte(u8 data)
{
	// Note: This function may contain STM32F103xx-specific code!

	// Communications logging (can be enabled in CM_DXL_COM.h for debugging purposes)
#if COMMS_LOGA == COMMS_LOG_DXLTX
	COMMS_LOGA_WRITE(data);
#endif
#if COMMS_LOGB == COMMS_LOG_DXLTX
	COMMS_LOGB_WRITE(data);
#endif

	// Send the data byte
	DISABLE_USART_INTERRUPTS();    // Disable the USART interrupts
	USART1->SR = ~USART_SR_TC;     // Explicitly clear the TC flag
	USART1->DR = (u16) data;       // Write the data to the USART1 TDR register
	USART1->CR1 |= USART_CR1_TCIE; // Enable the TC interrupt for USART1 (DXL)
	REENABLE_USART_INTERRUPTS();   // Re-enable the USART interrupts
}

//
// Initialisation functions
//

// Initialise the buffers and variables corresponding to the specified USART port
void USARTInit(u8 PORT) // It is assumed that this function is called for each port before any of the corresponding interrupts fire
{
	// Reset/initialise the variables for the required port
	if(PORT == USART_DXL)
	{
		ResetRxInfo(&DXLRx, USART_DXL);
		ResetTxInfo(&DXLTx, USART_DXL);
	}
	else if(PORT == USART_PC)
	{
		ResetRxInfo(&PCRx, USART_PC);
		ResetTxInfo(&PCTx, USART_PC);
	}

	// Clear the buffers and reset the error counters (required for USART_ZIG case)
	USARTClearBuffers(PORT, USART_CLEAR_ALL);
	USARTResetCounters(PORT, USART_CLEAR_ALL);
}

// Clear the circular data buffers for the specified USART port (set which buffer(s) to clear using the flag parameter, using USART_CLEAR_RX, USART_CLEAR_TX, USART_CLEAR_ALL)
void USARTClearBuffers(u8 PORT, u8 flag)
{
	// Disable interrupts
	DISABLE_HANDLER_INTERRUPTS();

	// Reset the required buffer read and write pointers
	if(PORT == USART_ZIG)
	{
		if(flag & USART_CLEAR_RX) ZIGRxBufReadPtr = ZIGRxBufWritePtr = 0;
	}
	else if(PORT == USART_DXL)
	{
		if(flag & USART_CLEAR_RX) DXLRx.BufReadPtr = DXLRx.BufWritePtr = 0;
		if(flag & USART_CLEAR_TX) DXLTx.BufReadPtr = DXLTx.BufWritePtr = 0;
	}
	else if(PORT == USART_PC)
	{
		if(flag & USART_CLEAR_RX) PCRx.BufReadPtr = PCRx.BufWritePtr = 0;
		if(flag & USART_CLEAR_TX) PCTx.BufReadPtr = PCTx.BufWritePtr = 0;
	}

	// Enable interrupts
	REENABLE_HANDLER_INTERRUPTS();
}

// Reset the counters for the specified USART port
void USARTResetCounters(u8 PORT, u8 flag)
{
	// Reset the required counters
	if(PORT == USART_ZIG)
	{
		if(flag & USART_CLEAR_RX) ZIGRxBufOverflowCount = 0;
	}
	else if(PORT == USART_DXL)
	{
		if(flag & USART_CLEAR_RX) DXLRx.BufPacketCount = DXLRx.BufOverflowCount = DXLRx.BufErrorCount = DXLRx.CheckSumErrorCount = DXLRx.OREErrorCount = 0;
		if(flag & USART_CLEAR_TX) DXLTx.BufPacketCount = DXLTx.BufOverflowCount = DXLTx.BufErrorCount = 0;
	}
	else if(PORT == USART_PC)
	{
		if(flag & USART_CLEAR_RX) PCRx.BufPacketCount = PCRx.BufOverflowCount = PCRx.BufErrorCount = PCRx.CheckSumErrorCount = PCRx.OREErrorCount = 0;
		if(flag & USART_CLEAR_TX) PCTx.BufPacketCount = PCTx.BufOverflowCount = PCTx.BufErrorCount = 0;
	}

	// Update the control table
	RxUpdateControlTable();
	TxUpdateControlTable();
}

// Update the control table
void USARTUpdateControlTable()
{
	// Update the Rx and Tx entries in the control table
	RxUpdateControlTable();
	TxUpdateControlTable();
}

//
// Communication functions
//

// Return whether data is available in the buffer for the specified USART port (all ports)
u8 RxDDataAvailable(u8 PORT)
{
	// Return whether data is available or not
	if(PORT == USART_ZIG)
		return (ZIGRxBufWritePtr != ZIGRxBufReadPtr);
	else if(PORT == USART_DXL)
		return RxBufPacketAvailable(&DXLRx);
	else if(PORT == USART_PC)
		return RxBufPacketAvailable(&PCRx);
	else
		return FALSE; // Return false if we don't recognise the port
}

// Retrieve a received data byte on the specified USART port (USART_ZIG only)
u8 RxDData(u8 PORT)
{
	// Retrieve the data from the appropriate buffer
	if(PORT == USART_ZIG)
		return ZIGRxBufReadByte();
	else
		return 0; // Return zero if we don't recognise the port
}

// Retrieve a received data packet on the specified USART port (USART_DXL and USART_PC only)
void RxDDataDP(u8 PORT, struct DxlPacket *DP)
{
	// Retrieve the data from the appropriate buffer
	if(PORT == USART_DXL)
		RxBufReadPacket(&DXLRx, DP);
	else if(PORT == USART_PC)
		RxBufReadPacket(&PCRx, DP);
	else
	{
		DP->ID = INVALID_ID;
		DP->Instruction = INST_NONE;
		DP->NumParams = 0;
	}
}

// Retrieve whether a transmission is currently underway on the specified USART port (all ports)
u8 TxDDataTransmitting(u8 PORT)
{
	// Return whether a buffer flush is currently underway
	if(PORT == USART_ZIG)
		return FALSE; // There is no USART_ZIG Tx buffer, so always false
	else if(PORT == USART_DXL)
		return DXLTx.Transmitting;
	else if(PORT == USART_PC)
		return PCTx.Transmitting;
	else
		return FALSE; // Return false if we don't recognise the port
}

// Transmit a data byte (directly and immediately) on the specified USART port (USART_ZIG only)
void TxDData(u8 PORT, u8 data)
{
#if ALLOW_ZIGBEE
	// Send the data on the appropriate port
	if(PORT == USART_ZIG)
	{
		// Send the required data byte
		USART_SendData(UART5, data);
		while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
	}
#endif
}

// Append a data packet to the Tx buffer of a specified USART port (USART_DXL and USART_PC only)
void TxDDataDP(u8 PORT, const struct DxlPacket *DP)
{
	// Retrieve the data from the appropriate buffer
	if(PORT == USART_DXL)
		TxBufAppendPacket(&DXLTx, DP);
	else if(PORT == USART_PC)
		TxBufAppendPacket(&PCTx, DP);
}

// Wait for the transmission of data packets to finish on the specified USART port
// Note: Additional packets may be added in the background in interrupts, making this function take longer than expected!
void WaitForTxDData(u8 PORT)
{
	// Wait until the TX handler has executed at least once (may already be the case)
	if(PORT == USART_DXL)
		while(!(DXLTx.Mutex & MUTEX_INT_EXECUTED));
	else if(PORT == USART_PC)
		while(!(PCTx.Mutex  & MUTEX_INT_EXECUTED));

	// Wait until no transmissions are happening anymore on the specified port
	while(TxDDataTransmitting(PORT));
}

//
// Interrupt service routines
//

// PC byte received interrupt service routine (USART3)
void __ISR_PC_USART()
{
	// Note: This function may contain STM32F103xx-specific code!
	// This interrupt has been written to handle 3 of the 10 possible USART interrupt sources:
	// - RXNE: Read data register not empty => Received content in the shift register has been transferred to the RDR register (read-shadowed by USART_DR)
	// - ORE:  Overrun error => Content finished receiving to the shift register, but the RXNE flag is still set, indicating that the current value of the RDR register hasn't been read out yet.
	// - TC:   Transmission complete => Sending of data has completed and the TDR register (write-shadowed by USART_DR) is empty
	// The remaining 7 USART3 interrupt sources should be disabled to avoid infinite USART interrupt loops that hang the CM730!
	// The RXNE flag is cleared via a read to USART_DR (i.e. a read to the shadowed RDR).
	// The ORE flag is cleared via a read to USART_SR, followed by a read to USART_DR (RDR).
	// The TC flag is cleared via a read to USART_SR, followed by a write to USART_DR (TDR), OR via a manual resetting of the flag (write 0).
	// The TC flag is used on this per-byte basis instead of the more efficient TXE flag so that the interrupt can be more responsive to the RXNE flags, and avoid ORE's as much as possible.

	// Read the USART3 status register (contains the interrupt flags)
	u16 USART_SR = USART3->SR; // Retrieves the lowest 16 bits of the 32-bit USART_SR status register

	// Check which interrupt flags are set
	u8 RXNESet = ((USART_SR & USART_SR_RXNE) != 0);
	u8 ORESet  = ((USART_SR & USART_SR_ORE ) != 0);
	u8 TCSet   = ((USART_SR & USART_SR_TC  ) != 0);

	// If we have received data from the PC over USART3...
	if(RXNESet || ORESet)
	{
		// Read one byte from the received data register
		u8 ReceivedData = (u8) USART3->DR; // USART communications are configured as 8N1, so this truncates bit 9, which isn't used.

		// Communications logging (can be enabled in CM_DXL_COM.h for debugging purposes)
#if COMMS_LOGA == COMMS_LOG_PCRX
		COMMS_LOGA_WRITE(ReceivedData);
#endif
#if COMMS_LOGB == COMMS_LOG_PCRX
		COMMS_LOGB_WRITE(ReceivedData);
#endif

		// Write the received byte into the raw PC Rx circular buffer
		PCRx.RawBuf[PCRx.RawBufWritePtr++] = ReceivedData;
		PCRx.RawBufWritePtr &= USART_RX_BUFMASK;
		if(PCRx.RawBufWritePtr == PCRx.RawBufReadPtr) // Circular buffer was full, so we must forcibly advance the read pointer to avoid the buffer now looking empty!
		{
			PCRx.RawBufReadPtr++;
			PCRx.RawBufReadPtr &= USART_RX_BUFMASK;
			GW_PCRX_OVERFLOW_CNT = (++PCRx.BufOverflowCount);
		}

		// Direct byte feedthrough PC --> DXL
#if FWD_PC_BYTES_DIRECTLY
		if(gbDXLForwarding == TRUE)
		{
			DXLTx.RawBuf[DXLTx.RawBufWritePtr++] = ReceivedData;
			DXLTx.RawBufWritePtr &= USART_TX_BUFMASK;
			if(DXLTx.RawBufWritePtr == DXLTx.RawBufReadPtr) // Circular buffer was full, so we must forcibly advance the read pointer to avoid the buffer now looking empty!
			{
				DXLTx.RawBufReadPtr++;
				DXLTx.RawBufReadPtr &= USART_TX_BUFMASK;
				GW_DXLTX_OVERFLOW_CNT = (++DXLTx.BufOverflowCount);
			}
			if(DXLTx.Transmitting != TRUE)
				SetInterruptPending(IRQ_DXL_TXHANDLER);
		}
#endif

		// Increment the ORE error counter if necessary
		if(ORESet)
			GW_PCRX_ORE_CNT = (++PCRx.OREErrorCount);

		// Trigger the lower priority Rx handler interrupt
		SetInterruptPending(IRQ_PC_RXHANDLER);
	}

	// If we have just finished sending a byte to the PC over USART3...
	else if(TCSet)
	{
		// Only do something if we are currently transmitting
		if(PCTx.Transmitting == TRUE)
		{
			// If we have another byte to send then do so, otherwise cease the transmission
			if(PCTx.RawBufReadPtr != PCTx.RawBufWritePtr)
			{
				// Send the next byte
				PCTxSendByte(PCTx.RawBuf[PCTx.RawBufReadPtr++]); // This internally writes to USART3->DR...
				PCTx.RawBufReadPtr &= USART_TX_BUFMASK;
			}
			else
			{
				// Signal that the transmission is over
				PCTx.Transmitting = FALSE;

				// Disable TC for USART3 (PC)
				USART3->CR1 &= ~USART_CR1_TCIE; // Disable the TC interrupt for USART3 (PC)
				USART3->SR = ~USART_SR_TC;      // Explicitly clear the TC flag

				// Trigger the lower priority Tx handler interrupt if we have more data pending
				if(PCTx.BufMoreData == TRUE)
					SetInterruptPending(IRQ_PC_TXHANDLER);
			}
		}
		else
		{
			// Disable TC for USART3 (PC)
			USART3->CR1 &= ~USART_CR1_TCIE; // Disable the TC interrupt for USART3 (PC)
			USART3->SR = ~USART_SR_TC;      // Explicitly clear the TC flag
		}
	}

	// Memory barrier to ensure that no funny business happens with the interrupts and interrupt flags
	__asm volatile("DSB;ISB");
}

// PC Rx handler interrupt service routine
void __ISR_PC_RXHANDLER()
{
	// Don't execute if the buffer mutex is locked
	if(PCRx.Mutex & MUTEX_LOCK)
	{
		PCRx.Mutex |= MUTEX_INT_WAS_BLOCKED;
		return;
	}
	PCRx.Mutex |= MUTEX_INT_EXECUTED;

	// Declare variables
	u8 dataAvailable, data = 0;

	// Keep reading and processing bytes from the raw Rx buffer
	while(1)
	{
		// Safely retrieve a byte from the raw Rx buffer
		DISABLE_USART_INTERRUPTS();
		dataAvailable = (PCRx.RawBufReadPtr != PCRx.RawBufWritePtr);
		if(dataAvailable)
		{
			data = PCRx.RawBuf[PCRx.RawBufReadPtr++];
			PCRx.RawBufReadPtr &= USART_RX_BUFMASK;
		}
		REENABLE_USART_INTERRUPTS();

		// Break if we have no more available data
		if(!dataAvailable) break;

		// Process the retrieved Rx byte
		RxProcessByte(&PCRx, data);
	}
}

// PC Tx handler interrupt service routine
void __ISR_PC_TXHANDLER()
{
	// Don't execute everything if the buffer mutex is locked
	if(PCTx.Mutex & MUTEX_LOCK)
	{
		// Signal that full execution of the Tx handler was blocked
		PCTx.Mutex |= MUTEX_INT_WAS_BLOCKED;
	}
	else
	{
		// Signal that the Tx handler is executing fully
		PCTx.Mutex |= MUTEX_INT_EXECUTED;

		// Flush packets from the packet buffer into the raw Tx buffer
		TxBufFlushPackets(&PCTx);
	}

	// Start a transmission of raw Tx data if some is available
	if(PCTx.Transmitting != TRUE)
	{
		// Declare variables
		u8 dataAvailable, data = 0;

		// Safely check whether there is data available to send
		DISABLE_USART_INTERRUPTS();
		dataAvailable = (PCTx.RawBufReadPtr != PCTx.RawBufWritePtr);
		if(dataAvailable)
		{
			data = PCTx.RawBuf[PCTx.RawBufReadPtr++];
			PCTx.RawBufReadPtr &= USART_TX_BUFMASK;
		}
		REENABLE_USART_INTERRUPTS();

		// Start a transmission if data is available
		if(dataAvailable)
		{
			// Signal that a transmission is starting
			PCTx.Transmitting = TRUE;

			// Send the first byte
			PCTxSendByte(data);
		}
	}
}

// DXL byte received interrupt service routine (USART1)
void __ISR_DXL_USART()
{
	// Note: This function may contain STM32F103xx-specific code!
	// This interrupt has been written to handle 3 of the 10 possible USART interrupt sources:
	// - RXNE: Read data register not empty => Received content in the shift register has been transferred to the RDR register (read-shadowed by USART_DR)
	// - ORE:  Overrun error => Content finished receiving to the shift register, but the RXNE flag is still set, indicating that the current value of the RDR register hasn't been read out yet.
	// - TC:   Transmission complete => Sending of data has completed and the TDR register (write-shadowed by USART_DR) is empty
	// The remaining 7 USART3 interrupt sources should be disabled to avoid infinite USART interrupt loops that hang the CM730!
	// The RXNE flag is cleared via a read to USART_DR (i.e. a read to the shadowed RDR).
	// The ORE flag is cleared via a read to USART_SR, followed by a read to USART_DR (RDR).
	// The TC flag is cleared via a read to USART_SR, followed by a write to USART_DR (TDR), OR via a manual resetting of the flag (write 0).
	// The TC flag is used on this per-byte basis instead of the more efficient TXE flag so that the interrupt can be more responsive to the RXNE flags, and avoid ORE's as much as possible.

	// Read the USART1 status register (contains the interrupt flags)
	u16 USART_SR = USART1->SR; // Retrieves the lowest 16 bits of the 32-bit USART_SR status register

	// Check which interrupt flags are set
	u8 RXNESet = ((USART_SR & USART_SR_RXNE) != 0);
	u8 ORESet  = ((USART_SR & USART_SR_ORE ) != 0);
	u8 TCSet   = ((USART_SR & USART_SR_TC  ) != 0);

	// If we have received data from the DXLs over USART1...
	if(RXNESet || ORESet)
	{
		// Read one byte from the received data register
		u8 ReceivedData = (u8) USART1->DR; // USART communications are configured as 8N1, so this truncates bit 9, which isn't used.

		// Communications logging (can be enabled in CM_DXL_COM.h for debugging purposes)
#if COMMS_LOGA == COMMS_LOG_DXLRX
		COMMS_LOGA_WRITE(ReceivedData);
#endif
#if COMMS_LOGB == COMMS_LOG_DXLRX
		COMMS_LOGB_WRITE(ReceivedData);
#endif

		// Write the received byte into the raw DXL Rx circular buffer
		DXLRx.RawBuf[DXLRx.RawBufWritePtr++] = ReceivedData;
		DXLRx.RawBufWritePtr &= USART_RX_BUFMASK;
		if(DXLRx.RawBufWritePtr == DXLRx.RawBufReadPtr) // Circular buffer was full, so we must forcibly advance the read pointer to avoid the buffer now looking empty!
		{
			DXLRx.RawBufReadPtr++;
			DXLRx.RawBufReadPtr &= USART_RX_BUFMASK;
			GW_DXLRX_OVERFLOW_CNT = (++DXLRx.BufOverflowCount);
		}

		// Increment the ORE error counter if necessary
		if(ORESet)
			GW_DXLRX_ORE_CNT = (++DXLRx.OREErrorCount);

		// Trigger the lower priority Rx handler interrupt
		SetInterruptPending(IRQ_DXL_RXHANDLER);
	}

	// If we have just finished sending a byte to the DXLs over USART1...
	else if(TCSet)
	{
		// Only do something if we are currently transmitting
		if(DXLTx.Transmitting == TRUE)
		{
			// If we have another byte to send then do so, otherwise cease the transmission
			if(DXLTx.RawBufReadPtr != DXLTx.RawBufWritePtr)
			{
				// Send the next byte
				DXLTxSendByte(DXLTx.RawBuf[DXLTx.RawBufReadPtr++]); // This internally writes to USART1->DR...
				DXLTx.RawBufReadPtr &= USART_TX_BUFMASK;
			}
			else
			{
				// Signal that the transmission is over
				DXLTx.Transmitting = FALSE;

				// Disable TC for USART1 (DXL)
				USART1->CR1 &= ~USART_CR1_TCIE; // Disable the TC interrupt for USART1 (DXL)
				USART1->SR = ~USART_SR_TC;      // Explicitly clear the TC flag

				// Enable/disable RX/TX pins for receiving
				GPIO_ResetBits(PORT_ENABLE_TX, PIN_ENABLE_TX); // TX Disable
				GPIO_SetBits(PORT_ENABLE_RX, PIN_ENABLE_RX);   // RX Enable

				// Trigger the lower priority Tx handler interrupt if we have more data pending
				if(DXLTx.BufMoreData == TRUE)
					SetInterruptPending(IRQ_DXL_TXHANDLER);
			}
		}
		else
		{
			// Disable TC for USART1 (DXL)
			USART1->CR1 &= ~USART_CR1_TCIE; // Disable the TC interrupt for USART1 (DXL)
			USART1->SR = ~USART_SR_TC;      // Explicitly clear the TC flag

			// Enable/disable RX/TX pins for receiving
			GPIO_ResetBits(PORT_ENABLE_TX, PIN_ENABLE_TX); // TX Disable
			GPIO_SetBits(PORT_ENABLE_RX, PIN_ENABLE_RX);   // RX Enable
		}
	}

	// Memory barrier to ensure that no funny business happens with the interrupts and interrupt flags
	__asm volatile("DSB;ISB");
}

// DXL Rx handler interrupt service routine
void __ISR_DXL_RXHANDLER()
{
	// Don't execute if the buffer mutex is locked
	if(DXLRx.Mutex & MUTEX_LOCK)
	{
		DXLRx.Mutex |= MUTEX_INT_WAS_BLOCKED;
		return;
	}
	DXLRx.Mutex |= MUTEX_INT_EXECUTED;

	// Declare variables
	u8 dataAvailable, data = 0;

	// Keep reading and processing bytes from the raw Rx buffer
	while(1)
	{
		// Safely retrieve a byte from the raw Rx buffer
		DISABLE_USART_INTERRUPTS();
		dataAvailable = (DXLRx.RawBufReadPtr != DXLRx.RawBufWritePtr);
		if(dataAvailable)
		{
			data = DXLRx.RawBuf[DXLRx.RawBufReadPtr++];
			DXLRx.RawBufReadPtr &= USART_RX_BUFMASK;
		}
		REENABLE_USART_INTERRUPTS();

		// Break if we have no more available data
		if(!dataAvailable) break;

		// Process the retrieved Rx byte
		RxProcessByte(&DXLRx, data);
	}
}

// DXL Tx handler interrupt service routine
void __ISR_DXL_TXHANDLER()
{
	// Don't execute everything if the buffer mutex is locked
	if(DXLTx.Mutex & MUTEX_LOCK)
	{
		// Signal that full execution of the Tx handler was blocked
		DXLTx.Mutex |= MUTEX_INT_WAS_BLOCKED;
	}
	else
	{
		// Signal that the Tx handler is executing fully
		DXLTx.Mutex |= MUTEX_INT_EXECUTED;

		// Flush packets from the packet buffer into the raw Tx buffer
		TxBufFlushPackets(&DXLTx);
	}

	// Start a transmission of raw Tx data if some is available
	if(DXLTx.Transmitting != TRUE)
	{
		// Declare variables
		u8 dataAvailable, data = 0;

		// Safely check whether there is data available to send
		DISABLE_USART_INTERRUPTS();
		dataAvailable = (DXLTx.RawBufReadPtr != DXLTx.RawBufWritePtr);
		if(dataAvailable)
		{
			data = DXLTx.RawBuf[DXLTx.RawBufReadPtr++];
			DXLTx.RawBufReadPtr &= USART_TX_BUFMASK;
		}
		REENABLE_USART_INTERRUPTS();

		// Start a transmission if data is available
		if(dataAvailable)
		{
			// Enable/disable RX/TX pins for sending
			GPIO_ResetBits(PORT_ENABLE_RX, PIN_ENABLE_RX); // RX Disable
			GPIO_SetBits(PORT_ENABLE_TX, PIN_ENABLE_TX);   // TX Enable

			// Signal that a transmission is starting
			DXLTx.Transmitting = TRUE;

			// Delay for a small amount of time to avoid the GPIO operation above being too soon before the first byte is sent
			u8 i; for(i = 0; i < 100; i++) TmpVar += i;

			// Send the first byte
			DXLTxSendByte(data);
		}
	}
}

// ZIG byte received interrupt service routine (UART5)
void __ISR_ZIG_USART()
{
#if ALLOW_ZIGBEE
	// If we have received data from the Zigbee over UART5...
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		// Read one byte from the received data register
		u8 ReceivedData = (u8) USART_ReceiveData(UART5);

		// Write the received byte into the ZIG Rx circular buffer
		ZIGRxBuf[ZIGRxBufWritePtr++] = ReceivedData;
		ZIGRxBufWritePtr &= USART_ZIG_RX_BUFMASK;
		if(ZIGRxBufWritePtr == ZIGRxBufReadPtr) // Circular buffer was full, so we must forcibly advance the read pointer to avoid the buffer now looking empty!
		{
			ZIGRxBufReadPtr++;
			ZIGRxBufReadPtr &= USART_ZIG_RX_BUFMASK;
			ZIGRxBufOverflowCount++;
		}
	}
#else
	// Clear all flags to make doubly sure (no interrupts should actually be enabled if ALLOW_ZIGBEE is 0) that we don't get stuck in an infinite interrupt loop here
	TmpVar += (u8) UART5->SR;                                          // Read from USART_SR
	UART5->SR = (u16) ~(USART_FLAG_RXNE|USART_FLAG_TC|USART_FLAG_LBD); // Clear the flags that can be manually cleared (CTS is not available in UART5)
	TmpVar += (u8) UART5->DR;                                          // Read from USART_DR
#endif

	// Memory barrier to ensure that no funny business happens with the interrupts and interrupt flags
	__asm volatile("DSB;ISB");
}

//
// Buffering and dynamixel forwarding
//

// Enable the storing of packets received from the DXLs in a buffer for local processing
void enableDXLBuffering(void)
{
	gbDXLBuffering = TRUE;
}

// Disable the storing of packets received from the DXLs in a buffer for local processing
void disableDXLBuffering(void)
{
	gbDXLBuffering = FALSE;
}

// Enable forwarding of all PC packets to the DXLs
void enableDXLForwarding(void)
{
	// Set the flag to true
	gbDXLForwarding = TRUE;
}

// Disable forwarding of all PC packets to the DXLs
void disableDXLForwarding(void)
{
	// Set the flag to false
	gbDXLForwarding = FALSE;
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
