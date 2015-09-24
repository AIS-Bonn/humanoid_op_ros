/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : CM_DXL_COM.c
* Author             : dudung
* Version            : V0.1
* Date               : 2010/11/03
* Description        : Implements the PC and Dynamixel communication architecture,
*                      and the general main function of the CM730
* Comment            : This file has been completely rewritten by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14).
*******************************************************************************/

// Includes
#include "CM_DXL_COM.h"
#include "stm32f10x_nvic.h"
#include "system_init.h"
#include "system_func.h"
#include "gyro_acc.h"
#include "dxl_defs.h"
#include "zigbee.h"
#include "sound.h"
#include "usart.h"
#include "isr.h"
#include "led.h"

// Includes - Library
#include <string.h> // For memcpy()

// Status return levels
#define RETURN_NO_PACKET       0      // Respond with the device status only to PING commands
#define RETURN_READ_PACKET     1      // Respond with the device status only to PING and READ commands
#define RETURN_ALL_PACKET      2      // Respond with the device status to all commands

// Parameter range array index
#define LOW_LIMIT              0      // Index in the gbCTParamRange[][2] array of the lower bound for the given parameter
#define HIGH_LIMIT             1      // Index in the gbCTParamRange[][2] array of the upper bound for the given parameter

// Default ROM values
#if IS_CM730
#define CM730_MODEL_NUMBER     0x7301 // Model number of the NimbRo-OP specific CM730
#elif IS_CM740
#define CM730_MODEL_NUMBER     0x7401 // Model number of the NimbRo-OP specific CM740
#endif
#define FIRMWARE_VERSION       0x87   // Version 0x87 as of modifications by Philipp Allgeuer (10/09/15 and later)
#define DEFAULT_ID             200    // Default CM730 device ID on the Dynamixel bus
#define BROADCASTING_ID        0xFE   // Device ID for broadcasting on the Dynamixel bus (every device listens)
#define DEF_RETURN_DELAY_TIME  0      // Delay before returning status packet = DEF_RETURN_DELAY_TIME*2us = 0us
#define CW_ANGLE_FIXED_LIMIT   0x0000 // Default CW angular limit (N/A for CM730)
#define CCW_ANGLE_FIXED_LIMIT  0x0FFF // Default CCW angular limit (N/A for CM730)
#define DEFAULT_MAX_TORQUE     0x03FF // Default maximum torque (N/A for CM730)
#define DEFAULT_ALARM_STATE    0x24   // Default value of the alarm registers
#define DEFAULT_OPERATING_MODE 0
#define DEFAULT_DOWN_CALIB     0
#define DEFAULT_UP_CALIB       0

// Misc defines
#define SERVO_MAX_TORQUE       0x03FF // Value to initialise the servo max torque and torque limit registers to on firmware start
#define BULK_READ_TIMEOUT      25     // Number of ms to wait for the next bulk read status packet before aborting
#define TRUE                   1
#define FALSE                  0

// Macros
#define SYSTEM_RESET           NVIC_GenerateSystemReset()

// Functions
u8   ConvertBulkRead(struct DxlPacket *DP);
u8   ConvertSyncWrite(struct DxlPacket *DP);
void ReadData(const struct DxlPacket *DP, u8 IsBulkRead);
u8   WriteDataCheck(const struct DxlPacket *DP);
void WriteData(const struct DxlPacket *DP);
void BroadcastPacket(struct DxlPacket *DP);
void BroadcastPacketWithDelay(struct DxlPacket *DP);
void ReturnPacket(struct DxlPacket *DP);
void ReturnPacketWithDelay(struct DxlPacket *DP);
void ReturnErrorStatus(u8 Error);
void ReturnErrorStatusWithDelay(u8 Error);

// Initial ROM control table parameter values
byte ROM_INITIAL_DATA[ROM_CONTROL_TABLE_LEN] =
{
	LOW_BYTE(CM730_MODEL_NUMBER),      // MODEL_NUMBER_L         0
	HIGH_BYTE(CM730_MODEL_NUMBER),     // MODEL_NUMBER_H         1
	FIRMWARE_VERSION,                  // VERSION                2
	DEFAULT_ID,                        // ID                     3
	DEFAULT_BAUD_RATE,                 // BAUD_RATE              4
	DEF_RETURN_DELAY_TIME,             // RETURN_DELAY_TIME      5
	LOW_BYTE(CW_ANGLE_FIXED_LIMIT),    // CW_ANGLE_LIMIT_L       6
	HIGH_BYTE(CW_ANGLE_FIXED_LIMIT),   // CW_ANGLE_LIMIT_H       7
	LOW_BYTE(CCW_ANGLE_FIXED_LIMIT),   // CCW_ANGLE_LIMIT_L      8
	HIGH_BYTE(CCW_ANGLE_FIXED_LIMIT),  // CCW_ANGLE_LIMIT_H      9
	0,                                 // SYSTEM_DATA2           10 (Reserved)
	80,                                // LIMIT_TEMPERATURE      11 (Degrees)
	VOLTAGE_LEVEL_0,                   // VOLTAGE_LOWER_LIMIT    12 (Decivolts)
	VOLTAGE_LEVEL_6,                   // VOLTAGE_UPPER_LIMIT    13 (Decivolts)
	LOW_BYTE(DEFAULT_MAX_TORQUE),      // MAX_TORQUE_L           14
	HIGH_BYTE(DEFAULT_MAX_TORQUE),     // MAX_TORQUE_H           15
	RETURN_READ_PACKET,                // RETURN_LEVEL           16
	DEFAULT_ALARM_STATE,               // ALARM_LED              17
	DEFAULT_ALARM_STATE,               // ALARM_SHUTDOWN         18
	DEFAULT_OPERATING_MODE,            // OPERATING_MODE         19
	LOW_BYTE(DEFAULT_DOWN_CALIB),      // DOWN_CALIBRATION_L     20
	HIGH_BYTE(DEFAULT_DOWN_CALIB),     // DOWN_CALIBRATION_H     21
	LOW_BYTE(DEFAULT_UP_CALIB),        // UP_CALIBRATION_L       22
	HIGH_BYTE(DEFAULT_UP_CALIB)        // UP_CALIBRATION_H       23
};

// Allowed control table parameter ranges (L > R => Cannot write into register using INST_WRITE)
byte gbCTParamRange[CONTROL_TABLE_LEN][2] =
{
	{1,0},     // MODEL_NUMBER_L         0   vvvv  Start ROM  vvvv
	{1,0},     // MODEL_NUMBER_H         1
	{1,0},     // VERSION                2
	{0,253},   // ID                     3
	{1,254},   // BAUD_RATE              4
	{0,254},   // RETURN_DELAY_TIME      5
	{0,255},   // CW_ANGLE_LIMIT_L       6
	{0,3},     // CW_ANGLE_LIMIT_H       7
	{0,255},   // CCW_ANGLE_LIMIT_L      8
	{0,3},     // CCW_ANGLE_LIMIT_H      9
	{1,0},     // SYSTEM_DATA2           10
	{0,150},   // LIMIT_TEMPERATURE      11
	{1,0},     // VOLTAGE_LOWER_LIMIT    12  Note: Write access to this has been disabled
	{1,0},     // VOLTAGE_UPPER_LIMIT    13  Note: Write access to this has been disabled
	{0,255},   // MAX_TORQUE_L           14
	{0,3},     // MAX_TORQUE_H           15
	{0,2},     // RETURN_LEVEL           16
	{1,0},     // ALARM_LED              17  Note: Write access to this has been disabled
	{1,0},     // ALARM_SHUTDOWN         18  Note: Write access to this has been disabled
	{0,255},   // OPERATING_MODE         19
	{1,0},     // DOWN_CALIBRATION_L     20
	{1,0},     // DOWN_CALIBRATION_H     21
	{1,0},     // UP_CALIBRATION_L       22
	{1,0},     // UP_CALIBRATION_H       23  ^^^^   End ROM   ^^^^
	{0,2},     // DYNAMIXEL_POWER        24  vvvv  Start RAM  vvvv
	{0,7},     // LED_PANEL              25
	{1,0},     // RGBLED5                26  Note: Write access to this has been disabled
	{1,0},     //                        27  Note: Write access to this has been disabled
	{0,255},   // RGBLED6                28
	{0,127},   //                        29
	{1,0},     // BUTTON                 30
	{1,0},     // BATTERY_VOLTAGE        31
	{1,0},     // GYRO_X                 32
	{1,0},     //                        33
	{1,0},     // GYRO_Y                 34
	{1,0},     //                        35
	{1,0},     // GYRO_Z                 36
	{1,0},     //                        37
	{1,0},     // ACC_X                  38
	{1,0},     //                        39
	{1,0},     // ACC_Y                  40
	{1,0},     //                        41
	{1,0},     // ACC_Z                  42
	{1,0},     //                        43
	{1,0},     // MAG_X                  44
	{1,0},     //                        45
	{1,0},     // MAG_Y                  46
	{1,0},     //                        47
	{1,0},     // MAG_Z                  48
	{1,0},     //                        49
	{1,0},     // ADC0_BATTERY           50
	{1,0},     //                        51
	{1,0},     // ADC1_MIC1              52
	{1,0},     //                        53
	{1,0},     // ADC2_MIC2              54
	{1,0},     //                        55
	{1,0},     // ADC3                   56
	{1,0},     //                        57
	{1,0},     // ADC4                   58
	{1,0},     //                        59
	{1,0},     // ADC5                   60
	{1,0},     //                        61
	{1,0},     // ADC6                   62
	{1,0},     //                        63
	{1,0},     // ADC7                   64
	{1,0},     //                        65
	{1,0},     // ADC8                   66
	{1,0},     //                        67
	{1,0},     // ADC9                   68
	{1,0},     //                        69
	{1,0},     // ADC10                  70
	{1,0},     //                        71
	{1,0},     // ADC11                  72
	{1,0},     //                        73
	{1,0},     // ADC12                  74
	{1,0},     //                        75
	{1,0},     // ADC13                  76
	{1,0},     //                        77
	{1,0},     // ADC14                  78
	{1,0},     //                        79
	{1,0},     // ADC15                  80
	{1,0},     //                        81
	{0,0xFF},  // BUZZER_DATA            82
	{0,0xFF},  // BUZZER_PLAY_LENGTH     83
	{1,0},     // ZIGBEE_ID              84
	{1,0},     //                        85
	{0,0xFF},  // TX_REMOCON_DATA        86
	{0,0xFF},  //                        87
	{1,0},     // RX_REMOCON_DATA        88
	{1,0},     //                        89
	{1,0},     // RX_REMOCON_DATA_ARR    90
	{1,0},     // DXLRX_PACKET_CNT       91
	{1,0},     //                        92
	{1,0},     // DXLRX_OVERFLOW_CNT     93
	{1,0},     //                        94
	{1,0},     // DXLRX_BUFERROR_CNT     95
	{1,0},     //                        96
	{1,0},     // DXLRX_CHKERROR_CNT     97
	{1,0},     //                        98
	{1,0},     // DXLRX_ORE_CNT          99
	{1,0},     //                        100
	{1,0},     // DXLTX_PACKET_CNT       101
	{1,0},     //                        102
	{1,0},     // DXLTX_OVERFLOW_CNT     103
	{1,0},     //                        104
	{1,0},     // DXLTX_BUFERROR_CNT     105
	{1,0},     //                        106
	{1,0},     // PCRX_PACKET_CNT        107
	{1,0},     //                        108
	{1,0},     // PCRX_OVERFLOW_CNT      109
	{1,0},     //                        110
	{1,0},     // PCRX_BUFERROR_CNT      111
	{1,0},     //                        112
	{1,0},     // PCRX_CHKERROR_CNT      113
	{1,0},     //                        114
	{1,0},     // PCRX_ORE_CNT           115
	{1,0},     //                        116
	{1,0},     // PCTX_PACKET_CNT        117
	{1,0},     //                        118
	{1,0},     // PCTX_OVERFLOW_CNT      119
	{1,0},     //                        120
	{1,0},     // PCTX_BUFERROR_CNT      121
	{1,0},     //                        122
	{0,255},   // MISC0                  123
	{0,255},   // MISC1                  124
	{0,255},   // MISC2                  125
	{0,255}    // MISC3                  126 ^^^^  End RAM  ^^^^
};

// Control table parameter data sizes in bytes (0 = N/A, 1 = 8-bit, 2 = 16-bit)
byte gbCTDataSize[CONTROL_TABLE_LEN] =
{
  2, // MODEL_NUMBER_L         0   vvvv  Start ROM  vvvv
  0, // MODEL_NUMBER_H         1
  1, // VERSION                2
  1, // ID                     3
  1, // BAUD_RATE              4
  1, // RETURN_DELAY_TIME      5
  2, // CW_ANGLE_LIMIT_L       6
  0, // CW_ANGLE_LIMIT_H       7
  2, // CCW_ANGLE_LIMIT_L      8
  0, // CCW_ANGLE_LIMIT_H      9
  1, // SYSTEM_DATA2           10
  1, // LIMIT_TEMPERATURE      11
  1, // VOLTAGE_LOWER_LIMIT    12
  1, // VOLTAGE_UPPER_LIMIT    13
  2, // MAX_TORQUE_L           14
  0, // MAX_TORQUE_H           15
  1, // RETURN_LEVEL           16
  1, // ALARM_LED              17
  1, // ALARM_SHUTDOWN         18
  1, // OPERATING_MODE         19
  2, // DOWN_CALIBRATION_L     20
  0, // DOWN_CALIBRATION_H     21
  2, // UP_CALIBRATION_L       22
  0, // UP_CALIBRATION_H       23  ^^^^   End ROM   ^^^^
  1, // DYNAMIXEL_POWER        24  vvvv  Start RAM  vvvv
  1, // LED_PANEL              25
  2, // RGBLED5                26
  0, //                        27
  2, // RGBLED6                28
  0, //                        29
  1, // BUTTON                 30
  1, // BATTERY_VOLTAGE        31
  2, // GYRO_X                 32
  0, //                        33
  2, // GYRO_Y                 34
  0, //                        35
  2, // GYRO_Z                 36
  0, //                        37
  2, // ACC_X                  38
  0, //                        39
  2, // ACC_Y                  40
  0, //                        41
  2, // ACC_Z                  42
  0, //                        43
  2, // MAG_X                  44
  0, //                        45
  2, // MAG_Y                  46
  0, //                        47
  2, // MAG_Z                  48
  0, //                        49
  2, // ADC0_BATTERY           50
  0, //                        51
  2, // ADC1_MIC1              52
  0, //                        53
  2, // ADC2_MIC2              54
  0, //                        55
  2, // ADC3                   56
  0, //                        57
  2, // ADC4                   58
  0, //                        59
  2, // ADC5                   60
  0, //                        61
  2, // ADC6                   62
  0, //                        63
  2, // ADC7                   64
  0, //                        65
  2, // ADC8                   66
  0, //                        67
  2, // ADC9                   68
  0, //                        69
  2, // ADC10                  70
  0, //                        71
  2, // ADC11                  72
  0, //                        73
  2, // ADC12                  74
  0, //                        75
  2, // ADC13                  76
  0, //                        77
  2, // ADC14                  78
  0, //                        79
  2, // ADC15                  80
  0, //                        81
  1, // BUZZER_DATA            82
  1, // BUZZER_PLAY_LENGTH     83
  2, // ZIGBEE_ID              84
  0, //                        85
  2, // TX_REMOCON_DATA        86
  0, //                        87
  2, // RX_REMOCON_DATA        88
  0, //                        89
  1, // RX_REMOCON_DATA_ARR    90
  2, // DXLRX_PACKET_CNT       91
  0, //                        92
  2, // DXLRX_OVERFLOW_CNT     93
  0, //                        94
  2, // DXLRX_BUFERROR_CNT     95
  0, //                        96
  2, // DXLRX_CHKERROR_CNT     97
  0, //                        98
  2, // DXLRX_ORE_CNT          99
  0, //                        100
  2, // DXLTX_PACKET_CNT       101
  0, //                        102
  2, // DXLTX_OVERFLOW_CNT     103
  0, //                        104
  2, // DXLTX_BUFERROR_CNT     105
  0, //                        106
  2, // PCRX_PACKET_CNT        107
  0, //                        108
  2, // PCRX_OVERFLOW_CNT      109
  0, //                        110
  2, // PCRX_BUFERROR_CNT      111
  0, //                        112
  2, // PCRX_CHKERROR_CNT      113
  0, //                        114
  2, // PCRX_ORE_CNT           115
  0, //                        116
  2, // PCTX_PACKET_CNT        117
  0, //                        118
  2, // PCTX_OVERFLOW_CNT      119
  0, //                        120
  2, // PCTX_BUFERROR_CNT      121
  0, //                        122
  1, // MISC0                  123
  1, // MISC1                  124
  1, // MISC2                  125
  1  // MISC3                  126 ^^^^  End RAM  ^^^^
};

// Global variables
struct DxlPacket DPtmp = {INVALID_ID, INST_NONE, 0, {0}};
struct DxlPacket DPBR = {INVALID_ID, INST_NONE, 0, {0}};
vu8 gbControlTable[CONTROL_TABLE_LEN+1+CT_EXTRA] = {0}; // Control table (one more byte than required is allocated, in case a GW_CONTROL_TABLE access is done on the highest index)
vu8 gbAlarmState = 0x00; // Alarm state (for error reporting by CM730)

// Main loop function for the whole firmware application (called from main() in main.c)
void Process(void)
{
	// Declare variables
	struct DxlPacket DP = {INVALID_ID, INST_NONE, 0, {0}};
	u8 Error, Converted;
	
	// Initialise the LED control table parameters
	GB_LED_PANEL = 0;
	GW_RGBLED5 = (0>>3) | ((255>>3)<<5) | ((  0>>3)<<10); // RGBLED5 = RGB(0,255,0) = Green
	GW_RGBLED6 = (0>>3) | ((  0>>3)<<5) | ((255>>3)<<10); // RGBLED6 = RGB(0,0,255) = Blue

	// Initialisation actions for the values in the control table
	OnControlTableWrite(P_LED_PANEL);
	OnControlTableWrite(P_RGBLED5);
	OnControlTableWrite(P_RGBLED6);

	// Clear the USART Rx buffers
	USARTClearBuffers(USART_PC, USART_CLEAR_RX);
	USARTClearBuffers(USART_DXL, USART_CLEAR_RX);

	// Main loop
	while(1)
	{
		// The following things can still be improved on in the way that the dynamixel protocol is handled:
		//   - Implement the INST_REG_WRITE, INST_SYNC_REG_WRITE and INST_ACTION instructions
		//   - Allow multiple instances of the same ID in the same bulk read
		//   - Add backup register feature and implement INST_SYSTEM_WRITE

		// Wait for an instruction packet from the PC
		RGBLED_SetColour(RGBLED5, 0, 255, 0); // Green => Waiting for packet
		while(!RxDDataAvailable(USART_PC));
		RxDDataDP(USART_PC, &DP);
		RGBLED_SetColour(RGBLED5, 255, 0, 255); // Magenta => Processing packet

		// Update the USART control table registers
		USARTUpdateControlTable();

		// Ignore this packet if it is not destined for us
		if((DP.ID != GB_ID) && (DP.ID != BROADCASTING_ID)) continue; // Note: If RxDDataDP() fails for whatever reason, this check filters out packets that are returned with INVALID_ID

		// Refer to the following link for information on the set of instructions, and their respective formats:
		// http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm

		// Initialise variables
		Converted = FALSE;

		// Handle special instruction: Repeat last bulk read
		if(DP.Instruction == INST_REPEAT_BULK)
		{
			// If we haven't saved a bulk read yet then ignore this instruction
			if(DPBR.Instruction != INST_BULK_READ) continue;

			// Send off the bulk read packet to the servos
			TxDDataDP(USART_DXL, &DPBR);

			// Copy out the old bulk read packet and pretend it's new
			const char* src = (const char *) &DPBR;
			char* dst = (char *) &DP;
			memcpy(dst, src, sizeof(struct DxlPacket));
			Converted = TRUE;
		}

		// Handle special instruction: Bulk read
		if(DP.Instruction == INST_BULK_READ)
		{
			// Copy out the bulk read packet to a special storage location
			if(Converted != TRUE)
			{
				const char* src = (const char *) &DP;
				char* dst = (char *) &DPBR;
				memcpy(dst, src, sizeof(struct DxlPacket));
			}

			// Expected parameter format is [0x00] [L1] [ID1] [Addr1] ... [LN] [IDN] [AddrN]
			// We convert the bulk read instruction in-place into a simple read instruction
			Converted = ConvertBulkRead(&DP);

			// If the packet conversion failed or our ID wasn't listed, then there is no need to process this packet any further
			if(Converted != TRUE) continue;
		}

		// Handle special instruction: Synchronous write
		if(DP.Instruction == INST_SYNC_WRITE)
		{
			// Expected parameter format is [Addr] [L] [ID1] [Data1] ... [DataL] ... [IDN] [Data1] ... [DataL]
			// We convert the synchronous write instruction in-place into a simple write instruction
			Converted = ConvertSyncWrite(&DP);

			// If the packet conversion failed or our ID wasn't listed, then there is no need to process this packet any further
			if(Converted != TRUE) continue;
		}

		// Handle all remaining instructions
		switch(DP.Instruction)
		{
			// Handle write instruction
			case INST_WRITE: // Expected parameter format is [Addr] [Data1] ... [DataN]
				Error = WriteDataCheck(&DP);
				if((GB_RETURN_LEVEL >= RETURN_ALL_PACKET) && (DP.ID != BROADCASTING_ID))
					ReturnErrorStatusWithDelay(Error);
				if(Error == NO_ERROR_BIT)
					WriteData(&DP);
				break;

			// Handle read instruction
			case INST_READ: // Expected parameter format is [Addr] [Count]
				if((GB_RETURN_LEVEL >= RETURN_READ_PACKET) && (DP.ID != BROADCASTING_ID))
					ReadData(&DP, Converted);
				break;

			// Handle system write instruction
			case INST_SYSTEM_WRITE: // Expected parameter format is [Addr] [Data] [0xF0] [0x55] [0x0F] [0xAA]
				if((DP.NumParams == 6) && (DP.Param[2] == 0xF0) && (DP.Param[3] == 0x55) && (DP.Param[4] == 0x0F) && (DP.Param[5] == 0xAA) && (DP.Param[0] < ROM_CONTROL_TABLE_LEN))
				{
					// Note: No action for the system write instruction has been implemented yet!
					//       Any future implementation does not need to return a status packet for this instruction.
				}
				break;

			// Handle ping instruction
			case INST_PING:
				ReturnErrorStatusWithDelay(NO_ERROR_BIT); // Note: For a broadcasted ping we just try to answer as quickly as possible, before the Dynamixels even had a chance to respond, instead of artificially inserting an additional delay.
				break;

			// Handle reset instruction
			case INST_RESET:
				if((GB_RETURN_LEVEL >= RETURN_ALL_PACKET) && (DP.ID != BROADCASTING_ID))
					ReturnErrorStatusWithDelay(NO_ERROR_BIT);
				SYSTEM_RESET;
				break;

			// Handle digital reset instruction
			case INST_DIGITAL_RESET:
				if((GB_RETURN_LEVEL >= RETURN_ALL_PACKET) && (DP.ID != BROADCASTING_ID))
					ReturnErrorStatusWithDelay(NO_ERROR_BIT);
				SYSTEM_RESET;
				break;

			// Unrecognised instruction
			default:
				if((GB_RETURN_LEVEL >= RETURN_ALL_PACKET) && (DP.ID != BROADCASTING_ID))
					ReturnErrorStatusWithDelay(INSTRUCTION_ERROR_BIT);
				break;
		}
	}
}

// Convert a bulk read instruction in-place into a simple read instruction
u8 ConvertBulkRead(struct DxlPacket *DP)
{
	// Declare variables
	u8 PrevID, i;
	u32 StartTime;

	// Error checking on the packet length
	if(DP->NumParams < 4) return FALSE;

	// Work out which ID precedes our own in the bulk read packet (if we're even listed)
	// Note: It is assumed here that our ID only appears once in the bulk read
	PrevID = INVALID_ID;
	for(i = 2; i < DP->NumParams - 1; i += 3)
	{
		if(DP->Param[i] == GB_ID)
		{
			DP->ID = GB_ID;
			DP->Instruction = INST_READ;
			DP->NumParams = 2;
			DP->Param[0] = DP->Param[i+1]; // [Addr]
			DP->Param[1] = DP->Param[i-1]; // [Count]
			i = 0;                         // Signal that our ID was found
			break;
		}
		PrevID = DP->Param[i];
	}

	// If our ID wasn't listed then we don't need to do anything
	if(i != 0) return FALSE;
	if(PrevID == BROADCASTING_ID) return FALSE;

	// If we are not the first ID in the bulk read then wait for a status packet from the device ID that should precede us
	if(PrevID != INVALID_ID)
	{
		// Clear out the DXL Rx buffer
		USARTClearBuffers(USART_DXL, USART_CLEAR_RX); // Note: Clears only the packet buffer, not the lower level raw Rx buffer

		// Enable siphoning of DXL Rx packets
		enableDXLBuffering();

		// Wait for a packet to arrive from the DXLs
		StartTime = gbMillisec; // Note: This limits the time for the *entire* bulk read to a given timeout value!
		while(1)
		{
			while(!RxDDataAvailable(USART_DXL))
			{
				if(gbMillisec - StartTime > BULK_READ_TIMEOUT) return FALSE;
				if(RxDDataAvailable(USART_PC)) return FALSE;  // If we receive a packet from the PC then assume that the PC thinks the bulk read is over, or stopped waiting for it...
			}
			RxDDataDP(USART_DXL, &DPtmp); // Note: If RxDDataDP() fails then DPtmp.ID becomes INVALID_ID, which doesn't satisfy the following check, so the returned packet from RxDDataDP() is essentially ignored
			if(DPtmp.ID == PrevID) break;
		}

		// Disable siphoning of DXL Rx packets again
		disableDXLBuffering();
	}

	// At this point it is our turn to return a bulk read status packet, so we allow the INST_READ packet constructed above to do its job
	return TRUE;
}

// Convert a synchronous write instruction in-place into a simple write instruction
u8 ConvertSyncWrite(struct DxlPacket *DP)
{
	// Declare variables
	u8 L, i, j;

	// Error checking on the packet length
	if(DP->NumParams < 4) return FALSE;

	// Retrieve the data length
	L = DP->Param[1];

	// Error checking on the data length
	if(L == 0) return FALSE;
	if(L >= DP->NumParams - 2) return FALSE;

	// Retrieve the parameters destined for our device ID (if our ID is even listed)
	for(i = 2; i < DP->NumParams - L; i += L + 1)
	{
		if(DP->Param[i] == GB_ID)
		{
			for(j = 1; j <= L; j++)
				DP->Param[j] = DP->Param[i+j];
			DP->NumParams = L + 1;   // New format is [Addr] [Data1] ... [DataL]
			DP->Instruction &= 0x7F; // Changes INST_SYNC_WRITE to INST_WRITE
			DP->ID = BROADCASTING_ID;
			return TRUE;
		}
	}

	// Return that our ID wasn't listed
	return FALSE;
}

// Construct a status packet with the contents of the required registers, as specified by the read instruction
void ReadData(const struct DxlPacket *DP, u8 IsBulkRead)
{
	// Declare variables
	u8 Error, Address, Count, i;

	// Initialise the error flag
	Error = NO_ERROR_BIT;

	// Check that we have enough parameters and that the requested address range is valid
	if(DP->NumParams < 2)
		Error = INSTRUCTION_ERROR_BIT;
	else
	{
		Address    = DP->Param[0];
		Count      = DP->Param[1];
#ifndef EXTRA_CONTROL_TABLE
		u8 EndAddress = Address + Count - 1;
		if((Address >= CONTROL_TABLE_LEN) || (EndAddress >= CONTROL_TABLE_LEN) || (EndAddress < Address)) Error = RANGE_ERROR_BIT;
		if((gbCTDataSize[Address] == 0) || (gbCTDataSize[EndAddress] == 2)) Error = RANGE_ERROR_BIT; // Comment out this line to allow unaligned READs
#endif
	}

	// Return a status packet with an error code if there was something wrong with the read instruction
	if(Error != NO_ERROR_BIT)
	{
		if(IsBulkRead == TRUE)
		{
			DPtmp.ID = GB_ID;
			DPtmp.Instruction = Error;
			DPtmp.NumParams = 0;
			BroadcastPacketWithDelay(&DPtmp);
		}
		else if((GB_RETURN_LEVEL >= RETURN_ALL_PACKET) && (DP->ID != BROADCASTING_ID))
			ReturnErrorStatusWithDelay(Error);
		return;
	}

	// Start constructing the required return packet
	DPtmp.ID = GB_ID;
	DPtmp.Instruction = NO_ERROR_BIT;
	DPtmp.NumParams = Count;

	// Copy out the required control table registers to the parameter array
	for(i = 0; i < Count; i++,Address++)
	{
#if ALLOW_ZIGBEE
		if(Address == P_RX_REMOCON_DATA)
			GW_RX_REMOCON_DATA = zgb_rx_data();
		else if(Address == P_RX_REMOCON_DATA_ARR)
			GB_RX_REMOCON_DATA_ARR = zgb_rx_check();
#endif
		if(Address == P_GYRO_X) // If this is a read of the gyro/acc sensor data, then reset the corresponding data history buffer
			GyroAccAvgBufClear = 1;
		DPtmp.Param[i] = GB_CONTROL_TABLE(Address);
	}

	// Send the required status packet
	if(IsBulkRead == TRUE)
		BroadcastPacketWithDelay(&DPtmp); // Send response to PC and DXLs
	else
		ReturnPacketWithDelay(&DPtmp);    // Send response to PC only
}

// Check the parameters of a write instruction
u8 WriteDataCheck(const struct DxlPacket *DP)
{
	// Declare variables
	u8 Address, EndAddress, i;

	// Make sure that there are at least two parameters to the write instruction
	if(DP->NumParams < 2) return INSTRUCTION_ERROR_BIT;

	// Retrieve the base address that the instruction wishes to write to
	Address    = DP->Param[0];
	EndAddress = Address + DP->NumParams - 2;

	// Check that the addresses lie within the control table
	if((Address >= CONTROL_TABLE_LEN) || (EndAddress >= CONTROL_TABLE_LEN) || (EndAddress < Address)) return RANGE_ERROR_BIT;

	// Check that the first address points to a valid register and that the last register referenced is completely written to
	if((gbCTDataSize[Address] == 0) || (gbCTDataSize[EndAddress] == 2)) return RANGE_ERROR_BIT; // Comment out this line to allow unaligned WRITEs

	// Check that the values that the instruction wishes to write are inside the predefined allowed ranges
	for(i = 1; i < DP->NumParams; i++,Address++)
	{
		if((DP->Param[i] < gbCTParamRange[Address][LOW_LIMIT]) || (DP->Param[i] > gbCTParamRange[Address][HIGH_LIMIT]))
			return RANGE_ERROR_BIT; // Note that if ...[LOW_LIMIT] > ...[HIGH_LIMIT] then this is always true and so writing to the corresponding register is effectively disabled!
	}

	// Return that the data to write is valid
	return NO_ERROR_BIT;
}

// Write the parameters from a write instruction into the control table as required
void WriteData(const struct DxlPacket *DP) // Assumes that WriteDataCheck() on the same DP returns/returned NO_ERROR_BIT!
{
	// Declare variables
	u8 Address, i;

	// Retrieve the base address that the instruction wishes to write to
	Address = DP->Param[0];

	// Write to each of the addresses in turn as required
	for(i = 1; i < DP->NumParams; i++,Address++)
	{
		GB_CONTROL_TABLE(Address) = DP->Param[i];
		OnControlTableWrite(Address);
	}
}

// Perform the required actions after a write to the control table
void OnControlTableWrite(u8 address)
{
	// Declare variables
	u32 lTemp;

	// Perform the required action based on the address that was written to
	switch(address)
	{
		// Baud rate register
		case P_BAUD_RATE:
			lTemp = 2000000;
			lTemp /= (GB_BAUD_RATE + 1); // Note: The value of the baud rate register is interpreted to mean a required baud rate of 2000000/(GB_BAUD_RATE + 1)!
			USART_Configuration(USART_DXL, lTemp);
			USART_Configuration(USART_PC , lTemp);
			break;

		// Dynamixel power register
		case P_DYNAMIXEL_POWER: // Note: It is a safety feature that if an out of range value is written into this register, the DXL power state doesn't change
			if(GB_DYNAMIXEL_POWER == 0)
			{
				DXLSetPower(OFF);
				disableDXLForwarding();
			}
			else if(GB_DYNAMIXEL_POWER == 1)
			{
				DXLSetPower(ON);
				enableDXLForwarding();
			}
			else if(GB_DYNAMIXEL_POWER == 2)
			{
				DXLSetPower(ON);
				disableDXLForwarding();
			}
			break;

		// LED panel register
		case P_LED_PANEL:
			LED_SetState(GB_LED_PANEL, ON);   // Turn the required LEDs on
			LED_SetState(~GB_LED_PANEL, OFF); // Turn all the other LEDs off
			break;

		// RGB LED 5 register
		case P_RGBLED5:
			RGBLED_SetColour(RGBLED5, (GW_RGBLED5&0x1F)<<3, ((GW_RGBLED5>>5)&0x1F)<<3, ((GW_RGBLED5>>10)&0x1F)<<3);
			break;

		// RGB LED 6 register
		case P_RGBLED6:
			RGBLED_SetColour(RGBLED6, (GW_RGBLED6&0x1F)<<3, ((GW_RGBLED6>>5)&0x1F)<<3, ((GW_RGBLED6>>10)&0x1F)<<3);
			break;

		// Buzzer registers (write the play length into GB_BUZZER_PLAY_LENGTH [e.g. 255], then write the buzzer data into GB_BUZZER_DATA [e.g. 22], the second action triggers the sound)
		case P_BUZZER_DATA:
			setBuzzerPlayLength(GB_BUZZER_PLAY_LENGTH);
			if((getBuzzerState() == 0) || (GB_BUZZER_PLAY_LENGTH == 0xFE))
			{
				setBuzzerData(GB_BUZZER_DATA);
				PlayBuzzer();
			}
			break;
		case P_BUZZER_PLAY_LENGTH:
			if(GB_BUZZER_PLAY_LENGTH == 0x00)
				setBuzzerOff();
			break;

		// Zigbee remote control TX register
		case P_TX_REMOCON_DATA:
#if ALLOW_ZIGBEE
			zgb_tx_data(GW_TX_REMOCON_DATA);
#endif
			break;

		// No action required for all other registers
		default:
			break;
	}
}

// Broadcast a status packet with the given data (to PC and DXL)
void BroadcastPacket(struct DxlPacket *DP)
{
	// Add concurrent error bits as required
	DP->Instruction |= gbAlarmState;

	// Confirm the device ID
	DP->ID = GB_ID;

	// Send the required status packet
	TxDDataDP(USART_DXL, DP);
	TxDDataDP(USART_PC, DP);
}

// Broadcast a status packet with the given data (to PC and DXL), first waiting out the required return delay time
void BroadcastPacketWithDelay(struct DxlPacket *DP)
{
	// Wait out the required status packet return delay time (in units of 2us)
	uDelay(((u32) GB_RETURN_DELAY_TIME) << 1);

	// Return the required packet
	BroadcastPacket(DP);
}

// Return a status packet with the given data
void ReturnPacket(struct DxlPacket *DP)
{
	// Add concurrent error bits as required
	DP->Instruction |= gbAlarmState;

	// Confirm the device ID
	DP->ID = GB_ID;

	// Send the required status packet
	TxDDataDP(USART_PC, DP);
}

// Return a status packet with the given data, first waiting out the required return delay time
void ReturnPacketWithDelay(struct DxlPacket *DP)
{
	// Wait out the required status packet return delay time (in units of 2us)
	uDelay(((u32) GB_RETURN_DELAY_TIME) << 1);

	// Return the required packet
	ReturnPacket(DP);
}

// Return a status packet with the given error code
void ReturnErrorStatus(u8 Error)
{
	// Add concurrent error bits as required
	Error |= gbAlarmState;

	// Construct the required status packet
	DPtmp.ID = GB_ID;
	DPtmp.Instruction = Error;
	DPtmp.NumParams = 0;

	// Send the required status packet
	TxDDataDP(USART_PC, &DPtmp);
}

// Return a status packet with the given error code, first waiting out the required return delay time
void ReturnErrorStatusWithDelay(u8 Error)
{
	// Wait out the required status packet return delay time (in units of 2us)
	uDelay(((u32) GB_RETURN_DELAY_TIME) << 1);

	// Return the required packet
	ReturnErrorStatus(Error);
}

// Broadcast message(s) on the dynamixel bus to initialise the configuration of the servos
void DXLServoConfig(void)
{
	// Note: This function does not clear the DXL Tx buffer, and so these packets will only
	//       get sent after whatever else is already in the buffer has already been sent.

	// Constants
	const u16 torque = SERVO_MAX_TORQUE;

	// Construct a packet to configure the return delay time
	DPtmp.ID          = BROADCASTING_ID;     // Broadcast to all devices on the bus
	DPtmp.Instruction = INST_WRITE;          // Write instruction
	DPtmp.NumParams   = 2;                   // Number of parameters
	DPtmp.Param[0]    = 0x05;                // Address of the return delay time register
	DPtmp.Param[1]    = 0x00;                // RETURN_DELAY_TIME = 0x00 => 0us delay for status packet

	// Send the constructed packet
	TxDDataDP(USART_DXL, &DPtmp);

	// Construct a packet to configure the status return level and alarm registers
	DPtmp.ID          = BROADCASTING_ID;     // Broadcast to all devices on the bus
	DPtmp.Instruction = INST_WRITE;          // Write instruction
	DPtmp.NumParams   = 4;                   // Number of parameters
	DPtmp.Param[0]    = 0x10;                // Address of the status return level register
	DPtmp.Param[1]    = RETURN_READ_PACKET;  // STATUS_RETURN_LEVEL = 0x01 => Return status packet for READ and PING instructions only
	DPtmp.Param[2]    = DEFAULT_ALARM_STATE; // ALARM_LED = 0x24 => Overheating and overload errors cause the servo LED to turn on
	DPtmp.Param[3]    = DEFAULT_ALARM_STATE; // ALARM_SHUTDOWN = 0x24 => Overheating and overload errors cause the servo to shut down

	// Send the constructed packet
	TxDDataDP(USART_DXL, &DPtmp);

	// Construct a packet to configure the max torque
	DPtmp.ID          = BROADCASTING_ID;     // Broadcast to all devices on the bus
	DPtmp.Instruction = INST_WRITE;          // Write instruction
	DPtmp.NumParams   = 3;                   // Number of parameters
	DPtmp.Param[0]    = 0x0E;                // Address of the max torque register
	DPtmp.Param[1]    = LOW_BYTE(torque);    // MAX_TORQUE = 0x03FF => Use 100% of the maximum available torque if required
	DPtmp.Param[2]    = HIGH_BYTE(torque);   // MAX_TORQUE = 0x03FF => Use 100% of the maximum available torque if required

	// Send the constructed packet
	TxDDataDP(USART_DXL, &DPtmp);

	// Construct a packet to configure the torque limit (Note however that torque limit resets to the value of max torque on servo power on)
	DPtmp.ID          = BROADCASTING_ID;     // Broadcast to all devices on the bus
	DPtmp.Instruction = INST_WRITE;          // Write instruction
	DPtmp.NumParams   = 3;                   // Number of parameters
	DPtmp.Param[0]    = 0x22;                // Address of the torque limit register
	DPtmp.Param[1]    = LOW_BYTE(torque);    // TORQUE_LIMIT = 0x03FF => Use 100% of the maximum available torque if required
	DPtmp.Param[2]    = HIGH_BYTE(torque);   // TORQUE_LIMIT = 0x03FF => Use 100% of the maximum available torque if required

	// Send the constructed packet
	TxDDataDP(USART_DXL, &DPtmp);

	// Construct a packet to configure the P gain (Note however that the P gain loses its value on servo power off)
	DPtmp.ID          = BROADCASTING_ID;     // Broadcast to all devices on the bus
	DPtmp.Instruction = INST_WRITE;          // Write instruction
	DPtmp.NumParams   = 2;                   // Number of parameters
	DPtmp.Param[0]    = 0x1C;                // Address of the P gain register
	DPtmp.Param[1]    = 16;                  // P_GAIN = 16 => Set internal servo position tracking P gain to 16

	// Send the constructed packet
	TxDDataDP(USART_DXL, &DPtmp);

	// Wait for all the packets to go through
	WaitForTxDData(USART_DXL);
}

// Broadcast a message on the dynamixel bus to disable the torque of every servo
void DXLServoTorqueOff(void)
{
	// Note: This function clears the DXL Tx buffer, and so should be used with caution as it will cause enqueued packets to be lost.

	// Construct the required broadcast packet to turn all the DXL torques off
	DPtmp.ID          = BROADCASTING_ID; // Broadcast to all devices on the bus
	DPtmp.Instruction = INST_WRITE;      // Write instruction
	DPtmp.NumParams   = 2;               // Number of parameters
	DPtmp.Param[0]    = 0x18;            // Address of the torque enable register
	DPtmp.Param[1]    = 0x00;            // Value to write = 0x00 => OFF (as opposed to 0x01 => ON)

	// Ensure the packet is the very next packet in the Tx packet buffer
	USARTClearBuffers(USART_DXL, USART_CLEAR_TX);

	// Send the torque off packet and wait for it to go through
	TxDDataDP(USART_DXL, &DPtmp);
	WaitForTxDData(USART_DXL);
}

// Initialise the control table
void InitControlTable(void)
{
	// Declare variables
	u8 bCount;

	// Initialise the values in the ROM section of the control table
	for(bCount = 0; bCount < ROM_CONTROL_TABLE_LEN; bCount++)
		GB_CONTROL_TABLE(bCount) = ROM_INITIAL_DATA[bCount];

	// Initialise the values in the RAM section of the control table
	for(bCount = ROM_CONTROL_TABLE_LEN; bCount < CONTROL_TABLE_LEN; bCount++)
	{
		if(gbCTParamRange[bCount][LOW_LIMIT] > gbCTParamRange[bCount][HIGH_LIMIT])
			GB_CONTROL_TABLE(bCount) = 0;
		else
			GB_CONTROL_TABLE(bCount) = gbCTParamRange[bCount][LOW_LIMIT];
	}
}
/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
