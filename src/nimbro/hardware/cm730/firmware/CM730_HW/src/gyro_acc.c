/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : gyro_acc.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 2011/01/15
* Description        : Contains the functions and defines for the gyro/acc sensors
* Comment            : This file has been heavily modified by Philipp Allgeuer
*                      <pallgeuer@ais.uni-bonn.de> for the NimbRo-OP (02/04/14),
*                      and is intended for use with CM730's that have the L3G4200D
*                      gyroscope chip and the LIS331DLH accelerometer chip.
*******************************************************************************/

// Includes
#include "gyro_acc.h"
#include "stm32f10x_lib.h"
#include "system_init.h"
#include "CM_DXL_COM.h"

// Defines
#define GYRO_CTRL_REG1  0x20
#define GYRO_CTRL_REG2  0x21
#define GYRO_CTRL_REG3  0x22
#define GYRO_CTRL_REG4  0x23
#define GYRO_CTRL_REG5  0x24
#define ACC_CTRL_REG1   0x20
#define ACC_CTRL_REG2   0x21
#define ACC_CTRL_REG3   0x22
#define ACC_CTRL_REG4   0x23
#define ACC_CTRL_REG5   0x24
#define MAX_AVG_POINTS  5

// SPI data Tx buffers
const u8 SPI_GyroTxBuf[3][3] = {{0xC0|0x28, 0xFF, 0xFF},  // Gyroscope: Bytes to send in order to read the registers 0x28, 0x2A and 0x2C (OUT_X, OUT_Y, OUT_Z)
                                {0xC0|0x2A, 0xFF, 0xFF},
                                {0xC0|0x2C, 0xFF, 0xFF}};
const u8 SPI_AccTxBuf[3][3]  = {{0xC0|0x28, 0xFF, 0xFF},  // Accelerometer: Bytes to send in order to read the registers 0x28, 0x2A and 0x2C (OUT_X, OUT_Y, OUT_Z)
                                {0xC0|0x2A, 0xFF, 0xFF},
                                {0xC0|0x2C, 0xFF, 0xFF}};

// SPI data Rx buffers
vu8 SPI_GyroRxBuf[3][3]; // Buffer to store the bytes read from the gyroscope
vu8 SPI_AccRxBuf[3][3];  // Buffer to store the bytes read from the accelerometer

// Data history buffer
vs16 GyroAccAvgBuf[MAX_AVG_POINTS][6] = {{0}}; // Stores a history of GyroX, GyroY, GyroZ, AccX, AccY, AccZ
vu8  GyroAccAvgBufFull = 0;                    // Flag specifying whether the history buffer is full
vu8  GyroAccAvgBufPtr = 0;                     // Pointer of where to write to next in the history buffer
vu8  GyroAccAvgBufClear = 1;                   // A flag that can be used to clear the history buffer

// Convert the received SPI Rx bytes into the required gyroscope and accelerometer readings and place them in the control table
void ProcessData()
{
	// Declare variables
	s16 RawGyroX, RawGyroY, RawGyroZ, RawAccX, RawAccY, RawAccZ;
	s32 GyroX, GyroY, GyroZ, AccX, AccY, AccZ;
	u8 i, num;

	// Note: The axes of the L3G4200D gyroscope chip are aligned such that on the NimbRo-OP:
	//       x is towards the back, y is towards the robot's right, z is upwards

	// Retrieve the raw gyroscope data from the corresponding SPI Rx buffer (and adjust for the chip's coordinate system indicated above)
	RawGyroX = -(((s16) SPI_GyroRxBuf[0][2] << 8) | SPI_GyroRxBuf[0][1]); // X(robot) = -X(chip)
	RawGyroY = -(((s16) SPI_GyroRxBuf[1][2] << 8) | SPI_GyroRxBuf[1][1]); // Y(robot) = -Y(chip)
	RawGyroZ =  (((s16) SPI_GyroRxBuf[2][2] << 8) | SPI_GyroRxBuf[2][1]); // Z(robot) =  Z(chip)

	// Note: The axes of the LIS331DLH accelerometer chip are aligned such that on the NimbRo-OP:
	//       x is towards the robot's left, y is towards the back, z is upwards

	// Retrieve the raw accelerometer data from the corresponding SPI Rx buffer (and adjust for the chip's coordinate system indicated above)
	RawAccX = -(((s16) SPI_AccRxBuf[1][2] << 8) | SPI_AccRxBuf[1][1]); // X(robot) = -Y(chip)
	RawAccY =  (((s16) SPI_AccRxBuf[0][2] << 8) | SPI_AccRxBuf[0][1]); // Y(robot) =  X(chip)
	RawAccZ =  (((s16) SPI_AccRxBuf[2][2] << 8) | SPI_AccRxBuf[2][1]); // Z(robot) =  Z(chip)

	// Check whether we need to clear the buffer
	if(GyroAccAvgBufClear != 0)
	{
		GyroAccAvgBufFull = 0;
		GyroAccAvgBufPtr = 0;
		GyroAccAvgBufClear = 0;
	}

	// Write the new values into the history buffer
	GyroAccAvgBuf[GyroAccAvgBufPtr][0] = RawGyroX;
	GyroAccAvgBuf[GyroAccAvgBufPtr][1] = RawGyroY;
	GyroAccAvgBuf[GyroAccAvgBufPtr][2] = RawGyroZ;
	GyroAccAvgBuf[GyroAccAvgBufPtr][3] = RawAccX;
	GyroAccAvgBuf[GyroAccAvgBufPtr][4] = RawAccY;
	GyroAccAvgBuf[GyroAccAvgBufPtr][5] = RawAccZ;
	GyroAccAvgBufPtr++;
	if(GyroAccAvgBufPtr == MAX_AVG_POINTS)
	{
		GyroAccAvgBufFull = 1;
		GyroAccAvgBufPtr = 0;
	}

	// Calculate the average of the values in the history buffer
	GyroX = GyroY = GyroZ = AccX = AccY = AccZ = 0;
	num = (GyroAccAvgBufFull == 1 ? MAX_AVG_POINTS : GyroAccAvgBufPtr);
	for(i = 0; i < num; i++)
	{
		GyroX += GyroAccAvgBuf[i][0];
		GyroY += GyroAccAvgBuf[i][1];
		GyroZ += GyroAccAvgBuf[i][2];
		AccX  += GyroAccAvgBuf[i][3];
		AccY  += GyroAccAvgBuf[i][4];
		AccZ  += GyroAccAvgBuf[i][5];
	}
	GyroX /= num;
	GyroY /= num;
	GyroZ /= num;
	AccX  /= num;
	AccY  /= num;
	AccZ  /= num;

	// Place the gyroscope values into the control table
	GW_GYRO_X = (s16) GyroX;
	GW_GYRO_Y = (s16) GyroY;
	GW_GYRO_Z = (s16) GyroZ;
	
	// Place the accelerometer values into the control table
	GW_ACC_X = (s16) AccX;
	GW_ACC_Y = (s16) AccY;
	GW_ACC_Z = (s16) AccZ;
}

// Write to a register of the L3G4200D gyroscope chip
void WriteGyroRegister(u16 address, u16 value)
{
	// Activate the gyroscope chip select line
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_GYRO_CS, PIN_SIG_GYRO_CS);

	// Send the register address to write to
	SPI_I2S_SendData(SPI2, address);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(SPI2);

	// Send the value to write to the register
	SPI_I2S_SendData(SPI2, value);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(SPI2);

	// Deactivate the gyroscope chip select line
	GPIO_SetBits(PORT_SIG_GYRO_CS, PIN_SIG_GYRO_CS);
}

// Configure the L3G4200D gyroscope chip
void ConfigureGyro()
{
	// Configure CTRL_REG1:
	// DR1-DR0 = 11 => Output data rate of 800Hz
	// BW1-BW0 = 11 => LPF2 cut-off frequency of 110Hz (in addition to the fixed 93Hz LPF1 / not relevant unless OUT_SEL1 = 1)
	// PD      = 1  => Normal mode
	// Zen     = 1  => Enable z-axis
	// Yen     = 1  => Enable y-axis
	// Xen     = 1  => Enable x-axis
	WriteGyroRegister(GYRO_CTRL_REG1, 0xFF);

	// Configure CTRL_REG4:
	// BDU     = 1  => Enable block data update mode
	// BLE     = 0  => Little endian (data LSB at lower address)
	// FS1-FS0 = 00 => Full scale range of +-250dps (corresponding to an s16 data range of -32768 to 32767)
	// ST1-ST0 = 00 => Disable self test
	// SIM     = 0  => SPI 4-wire interface
	WriteGyroRegister(GYRO_CTRL_REG4, 0x80);

	// Configure CTRL_REG5:
	// BOOT                = 0  => Normal mode (do NOT reboot memory content)
	// FIFO_EN             = 0  => Disable FIFO
	// HPEN                = 0  => Disable high pass filter
	// INT1_SEL1-INT1_SEL0 = 00 => Interrupt generation based on LPF1 output
	// OUT_SEL1-OUT_SEL0   = 00 => Output data based on LPF1 output
	WriteGyroRegister(GYRO_CTRL_REG5, 0x00);
}

// Write to a register of the LIS331DLH accelerometer chip
void WriteAccRegister(u16 address, u16 value)
{
	// Activate the accelerometer chip select line
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_ACC_CS, PIN_SIG_ACC_CS);

	// Send the register address to write to
	SPI_I2S_SendData(SPI2, address);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(SPI2);

	// Send the value to write to the register
	SPI_I2S_SendData(SPI2, value);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(SPI2);

	// Deactivate the accelerometer chip select line
	GPIO_SetBits(PORT_SIG_ACC_CS, PIN_SIG_ACC_CS);
}

// Configure the LIS331DLH accelerometer chip
void ConfigureAcc()
{
	// Configure CTRL_REG1:
	// PM2-PM0 = 001 => Normal power mode and data rate
	// DR1-DR0 = 01  => Output data rate of 100Hz, analog LPF cut-off frequency of 74Hz
	// Zen     = 1   => Enable z-axis
	// Yen     = 1   => Enable y-axis
	// Xen     = 1   => Enable x-axis
	WriteAccRegister(ACC_CTRL_REG1, 0x2F);

	// Configure CTRL_REG2:
	// BOOT        = 0  => Normal mode (do NOT reboot memory content)
	// HPM1-HPM0   = 00 => Disable high pass filter
	// FDS         = 0  => Bypass internal filter in filtered data selection
	// HPEN2       = 0  => HPF disabled for interrupt 2 source
	// HPEN1       = 0  => HPF disabled for interrupt 1 source
	// HPCF1-HPCF0 = 00 => HPF cut-off frequency configuration
	WriteAccRegister(ACC_CTRL_REG2, 0x00);

	// Configure CTRL_REG4:
	// BDU     = 1  => Enable block data update mode
	// BLE     = 0  => Little endian (data LSB at lower address)
	// FS1-FS0 = 01 => Full scale range of +-4g (corresponding to an s16 data range of -32768 to 32767)
	// STsign  = 0  => Positive self test sign
	// ST      = 0  => Disable selt test
	// SIM     = 0  => SPI 4-wire interface
	WriteAccRegister(ACC_CTRL_REG4, 0x90);
}

// Interrupt service routine to read the gyroscope and accelerometer data
void __ISR_GYRO_ACC_SPI() // Called at 259.3Hz (see isr.c)
{
	// Declare variables
	int i, j;

	// Read the gyroscope data
	for(i = 0;i < 3;i++)
	{
		// Activate the gyroscope chip select line
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		GPIO_ResetBits(PORT_SIG_GYRO_CS, PIN_SIG_GYRO_CS);

		// Send and receive the bytes required to read out one of the gyro values (i.e. one of OUT_X, OUT_Y, OUT_Z)
		for(j = 0;j < 3;j++)
		{
			SPI_I2S_SendData(SPI2, SPI_GyroTxBuf[i][j]);
			while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
			while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
			SPI_GyroRxBuf[i][j] = (u8) SPI_I2S_ReceiveData(SPI2);
		}

		// Deactivate the gyroscope chip select line
		GPIO_SetBits(PORT_SIG_GYRO_CS, PIN_SIG_GYRO_CS);
	}

	// Read the accelerometer data
	for(i = 0;i < 3;i++)
	{
		// Activate the accelerometer chip select line
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		GPIO_ResetBits(PORT_SIG_ACC_CS, PIN_SIG_ACC_CS);

		// Send and receive the bytes required to read out one of the gyro values (i.e. one of OUT_X, OUT_Y, OUT_Z)
		for(j = 0;j < 3;j++)
		{
			SPI_I2S_SendData(SPI2, SPI_AccTxBuf[i][j]);
			while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
			while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
			SPI_AccRxBuf[i][j] = (u8) SPI_I2S_ReceiveData(SPI2);
		}

		// Deactivate the accelerometer chip select line
		GPIO_SetBits(PORT_SIG_ACC_CS, PIN_SIG_ACC_CS);
	}

	// Process the received data
	ProcessData();
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
