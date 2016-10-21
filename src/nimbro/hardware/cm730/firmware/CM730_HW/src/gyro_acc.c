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

// SPI data Rx buffers
vu8 SPI_GyroRxBuf[9] = {0}; // Buffer to store the bytes read from the gyroscope
vu8 SPI_AccRxBuf[7] = {0};  // Buffer to store the bytes read from the accelerometer

// Data history buffer
vs16 GyroAccAvgBuf[MAX_AVG_POINTS][6] = {{0}}; // Stores a history of GyroX, GyroY, GyroZ, AccX, AccY, AccZ
vu8  GyroAccAvgBufFull = 0;                    // Flag specifying whether the history buffer is full
vu8  GyroAccAvgBufPtr = 0;                     // Pointer of where to write to next in the history buffer
vu8  GyroAccAvgBufClear = 1;                   // A flag that can be used to clear the history buffer

// Temperature variables
vu8 gbTempCounter = 0;

// Convert the received SPI Rx bytes into the required gyroscope and accelerometer readings and place them in the control table
void ProcessData()
{
	// Declare variables
	s16 RawGyroX, RawGyroY, RawGyroZ, RawAccX, RawAccY, RawAccZ;
	s32 GyroX, GyroY, GyroZ, AccX, AccY, AccZ;
	s8 Temperature;
	u8 i, num;

	// Note: The axes of the L3G4200D gyroscope chip are aligned such that on the NimbRo-OP:
	//       x is towards the back, y is towards the robot's right, z is upwards

	// Retrieve the raw gyroscope data from the corresponding SPI Rx buffer (and adjust for the chip's coordinate system indicated above)
	RawGyroX = -(((s16) SPI_GyroRxBuf[4] << 8) | SPI_GyroRxBuf[3]); // X(robot) = -X(chip)
	RawGyroY = -(((s16) SPI_GyroRxBuf[6] << 8) | SPI_GyroRxBuf[5]); // Y(robot) = -Y(chip)
	RawGyroZ =  (((s16) SPI_GyroRxBuf[8] << 8) | SPI_GyroRxBuf[7]); // Z(robot) =  Z(chip)

	// Retrieve the temperature inside the gyroscope chip
	Temperature = 50 - ((s8) SPI_GyroRxBuf[1]); // Note: The 50 here is only for convenience, the temperature is only a relative reading, not an absolute one!

	// Note: The axes of the LIS331DLH accelerometer chip are aligned such that on the NimbRo-OP:
	//       x is towards the robot's left, y is towards the back, z is upwards

	// Retrieve the raw accelerometer data from the corresponding SPI Rx buffer (and adjust for the chip's coordinate system indicated above)
	RawAccX = -(((s16) SPI_AccRxBuf[4] << 8) | SPI_AccRxBuf[3]); // X(robot) = -Y(chip)
	RawAccY =  (((s16) SPI_AccRxBuf[2] << 8) | SPI_AccRxBuf[1]); // Y(robot) =  X(chip)
	RawAccZ =  (((s16) SPI_AccRxBuf[6] << 8) | SPI_AccRxBuf[5]); // Z(robot) =  Z(chip)

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

	// Place the gyro temperature value into the control table
	GB_TEMPERATURE = Temperature;
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
	// DR1-DR0 = 10 => Output data rate of 400Hz
	// BW1-BW0 = 11 => LPF2 cut-off frequency of 110Hz (in addition to the fixed 93Hz LPF1 / not relevant unless OUT_SEL1 = 1)
	// PD      = 1  => Normal mode
	// Zen     = 1  => Enable z-axis
	// Yen     = 1  => Enable y-axis
	// Xen     = 1  => Enable x-axis
	WriteGyroRegister(GYRO_CTRL_REG1, 0xBF);

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
	u8 i;

	//
	// Gyroscope
	//

	// Update the temperature counter (gbTempCounter == 0 at a rate of 2.0Hz)
	gbTempCounter++;
	if(gbTempCounter >= 130)
		gbTempCounter = 0;

	// Decide which register to start reading and how many bytes to read
	u8 gyroReg = 0x28;
	u8 gyroLen = 6;
	if(gbTempCounter == 0)
	{
		gyroReg = 0x26;
		gyroLen = 8;
	}
	u8 gyroOffset = gyroReg - 0x25;

	// Activate the gyroscope chip select line
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_GYRO_CS, PIN_SIG_GYRO_CS);

	// Write the address to start reading from (RW = 1 for read, MS = 1 for auto-increment => 0xC0)
	SPI_I2S_SendData(SPI2, 0xC0|gyroReg);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_GyroRxBuf[0] = (u8) SPI_I2S_ReceiveData(SPI2);

	// Read the required number of registers from the gyro chip (e.g. OUT_TEMP, STATUS_REG, OUT_X, OUT_Y, OUT_Z)
	for(i = 0; i < gyroLen; i++)
	{
		SPI_I2S_SendData(SPI2, 0xFF);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
		SPI_GyroRxBuf[gyroOffset + i] = (u8) SPI_I2S_ReceiveData(SPI2);
	}

	// Deactivate the gyroscope chip select line
	GPIO_SetBits(PORT_SIG_GYRO_CS, PIN_SIG_GYRO_CS);

	//
	// Accelerometer
	//

	// Activate the accelerometer chip select line
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_ACC_CS, PIN_SIG_ACC_CS);

	// Write the address to start reading from (RW = 1 for read, MS = 1 for auto-increment => 0xC0)
	SPI_I2S_SendData(SPI2, 0xC0|0x28);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_AccRxBuf[0] = (u8) SPI_I2S_ReceiveData(SPI2);

	// Read the required number of registers from the accelerometer chip (e.g. OUT_X, OUT_Y, OUT_Z)
	for(i = 0; i < 6; i++)
	{
		SPI_I2S_SendData(SPI2, 0xFF);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
		SPI_AccRxBuf[i + 1] = (u8) SPI_I2S_ReceiveData(SPI2);
	}

	// Deactivate the accelerometer chip select line
	GPIO_SetBits(PORT_SIG_ACC_CS, PIN_SIG_ACC_CS);

	//
	// Process data
	//

	// Process the received data
	ProcessData();
}
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
