// Hardware interface for the CM730 board
// File:    CM730.h
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schueller <schuell1@cs.uni-bonn.de>
// Comment: This file is suitable for CM730 firmware versions of 0x82 and above (NimbRo-OP specific).

// Ensure header is included only once
#ifndef CM730_H
#define CM730_H

// Includes
#include <cm730/read_thread.h>
#include <cm730/dynamixel.h>
#include <config_server/parameter.h>
#include <plot_msgs/plot_manager.h>
#include <ros/time.h>
#include <sys/time.h>
#include <stddef.h>
#include <stdint.h>
#include <vector>

// Servo bulk read response struct
struct BRData
{
	// Constructor
	BRData() { std::memset(this, 0, sizeof(BRData)); position = -1; }
	
	// Bulk read parameters
	int id;
	int length;
	int startAddress;

	// Received data
	int position; // P_PRESENT_POSITION_*
};

// CM730 bulk read response struct
struct BRBoard
{
	// Constructor
	BRBoard() { std::memset(this, 0, sizeof(BRBoard)); }
	
	// Bulk read parameters
	int id;
	int length;
	int startAddress;

	// Received data
	unsigned char power;    // P_DYNAMIXEL_POWER
	unsigned char ledPanel; // P_LED_PANEL
	unsigned short rgbled5; // P_RGBLED5_*
	unsigned short rgbled6; // P_RGBLED6_*
	unsigned char button;   // P_BUTTON
	unsigned char voltage;  // P_BATTERY_VOLTAGE
	int gyroX;              // P_GYRO_X_*
	int gyroY;              // P_GYRO_Y_*
	int gyroZ;              // P_GYRO_Z_*
	int accX;               // P_ACC_X_*
	int accY;               // P_ACC_Y_*
	int accZ;               // P_ACC_Z_*
	int magX;               // P_MAG_X_*
	int magY;               // P_MAG_Y_*
	int magZ;               // P_MAG_Z_*
};

//
// CM730 class
//
class CM730
{
public:
	// Constructor/destructor
	CM730();
	virtual ~CM730();

	// Connection functions
	int connect();
	int ping(int id, struct timespec* abstime = NULL);
	void suspend(double suspendTime);
	void unsuspend();

	// Send instruction function
	int sendInstruction(int id, int inst);
	
	// Read functions
	int readByte(int id, int address, int* value, struct timespec* abstime = NULL);
	int readWord(int id, int address, int* value, struct timespec* abstime = NULL);
	int readData(int id, int address, void* data, size_t size, struct timespec* abstime = NULL);
	
	// Bulk read functions
	int readCM730(BRBoard* cm730Data);
	int bulkRead(std::vector<BRData>* servoData, BRBoard* cm730Data);
	int updateTxBRPacket(const std::vector<int>& servos);
	bool gotCM730Data() const { return m_gotCM730Data; } // Reset at the beginning of every bulk read

	// Write functions
	int writeByte(int id, int address, int value);
	int writeWord(int id, int address, int value);
	int writeData(int id, int address, void* data, size_t size);
	int syncWrite(int address, size_t numDataBytes, size_t numDevices, const uint8_t* data);
	int setDynamixelPower(int value);

	// Get functions
	inline bool isSuspended() { if(m_isSuspended && !m_unsuspendTime.isZero() && (ros::Time::now() >= m_unsuspendTime)) { unsuspend(); } return m_isSuspended; }
	inline int lastFailedID() const { return m_lastFailedID; }
	inline int servoFailCount(int id) const { return ((id >= 0) && (id < MAX_DEVICES) ? m_failCount[id] : -1); }

private:
	// Constants
	static const char* PATH;              // Path to the CM730 device
	static const int BAUDRATE = 1000000;  // 1 Mbps
	static const int MAX_TX_BYTES = 256;  // Tx buffer size of 256 bytes
	static const int MAX_RX_BYTES = 1024; // Rx buffer size of 1024 bytes
	static const int MAX_DEVICES = 256;   // Maximum number of devices on the bus (importantly, no device ID can exceed this value)

	// Allow overriding of a current suspension
	inline void pauseSuspend() { m_wasSuspended = m_isSuspended; m_isSuspended = false; }
	inline void resumeSuspend() { m_isSuspended = m_wasSuspended; }
	
	// Communications functions
	int txPacket(unsigned char* txp);
	int rxPacket(unsigned char* rxp, int size, struct timespec* abstime = NULL);
	void flushPort();

	// Packet helper functions
	bool parseBRPacket(unsigned char* rxp, int size, std::vector<BRData>* servoData, BRBoard* cm730Data);
	bool parseCM730Data(unsigned char* data, int size, BRBoard* cm730Data);
	int syncRxPacket(unsigned char* rxp, int* readSize);
	unsigned char checksum(unsigned char* packet);

	// Plotting
	plot_msgs::PlotManagerFS m_PM;

	// Config server parameters
	config_server::Parameter<bool> m_useBulkReadShortcut; // Parameter specifying whether to make use of the repeat last bulk read command or not (Note: If direct PC->DXL byte feedthrough is enabled in the CM730 firmware, then this should be FALSE, else TRUE, and that's important!)

	// Variables
	int m_fd;
	int m_lastFailedID;
	io::ReadThread m_readThread;
	pthread_t m_readThread_thread;
	unsigned char m_TxBulkRead[MAX_TX_BYTES];
	bool m_fullBRPacket;
	ros::Time m_lastFullBRPacket;
	int m_failCount[MAX_DEVICES];
	bool m_isSuspended;
	bool m_wasSuspended;
	ros::Time m_unsuspendTime;
	bool m_gotCM730Data;
	int m_lastSeenDynPow;

public:
	// Dynamixel communication success enumeration
	enum
	{
		RET_BAD_PARAM  = -1,
		RET_SUCCESS    = 0,
		RET_TX_CORRUPT = 1,
		RET_TX_FAIL    = 2,
		RET_RX_FAIL    = 3,
		RET_RX_TIMEOUT = 4,
		RET_RX_CORRUPT = 5,
		RET_MISC_FAIL  = 6
	};

	// Dynamixel packet byte positions
	enum
	{
		DP_STARTFF1    = 0,
		DP_STARTFF2    = 1,
		DP_ID          = 2,
		DP_LENGTH      = 3,
		DP_INSTRUCTION = 4,
		DP_ERRFLAGS    = 4,
		DP_PARAMETER   = 5
	};

	// Dynamixel packet IDs (DP_ID byte)
	enum
	{
		ID_NONE      = 0,
		ID_CM730     = 200,
		ID_BROADCAST = 254
	};

	// Dynamixel packet instructions (DP_INSTRUCTION byte)
	enum
	{
		INST_NONE           = 0x00,
		INST_PING           = 0x01,
		INST_READ           = 0x02,
		INST_WRITE          = 0x03,
		INST_REG_WRITE      = 0x04,
		INST_ACTION         = 0x05,
		INST_RESET          = 0x06,
		INST_DIGITAL_RESET  = 0x07,
		INST_SYSTEM_READ    = 0x0C,
		INST_SYSTEM_WRITE   = 0x0D,
		INST_REPEAT_BULK    = 0x0F,
		INST_SYNC_WRITE     = 0x83,
		INST_SYNC_REG_WRITE = 0x84,
		INST_BULK_READ      = 0x92
	};

	// Dynamixel packet error flags (DP_ERRFLAGS byte)
	enum
	{
		ERRBIT_VOLTAGE     = 0x01,
		ERRBIT_ANGLE_LIMIT = 0x02,
		ERRBIT_OVERHEATING = 0x04,
		ERRBIT_RANGE       = 0x08,
		ERRBIT_CHECKSUM    = 0x10,
		ERRBIT_OVERLOAD    = 0x20,
		ERRBIT_INSTRUCTION = 0x40
	};

	// CM730 register map (should match up with the defines in CM_DXL_COM.h, part of the CM730 firmware version 0x81 and above)
	enum
	{
		P_MODEL_NUMBER_L       = 0,
		P_MODEL_NUMBER_H       = 1,
		P_VERSION              = 2,
		P_ID                   = 3,
		P_BAUD_RATE            = 4,
		P_RETURN_DELAY_TIME    = 5,
		P_VOLTAGE_LOWER_LIMIT  = 12,
		P_VOLTAGE_UPPER_LIMIT  = 13,
		P_RETURN_LEVEL         = 16,
		P_ALARM_LED            = 17,
		P_DYNAMIXEL_POWER      = 24,
		P_LED_PANEL            = 25,
		P_RGBLED5_L            = 26,
		P_RGBLED5_H            = 27,
		P_RGBLED6_L            = 28,
		P_RGBLED6_H            = 29,
		P_BUTTON               = 30,
		P_BATTERY_VOLTAGE      = 31,
		P_GYRO_X_L             = 32,
		P_GYRO_X_H             = 33,
		P_GYRO_Y_L             = 34,
		P_GYRO_Y_H             = 35,
		P_GYRO_Z_L             = 36,
		P_GYRO_Z_H             = 37,
		P_ACC_X_L              = 38,
		P_ACC_X_H              = 39,
		P_ACC_Y_L              = 40,
		P_ACC_Y_H              = 41,
		P_ACC_Z_L              = 42,
		P_ACC_Z_H              = 43,
		P_MAG_X_L              = 44,
		P_MAG_X_H              = 45,
		P_MAG_Y_L              = 46,
		P_MAG_Y_H              = 47,
		P_MAG_Z_L              = 48,
		P_MAG_Z_H              = 49,
		P_ADC0_BATTERY_L       = 50,
		P_ADC0_BATTERY_H       = 51,
		P_ADC1_MIC1_L          = 52,
		P_ADC1_MIC1_H          = 53,
		P_ADC2_MIC2_L          = 54,
		P_ADC2_MIC2_H          = 55,
		P_ADC3_L               = 56,
		P_ADC3_H               = 57,
		P_ADC4_L               = 58,
		P_ADC4_H               = 59,
		P_ADC5_L               = 60,
		P_ADC5_H               = 61,
		P_ADC6_L               = 62,
		P_ADC6_H               = 63,
		P_ADC7_L               = 64,
		P_ADC7_H               = 65,
		P_ADC8_L               = 66,
		P_ADC8_H               = 67,
		P_ADC9_L               = 68,
		P_ADC9_H               = 69,
		P_ADC10_L              = 70,
		P_ADC10_H              = 71,
		P_ADC11_L              = 72,
		P_ADC11_H              = 73,
		P_ADC12_L              = 74,
		P_ADC12_H              = 75,
		P_ADC13_L              = 76,
		P_ADC13_H              = 77,
		P_ADC14_L              = 78,
		P_ADC14_H              = 79,
		P_ADC15_L              = 80,
		P_ADC15_H              = 81,
		P_BUZZER_DATA          = 82,
		P_BUZZER_PLAY_LENGTH   = 83,
		P_DXLRX_PACKET_CNT_L   = 91,
		P_DXLRX_PACKET_CNT_H   = 92,
		P_DXLRX_OVERFLOW_CNT_L = 93,
		P_DXLRX_OVERFLOW_CNT_H = 94,
		P_DXLRX_BUFERROR_CNT_L = 95,
		P_DXLRX_BUFERROR_CNT_H = 96,
		P_DXLRX_CHKERROR_CNT_L = 97,
		P_DXLRX_CHKERROR_CNT_H = 98,
		P_DXLRX_ORE_CNT_L      = 99,
		P_DXLRX_ORE_CNT_H      = 100,
		P_DXLTX_PACKET_CNT_L   = 101,
		P_DXLTX_PACKET_CNT_H   = 102,
		P_DXLTX_OVERFLOW_CNT_L = 103,
		P_DXLTX_OVERFLOW_CNT_H = 104,
		P_DXLTX_BUFERROR_CNT_L = 105,
		P_DXLTX_BUFERROR_CNT_H = 106,
		P_PCRX_PACKET_CNT_L    = 107,
		P_PCRX_PACKET_CNT_H    = 108,
		P_PCRX_OVERFLOW_CNT_L  = 109,
		P_PCRX_OVERFLOW_CNT_H  = 110,
		P_PCRX_BUFERROR_CNT_L  = 111,
		P_PCRX_BUFERROR_CNT_H  = 112,
		P_PCRX_CHKERROR_CNT_L  = 113,
		P_PCRX_CHKERROR_CNT_H  = 114,
		P_PCRX_ORE_CNT_L       = 115,
		P_PCRX_ORE_CNT_H       = 116,
		P_PCTX_PACKET_CNT_L    = 117,
		P_PCTX_PACKET_CNT_H    = 118,
		P_PCTX_OVERFLOW_CNT_L  = 119,
		P_PCTX_OVERFLOW_CNT_H  = 120,
		P_PCTX_BUFERROR_CNT_L  = 121,
		P_PCTX_BUFERROR_CNT_H  = 122,
		P_MISC0                = 123,
		P_MISC1                = 124,
		P_MISC2                = 125,
		P_MISC3                = 126,
		CM730_REGISTER_NUM
	};

	// Bulk read addresses
	enum
	{
		READ_CM730_ADDRESS = CM730::P_DYNAMIXEL_POWER, // CM730: Read the address range P_DYNAMIXEL_POWER (24) --> P_MAG_Z_H (49) = 26 bytes
		READ_CM730_LENGTH = CM730::P_MAG_Z_H - READ_CM730_ADDRESS + 1,                       
		READ_SERVO_ADDRESS = DynamixelMX::P_PRESENT_POSITION_L, // Read the address range P_PRESENT_POSITION_L (36) --> P_PRESENT_POSITION_H (37) = 2 bytes
		READ_SERVO_LENGTH = DynamixelMX::P_PRESENT_POSITION_H - READ_SERVO_ADDRESS + 1
	};
	
	// Dynamixel power states
	enum
	{
		DYNPOW_OFF = 0,
		DYNPOW_ON = 1,
		DYNPOW_ON_NODXLTX = 2
	};
};

#endif
// EOF