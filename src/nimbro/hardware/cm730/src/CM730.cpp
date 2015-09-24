// Hardware interface for the CM730 board
// File:    CM730.cpp
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schueller <schuell1@cs.uni-bonn.de>
// Comment: This file is suitable for CM730 firmware versions of 0x81 and above (NimbRo-OP specific).

// Includes - Local
#include <cm730/CM730.h>

// Includes - System
#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sstream>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/poll.h>
#include <termios.h>

// Includes - ROS
#include <ros/console.h>

// Defines - Configuration
#define CM730_COMMS_DEBUG             0 // Non-zero => Display TX and RX packets
#define CM730_COMMS_DEBUG_TX          0 // Non-zero => Display TX packets
#define BULKREAD_DETAILED_TIMESTAMPS  0 // Non-zero => Display bulk read timing information
#define DISPLAY_NONFAIL_TIMES         0 // Non-zero => Display bulk read times if no failure as well
#define DISPLAY_BULKREAD_TIME         0 // Non-zero => Display a message with the bulk read times
#define DISPLAY_FAIL_COUNTS           0 // Non-zero => Display the local servo fail counts

// Defines
#define COMMS_READ_TIMEOUT  10000000LL  // Read timeout in nanoseconds (it's ok if this is longer than a single robotcontrol time step, but it shouldn't be too long so that the read in the next cycle has a chance without a cycle being missed)
#define FULL_BR_PERIOD      5.0         // Maximum time to send repeat bulk read instructions instead of the full bulk read Tx packet

// Constants
const char* CM730::PATH = "/dev/cm730";

// Helper functions
inline void getTimeLimit(struct timespec* out);
inline int makeWord(unsigned char* pos);
inline int makeWordSigned(unsigned char* pos);
void dump(const char* prefix, const uint8_t* data, uint8_t len);

//
// CM730 class
//

// Constructor
CM730::CM730()
 : m_PM(0, "~")
 , m_useBulkReadShortcut("nopInterface/useBulkReadShortcut", true)
 , m_fd(-1)
 , m_lastFailedID(0)
 , m_fullBRPacket(true)
 , m_failCount() // Initialise all fail counts to zero
 , m_isSuspended(false)
 , m_wasSuspended(false)
 , m_unsuspendTime(0, 0)
 , m_gotCM730Data(false)
 , m_lastSeenDynPow(DYNPOW_OFF)
{
	// Initialise the bulk read TX packet
	memset(&m_TxBulkRead, 0, sizeof(m_TxBulkRead));
	updateTxBRPacket(std::vector<int>());
}

// Destructor
CM730::~CM730()
{
}

//
// Connection functions
//

// Connect to the CM730
int CM730::connect()
{
	// Declare variables
	struct termios config;
	struct serial_struct serinfo;

	// Open a serial connection to the CM730
	if((m_fd = open(PATH, O_RDWR | O_NOCTTY)) < 0)
	{
		perror("[CM730] Could not open serial connection");
		return -1;
	}

	// Lock the serial connection
	if(lockf(m_fd, F_TLOCK, 0) != 0)
	{
		perror("[CM730] Could not acquire serial lock (is another robotcontrol running?)");
		return -1;
	}

	// Initialise the terminal interface struct
	memset(&config, 0, sizeof(config));
	config.c_cflag     = B38400|CS8|CLOCAL|CREAD;
	config.c_iflag     = IGNPAR;
	config.c_oflag     = 0;
	config.c_lflag     = 0;
	config.c_cc[VTIME] = 0;
	config.c_cc[VMIN]  = 1;

	// Set the required terminal attributes
	if((tcsetattr(m_fd, TCSANOW, &config)) < 0)
	{
		perror("[CM730] Could not set terminal attributes");
		close(m_fd);
		m_fd = -1;
		return -2;
	}

	// Retrieve the serial attributes
	if(ioctl(m_fd, TIOCGSERIAL, &serinfo) < 0)
	{
		perror("[CM730] Could not get serial attributes");
		close(m_fd);
		m_fd = -1;
		return -2;
	}

	// Modify the serial attributes as required
	serinfo.flags |= ASYNC_LOW_LATENCY;
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / BAUDRATE;

	// Update the serial attributes with our changes
	if(ioctl(m_fd, TIOCSSERIAL, &serinfo) < 0)
	{
		perror("[CM730] Could not set serial attributes");
		close(m_fd);
		m_fd = -1;
		return -2;
	}

	// Initiate the read thread
	m_readThread.setFile(m_fd);
	pthread_create(&m_readThread_thread, 0, &io::ReadThread::start, &m_readThread);

	// Flush the serial port
	flushPort();

	// Return that the connection was successful
	return 0;
}

// Send a ping command
int CM730::ping(int id, struct timespec* abstime)
{
	// We send the packet:  [0xFF] [0xFF] [ID] [0x02] [PING] [Checksum]
	// And hope to receive: [0xFF] [0xFF] [ID] [0x02] [Status] [Checksum]

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};
	unsigned char rxp[MAX_RX_BYTES] = {0};
	unsigned char idbyte;
	int rxError;

	// Check the passed ID
	idbyte = (unsigned char) id;
	if(idbyte >= ID_BROADCAST)
		return RET_BAD_PARAM;

	// Construct the packet to send
	txp[DP_ID] = idbyte;
	txp[DP_LENGTH] = 2;
	txp[DP_INSTRUCTION] = INST_PING;

	// Send the ping packet
	if((rxError = txPacket(txp)) != RET_SUCCESS)
		return rxError;

	// Wait to receive a return packet (expect 6 bytes)
	if((rxError = rxPacket(rxp, 6, abstime)) == RET_SUCCESS)
		return rxp[DP_ERRFLAGS];
	else
		return rxError;
}

// Suspend the CM730 communications for a given amount of time
void CM730::suspend(double suspendTime)
{
	// Disable bytes from reaching the dynamixels for the duration of the suspend (disables PC --> DXL forwarding, i.e. TX to the DXL from the CM730)
	if(m_lastSeenDynPow != DYNPOW_OFF) // Note: We need to be careful here that we don't enable dynamixel power if it's not supposed to be on!
		setDynamixelPower(DYNPOW_ON_NODXLTX);

	// Suspend the communications as required
	if(suspendTime >= 0.0)
		m_unsuspendTime = ros::Time::now() + ros::Duration(suspendTime); // Time-limited comms suspension...
	else
		m_unsuspendTime.fromNSec(0); // Open-ended comms suspension...
	m_isSuspended = true;
	m_wasSuspended = true;

	// Plot an event for the communications suspension
	std::ostringstream sstream;
	sstream.precision(3);
	sstream << "CM730 Susp " << std::fixed << suspendTime << "s";
	m_PM.clear();
	m_PM.plotEvent(sstream.str());
	m_PM.publish();
}

// Unsuspend the CM730 communications
void CM730::unsuspend()
{
	// Unsuspend the communications as required
	m_isSuspended = false;
	m_wasSuspended = false;
	
	// Allow bytes to reach the dynamixels again (enables PC --> DXL forwarding, i.e. TX to the DXL from the CM730)
	if(m_lastSeenDynPow != DYNPOW_OFF) // Note: We need to be careful here that we don't enable dynamixel power if it's not supposed to be on!
		setDynamixelPower(DYNPOW_ON);
}

//
// Send instruction function
//

// Send a particular instruction (with no parameters)
int CM730::sendInstruction(int id, int inst)
{
	// We send the packet: [0xFF] [0xFF] [ID] [0x02] [INST] [Checksum]

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};
	int ret = RET_SUCCESS;

	// Construct the packet to send
	txp[DP_ID]            = (unsigned char) id;
	txp[DP_LENGTH]        = 2;
	txp[DP_INSTRUCTION]   = (unsigned char) inst;

	// Send the read packet
	if((ret = txPacket(txp)) != RET_SUCCESS)
		return RET_TX_FAIL;

	// Return success
	return RET_SUCCESS;
}

//
// Read functions
//

// Read one register byte from a device
int CM730::readByte(int id, int address, int* value, struct timespec* abstime)
{
	// We send the packet:  [0xFF] [0xFF] [ID] [0x04] [READ] [Addr] [0x01] [Checksum]
	// And hope to receive: [0xFF] [0xFF] [ID] [0x03] [Status] [Data] [Checksum]

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};
	unsigned char rxp[MAX_RX_BYTES] = {0};
	int ret = RET_SUCCESS;

	// Construct the packet to send
	txp[DP_ID]            = (unsigned char) id;
	txp[DP_LENGTH]        = 4;
	txp[DP_INSTRUCTION]   = INST_READ;
	txp[DP_PARAMETER]     = (unsigned char) address;
	txp[DP_PARAMETER + 1] = 1;

	// Send the read packet
	if((ret = txPacket(txp)) != RET_SUCCESS)
		return ret;

	// Wait to receive a return packet (expect 7 bytes)
	if((ret = rxPacket(rxp, 7, abstime)) != RET_SUCCESS)
		return ret;

	// Return the read byte
	*value = (int) rxp[DP_PARAMETER];

	// Return success
	return RET_SUCCESS;
}

// Read one register word from a device
int CM730::readWord(int id, int address, int* value, struct timespec* abstime)
{
	// We send the packet:  [0xFF] [0xFF] [ID] [0x04] [READ] [Addr] [0x02] [Checksum]
	// And hope to receive: [0xFF] [0xFF] [ID] [0x04] [Status] [Data1] [Data2] [Checksum]

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};
	unsigned char rxp[MAX_RX_BYTES] = {0};
	int ret = RET_SUCCESS;

	// Construct the packet to send
	txp[DP_ID]            = (unsigned char) id;
	txp[DP_LENGTH]        = 4;
	txp[DP_INSTRUCTION]   = INST_READ;
	txp[DP_PARAMETER]     = (unsigned char) address;
	txp[DP_PARAMETER + 1] = 2;

	// Send the read packet
	if((ret = txPacket(txp)) != RET_SUCCESS)
		return ret;

	// Wait to receive a return packet (expect 8 bytes)
	if((ret = rxPacket(rxp, 8, abstime)) != RET_SUCCESS)
		return ret;

	// Return the read word
	*value = makeWord(&rxp[DP_PARAMETER]);

	// Return success
	return RET_SUCCESS;
}

// Read register data from a device
int CM730::readData(int id, int address, void* data, size_t size, struct timespec* abstime)
{
	// We send the packet:  [0xFF] [0xFF] [ID] [0x04] [READ] [Addr] [N] [Checksum]
	// And hope to receive: [0xFF] [0xFF] [ID] [N+2] [Status] [Data1] ... [DataN] [Checksum]

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};
	unsigned char rxp[MAX_RX_BYTES] = {0};
	int ret = RET_SUCCESS;

	// Sanity check on number of bytes to read
	if((int) size + 6 > MAX_RX_BYTES)
		return RET_BAD_PARAM;

	// Construct the packet to send
	txp[DP_ID]            = (unsigned char) id;
	txp[DP_LENGTH]        = 4;
	txp[DP_INSTRUCTION]   = INST_READ;
	txp[DP_PARAMETER]     = (unsigned char) address;
	txp[DP_PARAMETER + 1] = size;

	// Send the read packet
	if((ret = txPacket(txp)) != RET_SUCCESS)
		return ret;

	// Wait to receive a return packet (expect size + 6 bytes)
	if((ret = rxPacket(rxp, size + 6, abstime)) != RET_SUCCESS)
		return ret;

	// Return the read data
	memcpy(data, &rxp[DP_PARAMETER], size);

	// Return success
	return RET_SUCCESS;
}

//
// Bulk read functions
//

// Perform a read of the CM730 only
int CM730::readCM730(BRBoard* cm730Data)
{
	// Reset the last failed ID variable
	m_lastFailedID = 0;
	
	// Reset whether we received CM730 data
	m_gotCM730Data = false;
	
	// Start timing if desired
#if DISPLAY_BULKREAD_TIME
	ros::Time tstart = ros::Time::now();
#endif

	// Declare variables
	unsigned char rxp[MAX_RX_BYTES] = {0};

	// Calculate the desired absolute timeout time
	struct timespec timeout_time;
	getTimeLimit(&timeout_time);
	
	// Error checking
	if((int) m_TxBulkRead[DP_LENGTH] < 6) // Check that there are at least 3 bytes payload, which should be the CM730 read specification
		return RET_BAD_PARAM;
	
	// Retrieve the register range to read from the CM730 from the bulk read packet (Note: Assumes the CM730 is the first device in the bulk read packet!)
	int len = m_TxBulkRead[DP_PARAMETER + 1];
	int id = m_TxBulkRead[DP_PARAMETER + 2];
	int address = m_TxBulkRead[DP_PARAMETER + 3];
	
	// Ensure at this point that the retrieved register range is truly for the CM730
	if(id != ID_CM730 || len >= (int) sizeof(rxp))
		return RET_BAD_PARAM;
	
	// Save the requested data headers
	cm730Data->id = id;
	cm730Data->length = len;
	cm730Data->startAddress = address;
	
	// Read the required register range from the CM730 (
	pauseSuspend();
	int rxError = readData(id, address, rxp, len, &timeout_time);
	resumeSuspend();
	
	// Handle the case where the read is unsuccessful
	if(rxError != RET_SUCCESS)
	{
		m_lastFailedID = id;
		if((id >= 0) && (id < MAX_DEVICES))
		{
			m_failCount[id]++;
#if DISPLAY_FAIL_COUNTS
			ROS_ERROR("ID %d failed: Total %d times now", id, m_failCount[id]);
#endif
		}
		else
			ROS_ERROR("Invalid ID %d failed", id);
#if DISPLAY_BULKREAD_TIME
		ROS_WARN("Total CM730 read time: %lfs [FAIL]", (ros::Time::now() - tstart).toSec());
#endif
		return rxError;
	}
	
	// Display the CM730 read time
#if DISPLAY_BULKREAD_TIME
	ROS_WARN("Total CM730 read time: %lfs", (ros::Time::now() - tstart).toSec());
#endif
	
	// Parse the received read response packet
	if(!parseCM730Data(rxp, len, cm730Data))
		return RET_RX_CORRUPT;

	// Return success
	return RET_SUCCESS;
}

// Perform a bulk read
int CM730::bulkRead(std::vector<BRData>* servoData, BRBoard* cm730Data)
{
	// Reset the last failed ID variable
	m_lastFailedID = 0;
	
	// Reset whether we received CM730 data
	m_gotCM730Data = false;

	// Save the start ROS time of the call to this function
#if BULKREAD_DETAILED_TIMESTAMPS || DISPLAY_BULKREAD_TIME
	std::vector<ros::Time> ts;
	ts.push_back(ros::Time::now());
#endif

	// Declare variables
	unsigned char rxp[MAX_RX_BYTES] = {0};
	int i, ret = RET_SUCCESS;

	// Calculate the desired absolute timeout time
	struct timespec timeout_time;
	getTimeLimit(&timeout_time);

	// Error checking
	if((int) m_TxBulkRead[DP_LENGTH] + 4 > MAX_TX_BYTES)
		return RET_BAD_PARAM;

	// Send the bulk read Tx packet
	ros::Time preBulkTx = ros::Time::now();
	if((preBulkTx - m_lastFullBRPacket).toSec() > FULL_BR_PERIOD)
		m_fullBRPacket = true;
	if(m_fullBRPacket || !m_useBulkReadShortcut()) // Note: m_fullBRPacket is (i.e. certainly should be) set to true in all locations where m_TxBulkRead is modified
	{
		if((ret = txPacket(m_TxBulkRead)) != RET_SUCCESS)
			return ret;
		m_fullBRPacket = false;
		m_lastFullBRPacket = preBulkTx;
	}
	else
	{
		if((ret = sendInstruction(ID_CM730, INST_REPEAT_BULK)) != RET_SUCCESS)
			return ret;
	}

	// Save the current ROS time
#if BULKREAD_DETAILED_TIMESTAMPS
	ts.push_back(ros::Time::now());
#endif

	// Give the OS some time (the assumption is that after 3.5ms the servos won't be done responding yet, and the buffer inside the read thread won't be overflowing yet with responses)
	usleep(3700); // This is an approximate equal of the time it takes before the CM730/servo responses come streaming in...

	// Calculate the number of requests that were made in the bulk read (i.e. the number of queried devices)
	int numberOfRequests = (m_TxBulkRead[DP_LENGTH] - 3) / 3;

	// Save the bulk read request data headers (these are used when parsing the bulk read return packets)
	for(i = 0; i < 3*numberOfRequests; i += 3)
	{
		// Retrieve the requested address, length and ID of the next listed device
		int len  = m_TxBulkRead[DP_PARAMETER + i + 1];
		int id   = m_TxBulkRead[DP_PARAMETER + i + 2];
		int addr = m_TxBulkRead[DP_PARAMETER + i + 3];

		// Save the request data headers into the appropriate struct
		if(id == ID_CM730)
		{
			cm730Data->id = id;
			cm730Data->length = len;
			cm730Data->startAddress = addr;
		}
		else
		{
			BRData* servoInfo = &servoData->at(id - 1); // The servo data is stored by ID in a zero-based array, hence subtract 1 here to shift to zero-based
			servoInfo->id = id;
			servoInfo->length = len;
			servoInfo->startAddress = addr;
		}
	}

	// Receive the returned status packets
	for(i = 0; i < 3*numberOfRequests; i += 3)
	{
		// Retrieve the requested length and ID
		int len = m_TxBulkRead[DP_PARAMETER + i + 1];
		int id  = m_TxBulkRead[DP_PARAMETER + i + 2];

		// Error checking
		if(len + 6 >= (int) sizeof(rxp))
		{
			ROS_ERROR_THROTTLE(0.3, "Requested read size for bulk read of ID %d is too big (%d bytes) for the local Rx buffer size", id, len + 6);
			return RET_RX_CORRUPT;
		}

		// Try to receive a packet of the appropriate length
		int rxError = rxPacket(rxp, len + 6, &timeout_time);

		// Handle case where packet fails to arrive or is otherwise corrupt
		if(rxError != RET_SUCCESS)
		{
			// If no servo responded at all then make sure the next bulk read is sent in full, just to be safe
			if(i <= 3) // Note: i == 0 should always correspond to the CM730, so if the first-queried servo just failed to respond we have i == 3...
				m_fullBRPacket = true;

			// Save the current ROS time
#if BULKREAD_DETAILED_TIMESTAMPS
			ts.push_back(ros::Time::now());
#endif

			// Log an error message
#if BULKREAD_DETAILED_TIMESTAMPS
			ROS_ERROR("Could not read answer for ID %d: Error %d", id, rxError);
#endif

			// Display timing information of the bulk read
#if DISPLAY_BULKREAD_TIME
			ROS_WARN("Total bulk read time: %lfs [FAIL]", (ros::Time::now() - ts[0]).toSec());
#endif

			// Save which ID failed (although the real cause could at times be another servo)
			m_lastFailedID = id;
			if((id >= 0) && (id < MAX_DEVICES))
			{
				m_failCount[id]++;
#if DISPLAY_FAIL_COUNTS
				ROS_ERROR("ID %d failed: Total %d times now", id, m_failCount[id]);
#endif
			}
			else
				ROS_ERROR("Invalid ID %d failed", id);

			// Display the recorded ROS times
#if BULKREAD_DETAILED_TIMESTAMPS
			if(ts.back() - ts[0] >= ros::Duration((double) COMMS_READ_TIMEOUT * 1e-9))
			{
				ROS_WARN("Total time: %lfs", (ts.back() - ts[0]).toSec());
				for(size_t j = 1; j < ts.size(); j++)
					ROS_WARN("Delta: %lfs (%lfs since start)", (ts[j] - ts[j-1]).toSec(), (ts[j] - ts[0]).toSec());
			}
#endif

			// Return the error that occurred
			return rxError;
		}

		// Save the current ROS time
#if BULKREAD_DETAILED_TIMESTAMPS
		ts.push_back(ros::Time::now());
#endif

		// Parse the received bulk read response packet
		parseBRPacket(rxp, len + 6, servoData, cm730Data);

		// Save the current ROS time
#if BULKREAD_DETAILED_TIMESTAMPS
		ts.push_back(ros::Time::now());
#endif
	}

	// Display timing information if required
#if DISPLAY_BULKREAD_TIME
	ROS_WARN("Total bulk read time: %lfs", (ros::Time::now() - ts[0]).toSec());
#endif
#if BULKREAD_DETAILED_TIMESTAMPS && DISPLAY_NONFAIL_TIMES
	ROS_WARN("Total time: %lfs", (ts.back() - ts[0]).toSec());
	for(size_t j = 1; j < ts.size(); j++)
		ROS_WARN("Delta: %lfs (%lfs since start)", (ts[j] - ts[j-1]).toSec(), (ts[j] - ts[0]).toSec());
#endif

	// Return success
	return RET_SUCCESS;
}

// Rebuild the Tx packet that is sent for each bulk read based on the passed list of servos
int CM730::updateTxBRPacket(const std::vector<int>& servos)
{
	// We want the packet: [0xFF] [0xFF] [0xFE] [3*N+3] [0x92] [0x00] [L1] [ID1] [Addr1] ... [LN] [IDN] [AddrN] [Checksum]

	// Declare variables
	int length = 3*(servos.size() + 1) + 3;
	int num = 0;

	// Error checking
	if(length + 4 > MAX_TX_BYTES)
		return RET_BAD_PARAM;

	// Initialise the bulk read packet
	m_TxBulkRead[DP_ID]                = (unsigned char) ID_BROADCAST;
	m_TxBulkRead[DP_LENGTH]            = (unsigned char) length;
	m_TxBulkRead[DP_INSTRUCTION]       = INST_BULK_READ;
	m_TxBulkRead[DP_PARAMETER + num++] = (unsigned char) 0x00;

	// Add the CM730 as the first device in the bulk read (important!)
	m_TxBulkRead[DP_PARAMETER + num++] = READ_CM730_LENGTH;  // Number of bytes to read
	m_TxBulkRead[DP_PARAMETER + num++] = ID_CM730;           // ID of the CM730
	m_TxBulkRead[DP_PARAMETER + num++] = READ_CM730_ADDRESS; // Start address of the region to read

	// Add the servos to the bulk read packet
	for(size_t i = 0; i < servos.size(); i++)
	{
		m_TxBulkRead[DP_PARAMETER + num++] = READ_SERVO_LENGTH;  // Number of bytes to read
		m_TxBulkRead[DP_PARAMETER + num++] = servos[i];          // ID of the i-th servo
		m_TxBulkRead[DP_PARAMETER + num++] = READ_SERVO_ADDRESS; // Start address of the region to read
	}

	// We have modified the m_TxBulkRead packet, so we need to force the next bulk read to send the full bulk read Tx packet
	m_fullBRPacket = true;

	// Return success
	return RET_SUCCESS;
}

//
// Write functions
//

// Write one register byte to a device
int CM730::writeByte(int id, int address, int value)
{
	// We send the packet: [0xFF] [0xFF] [ID] [0x04] [WRITE] [Addr] [Data] [Checksum]

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};

	// Construct the packet to send
	txp[DP_ID]           = (unsigned char) id;
	txp[DP_LENGTH]       = 4;
	txp[DP_INSTRUCTION]  = INST_WRITE;
	txp[DP_PARAMETER]    = (unsigned char) address;
	txp[DP_PARAMETER+1]  = (unsigned char) value;

	// Send the write packet and return
	return txPacket(txp);
}

// Write one register word to a device
int CM730::writeWord(int id, int address, int value)
{
	// We send the packet: [0xFF] [0xFF] [ID] [0x05] [WRITE] [Addr] [Data1] [Data2] [Checksum]

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};

	// Construct the packet to send
	txp[DP_ID]           = (unsigned char) id;
	txp[DP_LENGTH]       = 5;
	txp[DP_INSTRUCTION]  = INST_WRITE;
	txp[DP_PARAMETER]    = (unsigned char) address;
	txp[DP_PARAMETER+1]  = (unsigned char) (value & 0xFF);
	txp[DP_PARAMETER+2]  = (unsigned char) ((value >> 8) & 0xFF);

	// Send the write packet and return
	return txPacket(txp);
}

// Write register data to a device
int CM730::writeData(int id, int address, void* data, size_t size)
{
	// We send the packet: [0xFF] [0xFF] [ID] [N+3] [WRITE] [Addr] [Data1] ... [DataN] [Checksum]

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};

	// Error checking
	if((int) size + 7 > MAX_TX_BYTES)
		return RET_BAD_PARAM;

	// Construct the packet to send
	txp[DP_ID]          = (unsigned char) id;
	txp[DP_LENGTH]      = size + 3;
	txp[DP_INSTRUCTION] = INST_WRITE;
	txp[DP_PARAMETER]   = (unsigned char) address;
	memcpy(&txp[DP_PARAMETER+1], data, size);

	// Send the write packet and return
	return txPacket(txp);
}

// Perform a sync write
// address:      The starting address of the write on each device
// numDataBytes: The number of bytes to write for each device (i.e. results in a write to the register range "address --> address + numDataBytes - 1")
// numDevices:   The number of devices to write to (the data for exactly this number of devices should be stored in the data parameter)
// data:         A back to back byte array of IDs and corresponding data (Required array length = (numDataBytes + 1) * numDevices)
int CM730::syncWrite(int address, size_t numDataBytes, size_t numDevices, const uint8_t* data)
{
	// We send the packet:
	//     [0xFF] [0xFF] [ID] [(L+1)*N+4] [SYNC_WRITE] [Addr] [L] [[data]] [Checksum]
	// where L is numDataBytes and N is numDevices.
	// [[data]] is expected to be a back to back array of:
	//     [ID] [Data1] ... [DataL]
	// where L is numDataBytes.

	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};
	size_t dataSize;

	// Calculate the array and packet sizes
	dataSize = (numDataBytes + 1) * numDevices;

	// Error checking
	if((int) dataSize + 8 > MAX_TX_BYTES)
		return RET_BAD_PARAM;

	// Construct the packet to send
	txp[DP_ID]          = ID_BROADCAST;
	txp[DP_LENGTH]      = dataSize + 4;
	txp[DP_INSTRUCTION] = INST_SYNC_WRITE;
	txp[DP_PARAMETER]   = (unsigned char) address;
	txp[DP_PARAMETER+1] = (unsigned char) numDataBytes;
	memcpy(&txp[DP_PARAMETER+2], data, dataSize);

	// Send the write packet and return
	return txPacket(txp);
}

// Set the dyamixel power state of the CM730
int CM730::setDynamixelPower(int value)
{
	// Error checking
	if(value < DYNPOW_OFF || value > DYNPOW_ON_NODXLTX)
	{
		ROS_ERROR("Attempted to write bad value %d to the CM730 Dynamixel Power register!", value);
		return RET_BAD_PARAM;
	}
	
	// Write the appropriate register
	int ret = writeByte(ID_CM730, P_DYNAMIXEL_POWER, value);
	if(ret != RET_SUCCESS)
		ROS_ERROR("Failed to write %d to the CM730 Dynamixel Power register! (Error: %d)", value, ret);
	
	// Return value
	return ret;
}

//
// Communications functions
//

// Transmit a packet to the CM730
int CM730::txPacket(unsigned char* txp)
{
	// Don't do anything if the communications are suspended
	if(isSuspended()) return RET_TX_FAIL;

	// Finalise the Tx packet
	int length = txp[DP_LENGTH] + 4;
	txp[0] = 0xFF;
	txp[1] = 0xFF;
	txp[length-1] = checksum(txp);

	// Clear the Rx packet buffer inside the read thread
	flushPort();

	// Display the packet being sent
#if CM730_COMMS_DEBUG || CM730_COMMS_DEBUG_TX
	dump("TX", txp, length);
#endif

	// Write the packet to the serial port
	if(write(m_fd, txp, length) == length)
	{
		flushPort(); // Clear the Rx packet buffer again...
		return RET_SUCCESS;
	}
	else
	{
		ROS_ERROR("CM730 txPacket() write error: %s", strerror(errno));
		return RET_TX_FAIL;
	}
}

// Receive a packet from the CM730
int CM730::rxPacket(unsigned char* rxp, int size, struct timespec* abstime)
{
	// Don't do anything if the communications are suspended
	if(isSuspended()) return RET_TX_FAIL;

	// Declare variables
	int readErr, readSize = 0;

	// Calculate the absolute timeout time if one wasn't provided
	struct timespec timeout;
	if(!abstime)
	{
		getTimeLimit(&timeout);
		abstime = &timeout;
	}

	// Read until the appropriate number of bytes are received or an error occurs
	while(1)
	{
		// Wait to receive at least as many bytes as we are still missing from our packet
		readErr = m_readThread.read(rxp + readSize, size - readSize, abstime); // Note: This should be the only location that retrieves Rx buffer data from the read thread!

		// Abort if we have timed out
		if(readErr == -ETIMEDOUT)
			return RET_RX_TIMEOUT;

		// Abort if an error occurred
		if(readErr < 0)
			return RET_RX_FAIL;

		// Display the received data
#if CM730_COMMS_DEBUG
		dump("RX       ", rxp + readSize, readErr);
#endif

		// Make note of the number of bytes we received 
		readSize += readErr;

		// Synchronise the start of rxp with the start of a packet
		syncRxPacket(rxp, &readSize);

		// See if we have read enough bytes
		if(readSize >= size) // Note: It should be impossible for us to ever have read more than size bytes, so theoretically readSize == size would suffice here...
		{
			// Display the received packet
#if CM730_COMMS_DEBUG
			dump("RX packet", rxp, readSize);
#endif

			// Retrieve the packet length
			int packetSize = rxp[DP_LENGTH] + 4;
			
			// Discard packets that are too short
			if(packetSize < size) // The packet at the beginning of rxp is too short, but we have still received the right number of bytes, so there must be another packet, which might be ours! Because we already know that we have received more bytes it is unlikely that we cause timeouts with this heuristic that otherwise would not have been.
			{
				memset(rxp, 0, packetSize);
				syncRxPacket(rxp, &readSize);
				if(readSize < size) continue;
			}
			
			// Check that the packet is of the length that we were expecting
			if(packetSize != size)
			{
				ROS_ERROR_THROTTLE(0.1, "Wrong Rx packet length: %d (expected %d)", packetSize, (int) size);
				return RET_RX_CORRUPT;
			}

			// Check that the checksum of the packet is correct
			int txChecksum = checksum(rxp);
			if(txChecksum != rxp[DP_LENGTH + rxp[DP_LENGTH]]) // This is safe as at this point we must have rxp[DP_LENGTH] + 4 == size
			{
				ROS_ERROR_THROTTLE(0.1, "Wrong Rx packet checksum: %d (expected %d)", (int) rxp[DP_LENGTH + rxp[DP_LENGTH]], txChecksum);
				return RET_RX_CORRUPT;
			}

			// Return success
			return RET_SUCCESS;
		}
	}
}

// Flush the Rx buffer data (reset to a clean slate)
void CM730::flushPort()
{
	// Tell the read thread to flush the Rx buffers
	m_readThread.flush();
}

//
// Packet helper functions
//

// Parse a bulk read return packet for the information it contains
bool CM730::parseBRPacket(unsigned char* rxp, int size, std::vector<BRData>* servoData, BRBoard* cm730Data)
{
	// Parse the bulk read return packet based on whether it's the CM730 or a servo
	if(rxp[DP_ID] == ID_CM730)
	{
		// Parse the CM730 data
		return parseCM730Data(rxp + DP_PARAMETER, size - DP_PARAMETER - 1, cm730Data);
	}
	else
	{
		// Check the servo ID
		if(rxp[DP_ID] > servoData->size())
		{
			ROS_WARN_THROTTLE(0.3, "Servo bulk read return packet comes from ID %d, which is outside our servo data array size (%d)", (int) rxp[DP_ID], (int) servoData->size());
			return false;
		}

		// Retrieve the servo information struct based on the ID
		BRData* servoInfo = &servoData->at(rxp[DP_ID] - 1); // The servo data is stored by ID in a zero-based array, hence subtract 1 here to shift to zero-based

		// Check the packet size
		if(size != DP_PARAMETER + servoInfo->length + 1)
		{
			ROS_WARN_THROTTLE(0.3, "Servo bulk read return packet (%d bytes) cannot be parsed as it is not of the expected size (%d bytes)", size, DP_PARAMETER + servoInfo->length + 1);
			return false;
		}

		// Calculate the register data address offset
		int offset = DP_PARAMETER - servoInfo->startAddress;

		// Save the returned data
		servoInfo->position = makeWord(rxp + offset + DynamixelMX::P_PRESENT_POSITION_L);
		
		// Return that we got some data
		return true;
	}
}

// Parse the data of a CM730 packet (data is assumed to point to the first of size bytes of payload data)
bool CM730::parseCM730Data(unsigned char* data, int size, BRBoard* cm730Data)
{
	// Check that our packet is of the right size
	if(size != cm730Data->length)
	{
		ROS_WARN_THROTTLE(0.3, "CM730 packet (%d payload bytes) cannot be parsed as it is not of the expected size (%d bytes)", size, cm730Data->length);
		return false;
	}

	// Calculate the register data address offset
	int offset = -cm730Data->startAddress;

	// Save the returned data
	cm730Data->power    = data[offset + P_DYNAMIXEL_POWER];
	cm730Data->ledPanel = data[offset + P_LED_PANEL];
	cm730Data->rgbled5  = makeWord(data + offset + P_RGBLED5_L);
	cm730Data->rgbled6  = makeWord(data + offset + P_RGBLED6_L);
	cm730Data->button   = data[offset + P_BUTTON];
	cm730Data->voltage  = data[offset + P_BATTERY_VOLTAGE];
	cm730Data->gyroX    = makeWordSigned(data + offset + P_GYRO_X_L);
	cm730Data->gyroY    = makeWordSigned(data + offset + P_GYRO_Y_L);
	cm730Data->gyroZ    = makeWordSigned(data + offset + P_GYRO_Z_L);
	cm730Data->accX     = makeWordSigned(data + offset + P_ACC_X_L);
	cm730Data->accY     = makeWordSigned(data + offset + P_ACC_Y_L);
	cm730Data->accZ     = makeWordSigned(data + offset + P_ACC_Z_L);
	cm730Data->magX     = makeWordSigned(data + offset + P_MAG_X_L);
	cm730Data->magY     = makeWordSigned(data + offset + P_MAG_Y_L);
	cm730Data->magZ     = makeWordSigned(data + offset + P_MAG_Z_L);
	
	// Save the dynamixel power state for our own purposes
	m_lastSeenDynPow = cm730Data->power;
	
	// Indicate that we received CM730 data
	m_gotCM730Data = true;
	
	// Return that we got some data
	return true;
}

// Synchronise the start of a packet with the start of the passed Rx data buffer (returns the number of bytes that are discarded in the process)
int CM730::syncRxPacket(unsigned char* rxp, int* readSize)
{
	// If we don't have at least 3 bytes then we can't sync anything
	if(*readSize < 3)
		return 0;

	// If the start of our received data is already the start of a packet then all is good
	if(rxp[0] == 0xFF && rxp[1] == 0xFF && rxp[2] != 0xFF)
		return 0;

	// Declare variables
	bool Found = false;
	int i, origSize = *readSize;

	// Search for the first occurrence of a packet header [0xFF] [0xFF] [non-0xFF]
	for(i = 1; i < (*readSize - 2); i++)
	{
		if(rxp[i] == 0xFF && rxp[i+1] == 0xFF && rxp[i+2] != 0xFF)
		{
			Found = true;
			break;
		}
	}

	// First found packet header starts at the zero-based index i => Remove the first i bytes of the Rx buffer
	if(Found)
	{
		size_t copySize = *readSize - i;
		memmove(rxp, rxp + i, copySize);
		*readSize = copySize;
		return i;
	}

	// Check if the last 0 to 2 bytes are a possible incomplete packet header
	if(rxp[*readSize-1] != 0xFF)
	{
		*readSize = 0;
		return origSize;
	}
	else if(rxp[*readSize-2] != 0xFF)
	{
		rxp[0] = 0xFF;
		*readSize = 1;
		return origSize - 1;
	}
	else
	{
		rxp[0] = 0xFF;
		rxp[1] = 0xFF;
		*readSize = 2;
		return origSize - 2;
	}
}

// Calculate the checksum of a packet (assumes that the packet is valid in terms of the length parameter!)
unsigned char CM730::checksum(unsigned char* packet)
{
	// Declare variables
	unsigned char checksum = 0x00;

	// Sum up the bytes of the package
	for(int i = DP_ID; i < packet[DP_LENGTH] + 3; i++)
		checksum += packet[i];

	// Calculate the 1's complement of the sum => This is now the true checksum
	return (~checksum);
}

//
// Helper functions
//

// Calculate a timeout time exactly COMMS_READ_TIMEOUT (ns) into the future based on the current time
inline void getTimeLimit(struct timespec* out)
{
	clock_gettime(CLOCK_MONOTONIC, out);
	out->tv_sec += (out->tv_nsec + COMMS_READ_TIMEOUT) / 1000000000LL;
	out->tv_nsec = (out->tv_nsec + COMMS_READ_TIMEOUT) % 1000000000LL;
}

// Make a word out of two consecutive bytes (pos[0] = Low byte, pos[1] = High byte)
inline int makeWord(unsigned char* pos)
{
	return (uint16_t)(((int) pos[1] << 8) | pos[0]);
}

// Make a signed word out of two consecutive bytes (pos[0] = Low byte, pos[1] = High byte)
inline int makeWordSigned(unsigned char* pos)
{
	return (int16_t)(((int) pos[1] << 8) |  pos[0]);
}

// Dump the contents of a byte array to the screen
void dump(const char* prefix, const uint8_t* data, uint8_t len)
{
	printf("%s:", prefix);
	for(int i = 0; i < len; i++)
		printf(" %02X", data[i]);
	printf("\n");
}
// EOF