// Hardware interface for the CM740 board with megapacket support
// File:    CM730.cpp
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schueller <schuell1@cs.uni-bonn.de>
//          Grzegorz Ficht <gficht@cs.uni-bonn.de>
// Comment: This file is suitable for CM730 firmware versions of 0x88 and above (NimbRo-OP specific with megapacket support).

// Includes - Local
#include <cm730/CM740mp.h>

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

// Defines
#define COMMS_READ_TIMEOUT  10000000LL  // Read timeout in nanoseconds (it's ok if this is longer than a single robotcontrol time step, but it shouldn't be too long so that the read in the next cycle has a chance without a cycle being missed)

// Namespaces
using namespace cm740;

// Helper functions
inline int makeWord(unsigned char* pos);
inline int makeWordSigned(unsigned char* pos);
inline void getTimeLimit(struct timespec* out);
unsigned char getLowerByte(unsigned short value);
unsigned char getUpperByte(unsigned short value);
void dump1(const char* prefix, const uint8_t* data, uint8_t len);

// Write a megapacket to the cm740, assumes that the CM740 is also configured.
int CM740mp::writeMegapacket(std::vector<MPData>& Servos, MPBoard& Board)
{
	// We send the packet: [0xFF] [0xFF] [ID] [N+3] [WRITE] [Addr] [Data1] ... [DataN] [Checksum]
	// Declare variables
	unsigned char txmp[MAX_TX_BYTES] = {0};
	int length = 0;
	// Construct the packet to send
	txmp[DP_ID]          = ID_CM730;
	txmp[DP_INSTRUCTION] = INST_MEGA_WRITE;
	// First write board data(LEDs)
	txmp[DP_PARAMETER+length]   = 5;
	txmp[DP_PARAMETER+length+1] = ID_CM730;
	txmp[DP_PARAMETER+length+2] = Board.ledPanelCmd;
	txmp[DP_PARAMETER+length+3] = getLowerByte(Board.rgbled5Cmd);
	txmp[DP_PARAMETER+length+4] = getUpperByte(Board.rgbled5Cmd);
	length += 5;

	for(unsigned int i = 0; i < Servos.size(); i++)
	{
		if(Servos[i].f_writePosition == true)
		{
			txmp[DP_PARAMETER+length]   = 11; // 7 bytes + length and id byte
			txmp[DP_PARAMETER+length+1] = Servos[i].id;
			txmp[DP_PARAMETER+length+2] = Servos[i].torqueEnable;
			txmp[DP_PARAMETER+length+3] = Servos[i].complianceSlope; // the same compliance slope in both directions
			txmp[DP_PARAMETER+length+4] = Servos[i].complianceSlope; // the same compliance slope in both directions
			txmp[DP_PARAMETER+length+5] = getLowerByte(Servos[i].positionGoal);
			txmp[DP_PARAMETER+length+6] = getUpperByte(Servos[i].positionGoal);
			txmp[DP_PARAMETER+length+7] = getLowerByte(Servos[i].speedGoal);
			txmp[DP_PARAMETER+length+8] = getUpperByte(Servos[i].speedGoal);
			txmp[DP_PARAMETER+length+9] = getLowerByte(Servos[i].torqueGoal);
			txmp[DP_PARAMETER+length+10]= getUpperByte(Servos[i].torqueGoal);
			length += 11;
		}
		else
		{
			txmp[DP_PARAMETER+length] = 3; // 1 byte + length and id byte
			txmp[DP_PARAMETER+length+1] = Servos[i].id;
			txmp[DP_PARAMETER+length+2] = Servos[i].torqueEnable;
			length += 3;
		}
	}
	txmp[DP_LENGTH] = (unsigned char)(length + 2);

	// Send the write packet and return
	return txMegapacket(txmp);
}


// Receive and parse a megapacket, save the servo/board data in it to the device lists
int CM740mp::receiveMegapacket(std::vector<MPData>& Servos, MPBoard& Board)
{
	unsigned char rxmp[MAX_RX_BYTES] = {0};
	int size=Board.length+6;// extra 6 bytes = [FF][FF][ID][LEN][INST]...[CRC]
	int ret;
	for(unsigned int device=0; device<Servos.size(); device++)
	{
		size += Servos[device].length;
	}
	struct timespec timeout_time;

	getTimeLimit(&timeout_time);
	// TODO write an rxMegapacket() function to not wait for a packet.
	ret = rxPacket(rxmp,size,&timeout_time);

	// Parse the megapacket
//	if((rxmp[DP_ID] == ID_CM730) && (rxmp[DP_PARAMETER]>0))
	if(ret == RET_SUCCESS)
	{// correct packet received
		unsigned char dataMarker=0;
		// TODO add support to handle just the board, if no servos initialised
			for(unsigned int device=0; device<Servos.size();) // board doesn't count as device
			{
				switch(rxmp[DP_PARAMETER+dataMarker+1])
				{
				// if it's the board
				case ID_CM730:
					parseBoardData(Board, rxmp + DP_PARAMETER + dataMarker);
//					ROS_INFO("current packets: %d, old packets: %d",Board.health,Board.healthPrevious);
					dataMarker += rxmp[DP_PARAMETER+dataMarker];
					break;

				//if it's a different device
				default:
					parseServoData(Servos[device], rxmp + DP_PARAMETER + dataMarker);
					dataMarker += rxmp[DP_PARAMETER+dataMarker];
					device++;
					break;

				}
			}
		flushPort();
		return ret;
	}
	else if(ret == RET_RX_CORRUPT)
	{// there was a packet but it wasn't correct somehow

		flushPort();
		return ret;
	}
	else return ret;
}

// enables megapacket mode
int CM740mp::enableMegapacket(bool enable)
{
	int ret;
	ret = setDynamixelPower(enable == true ? DYNPOW_ON_MPACKET : DYNPOW_OFF);
	m_inMegapacketMode = (ret == RET_SUCCESS? true : false);
	return ret;
}

// copy data from megapacket to board
int CM740mp::parseBoardData(MPBoard& Board, unsigned char* data)
{
	Board.healthPrevious = Board.health;
	Board.status   = data[2];
	Board.health   = data[3];
	Board.power    = data[4];
	Board.ledPanel = data[5];
	Board.rgbled5  = makeWord(data+6);
	Board.rgbled6  = makeWord(data+8);
	Board.button   = data[10];
	Board.voltage  = data[11];
	Board.gyroX    = makeWordSigned(data + 12);
	Board.gyroY    = makeWordSigned(data + 14);
	Board.gyroZ    = makeWordSigned(data + 16);
	Board.accX     = makeWordSigned(data + 18);
	Board.accY     = makeWordSigned(data + 20);
	Board.accZ     = makeWordSigned(data + 22);
	Board.magX     = makeWordSigned(data + 24);
	Board.magY     = makeWordSigned(data + 26);
	Board.magZ     = makeWordSigned(data + 28);
	return packetsReceived(Board.healthPrevious, Board.health);
}

// copy data from megapacket to servo
int CM740mp::parseServoData(MPData& Servo, unsigned char* data)
{
	// data[0] is length
	// data[1] is id
	Servo.healthPrevious = Servo.health;
	Servo.status      = data[2];
	Servo.health      = data[3];
	Servo.position    = makeWord(data+4);
// 	Servo.speed       = makeWord(data+6);
//	Servo.torque      = makeWord(data+8);
// 	Servo.torque      = (uint16_t)(((int) (data[9]&0x03) << 8) | data[8]);
// 	Servo.voltage     = data[10];
// 	Servo.temperature = data[11];
	return packetsReceived(Servo.healthPrevious, Servo.health);
}

// configures the dxl device structures inside the CM740 and prepares them for communication on the dxl bus
int CM740mp::configureMegapacket(std::vector<MPData>& Servos, MPBoard& Board)
{
	// We send the packet: [0xFF] [0xFF] [ID] [N+2] [MEGA_CONFIG] [DEV_1_ID] [DEV_1_TYPE] [DEV_N_ID] [DEV_N_TYPE] ... [Checksum]
	// Declare variables
	unsigned char txp[MAX_TX_BYTES] = {0};
	int size = Servos.size()+1; // +1 is the board

	// if we want a board in the megapacket data
		// Error checking
		if((int) size*2 + 6 > MAX_TX_BYTES)
			return RET_BAD_PARAM;

	// Construct the packet to send
	txp[DP_ID]          = ID_CM730;
	txp[DP_LENGTH]      = size*2 + 2;
	txp[DP_INSTRUCTION] = INST_MEGA_CONFIG;
	txp[DP_PARAMETER]   = ID_CM730;		// type CM730
	txp[DP_PARAMETER+1] = 1;			// type CM730
	for(int i=1;i<size;i++)
	{
		txp[DP_PARAMETER+i*2]   = Servos[i-1].id;
		txp[DP_PARAMETER+i*2+1] = Servos[i-1].type;
	}
	m_isMegapacketConfigured = true;
	// Send the write packet and return
	return txPacket(txp);
}

//
// helper functions
//

//adds servo to the list of devices;
void CM740mp::addMegapacketServo(std::vector<MPData>& Servos,unsigned char id, unsigned char type)
{
	MPData tempServo;
	tempServo.id=id;
	tempServo.type=type;
	Servos.push_back(tempServo);
}

// gets the number of times the servo has been read out since last megapacket. Assumes that less than 256 times -> read outs happen constantly.
int CM740mp::packetsReceived(unsigned char oldPackets, unsigned char currentPackets)
{
	if(oldPackets > currentPackets)
		return (int)(currentPackets+256-oldPackets);
	else
		return (int)(currentPackets-oldPackets);
}

// Transmit a packet to the CM730
int CM740mp::txMegapacket(unsigned char* txp)
{
	// Don't do anything if the communications are suspended
	if(isSuspended()) return RET_TX_FAIL;

	// Finalise the Tx packet
	int length = txp[DP_LENGTH] + 4;
	txp[0] = 0xFF;
	txp[1] = 0xFF;
	txp[length-1] = checksum(txp);

	// Clear the Rx packet buffer inside the read thread
//	flushPort();

	// Display the packet being sent
//	dump1("TX", txp, length);

	// Write the packet to the serial port
	if(write(m_fd, txp, length) == length)
	{
//		flushPort(); // Clear the Rx packet buffer again...
		return RET_SUCCESS;
	}
	else
	{
		ROS_ERROR("CM740 txMegapacket() write error: %s", strerror(errno));
		return RET_TX_FAIL;
	}
}

//
// Helper functions
//

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

// gets the lower byte of a 16-bit value
unsigned char getLowerByte(unsigned short value)
{
	return (unsigned char)(value & 0xFF);
}

// gets the upper byte of a 16-bit value
unsigned char getUpperByte(unsigned short value)
{
	return (unsigned char)(value >> 8);
}

// Calculate a timeout time exactly COMMS_READ_TIMEOUT (ns) into the future based on the current time
inline void getTimeLimit(struct timespec* out)
{
	clock_gettime(CLOCK_MONOTONIC, out);
	out->tv_sec += (out->tv_nsec + COMMS_READ_TIMEOUT) / 1000000000LL;
	out->tv_nsec = (out->tv_nsec + COMMS_READ_TIMEOUT) % 1000000000LL;
}


// Dump the contents of a byte array to the screen
void dump1(const char* prefix, const uint8_t* data, uint8_t len)
{
	printf("%s:", prefix);
	for(int i = 0; i < len; i++)
		printf(" %02X", data[i]);
	printf("\n");
}
