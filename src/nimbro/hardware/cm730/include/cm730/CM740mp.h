// Hardware interface for the CM740 board with megapacket support
// File:    CM730.cpp
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schueller <schuell1@cs.uni-bonn.de>
//          Grzegorz Ficht <gficht@cs.uni-bonn.de>
// Comment: This file is suitable for CM730 firmware versions of 0x88 and above (NimbRo-OP specific with megapacket support).

// Ensure header is included only once
#ifndef CM740MP_H
#define CM740MP_H

// Includes
#include <cm730/CM730.h>

namespace cm740
{

// megapacket servo data struct
struct MPData
{
	// Constructor
	MPData() { std::memset(this, 0, sizeof(MPData)); length=6; f_writePosition = false; }

	// Megapacket read parameters
	unsigned char length;			// subpacket length
	unsigned char id;
	unsigned char type;				// not received, only written at configuration
	unsigned char status;
	unsigned char health;
	unsigned char healthPrevious;	// not received, copied before receiving new health information.

	// flags
	bool f_writePosition;

	// Received data
	unsigned short position; 		// P_PRESENT_POSITION_*
// 	unsigned short speed; 			// P_PRESENT_SPEED_*
// 	unsigned short torque;			// P_PRESENT_TORQUE_*
// 	unsigned char  voltage;			// P_PRESENT_VOLTAGE_*
// 	unsigned char  temperature;		// P_PRESENT_TEMPERATURE_*

	// Sent data
	unsigned char  torqueEnable;    //  0/1 = torque off/on
	unsigned char  complianceSlope;	// 1024/complianceSlope = P_GAIN = effort;
	unsigned short positionGoal;	// 4095 = max position -> 300deg
	unsigned short speedGoal;		//    0 = max speed
	unsigned short torqueGoal;		// 1023 = max torque

};

// megapacket board data struct
struct MPBoard
{
	// Constructor
	MPBoard() { std::memset(this, 0, sizeof(MPBoard)); length = 30;}

	// Megapacket read parameters
	unsigned char length;		// subpacket length
	unsigned char id;
	unsigned char status;
	unsigned char health;
	unsigned char healthPrevious;

	// Received data
	unsigned char  power;		// P_DYNAMIXEL_POWER
	unsigned char  ledPanel;	// P_LED_PANEL
	unsigned short rgbled5;		// P_RGBLED5_*
	unsigned short rgbled6;		// P_RGBLED6_*
	unsigned char  button;		// P_BUTTON
	unsigned char  voltage;		// P_BATTERY_VOLTAGE
	signed short gyroX;       	// P_GYRO_X_*
	signed short gyroY;       	// P_GYRO_Y_*
	signed short gyroZ;       	// P_GYRO_Z_*
	signed short accX;        	// P_ACC_X_*
	signed short accY;        	// P_ACC_Y_*
	signed short accZ;        	// P_ACC_Z_*
	signed short magX;        	// P_MAG_X_*
	signed short magY;        	// P_MAG_Y_*
	signed short magZ;        	// P_MAG_Z_*

	// Sent data
	unsigned char  ledPanelCmd;	// P_LED_PANEL
	unsigned short rgbled5Cmd;	// P_RGBLED5_*

};

//
// CM740 with megapacket support class
//
class CM740mp : public cm730::CM730
{
public:
	CM740mp(const std::string& resourcePath, const std::string& configParamPath) : CM730(resourcePath, configParamPath, boost::make_shared<cm730::DynamixelMX>()), m_inMegapacketMode(false), m_isMegapacketConfigured(false), m_writeMegapacket(false) {}
	int  writeMegapacket(std::vector<MPData>& Servos, MPBoard& Board);
	int  receiveMegapacket(std::vector<MPData>& Servos, MPBoard& Board);
	int  configureMegapacket(std::vector<MPData>& Servos, MPBoard& Board);
	void addMegapacketServo(std::vector<MPData>& Servos,unsigned char id, unsigned char type);
	int  parseBoardData(MPBoard& Board, unsigned char* data);
	int  parseServoData(MPData& Servo, unsigned char* data);
	int  enableMegapacket(bool enable);
	int  packetsReceived(unsigned char oldPackets, unsigned char currentPackets);
	int  txMegapacket(unsigned char* txp);
// 	int  rxMegapacket(unsigned char* txp);
private:
	bool m_inMegapacketMode;
	bool m_isMegapacketConfigured;
	bool m_writeMegapacket;
};

}

#endif
// EOF
