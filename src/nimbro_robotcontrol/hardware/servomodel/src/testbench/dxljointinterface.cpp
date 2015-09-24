// Dynamixel joint interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "dxljointinterface.h"

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <QDebug>

#include <dynamixel.h>

#include <ros/console.h>

const int BAUD_NUM = 1;
const double TICKS_TO_RAD = 2.0 * M_PI / 4096.0;
const double VEL_TO_RAD_PER_S = 2.0 * M_PI * 0.114 / 60.0;
const double TORQUE_TO_NM = 0.0045 * 6.0 / 4.1;

// Register numbers
enum Register
{
	// ROM
	REG_CW_ANGLE_LIMIT      = 6,
	REG_CCW_ANGLE_LIMIT     = 8,
	REG_MAX_TORQUE          = 14,

	// RAM
	REG_TORQUE_ENABLE       = 24,
	REG_D_GAIN              = 26,
	REG_I_GAIN              = 27,
	REG_P_GAIN              = 28,
	REG_GOAL_POSITION       = 30,
	REG_MOVING_SPEED        = 32,
	REG_TORQUE_LIMIT        = 34,
	REG_PRESENT_POSITION    = 36,
	REG_PRESENT_SPEED       = 38,
	REG_PRESENT_LOAD        = 40,
	REG_PRESENT_TEMPERATURE = 43,
	REG_CURRENT             = 68,
	REG_TORQUE_CONTROL_EN   = 70,
	REG_GOAL_TORQUE         = 71
};

DXLJointInterface::DXLJointInterface(int servoID)
 : IJointInterface(servoID)
 , m_goalPosition(0)
 , m_goalSpeed(0)
 , m_currentPosition(0)
 , m_currentVelocity(0)
 , m_torqueLimit(0)
 , m_tickOffset(2048)
{
}

DXLJointInterface::~DXLJointInterface()
{
}

bool DXLJointInterface::init()
{
	if(!dxl_initialize(0, BAUD_NUM))
		return false;

	dxl_write_byte(200, 24, 1);
// 	sleep(1);

// 	dxl_ping(id());
// 	if(dxl_get_result() != COMM_RXSUCCESS)
// 	{
// 		ROS_ERROR("Could not find dynamixel with ID %d", id());
// 		return false;
// 	}

	dxl_write_word(id(), REG_TORQUE_LIMIT, 1023);
	dxl_write_word(id(), REG_CW_ANGLE_LIMIT, 0);

	setControlType(CT_POSITION);
	dxl_write_word(id(), REG_TORQUE_ENABLE, 1);

	dxl_write_byte(id(), REG_P_GAIN, 2);
	dxl_write_byte(id(), REG_I_GAIN, 0);
	dxl_write_byte(id(), REG_D_GAIN, 0);

	return true;
}

void DXLJointInterface::setOffsetTicks(int ticks)
{
	m_tickOffset = ticks;
}

void DXLJointInterface::setGoalPosition(double goalPosition)
{
	m_goalPosition = goalPosition;
}

void DXLJointInterface::setGoalSpeed(double speed)
{
	m_goalSpeed = speed;
}

void DXLJointInterface::setGoalTorque(double torque)
{
	m_goalTorque = torque;
}

void DXLJointInterface::setTorqueLimit(double torqueLimit)
{
	m_torqueLimit = torqueLimit;
}

void DXLJointInterface::setControlType(IJointInterface::ControlType ct)
{
	IJointInterface::setControlType(ct);

	dxl_write_word(id(), REG_CW_ANGLE_LIMIT, 0);

	if(ct == CT_POSITION)
		dxl_write_word(id(), REG_CCW_ANGLE_LIMIT, 4095);
	else
		dxl_write_word(id(), REG_CCW_ANGLE_LIMIT, 0);

	if(ct == CT_TORQUE)
	{
		dxl_write_byte(id(), REG_TORQUE_CONTROL_EN, 1);
		dxl_write_word(id(), REG_GOAL_TORQUE, 0);
	}
	else
		dxl_write_byte(id(), REG_TORQUE_CONTROL_EN, 0);

	dxl_write_word(id(), 24, 1);   // torque enable
}

void DXLJointInterface::process()
{
	switch(controlType())
	{
		case CT_POSITION:
		{
			int goalTicks = m_goalPosition / TICKS_TO_RAD;
			dxl_write_word(id(), REG_GOAL_POSITION, goalTicks + m_tickOffset);
		}
			break;
		case CT_VELOCITY:
		{
			int velTicks = m_goalSpeed / VEL_TO_RAD_PER_S;
			if(velTicks < 0)
			{
				velTicks = (1 << 10) | (-velTicks);
			}
			dxl_write_word(id(), REG_MOVING_SPEED, velTicks);
		}
			break;
		case CT_TORQUE:
		{
			int torqueTicks = m_goalTorque / TORQUE_TO_NM;
			if(torqueTicks < 0)
			{
				torqueTicks = (1 << 10) | (-torqueTicks);
			}
			if(torqueTicks == 0)
				torqueTicks = 1;
			dxl_write_word(id(), REG_GOAL_TORQUE, torqueTicks);
		}
			break;
	}

// 	int torqueTicks = 1023.0 * (m_torqueLimit / 4.4); // at 12V
// 	if(torqueTicks > 1023)
// 		torqueTicks = 1023;
// 	printf("torque: %4d\n", torqueTicks);
//
// 	dxl_write_word(id(), 34, torqueTicks);

	dxl_set_txpacket_id(id());
	dxl_set_txpacket_instruction(INST_READ);
	dxl_set_txpacket_length(2 + 2);
	dxl_set_txpacket_parameter(0, 36); // start address: present pos
	dxl_set_txpacket_parameter(1, 6);  // #bytes

	dxl_txrx_packet();

	if(dxl_get_rxpacket_length() != 6+2)
	{
		fprintf(stderr, "DXL: Got malformed read answer\n");
		return;
	}

	int currentTicks = (dxl_get_rxpacket_parameter(1) << 8) | dxl_get_rxpacket_parameter(0);
	m_currentPosition = TICKS_TO_RAD * (currentTicks - m_tickOffset);

	double lastVel = m_currentVelocity;
	int velTicks = (dxl_get_rxpacket_parameter(3) << 8) | dxl_get_rxpacket_parameter(2);
	if(velTicks & (1 << 10))
		velTicks = -(velTicks & 1023);
	m_currentVelocity = VEL_TO_RAD_PER_S * velTicks;

	int loadTicks = (dxl_get_rxpacket_parameter(5) << 8) | dxl_get_rxpacket_parameter(4);
	if(loadTicks & (1 << 10))
		loadTicks = -(loadTicks & 1023);
	m_currentLoad = (1.0 / 1023.0) * loadTicks;

	int current = dxl_read_word(id(), 68) - 2048;
	m_currentCurrent = 0.0045 * current;

	m_currentAcc = (m_currentVelocity - lastVel) / 0.02;
}

double DXLJointInterface::currentPosition() const
{
	return m_currentPosition;
}

double DXLJointInterface::currentVelocity() const
{
	return m_currentVelocity;
}

double DXLJointInterface::currentCurrent() const
{
	return m_currentCurrent;
}

double DXLJointInterface::currentLoad() const
{
	return m_currentLoad;
}

double DXLJointInterface::currentAcceleration() const
{
	return m_currentAcc;
}

double DXLJointInterface::currentTemperature()
{
    return dxl_read_byte(id(), REG_PRESENT_TEMPERATURE);
}

void DXLJointInterface::relax()
{
	if(controlType() == CT_TORQUE)
	{
		dxl_write_word(id(), REG_GOAL_TORQUE, 1);
		sleep(3);
	}

	dxl_write_word(id(), REG_TORQUE_ENABLE, 0);
	dxl_write_word(id(), REG_TORQUE_LIMIT, 1023);

	m_currentVelocity = 0;
	m_currentAcc = 0;
}

void DXLJointInterface::setPValue(int value)
{
	dxl_write_byte(id(), REG_P_GAIN, value);
}

double DXLJointInterface::maximumAngle() const
{
	return TICKS_TO_RAD * (4095 - m_tickOffset);
}

double DXLJointInterface::minimumAngle() const
{
	return TICKS_TO_RAD * (-m_tickOffset);
}


