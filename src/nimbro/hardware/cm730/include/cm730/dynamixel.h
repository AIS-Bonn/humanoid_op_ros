// Definitions for the MX series of Dynamixel servos
// File: dynamixel.h

// Ensure header is included only once
#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

// DynamixelMX class
class DynamixelMX
{
public:
	// MX series register map
	enum
	{
		P_MODEL_NUMBER_L        = 0,
		P_MODEL_NUMBER_H        = 1,
		P_VERSION               = 2,
		P_ID                    = 3,
		P_BAUD_RATE             = 4,
		P_RETURN_DELAY_TIME     = 5,
		P_CW_ANGLE_LIMIT_L      = 6,
		P_CW_ANGLE_LIMIT_H      = 7,
		P_CCW_ANGLE_LIMIT_L     = 8,
		P_CCW_ANGLE_LIMIT_H     = 9,
		P_LIMIT_TEMPERATURE     = 11,
		P_VOLTAGE_LOWER_LIMIT   = 12,
		P_VOLTAGE_UPPER_LIMIT   = 13,
		P_MAX_TORQUE_L          = 14,
		P_MAX_TORQUE_H          = 15,
		P_RETURN_LEVEL          = 16,
		P_ALARM_LED             = 17,
		P_ALARM_SHUTDOWN        = 18,
		P_TORQUE_ENABLE         = 24,
		P_LED                   = 25,
		P_D_GAIN                = 26,
		P_I_GAIN                = 27,
		P_P_GAIN                = 28,
		P_GOAL_POSITION_L       = 30,
		P_GOAL_POSITION_H       = 31,
		P_MOVING_SPEED_L        = 32,
		P_MOVING_SPEED_H        = 33,
		P_TORQUE_LIMIT_L        = 34,
		P_TORQUE_LIMIT_H        = 35,
		P_PRESENT_POSITION_L    = 36,
		P_PRESENT_POSITION_H    = 37,
		P_PRESENT_SPEED_L       = 38,
		P_PRESENT_SPEED_H       = 39,
		P_PRESENT_LOAD_L        = 40,
		P_PRESENT_LOAD_H        = 41,
		P_PRESENT_VOLTAGE       = 42,
		P_PRESENT_TEMPERATURE   = 43,
		P_REGISTERED            = 44,
		P_MOVING                = 46,
		P_LOCK                  = 47,
		P_PUNCH_L               = 48,
		P_PUNCH_H               = 49,
		P_CURRENT_L             = 68,
		P_CURRENT_H             = 69,
		P_TORQUE_CONTROL_ENABLE = 70,
		P_GOAL_TORQUE_L         = 71,
		P_GOAL_TORQUE_H         = 72,
		P_GOAL_ACCELERATION     = 73,
		MX_REGISTER_NUM
	};
};

#endif /* DYNAMIXEL_H */
// EOF