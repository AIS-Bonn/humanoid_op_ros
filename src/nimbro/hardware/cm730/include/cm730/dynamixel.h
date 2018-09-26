// Definitions for the MX series of Dynamixel servos
// File: dynamixel.h

// Ensure header is included only once
#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

// Includes
#include <string>

// CM730 namespace
namespace cm730
{
	// Dynamixel base class
	class DynamixelBase
	{
	public:
		// Servo type enumeration
		enum ServoType
		{
			MX_SERVOS,
			X_SERVOS
		};

		// Constructor/destructor
		DynamixelBase(ServoType type) : type(type) {};
		virtual ~DynamixelBase() = default;

		// Get functions (Note: All addresses and lengths should be in the range of a single unsigned byte, i.e. 0-255)
		virtual std::string name() const = 0;
		virtual int numRegisters() const = 0;
		virtual int readAddress() const = 0;
		virtual int readLength() const = 0;
		virtual int prodAddress() const = 0;         // Must be a 1 byte register
		virtual int prodValue() const = 0;           // Must be a 1 byte value
		virtual int addressPresentPos() const = 0;   // Must be a 2 byte register
		virtual int addressReturnLevel() const = 0;  // Must be a 1 byte register
		virtual int addressTorqueEnable() const = 0; // Must be a 1 byte register
		virtual int addressTorqueLimit() const = 0;  // Must be a 2 byte register
		virtual int fullTorque() const = 0;
		virtual int fullEffort() const = 0;

		// Data members
		const ServoType type;
	};

	// DynamixelMX class
	class DynamixelMX : public DynamixelBase
	{
	public:
		// Constructor/destructor
		DynamixelMX() : DynamixelBase(MX_SERVOS) {}
		virtual ~DynamixelMX() = default;

		// Get functions
		virtual std::string name() const override { return "MX"; }
		virtual int numRegisters() const override { return NUM_REGISTERS; }
		virtual int readAddress() const override { return P_PRESENT_POSITION_L; }
		virtual int readLength() const override { return P_PRESENT_POSITION_H - P_PRESENT_POSITION_L + 1; }
		virtual int prodAddress() const override { return P_ALARM_SHUTDOWN; }
		virtual int prodValue() const override { return 0x24; }
		virtual int addressPresentPos() const override { return P_PRESENT_POSITION_L; }
		virtual int addressReturnLevel() const override { return P_RETURN_LEVEL; }
		virtual int addressTorqueEnable() const override { return P_TORQUE_ENABLE; }
		virtual int addressTorqueLimit() const override { return P_TORQUE_LIMIT_L; }
		virtual int fullTorque() const override { return 1023; } // The torque limit value that corresponds to full torque
		virtual int fullEffort() const override { return 32; } // The P gain value that an effort of 1.0 corresponds to

		// Joint command sync write data: The memory layout reflects the actual layout in the message and in the Dynamixel hardware, hence the __attribute__((packed)).
		struct JointCmdSyncWriteData
		{
			uint8_t id;             //!< Servo ID
			uint8_t p_gain;         //!< P_P_GAIN
			uint8_t nothing;        //!< Unused register
			uint16_t goal_position; //!< P_GOAL_POSITION
		} __attribute__((packed));

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
			NUM_REGISTERS
		};
	};

	// DynamixelX class
	class DynamixelX : public DynamixelBase
	{
	public:
		// Constructor/destructor
		DynamixelX() : DynamixelBase(X_SERVOS) {}
		virtual ~DynamixelX() = default;

		// Get functions
		virtual std::string name() const override { return "X"; }
		virtual int numRegisters() const override { return NUM_REGISTERS; }
		virtual int readAddress() const override { return P_PRESENT_POSITION_0; }
		virtual int readLength() const override { return P_PRESENT_POSITION_1 - P_PRESENT_POSITION_0 + 1; }
		virtual int prodAddress() const override { return P_SHUTDOWN; }
		virtual int prodValue() const override { return 0x24; }
		virtual int addressPresentPos() const override { return P_PRESENT_POSITION_0; }
		virtual int addressReturnLevel() const override { return P_RETURN_LEVEL; }
		virtual int addressTorqueEnable() const override { return P_TORQUE_ENABLE; }
		virtual int addressTorqueLimit() const override { return P_GOAL_PWM_L; }
		virtual int fullTorque() const override { return 885; } // The PWM limit value that corresponds to full PWM
		virtual int fullEffort() const override { return 628; } // The P gain value that an effort of 1.0 corresponds to

		// Joint command sync write data (P gain): The memory layout reflects the actual layout in the message and in the Dynamixel hardware, hence the __attribute__((packed)).
		struct JointCmdSyncWriteDataPG
		{
			uint8_t id;             //!< Servo ID
			uint16_t p_gain;        //!< P_POSITION_P_GAIN
		} __attribute__((packed));

		// Joint command sync write data (goal position): The memory layout reflects the actual layout in the message and in the Dynamixel hardware, hence the __attribute__((packed)).
		struct JointCmdSyncWriteDataGP
		{
			uint8_t id;             //!< Servo ID
			uint32_t goal_position; //!< P_GOAL_POSITION
		} __attribute__((packed));

		// X series register map
		enum
		{
			P_MODEL_NUMBER_L         = 0,
			P_MODEL_NUMBER_H         = 1,
			P_VERSION                = 2,
			P_ID                     = 7,
			P_BAUD_RATE              = 8,
			P_RETURN_DELAY_TIME      = 9,
			P_DRIVE_MODE             = 10,
			P_OPERATING_MODE         = 11,
			P_SECONDARY_ID           = 12,
			P_PROTOCOL_VERSION       = 13,
			P_HOMING_OFFSET_0        = 20,
			P_HOMING_OFFSET_1        = 21,
			P_HOMING_OFFSET_2        = 22,
			P_HOMING_OFFSET_3        = 23,
			P_MOVING_THRESHOLD_0     = 24,
			P_MOVING_THRESHOLD_1     = 25,
			P_MOVING_THRESHOLD_2     = 26,
			P_MOVING_THRESHOLD_3     = 27,
			P_TEMPERATURE_LIMIT      = 31,
			P_MAX_VOLTAGE_LIMIT_L    = 32,
			P_MAX_VOLTAGE_LIMIT_H    = 33,
			P_MIN_VOLTAGE_LIMIT_L    = 34,
			P_MIN_VOLTAGE_LIMIT_H    = 35,
			P_PWM_LIMIT_L            = 36,
			P_PWM_LIMIT_H            = 37,
			P_CURRENT_LIMIT_L        = 38,
			P_CURRENT_LIMIT_H        = 39,
			P_ACCELERATION_LIMIT_0   = 40,
			P_ACCELERATION_LIMIT_1   = 41,
			P_ACCELERATION_LIMIT_2   = 42,
			P_ACCELERATION_LIMIT_3   = 43,
			P_VELOCITY_LIMIT_0       = 44,
			P_VELOCITY_LIMIT_1       = 45,
			P_VELOCITY_LIMIT_2       = 46,
			P_VELOCITY_LIMIT_3       = 47,
			P_MAX_POSITION_LIMIT_0   = 48,
			P_MAX_POSITION_LIMIT_1   = 49,
			P_MAX_POSITION_LIMIT_2   = 50,
			P_MAX_POSITION_LIMIT_3   = 51,
			P_MIN_POSITION_LIMIT_0   = 52,
			P_MIN_POSITION_LIMIT_1   = 53,
			P_MIN_POSITION_LIMIT_2   = 54,
			P_MIN_POSITION_LIMIT_3   = 55,
			P_EXTERNAL_PORT_MODE_1   = 56,
			P_EXTERNAL_PORT_MODE_2   = 57,
			P_EXTERNAL_PORT_MODE_3   = 58,
			P_SHUTDOWN               = 63,
			P_TORQUE_ENABLE          = 64,
			P_LED                    = 65,
			P_RETURN_LEVEL           = 68,
			P_REGISTERED_INSTR       = 69,
			P_HARDWARE_ERROR_STATUS  = 70,
			P_VELOCITY_I_GAIN_L      = 76,
			P_VELOCITY_I_GAIN_H      = 77,
			P_VELOCITY_P_GAIN_L      = 78,
			P_VELOCITY_P_GAIN_H      = 79,
			P_POSITION_D_GAIN_L      = 80,
			P_POSITION_D_GAIN_H      = 81,
			P_POSITION_I_GAIN_L      = 82,
			P_POSITION_I_GAIN_H      = 83,
			P_POSITION_P_GAIN_L      = 84,
			P_POSITION_P_GAIN_H      = 85,
			P_FEEDFWD_2ND_GAIN_L     = 88,
			P_FEEDFWD_2ND_GAIN_H     = 89,
			P_FEEDFWD_1ST_GAIN_L     = 90,
			P_FEEDFWD_1ST_GAIN_H     = 91,
			P_BUS_WATCHDOG           = 98,
			P_GOAL_PWM_L             = 100,
			P_GOAL_PWM_H             = 101,
			P_GOAL_CURRENT_L         = 102,
			P_GOAL_CURRENT_H         = 103,
			P_GOAL_VELOCITY_0        = 104,
			P_GOAL_VELOCITY_1        = 105,
			P_GOAL_VELOCITY_2        = 106,
			P_GOAL_VELOCITY_3        = 107,
			P_PROFILE_ACCELERATION_0 = 108,
			P_PROFILE_ACCELERATION_1 = 109,
			P_PROFILE_ACCELERATION_2 = 110,
			P_PROFILE_ACCELERATION_3 = 111,
			P_PROFILE_VELOCITY_0     = 112,
			P_PROFILE_VELOCITY_1     = 113,
			P_PROFILE_VELOCITY_2     = 114,
			P_PROFILE_VELOCITY_3     = 115,
			P_GOAL_POSITION_0        = 116,
			P_GOAL_POSITION_1        = 117,
			P_GOAL_POSITION_2        = 118,
			P_GOAL_POSITION_3        = 119,
			P_REALTIME_TICK_L        = 120,
			P_REALTIME_TICK_H        = 121,
			P_MOVING                 = 122,
			P_MOVING_STATUS          = 123,
			P_PRESENT_PWM_L          = 124,
			P_PRESENT_PWM_H          = 125,
			P_PRESENT_CURRENT_L      = 126,
			P_PRESENT_CURRENT_H      = 127,
			P_PRESENT_VELOCITY_0     = 128,
			P_PRESENT_VELOCITY_1     = 129,
			P_PRESENT_VELOCITY_2     = 130,
			P_PRESENT_VELOCITY_3     = 131,
			P_PRESENT_POSITION_0     = 132,
			P_PRESENT_POSITION_1     = 133,
			P_PRESENT_POSITION_2     = 134,
			P_PRESENT_POSITION_3     = 135,
			P_VELOCITY_TRAJECTORY_0  = 136,
			P_VELOCITY_TRAJECTORY_1  = 137,
			P_VELOCITY_TRAJECTORY_2  = 138,
			P_VELOCITY_TRAJECTORY_3  = 139,
			P_POSITION_TRAJECTORY_0  = 140,
			P_POSITION_TRAJECTORY_1  = 141,
			P_POSITION_TRAJECTORY_2  = 142,
			P_POSITION_TRAJECTORY_3  = 143,
			P_PRESENT_VOLTAGE_L      = 144,
			P_PRESENT_VOLTAGE_H      = 145,
			P_PRESENT_TEMPERATURE    = 146,
			P_EXTERNAL_PORT_DATA_1_L = 152,
			P_EXTERNAL_PORT_DATA_1_H = 153,
			P_EXTERNAL_PORT_DATA_2_L = 154,
			P_EXTERNAL_PORT_DATA_2_H = 155,
			P_EXTERNAL_PORT_DATA_3_L = 156,
			P_EXTERNAL_PORT_DATA_3_H = 157,
			NUM_REGISTERS
		};
	};
}

#endif /* DYNAMIXEL_H */
// EOF
