// Header include for the limb control PlayCommands service
// Author: Philipp Allgeuer

// Ensure header is only included once
#ifndef PLAYCOMMANDSSRV_H
#define PLAYCOMMANDSSRV_H

// Includes
#include <limb_control/PlayCommands.h>

// Limbcontrol namespace
namespace limb_control
{
	// Enumerations
	enum LimbCmdType
	{
		TYPE_INVALID = 0,
		TYPE_SETPOINT_TIME,
		TYPE_SETPOINT_VEL,
		TYPE_SINE_WAVE,
		NUM_TYPES
	};

	// Constants
	static const LimbCommand::_timeref_type INVALID_TIME_REF = 0;
	static const double M_2PI = 6.2831853071795864769;
}

#endif /* PLAYCOMMANDSSRV_H */
// EOF