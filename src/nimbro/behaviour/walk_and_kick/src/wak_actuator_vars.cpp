// Walk and kick: Actuator variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_actuator_vars.h>

// Namespaces
using namespace walk_and_kick;

//
// ActuatorVars class
//

// Reset function
void ActuatorVars::reset()
{
	// Reset the data members
	halt = true;
	GCV.setZero();
	doKick = false;
	rightKick = true;
	doDive = DD_NONE;
	gazeYaw = 0.0f;
	gazePitch = 0.0f;
}
// EOF