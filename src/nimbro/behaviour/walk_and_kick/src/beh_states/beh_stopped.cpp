// Walk and kick behaviour state: Stopped
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/beh_stopped.h>

// Namespaces
using namespace walk_and_kick;

//
// BehStopped class
//

// Constructor
BehStopped::BehStopped(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void BehStopped::handleActivation(bool nowActive)
{
}

// Execute function
void BehStopped::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	// Ensure we are halted and looking straight ahead
	AV.halt = true;
	AV.GCV.setZero();
	AV.gazeYaw = 0.0f;
	AV.gazePitch = config.gazePitchNeutral();
	AV.doKick = false;
	AV.rightKick = true;
	AV.doDive = DD_NONE;
}
// EOF