// Walk and kick behaviour state: Unknown state
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/beh_unknown_state.h>

// Namespaces
using namespace walk_and_kick;

//
// BehUnknownState class
//

// Constructor
BehUnknownState::BehUnknownState(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void BehUnknownState::handleActivation(bool nowActive)
{
	// Display a warning message if the unknown state is activated
	if(nowActive)
		ROS_WARN("The %s behaviour state has been activated => This should never happen!", name().c_str());
}

// Execute function
void BehUnknownState::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	// Display a warning message if the unknown state is executed
	ROS_WARN_THROTTLE(0.4, "The %s behaviour state is being executed => This should never happen!", name().c_str());

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