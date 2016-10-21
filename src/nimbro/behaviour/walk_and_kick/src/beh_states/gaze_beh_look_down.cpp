// Walk and kick gaze behaviour state: Look down
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/gaze_beh_look_down.h>

// Namespaces
using namespace walk_and_kick;

//
// GazeBehLookDown class
//

// Constructor
GazeBehLookDown::GazeBehLookDown(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GazeBehLookDown::handleActivation(bool nowActive)
{
}

// Execute function
void GazeBehLookDown::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Gaze down
	AV.gazePitch = config.gazePitchMax();
	AV.gazeYaw = 0.0f;
}
// EOF