// Walk and kick gaze behaviour state: Look at ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/gaze_beh_look_at_ball.h>

// Namespaces
using namespace walk_and_kick;

//
// GazeBehLookAtBall class
//

// Constructor
GazeBehLookAtBall::GazeBehLookAtBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GazeBehLookAtBall::handleActivation(bool nowActive)
{
}

// Execute function
void GazeBehLookAtBall::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Gaze at the ball
	WBS.gazeAtBall(AV, lastAV);
}
// EOF