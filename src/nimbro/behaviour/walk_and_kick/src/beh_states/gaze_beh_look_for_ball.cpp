// Walk and kick gaze behaviour state: Look for ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/gaze_beh_look_for_ball.h>

// Namespaces
using namespace walk_and_kick;

//
// GazeBehLookForBall class
//

// Constructor
GazeBehLookForBall::GazeBehLookForBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID), GazeBehLookAround(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GazeBehLookForBall::handleActivation(bool nowActive)
{
	// Handle the activation of the base class
	GazeBehLookAround::handleActivation(nowActive);

	// Reset variables
	m_haveBall = true;
}

// Execute function
void GazeBehLookForBall::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Gaze at the ball if you can see it, otherwise gaze to look for it
	if(SV.haveBall)
	{
		// Gaze at the ball
		WBS.gazeAtBall(AV, lastAV);

		// Reset any gaze spline that may have been in progress
		resetGazeSpline();

		// Indicate that we have seen the ball
		m_haveBall = true;
	}
	else
	{
		// Look around using the base class behaviour
		GazeBehLookAround::execute(AV, lastAV, (justActivated || m_haveBall));

		// Indicate that we have lost the ball
		m_haveBall = false;
	}
}
// EOF