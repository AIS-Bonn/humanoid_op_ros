// Walk and kick behaviour state: Dive for ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/beh_dive_for_ball.h>

// Namespaces
using namespace walk_and_kick;

//
// BehDiveForBall class
//

// Constructor
BehDiveForBall::BehDiveForBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID), GazeBehLookForBall(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void BehDiveForBall::handleActivation(bool nowActive)
{
	// Handle activation of the base class
	GazeBehLookForBall::handleActivation(nowActive);

	// Reset variables
	m_diveLock = false;
}

// Execute function
void BehDiveForBall::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Look for the ball
	GazeBehLookForBall::execute(AV, lastAV, justActivated);

	//
	// Walking control
	//

	// Stop walking and don't do anything else
	AV.halt = true;
	AV.GCV.setZero();
	AV.doKick = false;
	AV.rightKick = true;

	//
	// Diving control
	//

	// Decide whether we are in a state yet where we can command the dive
	if(!diveDirectionValid(GV.diveIfPossible) || GV.diveIfPossible == DD_NONE)
	{
		AV.doDive = DD_NONE;
		m_diveLock = false;
	}
	else if(SV.isWalking())
	{
		AV.doDive = DD_NONE;
		m_diveLock = true;
	}
	else if(SV.isStanding())
	{
		AV.doDive = GV.diveIfPossible;
		m_diveLock = true;
	}
	else
	{
		AV.doDive = DD_NONE;
		m_diveLock = false;
	}

	// Plotting
	if(config.plotData())
		PM.plotScalar(m_diveLock * PMSCALE_LOCK, PM_DFB_DIVELOCK);

	// Visualisation markers
	if(MM.willPublish())
	{
		if(m_diveLock || (!SV.isWalking() && !SV.isStanding()))
		{
			MM.SubStateText.setText(diveDirectionName(GV.diveIfPossible));
			MM.SubStateText.updateAdd();
			const float arrowSize = 0.5f;
			MM.KBKickVector.setPoint(0, 0.0, 0.0);
			if(GV.diveIfPossible == DD_NONE)
				MM.KBKickVector.setPoint(1, 0.0, 0.01);
			else if(GV.diveIfPossible == DD_LEFT)
				MM.KBKickVector.setPoint(1, 0.0, arrowSize);
			else if(GV.diveIfPossible == DD_RIGHT)
				MM.KBKickVector.setPoint(1, 0.0, -arrowSize);
			else if(GV.diveIfPossible == DD_SIT)
				MM.KBKickVector.setPoint(1, arrowSize, 0.0);
			else
				MM.KBKickVector.setPoint(1, -arrowSize, 0.0);
			MM.KBKickVector.show();
		}
	}
}
// EOF