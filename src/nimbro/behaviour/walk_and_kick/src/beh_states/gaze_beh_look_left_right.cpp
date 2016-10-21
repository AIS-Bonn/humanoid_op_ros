// Walk and kick gaze behaviour state: Look left right
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/gaze_beh_look_left_right.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// GazeBehLookLeftRight class
//

// Constructor
GazeBehLookLeftRight::GazeBehLookLeftRight(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GazeBehLookLeftRight::handleActivation(bool nowActive)
{
	// Reset variables
	m_gazeFreq = 0.0f;
	m_gazePhaseOff = 0.0f;
}

// Execute function
void GazeBehLookLeftRight::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Calculate the gaze oscillation parameters if we haven't yet
	if(m_gazeFreq <= 0.0f)
	{
		m_gazeFreq = coerce(config.llrGazeFreqScaler() * config.gazeVelLimit() / config.gazeYawAbsMax(), 0.5f, 5.0f);
		m_gazePhaseOff = asin(coerceAbs(lastAV.gazeYaw / config.gazeYawAbsMax(), 1.0f));
	}

	// Look left and right to try to see things on the field
	AV.gazeYaw = config.gazeYawAbsMax() * sin(m_gazeFreq*WBS.stateTime() + m_gazePhaseOff);
	AV.gazePitch = config.llrGazePitch();

	// Plotting
	if(config.plotData())
		PM.plotScalar(m_gazeFreq, PM_LLR_GAZEFREQ);
}
// EOF