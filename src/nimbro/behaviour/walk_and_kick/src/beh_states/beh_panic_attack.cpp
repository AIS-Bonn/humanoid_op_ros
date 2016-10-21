// Walk and kick behaviour state: Panic attack
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/beh_panic_attack.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// BehPanicAttack class
//

// Constructor
BehPanicAttack::BehPanicAttack(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void BehPanicAttack::handleActivation(bool nowActive)
{
}

// Execute function
void BehPanicAttack::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	// Calculate the direction of the goal we wish to score in
	float angleToScore = picut((SV.goalSign > 0 ? 0.0f : M_PI) - SV.robotPose.z());

	// Nod the robot's head up and down
	bool updown = ((((int) (2.0f*WBS.stateTime())) & 1) != 0);

	// Ensure we are halted and look in the direction that we are currently trying to score
	AV.halt = true;
	AV.GCV.setZero();
	AV.gazeYaw = coerceAbs(angleToScore, config.gazeYawAbsMax());
	AV.gazePitch = config.gazePitchMin() + (updown ? 0.0f : 0.15f);
	AV.doKick = false;
	AV.rightKick = true;
	AV.doDive = DD_NONE;
}
// EOF