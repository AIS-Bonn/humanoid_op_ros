// Walk and kick walk behaviour state: Walk to pose
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/walk_beh_walk_to_pose.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// WalkBehWalkToPose class
//

// Constructor
WalkBehWalkToPose::WalkBehWalkToPose(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());

	// Config parameter callbacks
	config.wtpArrivedWormTime.setCallback(boost::bind(&TheWorm::updateWormTime, &m_arrivedWorm, &config.wtpArrivedWormTime), true);
}

// Handle activation function
void WalkBehWalkToPose::handleActivation(bool nowActive)
{
	// Reset variables
	m_arrived = false;
	m_arrivedWorm.reset();
}

// Execute function
void WalkBehWalkToPose::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Walking control
	//

	// Compute the required GCV and vote whether we have arrived at our target
	float distCost = 0.0f;
	if(GV.targetPoseValid)
	{
		// Decide on a walking target tolerance
		float walkingTargetTol = (GV.targetPoseTol >= 0.0f ? GV.targetPoseTol : config.wtpArrivedCostMaxDefault());

		// Set a GCV suitable for walking to the target pose and check whether we have arrived at our destination
		float distCost = WBS.walkToGlobalPose(AV, GV.targetPose.x(), GV.targetPose.y(), GV.targetPose.z()); // Note: The return value is actually a virtual distance 'cost' that is a linear combination of distance and orientation errors
		WBS.setWalkingTargetTol(walkingTargetTol);
		m_arrivedWorm.vote(distCost < walkingTargetTol);
	}
	else
	{
		// No valid target pose, so slow down and double-vote that we've arrived
		AV.GCV.setZero();
		m_arrivedWorm.vote(true, 2);

		// Set the walking target
		WBS.setWalkingTarget(Vec2f::Zero());
	}

	// See whether we have arrived at our target or not
	if(m_arrivedWorm.unanimous())
		m_arrived = m_arrivedWorm.decision();

	// We wish to halt if we have arrived
	AV.halt = m_arrived;
	AV.doKick = false;
	AV.rightKick = true;
	AV.doDive = DD_NONE;

	// Print info about the walk to pose state
	if(config.debugMsgWTP() && WBS.stateCycle() % 20 == 1)
	{
		printf("WTP: %s(%.2f, %.2f, %.2f) %s(%.2f, %.2f, %.2f) %s(%.2f) COUNT(%d) GCV(%.2f, %.2f, %.2f)\n",
		       (GV.targetPoseValid ? "TARGET" : "target"), GV.targetPose.x(), GV.targetPose.y(), GV.targetPose.z(), (SV.haveRobotPose ? "POSE" : "pose"),
		       SV.robotPose.x(), SV.robotPose.y(), SV.robotPose.z(), (m_arrivedWorm.decision() ? "DIST" : "dist"), distCost, m_arrivedWorm.count(), AV.GCV.x(), AV.GCV.y(), AV.GCV.z());
	}

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(distCost, PM_WTP_DISTCOST);
		PM.plotScalar(m_arrived * PMSCALE_ARRIVED, PM_WTP_ARRIVED);
	}
}
// EOF