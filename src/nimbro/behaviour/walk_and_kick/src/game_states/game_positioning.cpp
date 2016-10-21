// Walk and kick game state: Positioning
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_positioning.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// GamePositioning class
//

// Constructor
GamePositioning::GamePositioning(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : WAKGameState(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GamePositioning::handleActivation(bool nowActive)
{
	// Reset variables
	m_specialTarget.clear();
}

// Execute function
void GamePositioning::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
	// Initialise the required behaviour state
	int reqBehState = WAKBehManager::BS_WALK_TO_POSE_LOOK_AROUND;

	// Calculate the desired pose to be in for the current kickoff
	Vec3f targetPose = WGS.getKickoffPoseTarget();

	// Calculate an appropriate pose target if we are in the set play state
	if(SV.playState == PS_SET)
	{
		reqBehState = WAKBehManager::BS_WALK_TO_POSE_LOOK_FOR_BALL;
		if(m_specialTarget.hasExpired(config.targetPoseReevalTime(), SV.now) || SV.robotPoseIsNew)
			setTargetToQuickLegalPose();
	}
	else
		m_specialTarget.clear();

	// Override the target with the special positioning target if one exists
	if(m_specialTarget.exists())
		targetPose = m_specialTarget.value();

	// Force a common target global pose if required
	if(config.posCommonTargetEnable())
	{
		reqBehState = WAKBehManager::BS_WALK_TO_POSE_LOOK_AROUND;
		switch(abs(config.posCommonTargetX()))
		{
			default: case 0: targetPose.x() = 0.0f; break;
			case 1: targetPose.x() = sign(config.posCommonTargetX()) * field.circleRadius(); break;
			case 2: targetPose.x() = sign(config.posCommonTargetX()) * (field.fieldLengthH() - field.penaltyMarkDist()); break;
			case 3: targetPose.x() = sign(config.posCommonTargetX()) * (field.fieldLengthH() - field.goalAreaLength()); break;
		}
		switch(abs(config.posCommonTargetY()))
		{
			default: case 0: targetPose.y() = 0.0f; break;
			case 1: targetPose.y() = sign(config.posCommonTargetY()) * field.circleRadius(); break;
			case 2: targetPose.y() = sign(config.posCommonTargetY()) * field.fieldWidthH(); break;
		}
		targetPose.z() = picut(M_PI_4 * config.posCommonTargetRot());
	}

	// Command the robot to perform positioning to the required target pose
	GV.forceBehStateByID = reqBehState;
	GV.suggestFoot = GameVars::FS_EITHER_FOOT;
	GV.dribbleIfPossible = false;
	GV.kickIfPossible = false;
	GV.diveIfPossible = DD_NONE;
	GV.ballTargetConf = 0.0f;
	GV.ballTargetDir.setZero();
	GV.ballTargetWedge = 0.0f;
	GV.ballTargetType = GameVars::BTT_UNKNOWN;
	GV.targetPose = targetPose;
	GV.targetPoseTol = config.posArrivedCostMax();
	GV.targetPoseValid = true;
}

// Set a special positioning target that promotes getting to a legal pose quickly
void GamePositioning::setTargetToQuickLegalPose()
{
	// Decide on how far inside our own territory we should aim for
	float R = field.circleRadius();
	float D = config.posPoseLegalityBuffer() + config.posArrivedCostMax();

	// Calculate and set the special target as required
	if(!SV.haveRobotPose)
		m_specialTarget.set(WGS.applyGoalSign(Vec3f(0.0f, 0.0f, 0.0f)), SV.now); // Set the target to the centre of the field if we are not localised
	else if(SV.kickoffType == KT_MANUAL)
		m_specialTarget.set(SV.robotPose, SV.now);
	else
	{
		Vec3f targetPoseGS;
		Vec3f robotPoseGS = WGS.applyGoalSign(SV.robotPose);
		float maxY = sqrt(coerceMin(R*(R + 2.0f*D), 0.0f)); // Mathematically we must have R < maxY < R + D, assuming D > 0
		if(SV.kickoffType == KT_ATTACKING)
		{
			targetPoseGS.x() = -D;
			targetPoseGS.y() = coerceAbs(robotPoseGS.y(), maxY);
		}
		else // If KT_DEFENDING or KT_DROPBALL
		{
			float RD = R + D;
			float absY = fabs(robotPoseGS.y());
			float signY = sign(robotPoseGS.y());
			float minY = interpolateCoerced(-D, RD, 0.0f, maxY, robotPoseGS.x());
			targetPoseGS.y() = coerceAbs(signY * interpolateCoerced(0.0f, maxY, minY, maxY, absY), RD); // In absolute y terms, a robot pose of [0,maxY] is mapped linearly to a target pose [minY,maxY]
			targetPoseGS.x() = -std::max<float>(sqrt(RD*RD - targetPoseGS.y()*targetPoseGS.y()), D);
		}
		Vec2f robotToTargetVec = (targetPoseGS - robotPoseGS).head<2>();
		float zForCentre = eigenAngleOf<float, 3>(-targetPoseGS);
		float zForTargetDirn = eigenAngleOf(robotToTargetVec);
		float distU = interpolateCoerced(config.posArrivedCostMax(), D, 0.0f, 1.0f, robotToTargetVec.norm());
		targetPoseGS.z() = picut(zForCentre + distU*picut(zForTargetDirn - zForCentre));
		m_specialTarget.set(WGS.applyGoalSign(targetPoseGS), SV.now);
	}
}
// EOF