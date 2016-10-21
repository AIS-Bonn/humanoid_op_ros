// Walk and kick game state: Wait for ball in play
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_wait_for_ball_in_play.h>
#include <walk_and_kick/wak_utils.h>
#include <rc_utils/math_vec_mat.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// GameWaitForBallInPlay class
//

// Constructor
GameWaitForBallInPlay::GameWaitForBallInPlay(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : WAKGameState(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GameWaitForBallInPlay::handleActivation(bool nowActive)
{
	// Reset variables
	m_targetPose.clear();
}

// Execute function
void GameWaitForBallInPlay::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
	// Calculate a new target walking pose if the current one is up for review
	if(m_targetPose.hasExpired(config.targetPoseReevalTime(), SV.now) || SV.robotPoseIsNew)
	{
		// Calculate the desired point to be at for the current kickoff
		Vec2f targetPose = WGS.getKickoffPoseTarget().head<2>();

		// If we are localised then adjust the target pose to avoid walking into the centre circle
		if(SV.haveRobotPose)
		{
			// Retrieve the 2D robot pose
			Vec2f robotPose = SV.robotPose2D;

			// Calculate the minimum of the relevant radii from the centre
			float radiusTargetPose = targetPose.norm();
			float radiusIllegal = field.circleRadius() + config.posPoseLegalityBuffer();
			float radiusRobotPose = robotPose.norm();
			float radiusHalo = coerceMin(std::min(radiusTargetPose, std::min(radiusIllegal, radiusRobotPose)), 0.0f);

			// Calculate a halo path around the centre halo from the robot pose to the target pose
			Vec2fArray path;
			Vec2f walkDirnNormal;
			float pathLen = WAKUtils::calculateHaloPath(walkDirnNormal, path, robotPose, targetPose, Vec2f::Zero(), radiusHalo, true);
			bool isDirect = (path.size() <= 2);

			// Adjust the walking direction for illegal robots
			if(!isDirect)
			{
				float angleAdjustMag = interpolateCoerced<float>(0.0f, radiusIllegal, M_PI_2, 0.0f, radiusRobotPose);
				float angleAdjust = angleAdjustMag * sign(walkDirnNormal.x() * robotPose.y() - robotPose.x() * walkDirnNormal.y());
				eigenRotateCCW(walkDirnNormal, angleAdjust);
			}

			// Create a fake target pose that is direct line of sight for the robot
			targetPose = robotPose + pathLen * walkDirnNormal;
		}

		// Calculate the target direction to face the centre
		float zForCentre = eigenAngleOf<float, 2>(-targetPose);

		// TODO: RoboCup hack to avoid problems if possible when starting with a bad localisation
		if(SV.haveBallPose && SV.haveRobotPose && SV.ballPose.norm() > 1.2f)
		{
			Vec2f robotToBallUnitVec = SV.ballPose - SV.robotPose2D;
			eigenNormalize(robotToBallUnitVec);
			targetPose = SV.ballPose - 1.0f*robotToBallUnitVec;
			zForCentre = (SV.goalSign > 0 ? 0.0 : M_PI);
		}

		// Set the newly calculated target pose
		m_targetPose.set(withZ(targetPose, zForCentre), SV.now);
	}

	// Command the robot to perform positioning to the required target pose
	GV.forceBehStateByID = WAKBehManager::BS_WALK_TO_POSE_LOOK_FOR_BALL;
	GV.suggestFoot = GameVars::FS_EITHER_FOOT;
	GV.dribbleIfPossible = false;
	GV.kickIfPossible = false;
	GV.diveIfPossible = DD_NONE;
	GV.ballTargetConf = 0.0f;
	GV.ballTargetDir.setZero();
	GV.ballTargetWedge = 0.0f;
	GV.ballTargetType = GameVars::BTT_UNKNOWN;
	GV.targetPose = m_targetPose.value();
	GV.targetPoseTol = config.posArrivedCostMax();
	GV.targetPoseValid = true;
}
// EOF