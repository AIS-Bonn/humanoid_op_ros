// Walk and kick game state: Default goalie
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_default_goalie.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// GameDefaultGoalie class
//

// Constructor
GameDefaultGoalie::GameDefaultGoalie(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : GameDefaultBallHandling(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GameDefaultGoalie::handleActivation(bool nowActive)
{
	// Reset the state variables
	m_cmdTarget.clear();
	m_reqBehState = WAKBehManager::BS_WALK_TO_POSE_LOOK_FOR_BALL;
}

// Execute function
void GameDefaultGoalie::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
	float scoringDist = INFINITY;
	if(SV.haveBallPose)
	{
		Vec2f ballPoseGS = SV.goalSign * SV.ballPose;
		float ballPoseGSYAbs = fabs(ballPoseGS.y());
		float ballDistFromGoalLine = ballPoseGS.x() + field.fieldLengthH();
		float ballDistOutsideGoalPost = ballPoseGSYAbs - field.goalWidthH();
		if(ballDistOutsideGoalPost > 0.0f)
			scoringDist = sqrt(ballDistFromGoalLine*ballDistFromGoalLine + ballDistOutsideGoalPost*ballDistOutsideGoalPost);
		else
			scoringDist = ballDistFromGoalLine;
	}

	float nominalX = SV.goalSign * (config.posPoseGoalieX() - field.fieldLengthH());

	Vec3f targetPose;
	targetPose.x() = nominalX;
	targetPose.y() = 0.0f;
	targetPose.z() = (SV.goalSign > 0 ? 0.0f : M_PI);

	m_cmdTarget.set(targetPose, SV.now); // TODO: Hack to always just take the current target pose
	m_reqBehState = WAKBehManager::BS_LOOK_DOWN; // TODO: Hack to always look down

	DiveDirection diveIfPossible = DD_NONE;
	if(diveDirectionValid(SV.diveDecision) && SV.diveDecision != DD_NONE)
	{
		diveIfPossible = SV.diveDecision;
		m_reqBehState = WAKBehManager::BS_DIVE_FOR_BALL;
	}

	// Command the robot to walk to the required pose
	GV.forceBehStateByID = m_reqBehState;
	GV.suggestFoot = GameVars::FS_EITHER_FOOT;
	GV.dribbleIfPossible = false;
	GV.kickIfPossible = false;
	GV.diveIfPossible = diveIfPossible;
	GV.ballTargetConf = 0.0f;
	GV.ballTargetDir.setZero();
	GV.ballTargetWedge = 0.0f;
	GV.ballTargetType = GameVars::BTT_UNKNOWN;
	GV.targetPose = m_cmdTarget.value();
	GV.targetPoseTol = config.dgArrivedCostMax();
	GV.targetPoseValid = true;

	// Plotting
	if(config.plotData())
		PM.plotScalar(scoringDist, PM_DG_SCORINGDIST);
}
// EOF