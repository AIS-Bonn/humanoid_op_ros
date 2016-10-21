// Walk and kick game state: Penalty goalie
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_penalty_goalie.h>

// Namespaces
using namespace walk_and_kick;

//
// GamePenaltyGoalie class
//

// Constructor
GamePenaltyGoalie::GamePenaltyGoalie(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : WAKGameState(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GamePenaltyGoalie::handleActivation(bool nowActive)
{
}

// Execute function
void GamePenaltyGoalie::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
	// Issue a warning
	ROS_WARN_THROTTLE(5.0, "Help! I don't know how to be a penalty goalie yet!"); // TODO: Implement

	// Command the robot to stop
	GV.forceBehStateByID = WAKBehManager::BS_STOPPED;
	GV.suggestFoot = GameVars::FS_EITHER_FOOT;
	GV.dribbleIfPossible = false;
	GV.kickIfPossible = false;
	GV.diveIfPossible = DD_NONE;
	GV.ballTargetConf = 0.0f;
	GV.ballTargetDir.setZero();
	GV.ballTargetWedge = 0.0f;
	GV.ballTargetType = GameVars::BTT_UNKNOWN;
	GV.targetPose.setZero();
	GV.targetPoseTol = -1.0f;
	GV.targetPoseValid = false;
}
// EOF