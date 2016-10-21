// Walk and kick game state: Stopped
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_stopped.h>

// Namespaces
using namespace walk_and_kick;

//
// GameStopped class
//

// Constructor
GameStopped::GameStopped(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : WAKGameState(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GameStopped::handleActivation(bool nowActive)
{
}

// Execute function
void GameStopped::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
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