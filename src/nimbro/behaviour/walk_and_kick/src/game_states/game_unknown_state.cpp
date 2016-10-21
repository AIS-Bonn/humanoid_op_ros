// Walk and kick game state: Unknown state
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_unknown_state.h>

// Namespaces
using namespace walk_and_kick;

//
// GameUnknownState class
//

// Constructor
GameUnknownState::GameUnknownState(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : WAKGameState(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GameUnknownState::handleActivation(bool nowActive)
{
	// Display a warning message if the unknown state is activated
	if(nowActive)
		ROS_WARN("The %s game state has been activated => This should never happen!", name().c_str());
}

// Execute function
void GameUnknownState::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
	// Display a warning message if the unknown state is executed
	ROS_WARN_THROTTLE(0.4, "The %s game state is being executed => This should never happen!", name().c_str());

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