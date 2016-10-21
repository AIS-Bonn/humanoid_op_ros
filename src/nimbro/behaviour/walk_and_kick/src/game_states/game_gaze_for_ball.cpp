// Walk and kick game state: Gaze for ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_gaze_for_ball.h>

// Namespaces
using namespace walk_and_kick;

//
// GameGazeForBall class
//

// Constructor
GameGazeForBall::GameGazeForBall(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : WAKGameState(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GameGazeForBall::handleActivation(bool nowActive)
{
}

// Execute function
void GameGazeForBall::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
	// Command the robot to stop
	GV.forceBehStateByID = WAKBehManager::BS_LOOK_FOR_BALL;
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