// Walk and kick game state: Panic attack
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_panic_attack.h>

// Namespaces
using namespace walk_and_kick;

//
// GamePanicAttack class
//

// Constructor
GamePanicAttack::GamePanicAttack(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : WAKGameState(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GamePanicAttack::handleActivation(bool nowActive)
{
	// Reset variables
	m_panicIsOver = false;
}

// Execute function
void GamePanicAttack::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
	// The panic is over if a certain time has elapsed
	if(WGS.stateTime() >= config.paTimeout())
		m_panicIsOver = true;

	// Command the robot to stop
	GV.forceBehStateByID = WAKBehManager::BS_PANIC_ATTACK;
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