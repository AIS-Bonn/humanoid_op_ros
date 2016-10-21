// Walk and kick game state: Wait for ball in play
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_WAIT_FOR_BALL_IN_PLAY_H
#define GAME_WAIT_FOR_BALL_IN_PLAY_H

// Includes
#include <walk_and_kick/wak_game_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GameWaitForBallInPlay
	* 
	* @brief A walk and kick game state that waits for the ball to be in play.
	**/
	class GameWaitForBallInPlay : public WAKGameState
	{
	public:
		// Constructor
		GameWaitForBallInPlay(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// Target pose
		DynamicTargetPose m_targetPose;
	};
}

#endif
// EOF