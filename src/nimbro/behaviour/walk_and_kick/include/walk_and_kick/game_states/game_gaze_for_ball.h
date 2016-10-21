// Walk and kick game state: Gaze for ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_GAZE_FOR_BALL_H
#define GAME_GAZE_FOR_BALL_H

// Includes
#include <walk_and_kick/wak_game_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GameGazeForBall
	* 
	* @brief A walk and kick game state that makes the robot look for the ball by gaze only.
	**/
	class GameGazeForBall : public WAKGameState
	{
	public:
		// Constructor
		GameGazeForBall(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF