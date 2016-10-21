// Walk and kick game state: Penalty ball handling
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_PENALTY_BALL_HANDLING_H
#define GAME_PENALTY_BALL_HANDLING_H

// Includes
#include <walk_and_kick/wak_game_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GamePenaltyBallHandling
	* 
	* @brief A walk and kick game state that implements ball handling for taking a penalty kick.
	**/
	class GamePenaltyBallHandling : public WAKGameState
	{
	public:
		// Constructor
		GamePenaltyBallHandling(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF