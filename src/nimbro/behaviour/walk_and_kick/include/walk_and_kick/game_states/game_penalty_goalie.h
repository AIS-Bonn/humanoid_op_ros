// Walk and kick game state: Penalty goalie
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_PENALTY_GOALIE_H
#define GAME_PENALTY_GOALIE_H

// Includes
#include <walk_and_kick/wak_game_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GamePenaltyGoalie
	* 
	* @brief A walk and kick game state that implements goalie behaviours for defending a penalty kick.
	**/
	class GamePenaltyGoalie : public WAKGameState
	{
	public:
		// Constructor
		GamePenaltyGoalie(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF