// Walk and kick game state: Unknown state
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_UNKNOWN_STATE_H
#define GAME_UNKNOWN_STATE_H

// Includes
#include <walk_and_kick/wak_game_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GameUnknownState
	* 
	* @brief A walk and kick game state that stands in for an unknown state or ID.
	**/
	class GameUnknownState : public WAKGameState
	{
	public:
		// Constructor
		GameUnknownState(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF