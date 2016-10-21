// Walk and kick game state: Stopped
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_STOPPED_H
#define GAME_STOPPED_H

// Includes
#include <walk_and_kick/wak_game_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GameStopped
	* 
	* @brief A walk and kick game state that forces the robot to the stopped behaviour state.
	**/
	class GameStopped : public WAKGameState
	{
	public:
		// Constructor
		GameStopped(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF