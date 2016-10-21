// Walk and kick game state: Default goalie
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_DEFAULT_GOALIE_H
#define GAME_DEFAULT_GOALIE_H

// Includes
#include <walk_and_kick/wak_game_state.h>
#include <walk_and_kick/game_states/game_default_ball_handling.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GameDefaultGoalie
	* 
	* @brief A walk and kick game state that implements default goalie behaviours for playing soccer.
	**/
	class GameDefaultGoalie : public GameDefaultBallHandling
	{
	public:
		// Constructor
		GameDefaultGoalie(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// State variables
		DynamicTargetPose m_cmdTarget;
		int m_reqBehState;
	};
}

#endif
// EOF