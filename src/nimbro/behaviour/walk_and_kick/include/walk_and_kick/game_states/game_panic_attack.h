// Walk and kick game state: Panic attack
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAME_PANIC_ATTACK_H
#define GAME_PANIC_ATTACK_H

// Includes
#include <walk_and_kick/wak_game_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GamePanicAttack
	* 
	* @brief A walk and kick game state that induces a panic attack.
	**/
	class GamePanicAttack : public WAKGameState
	{
	public:
		// Constructor
		GamePanicAttack(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated);

		// Panic functions
		bool panicIsOver() const { return m_panicIsOver; }

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

		// State variables
		bool m_panicIsOver;
	};
}

#endif
// EOF