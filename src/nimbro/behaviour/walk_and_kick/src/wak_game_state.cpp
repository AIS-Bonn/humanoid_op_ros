// Walk and kick: Game states base implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_game_state.h>
#include <walk_and_kick/wak_game_manager.h>

// Namespaces
using namespace walk_and_kick;

//
// WAKGameState class
//

// Constructor
WAKGameState::WAKGameState(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID)
 : config(config)
 , SV(SV)
 , WGS(WGS)
 , field(WGS.field)
 , PM(WGS.PM)
 , MM(WGS.MM)
 , m_id(ID)
 , m_name(WAKGameManager::gameStateName(ID))
 , m_active(false)
{
	// Reset the class
	reset();

	// Register the game state
	WGS.registerState(this, m_id, m_name);
}
// EOF