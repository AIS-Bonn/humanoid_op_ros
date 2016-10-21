// Walk and kick: Behaviour states base implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/wak_beh_manager.h>

// Namespaces
using namespace walk_and_kick;

//
// WAKBehState class
//

// Constructor
WAKBehState::WAKBehState(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID)
 : config(config)
 , SV(SV)
 , WBS(WBS)
 , GV(WBS.GV)
 , WGS(WGS)
 , field(WBS.field)
 , PM(WBS.PM)
 , MM(WBS.MM)
 , m_id(ID)
 , m_name(WAKBehManager::behStateName(ID))
 , m_active(false)
{
	// Reset the class
	reset();

	// Register the behaviour state
	WBS.registerState(this, m_id, m_name);
}
// EOF