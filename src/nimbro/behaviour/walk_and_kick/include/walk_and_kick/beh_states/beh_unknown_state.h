// Walk and kick behaviour state: Unknown state
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef BEH_UNKNOWN_STATE_H
#define BEH_UNKNOWN_STATE_H

// Includes
#include <walk_and_kick/wak_beh_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class BehUnknownState
	* 
	* @brief A walk and kick behaviour state that stands in for an unknown state or ID.
	**/
	class BehUnknownState : public WAKBehState
	{
	public:
		// Constructor
		BehUnknownState(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF