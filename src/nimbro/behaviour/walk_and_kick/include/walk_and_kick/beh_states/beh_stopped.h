// Walk and kick behaviour state: Stopped
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef BEH_STOPPED_H
#define BEH_STOPPED_H

// Includes
#include <walk_and_kick/wak_beh_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class BehStopped
	* 
	* @brief A walk and kick behaviour state that simply tells the robot to stop in place.
	**/
	class BehStopped : public WAKBehState
	{
	public:
		// Constructor
		BehStopped(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}

#endif
// EOF