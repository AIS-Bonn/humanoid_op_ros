// Walk and kick gaze behaviour state: Look down
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAZE_BEH_LOOK_DOWN_H
#define GAZE_BEH_LOOK_DOWN_H

// Includes
#include <walk_and_kick/wak_beh_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GazeBehLookDown
	* 
	* @brief A walk and kick gaze behaviour state that looks down.
	**/
	class GazeBehLookDown : public virtual WAKBehState
	{
	public:
		// Constructor
		GazeBehLookDown(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);
		virtual ~GazeBehLookDown() {}

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}


#endif
// EOF