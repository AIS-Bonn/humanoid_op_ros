// Walk and kick gaze behaviour state: Look left right
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAZE_BEH_LOOK_LEFT_RIGHT_H
#define GAZE_BEH_LOOK_LEFT_RIGHT_H

// Includes
#include <walk_and_kick/wak_beh_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GazeBehLookLeftRight
	* 
	* @brief A walk and kick gaze behaviour state that looks left and right.
	**/
	class GazeBehLookLeftRight : public virtual WAKBehState
	{
	public:
		// Constructor
		GazeBehLookLeftRight(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);
		virtual ~GazeBehLookLeftRight() {}

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// State variables
		float m_gazeFreq;
		float m_gazePhaseOff;
	};
}

#endif
// EOF