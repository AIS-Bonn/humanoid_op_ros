// Walk and kick gaze behaviour state: Look at ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAZE_BEH_LOOK_AT_BALL_H
#define GAZE_BEH_LOOK_AT_BALL_H

// Includes
#include <walk_and_kick/wak_beh_state.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GazeBehLookAtBall
	* 
	* @brief A walk and kick gaze behaviour state that looks at the ball (and does not move the head from the last commanded pose if the ball is not seen).
	**/
	class GazeBehLookAtBall : public virtual WAKBehState
	{
	public:
		// Constructor
		GazeBehLookAtBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);
		virtual ~GazeBehLookAtBall() {}

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);
	};
}


#endif
// EOF