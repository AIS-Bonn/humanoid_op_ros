// Walk and kick gaze behaviour state: Look for ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef GAZE_BEH_LOOK_FOR_BALL_H
#define GAZE_BEH_LOOK_FOR_BALL_H

// Includes
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/beh_states/gaze_beh_look_around.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GazeBehLookForBall
	* 
	* @brief A walk and kick gaze behaviour state that looks for the ball.
	**/
	class GazeBehLookForBall : public GazeBehLookAround
	{
	public:
		// Constructor
		GazeBehLookForBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);
		virtual ~GazeBehLookForBall() {}

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// State variables
		bool m_haveBall;
	};
}


#endif
// EOF