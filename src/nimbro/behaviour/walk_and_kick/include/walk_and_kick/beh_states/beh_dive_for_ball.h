// Walk and kick behaviour state: Dive for ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef BEH_DIVE_FOR_BALL_H
#define BEH_DIVE_FOR_BALL_H

// Includes
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/beh_states/gaze_beh_look_for_ball.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class BehDiveForBall
	* 
	* @brief A walk and kick behaviour state that dives for the ball.
	**/
	class BehDiveForBall : public GazeBehLookForBall
	{
	public:
		// Constructor
		BehDiveForBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

		// Get functions
		bool diveLock() const { return m_diveLock; } // Returns whether diving is currently locked on

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// State variables
		bool m_diveLock;
	};
}

#endif
// EOF