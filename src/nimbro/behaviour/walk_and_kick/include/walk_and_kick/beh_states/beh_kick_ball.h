// Walk and kick behaviour state: Kick ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef BEH_KICK_BALL_H
#define BEH_KICK_BALL_H

// Includes
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/beh_states/gaze_beh_look_for_ball.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class BehKickBall
	* 
	* @brief A walk and kick behaviour state that kicks the ball towards a target.
	**/
	class BehKickBall : public GazeBehLookForBall
	{
	public:
		// Constructor
		BehKickBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

		// Kick check functions
		bool okToKick() const;
		bool stillOkToKick() const;

		// Get functions
		bool kickLock() const { return m_kickLock; } // Returns whether kicking is currently locked on
		bool bestKickRight() const; // Returns whether the right foot is currently the best kick foot

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// State variables
		bool m_kickLock;
	};
}

#endif
// EOF