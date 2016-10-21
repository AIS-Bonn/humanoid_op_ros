// Walk and kick behaviour state: Go behind ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef BEH_GO_BEHIND_BALL_H
#define BEH_GO_BEHIND_BALL_H

// Includes
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/beh_states/gaze_beh_look_for_ball.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class BehGoBehindBall
	* 
	* @brief A walk and kick behaviour state that lines up the robot with the ball and a target.
	**/
	class BehGoBehindBall : public GazeBehLookForBall
	{
	public:
		// Constructor
		BehGoBehindBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

		// Get functions
		BAType ballActionTip() const { return m_ballActionTip; }
		bool useRightFoot() const { return m_useRightFoot; }

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// Constants
		static const float MinReqBallDirX;
		static const float MinRadius;

		// Helper functions
		float minDesiredBallDist(float ballAngle, float reqBallDirX) const;

		// State variables
		bool m_useRightFoot;
		TheWorm m_changeToRightFoot;
		Counter m_unforceFoot;
		Counter m_stuck;
		BAType m_ballActionTip;

		// Misc variables
		Vec2fArray m_path; // Not part of the behaviour state => This is only used temporarily for plotting to avoid constant memory allocations
	};
}

#endif
// EOF