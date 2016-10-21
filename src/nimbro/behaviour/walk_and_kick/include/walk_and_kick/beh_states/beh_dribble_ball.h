// Walk and kick behaviour state: Dribble ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef BEH_DRIBBLE_BALL_H
#define BEH_DRIBBLE_BALL_H

// Includes
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/beh_states/gaze_beh_look_for_ball.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class BehDribbleBall
	* 
	* @brief A walk and kick behaviour state that dribbles the ball towards a target.
	**/
	class BehDribbleBall : public GazeBehLookForBall
	{
	public:
		// Constructor
		BehDribbleBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

		// Dribble check functions
		bool okToDribble() const;
		bool stillOkToDribble() const;

		// Helper functions
		float minBallTargetWedge() const;
		float calcArcTheta(const Vec2f& vector) const;
		float calcPathAngle(bool useRightFoot, const Vec2f& robotE, Vec2f& robotToBehindBallE, Vec2f& robotToPathTargetE) const;
		float calcPathInterpFactor(const Vec2f& robotToBehindBallE, float localPathAngle) const;
		float calcWalkAngle(const Vec2f& robotToBehindBallE, float interpFactor, float localPathAngle, float pathAngle) const;

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// State variables
		bool m_useRightFoot;
		TheWorm m_changeToRightFoot;
	};
}

#endif
// EOF