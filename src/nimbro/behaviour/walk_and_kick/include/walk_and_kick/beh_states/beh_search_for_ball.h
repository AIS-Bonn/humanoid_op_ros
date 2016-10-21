// Walk and kick behaviour state: Search for ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef BEH_SEARCH_FOR_BALL_H
#define BEH_SEARCH_FOR_BALL_H

// Includes
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/beh_states/gaze_beh_look_for_ball.h>
#include <rc_utils/ros_time.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class BehSearchForBall
	* 
	* @brief A walk and kick behaviour state that searches for the ball.
	**/
	class BehSearchForBall : public GazeBehLookForBall
	{
	public:
		// Search for ball walk state enumeration
		enum SFBWalkState
		{
			SFB_WS_UNKNOWN = 0,  // Unknown search for ball walk state
			SFB_WS_STAYCOOL,     // Stay cool and hope that the ball will turn up again soon by itself
			SFB_WS_BACKUP,       // Walk backwards a little
			SFB_WS_SPIN,         // Spin on the spot
			SFB_WS_GOTOBALLHYP,  // Go to a ball hypothesis
			SFB_WS_GOTOCENTRE,   // Go to the centre
			SFB_WS_SPINHERE,     // Spin on the spot
			SFB_WS_WALKTOMARK,   // Walk to a penalty mark
			SFB_WS_WALKFWDS,     // Just walk forwards
			SFB_WS_COUNT
		};
		static bool sfbWalkStateValid(int state) { return (state > SFB_WS_UNKNOWN && state < SFB_WS_COUNT); }
		static bool sfbWalkStateValid(SFBWalkState state) { return sfbWalkStateValid((int) state); }
		static const std::string& sfbWalkStateName(SFBWalkState state) { if(sfbWalkStateValid(state)) return SFBWalkStateName[state]; else return SFBWalkStateName[SFB_WS_UNKNOWN]; }
	private:
		static const std::string SFBWalkStateName[SFB_WS_COUNT];

	public:
		// Ball hypothesis type enumeration
		enum BallHypType
		{
			BHT_NONE = 0,        // No ball hypothesis
			BHT_TEAM_COMMS,      // Ball hypothesis from team communications
			BHT_COUNT
		};
		static bool ballHypTypeValid(int type) { return (type >= BHT_NONE && type < BHT_COUNT); }
		static bool ballHypTypeValid(BallHypType type) { return ballHypTypeValid((int) type); }
		static const std::string& ballHypTypeName(BallHypType type) { if(ballHypTypeValid(type)) return BallHypTypeName[type]; else return BallHypTypeName[BHT_NONE]; }
	private:
		static const std::string BallHypTypeName[BHT_COUNT];

	public:
		// Constructor
		BehSearchForBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID);

		// Execute function
		virtual void execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated);

		// Reset search function
		void resetSearch();

		// Request search for ball state functions (allows the request of a particular initial state for the next time search for ball activates)
		void clearSfbStateRequest() { m_reqState = SFB_WS_UNKNOWN; m_reqData = 0; rc_utils::zeroRosTime(m_reqTime); }
		void requestSfbState(SFBWalkState state, int data = 0) { if(sfbWalkStateValid(state)) { m_reqState = state; m_reqData = data; m_reqTime = SV.now; } else clearSfbStateRequest(); }
		void refreshSfbStateRequest() { if(haveSfbStateRequest()) m_reqTime = SV.now; }
		float timeSinceSfbStateRequest() const { return (m_reqTime.isZero() ? INFINITY : (SV.now - m_reqTime).toSec()); }
		bool haveSfbStateRequest() const { return (sfbWalkStateValid(m_reqState) && timeSinceSfbStateRequest() < config.sfbStateRequestTimeout()); }

	protected:
		// Handle activation function
		virtual void handleActivation(bool nowActive);

	private:
		// Persistent variables struct
		struct PersistentVars
		{
			PersistentVars() { reset(); }
			void reset();
			SFBWalkState nextState;
			BallHypType ballHypType;
			float timeout;
			Vec2f target;
			int dirn;
		};

		// State helper functions
		void changeSfbState(SFBWalkState newSfbState, SFBWalkState nextState = SFB_WS_UNKNOWN); // Worker function to actually perform a change of the current search for ball walk state

		// Suspend/resume functions
		void suspendState();
		void resumeState();
		void clearSuspendedState();
		bool haveSuspendedState() const;

		// Helper functions
		float calcBallHypValue(const Vec2f& ballHypPose);
		float calcTimeout(const Vec2f& target);
		int calcSpinDirnToTarget(const Vec2f& target);
		bool spinningToTargetIsBetter(const Vec2f& target);
		float backupFactorFromLine(float margin, float distToLine, float cosAlpha);
		float backupFactorFromPoint(float margin, float vecToPointX, float vecToPointY, float theta);

		// Search for ball state request variables
		SFBWalkState m_reqState;
		int m_reqData;
		ros::Time m_reqTime;

		// Last activated state variables
		SFBWalkState m_lastActState;
		ros::Time m_lastActTime;
		ros::Time m_lastActSpinTime;
		float m_lastActElapsed;

		// Current search for ball walk state
		SFBWalkState m_walkState;
		ros::Time m_walkStateTime;
		bool m_walkStateIsRequest;
		bool m_walkStateIsResumed;
		bool m_walkStateIsNew;

		// Search for ball variables
		PersistentVars m_PV;
		float m_factor;
		float m_walkFwdTime;
		Counter m_doneCounter;
		Counter m_failCounter;

		// Search for ball resume variables
		SFBWalkState m_resumeWalkState;
		float m_resumeElapsed;
		PersistentVars m_resumePV;

		// Team communications variables
		Vec2f m_lastBallHypPose;
		bool m_lastBallHypPoseValid;

		// Visualisation variables
		BallHypType m_visBallHypType;
	};
}


#endif
// EOF