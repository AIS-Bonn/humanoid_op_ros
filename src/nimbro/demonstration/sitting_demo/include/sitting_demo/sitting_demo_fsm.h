// Finite state machine for motion demonstration on a sitting robot
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef SITTING_DEMO_FSM_H
#define SITTING_DEMO_FSM_H

// Includes
#include <state_controller/state_controller.h>
#include <limb_control/PlayCommandsSrv.h>
#include <ros/time.h>

// Sitting demo namespace
namespace sitting_demo
{
	// Namespaces
	using namespace statecontroller;

	// Class forward-declarations
	class SittingDemo;

	// State controller
	class SittingDemoSC;

	// States
	class InitDemoState;
	class WaitForSittingState;
	class WaitForTimeState;
	class WaitForMotionPlayerState;
	class PlanDemoActionState;

	// State IDs
	enum SDSCStateID
	{
		INIT_DEMO,                  // Initialise the sitting demo
		WAIT_FOR_SITTING,           // Wait until the robot enters the sitting state
		WAIT_FOR_TIME,              // Wait for a given number of seconds, or until a certain absolute time has been reached
		WAIT_FOR_MOTION_PLAYER,     // Wait for the motion player to finish whatever it's doing
		WAIT_FOR_LIMB_CONTROL,      // Wait for limb control to finish doing whatever it's doing
		PLAN_DEMO_ACTION,           // Plan a sequence of states that comprises a demo action
		RETURN_TO_SITTING,          // Return to the sitting pose in terms of the head, arm, knee and ankle servos
		START_MOTION_PLAYBACK,      // Start the playback of a motion using the motion player
		STOP_MOTION_PLAYBACK,       // Wait for motion playback using the motion player to cease
		START_LEG_DANGLING,         // Start the leg dangling background behaviour
		STOP_LEG_DANGLING,          // Stop the leg dangling background behaviour
		START_HEAD_IDLING,          // Start the head idling background behaviour
		STOP_HEAD_IDLING            // Stop the head idling background behaviour
	};

	// Sitting demo state controller
	class SittingDemoSC : public StateController
	{
	public:
		// Constructors
		SittingDemoSC(SittingDemo* SD);

		// Pointer to main sitting demo object
		SittingDemo* const m_SD;

		// State controller callbacks
		virtual bool preStepCallback();
		virtual void preActivateCallback(bool willCallActivate);
		virtual void preExecuteCallback();

		// Get functions
		inline const ros::Time& now() const { return m_curStepTime; }

	protected:
		// Background motion enable/disable functions
		void enableLegDangle();
		void disableLegDangle();
		void enableHeadIdle();
		void disableHeadIdle();

		// Background motion handlers
		void handleLegDangle();
		void handleHeadIdle();

		// Leg dangle variables
		bool m_legDangleEnabled;

		// Head idle variables
		bool m_headIdleEnabled;

		// Internal variables
		ros::Time m_curStepTime;
	};

	// Init demo state
	class InitDemoState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		InitDemoState(SittingDemoSC *sc) : GenState(sc, INIT_DEMO, "INIT_DEMO") {}

	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};

	// Wait for sitting state
	class WaitForSittingState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		WaitForSittingState(SittingDemoSC *sc) : GenState(sc, WAIT_FOR_SITTING, "WAIT_FOR_SITTING") {}
		
	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};

	// Wait for time state
	class WaitForTimeState : public GenState<SittingDemoSC>
	{
	public:
		// Constants
		static const ros::Time NullROSTime;

		// Constructor
		WaitForTimeState(SittingDemoSC* sc, double waitTime, ros::Time endTime = NullROSTime) : GenState(sc, WAIT_FOR_TIME, "WAIT_FOR_TIME"), m_waitTime(waitTime), m_endTime(endTime) {}

	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
		
		// State parameters
		double m_waitTime;
		ros::Time m_endTime;
	};

	// Wait for motion player state
	class WaitForMotionPlayerState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		WaitForMotionPlayerState(SittingDemoSC *sc) : GenState(sc, WAIT_FOR_MOTION_PLAYER, "WAIT_FOR_MOTION_PLAYER") {}

	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};

	// Wait for limb control state
	class WaitForLimbControlState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		WaitForLimbControlState(SittingDemoSC *sc) : GenState(sc, WAIT_FOR_LIMB_CONTROL, "WAIT_FOR_LIMB_CONTROL") {}

	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);

		// State variables
		const limb_control::PlayCommandsRequest PCmdReqNull;
	};

	// Plan action state
	class PlanDemoActionState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		PlanDemoActionState(SittingDemoSC* sc) : GenState(sc, PLAN_DEMO_ACTION, "PLAN_DEMO_ACTION") {}

	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};

	// Return to sitting state
	class ReturnToSittingState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		ReturnToSittingState(SittingDemoSC* sc) : GenState(sc, RETURN_TO_SITTING, "RETURN_TO_SITTING") {}

	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);

		// State variables
		limb_control::PlayCommandsRequest PCmdReq;
	};

	// Start motion playback state
	class StartMotionPlaybackState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		StartMotionPlaybackState(SittingDemoSC *sc, int motion) : GenState(sc, START_MOTION_PLAYBACK, "START_MOTION_PLAYBACK"), m_motion(motion) {}

	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);

		// State parameters
		const int m_motion;
	};

	// Stop motion playback state
	class StopMotionPlaybackState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		StopMotionPlaybackState(SittingDemoSC *sc) : GenState(sc, STOP_MOTION_PLAYBACK, "STOP_MOTION_PLAYBACK") {}

	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};

	// Start leg dangling state
	class StartLegDanglingState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		StartLegDanglingState(SittingDemoSC *sc) : GenState(sc, START_LEG_DANGLING, "START_LEG_DANGLING") {}

	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
	};

	// Stop leg dangling state
	class StopLegDanglingState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		StopLegDanglingState(SittingDemoSC *sc) : GenState(sc, STOP_LEG_DANGLING, "STOP_LEG_DANGLING") {}

	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
	};

	// Start head idling state
	class StartHeadIdlingState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		StartHeadIdlingState(SittingDemoSC *sc) : GenState(sc, START_HEAD_IDLING, "START_HEAD_IDLING") {}

	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
	};

	// Stop head idling state
	class StopHeadIdlingState : public GenState<SittingDemoSC>
	{
	public:
		// Constructor
		StopHeadIdlingState(SittingDemoSC *sc) : GenState(sc, STOP_HEAD_IDLING, "STOP_HEAD_IDLING") {}

	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
	};
}

#endif /* SITTING_DEMO_FSM_H */
// EOF