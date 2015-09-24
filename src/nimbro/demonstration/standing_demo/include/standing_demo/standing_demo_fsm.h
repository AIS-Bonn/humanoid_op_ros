// Motion demonstration on a standing robot, copied from standing_demo
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Ensure header is only included once
#ifndef STANDING_DEMO_FSM_H
#define STANDING_DEMO_FSM_H

// Includes
#include <state_controller/state_controller.h>
#include <limb_control/PlayCommandsSrv.h>
#include <ros/time.h>

// Standing demo namespace
namespace standing_demo
{
	// Namespaces
	using namespace statecontroller;
	
	// Class forward-declarations
	class StandingDemo;
	
	// State controller
	class StandingDemoSC;
	
	// States
	class InitDemoState;
	class WaitForStandingState;
	class WaitForTimeState;
	class WaitForMotionPlayerState;
	class PlanDemoActionState;
	
	// State IDs
	enum SDSCStateID
	{
		INIT_DEMO,                  // Initialise the standing demo
		WAIT_FOR_STANDING,           // Wait until the robot enters the standing state
		WAIT_FOR_TIME,              // Wait for a given number of seconds, or until a certain absolute time has been reached
		WAIT_FOR_MOTION_PLAYER,     // Wait for the motion player to finish whatever it's doing
		WAIT_FOR_LIMB_CONTROL,      // Wait for limb control to finish doing whatever it's doing
		PLAN_DEMO_ACTION,           // Plan a sequence of states that comprises a demo action
		RETURN_TO_STANDING,          // Return to the standing pose in terms of the head, arm, knee and ankle servos
		START_MOTION_PLAYBACK,      // Start the playback of a motion using the motion player
		STOP_MOTION_PLAYBACK,       // Wait for motion playback using the motion player to cease
		START_HEAD_IDLING,          // Start the head idling background behaviour
		STOP_HEAD_IDLING            // Stop the head idling background behaviour
	};
	
	// Standing demo state controller
	class StandingDemoSC : public StateController
	{
	public:
		// Constructors
		StandingDemoSC(StandingDemo* SD);
		
		// Pointer to main standing demo object
		StandingDemo* const m_SD;
		
		// State controller callbacks
		virtual bool preStepCallback();
		virtual void preActivateCallback(bool willCallActivate);
		virtual void preExecuteCallback();
		
		// Get functions
		inline const ros::Time& now() const { return m_curStepTime; }
		
	protected:
		// Background motion enable/disable functions
		void enableHeadIdle();
		void disableHeadIdle();
		
		// Background motion handlers
		void handleLegDangle();
		void handleHeadIdle();
		
		// Head idle variables
		bool m_headIdleEnabled;
		
		// Internal variables
		ros::Time m_curStepTime;
	};
	
	// Init demo state
	class InitDemoState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		InitDemoState(StandingDemoSC *sc) : GenState(sc, INIT_DEMO, "INIT_DEMO") {}
		
	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};
	
	// Wait for standing state
	class WaitForStandingState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		WaitForStandingState(StandingDemoSC *sc) : GenState(sc, WAIT_FOR_STANDING, "WAIT_FOR_STANDING") {}
		
	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};
	
	// Wait for time state
	class WaitForTimeState : public GenState<StandingDemoSC>
	{
	public:
		// Constants
		static const ros::Time NullROSTime;
		
		// Constructor
		WaitForTimeState(StandingDemoSC* sc, double waitTime, ros::Time endTime = NullROSTime) : GenState(sc, WAIT_FOR_TIME, "WAIT_FOR_TIME"), m_waitTime(waitTime), m_endTime(endTime) {}
		
	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
		
		// State parameters
		double m_waitTime;
		ros::Time m_endTime;
	};
	
	// Wait for motion player state
	class WaitForMotionPlayerState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		WaitForMotionPlayerState(StandingDemoSC *sc) : GenState(sc, WAIT_FOR_MOTION_PLAYER, "WAIT_FOR_MOTION_PLAYER") {}
		
	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};
	
	// Wait for limb control state
	class WaitForLimbControlState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		WaitForLimbControlState(StandingDemoSC *sc) : GenState(sc, WAIT_FOR_LIMB_CONTROL, "WAIT_FOR_LIMB_CONTROL") {}
		
	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
		
		// State variables
		const limb_control::PlayCommandsRequest PCmdReqNull;
	};
	
	// Plan action state
	class PlanDemoActionState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		PlanDemoActionState(StandingDemoSC* sc) : GenState(sc, PLAN_DEMO_ACTION, "PLAN_DEMO_ACTION") {}
		
	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};
	
	// Return to standing state
	class ReturnToStandingState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		ReturnToStandingState(StandingDemoSC* sc) : GenState(sc, RETURN_TO_STANDING, "RETURN_TO_STANDING") {}
		
	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
		
		// State variables
		limb_control::PlayCommandsRequest PCmdReq;
	};
	
	// Start motion playback state
	class StartMotionPlaybackState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		StartMotionPlaybackState(StandingDemoSC *sc, int motion) : GenState(sc, START_MOTION_PLAYBACK, "START_MOTION_PLAYBACK"), m_motion(motion) {}
		
	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
		
		// State parameters
		const int m_motion;
	};
	
	// Stop motion playback state
	class StopMotionPlaybackState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		StopMotionPlaybackState(StandingDemoSC *sc) : GenState(sc, STOP_MOTION_PLAYBACK, "STOP_MOTION_PLAYBACK") {}
		
	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};
	
	// Start head idling state
	class StartHeadIdlingState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		StartHeadIdlingState(StandingDemoSC *sc) : GenState(sc, START_HEAD_IDLING, "START_HEAD_IDLING") {}
		
	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
	};
	
	// Stop head idling state
	class StopHeadIdlingState : public GenState<StandingDemoSC>
	{
	public:
		// Constructor
		StopHeadIdlingState(StandingDemoSC *sc) : GenState(sc, STOP_HEAD_IDLING, "STOP_HEAD_IDLING") {}
		
	protected:
		// State callbacks
		virtual void activate(cycle_t cyc);
		virtual action_t execute(cycle_t cyc);
	};
}

#endif