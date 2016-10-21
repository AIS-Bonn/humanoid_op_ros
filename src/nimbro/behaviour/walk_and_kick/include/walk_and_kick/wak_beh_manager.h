// Walk and kick: Class for management of the behaviour states
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_BEH_MANAGER_H
#define WAK_BEH_MANAGER_H

// Includes - General
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_game_vars.h>
#include <walk_and_kick/wak_sensor_vars.h>
#include <walk_and_kick/wak_actuator_vars.h>
#include <walk_and_kick/wak_ros_interface.h>
#include <walk_and_kick/wak_game_shared.h>
#include <walk_and_kick/wak_beh_shared.h>
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/wak_beh_state_combined.h>

// Includes - Behaviour states
#include <walk_and_kick/beh_states/beh_unknown_state.h>
#include <walk_and_kick/beh_states/beh_stopped.h>
#include <walk_and_kick/beh_states/beh_panic_attack.h>
#include <walk_and_kick/beh_states/beh_search_for_ball.h>
#include <walk_and_kick/beh_states/beh_go_behind_ball.h>
#include <walk_and_kick/beh_states/beh_dribble_ball.h>
#include <walk_and_kick/beh_states/beh_kick_ball.h>
#include <walk_and_kick/beh_states/beh_dive_for_ball.h>
#include <walk_and_kick/beh_states/gaze_beh_look_around.h>
#include <walk_and_kick/beh_states/gaze_beh_look_at_ball.h>
#include <walk_and_kick/beh_states/gaze_beh_look_down.h>
#include <walk_and_kick/beh_states/gaze_beh_look_for_ball.h>
#include <walk_and_kick/beh_states/gaze_beh_look_left_right.h>
#include <walk_and_kick/beh_states/walk_beh_walk_to_pose.h>

// Walk and kick namespace
namespace walk_and_kick
{
	// Combined behaviour states
	typedef WAKBehStateCombined<WalkBehWalkToPose, GazeBehLookAround> BehWalkToPoseLookAround;
	typedef WAKBehStateCombined<WalkBehWalkToPose, GazeBehLookAtBall> BehWalkToPoseLookAtBall;
	typedef WAKBehStateCombined<WalkBehWalkToPose, GazeBehLookForBall> BehWalkToPoseLookForBall;
	typedef WAKBehStateCombined<WalkBehWalkToPose, GazeBehLookLeftRight> BehWalkToPoseLookLeftRight;

	/**
	* @class WAKBehManager
	* 
	* @brief A class that manages and executes the walk and kick behaviour states.
	**/
	class WAKBehManager
	{
	public:
		// Behaviour state ID enumeration
		enum BehStateID
		{
			BS_UNKNOWN = 0,
			BS_STOPPED,                       // Abbreviation: STP
			BS_PANIC_ATTACK,                  // Abbreviation: PA
			BS_SEARCH_FOR_BALL,               // Abbreviation: SFB
			BS_GO_BEHIND_BALL,                // Abbreviation: GBB
			BS_DRIBBLE_BALL,                  // Abbreviation: DB
			BS_KICK_BALL,                     // Abbreviation: KB
			BS_DIVE_FOR_BALL,                 // Abbreviation: DFB
			BS_LOOK_AROUND,                   // Abbreviation: LA
			BS_LOOK_AT_BALL,                  // Abbreviation: LAB
			BS_LOOK_DOWN,                     // Abbreviation: LD
			BS_LOOK_FOR_BALL,                 // Abbreviation: LFB
			BS_LOOK_LEFT_RIGHT,               // Abbreviation: LLR
			BS_WALK_TO_POSE,                  // Abbreviation: WTP
			BS_WALK_TO_POSE_LOOK_AROUND,      // Abbreviation: WTPLA
			BS_WALK_TO_POSE_LOOK_AT_BALL,     // Abbreviation: WTPLAB
			BS_WALK_TO_POSE_LOOK_FOR_BALL,    // Abbreviation: WTPLFB
			BS_WALK_TO_POSE_LOOK_LEFT_RIGHT,  // Abbreviation: WTPLLR
			BS_COUNT
		};
		static bool behStateValid(int BSI) { return (BSI > BS_UNKNOWN && BSI < BS_COUNT); }
		static bool behStateValid(BehStateID BSI) { return behStateValid((int) BSI); }
		static const std::string& behStateName(BehStateID BSI) { return behStateName((int) BSI); }
		static const std::string& behStateName(int BSI) { if(behStateValid(BSI)) return BehStateName[BSI]; else return BehStateName[BS_UNKNOWN]; }
	private:
		static const std::string BehStateName[BS_COUNT];

	public:
		// Constructor/destructor
		WAKBehManager(WAKConfig& config, const SensorVars& SV, const WAKRosInterface& RI, const WAKGameShared& WGS);
		virtual ~WAKBehManager();

		// Reset functions
		void reset();             // Resets all normal variables, the state machine and all updated variables
		void resetVars();         // Resets all normal variables
		void resetStateMachine(); // Resets all normal variables and the state machine

		// Behaviour state registration function
		void registerState(WAKBehState* state, int ID, const std::string& name);

		// Constant access to behaviour states
		const WAKBehState* currentState() const { return m_curState; }
		const BehStopped* stoppedState() const { return Stopped; }

		// Get functions
		const WAKBehShared& getWBS() const { return WBS; }
		const ActuatorVars& AV() const { return m_AV; }
		const ActuatorVars& lastAV() const { return m_lastAV; }
		cycle_t wakCycle() const { return m_wakCycle; }
		cycle_t stateCycle() const { return m_stateCycle; }
		BAType ballAction() const { return m_ballAction; }

		// Update function
		void updateManager(const GameVars& GV, cycle_t wakCycle);

		// Execute function
		void execute();

	private:
		// State decision function
		WAKBehState* decideState();

		// Config parameters
		WAKConfig& config;

		// Sensor variables
		const SensorVars& SV;

		// ROS interface
		const WAKRosInterface& RI;

		// Shared game variables
		const WAKGameShared& WGS;

		// Shared behaviour variables
		WAKBehShared WBS;

		// Behaviour states
		BehUnknownState* UnknownState;
		BehStopped* Stopped;
		BehPanicAttack* PanicAttack;
		BehSearchForBall* SearchForBall;
		BehGoBehindBall* GoBehindBall;
		BehDribbleBall* DribbleBall;
		BehKickBall* KickBall;
		BehDiveForBall* DiveForBall;
		GazeBehLookAround* LookAround;
		GazeBehLookAtBall* LookAtBall;
		GazeBehLookDown* LookDown;
		GazeBehLookForBall* LookForBall;
		GazeBehLookLeftRight* LookLeftRight;
		WalkBehWalkToPose* WalkToPose;
		BehWalkToPoseLookAround* WalkToPoseLookAround;
		BehWalkToPoseLookAtBall* WalkToPoseLookAtBall;
		BehWalkToPoseLookForBall* WalkToPoseLookForBall;
		BehWalkToPoseLookLeftRight* WalkToPoseLookLeftRight;

		// Current behaviour state
		WAKBehState* m_curState;

		// Behaviour state retrieval
		WAKBehState* stateForID(int ID);
		std::string stateNameForID(int ID);
		std::map<int, WAKBehState*> m_stateMap;
		std::map<int, std::string> m_stateNameMap;

		// Game variable inputs
		GameVars m_GV;

		// Actuator variable outputs
		ActuatorVars m_AV;
		ActuatorVars m_lastAV;

		// Decide state variables
		BAType m_ballAction;
		TheWorm m_kickWorm;
		TheWorm m_dribbleWorm;
		TheWorm m_stillDribbleWorm;
		bool m_allowBreakDribble;

		// Walking target variables
		bool haveWalkingTarget() const { return m_walkingTargetValid; }
		Vec2f getWalkingTarget() const { return m_walkingTarget; }
		float getWalkingTargetTol() const { return m_walkingTargetTol; }
		void setWalkingTarget(const Vec2f& target, float tol = -1.0f) { m_walkingTarget = target; m_walkingTargetTol = tol; m_walkingTargetValid = true; }
		void setWalkingTargetTol(float tol) { m_walkingTargetTol = tol; }
		void invalidateWalkingTarget() { m_walkingTargetValid = false; RI.MM->WalkingTarget.setColor(1.0, 0.0, 0.0); RI.MM->WalkingTargetTol.setColor(1.0, 0.0, 0.0); }
		Vec2f m_walkingTarget;
		float m_walkingTargetTol;
		bool m_walkingTargetValid;

		// Cycle numbers
		cycle_t m_wakCycle;
		cycle_t m_stateCycle;

		// Friend classes
		friend class WalkAndKick;
		friend class WAKBehShared;
	};
}

#endif
// EOF
