// Header for unit testing of the State Controller Library
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef TEST_STATE_CONTROLLER_H
#define TEST_STATE_CONTROLLER_H

// Include configurations
#ifndef VERBOSE_TEST
//#define VERBOSE_TEST // Comment this line out to inhibit printing to console
#endif /* VERBOSE_TEST */

// Includes
#include <iostream>
#include <iomanip>
#include <gtest/gtest.h>
#include <test_utilities/test_utilities.h>
#include <state_controller/state_controller.h>

// State controller test namespace
namespace statecontrollertest
{
	// Namespaces
	using namespace testutilities;
	using namespace statecontroller;
	
	// Enumerations
	enum DemoSCStateID
	{
		LOOK_FOR_BALL,
		WALK_TO_BALL,
		LINE_UP_GOAL,
		KICK_BALL,
		TEST_STATE
	};

	// Class declarations
	class DemoSC;
	class LookForBallState;
	class WalkToBallState;
	class LineUpGoalState;
	class KickBallState;
	class TestState;

	//
	// DemoSC state controller class
	//
	class DemoSC : public StateController
	{
	public:
		// Constructors
		DemoSC() : StateController("DemoSC") // Note: You may also choose to use unique names for each instance of the derived state controller class by implementing passing a name as a constructor parameter
			, g_state_num_exec(0)
			, g_state_num_deact(0)
			, g_test_sc_callbacks(false)
			, g_tested_term(false)
			, g_test_exec(false)
			, g_test_deact(false)
		{
			// Initialise arrays (not in initialiser list due to C4351 in MSVC)
			for(int i = 0; i < 6; i++) { g_scc[i] = 0; }
			for(int i = 0; i < 3; i++) { g_sc[i][0] = 0; g_sc[i][1] = 0; g_sc[i][2] = 0; }
		}

		// Step callbacks
		virtual bool preStepCallback();
		virtual void preActivateCallback(bool willCallActivate);
		virtual void preExecuteCallback();
		virtual bool postExecuteCallback();
		virtual void onTerminateCallback();
		virtual void postStepCallback();

		// Test variables for test_State unit test
		int g_state_num_exec;
		int g_state_num_deact;

		// Test variables for test_StateController
		int g_scc[6];
		int g_sc[3][3];
		bool g_test_sc_callbacks;
		bool g_tested_term;
		bool g_test_exec;
		bool g_test_deact;
	};

	//
	// State classes
	//

	// LookForBallState class
	// Demonstrates: Deriving directly from State for more flexibility / state with no state parameters / overriding all available state callbacks
	class LookForBallState : public State
	{
	public:
		// Constructors
		LookForBallState(DemoSC *sc) : State(LOOK_FOR_BALL, "LOOK_FOR_BALL", sc->name), sc(sc) {}

	protected:
		// Protected state properties
		StateController* scref() const { return static_cast<StateController*>(sc); }
		
		// State callbacks
		void activate(cycle_t cyc);
		action_t execute(cycle_t cyc);
		void deactivate(cycle_t cyc, bool wasExecuted);
		
		// State controller pointer
		DemoSC* const sc;
	};

	// WalkToBallState class
	// Demonstrates: Deriving from GenState / state with large number of state parameters / overriding all available state callbacks
	class WalkToBallState : public GenState<DemoSC>
	{
	public:
		// State parameter data structure type
		struct StateParams
		{
			float target_x;
			float target_y;
			int gait_type;
			float gait_param_1;
			float gait_param_2;
			bool use_fast_mode;
		};

		// Constructors
		WalkToBallState(DemoSC *sc, const StateParams& sp) : GenState(sc, WALK_TO_BALL, "WALK_TO_BALL"), sp(sp) {}

	protected:
		// State callbacks
		void activate(cycle_t cyc);
		action_t execute(cycle_t cyc);
		void deactivate(cycle_t cyc, bool wasExecuted);
		
		// State parameters
		StateParams sp;
	};

	// LineUpGoalState class
	// Demonstrates: Deriving from GenState / state with no state parameters / overriding all available state callbacks
	class LineUpGoalState : public GenState<DemoSC>
	{
	public:
		// Constructors
		LineUpGoalState(DemoSC *sc) : GenState(sc, LINE_UP_GOAL, "LINE_UP_GOAL") {}

	protected:
		// State callbacks
		void activate(cycle_t cyc);
		action_t execute(cycle_t cyc);
		void deactivate(cycle_t cyc, bool wasExecuted);
	};

	// KickBallState class
	// Demonstrates: Deriving from GenState / state with small number of state parameters / choosing to override deactivate() callback only in addition to execute()
	class KickBallState : public GenState<DemoSC>
	{
	public:
		// Constructors
		KickBallState(DemoSC *sc, int kick_type) : GenState(sc, KICK_BALL, "KICK_BALL"), sp_kick_type(kick_type) {}

		// Test functions (used in the test_State unit test)
		void testProtected();

	protected:
		// State callbacks
		action_t execute(cycle_t cyc);
		void deactivate(cycle_t cyc, bool wasExecuted);
		
		// State parameters
		int sp_kick_type;
	};

	// TestState class
	// Used by unit tests to test special case scenarios
	class TestState : public GenState<DemoSC>
	{
	public:
		// Constructors
		TestState(DemoSC* sc) : GenState(sc, TEST_STATE, "TEST_STATE") { DISPLAY(std::cout << "TestState: In constructor" << std::endl); }
		virtual ~TestState() { DISPLAY(std::cout << "TestState: In destructor" << std::endl); }

	protected:
		// State callbacks
		void activate(cycle_t cyc);
		action_t execute(cycle_t cyc);
		void deactivate(cycle_t cyc, bool wasExecuted);
	};
}

#endif /* TEST_STATE_CONTROLLER_H */
// EOF