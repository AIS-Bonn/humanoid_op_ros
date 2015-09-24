// Unit testing of the State Controller Library
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <../test/test_state_controller.h>

// Namespaces
using namespace std;
using namespace statecontrollertest;

//
// DemoSC class
//

// Pre-step callback
bool DemoSC::preStepCallback()
{
	// Only do something if we are currently performing the corresponding unit test
	if(g_test_sc_callbacks)
	{
		// Display callback text
		DISPLAY(cout << "In callback: preStepCallback()" << endl);

		// Increment appropriate callback execution count
		g_scc[0]++;
	}

	// Return that we do not wish to force a state transition
	return false;
}

// Pre-activation callback
void DemoSC::preActivateCallback(bool willCallActivate)
{
	// Only do something if we are currently performing the corresponding unit test
	if(g_test_sc_callbacks)
	{
		// Display callback text
		DISPLAY(cout << "In callback: preActivateCallback() [willCallActivate = " << (willCallActivate ? "true" : "false") << "]" << endl);

		// Increment appropriate callback execution count
		g_scc[1]++;
	}
}

// Pre-execution callback
void DemoSC::preExecuteCallback()
{
	// Only do something if we are currently performing the corresponding unit test
	if(g_test_sc_callbacks)
	{
		// Display callback text
		DISPLAY(cout << "In callback: preExecuteCallback()" << endl);

		// Increment appropriate callback execution count
		g_scc[2]++;
	}
}

// Post-execution callback
bool DemoSC::postExecuteCallback()
{
	// Only do something if we are currently performing the corresponding unit test
	if(g_test_sc_callbacks)
	{
		// Display callback text
		DISPLAY(cout << "In callback: postExecuteCallback()" << endl);

		// Increment appropriate callback execution count
		g_scc[3]++;
	}

	// Return that we do not wish to force a state transition
	return false;
}

// Termination callback
void DemoSC::onTerminateCallback()
{
	// Only do something if we are currently performing the corresponding unit test
	if(g_test_sc_callbacks)
	{
		// Display callback text
		DISPLAY(cout << "In callback: onTerminateCallback()" << endl);

		// Increment appropriate callback execution count
		g_scc[4]++;
	}
}

// Post-step callback
void DemoSC::postStepCallback()
{
	// Only do something if we are currently performing the corresponding unit test
	if(g_test_sc_callbacks)
	{
		// Display callback text
		DISPLAY(cout << "In callback: postStepCallback()" << endl);

		// Increment appropriate callback execution count
		g_scc[5]++;
	}
}

//
// LookForBallState class
//

// Activate callback
void LookForBallState::activate(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "Activating   LookForBallState" << endl);

	// Increment appropriate callback execution count
	sc->g_sc[LOOK_FOR_BALL][0]++;
}

// Execute callback
action_t LookForBallState::execute(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "Executing    LookForBallState" << endl);

	// Increment appropriate callback execution count
	sc->g_sc[LOOK_FOR_BALL][1]++;

	// Decide whether to change state
	if(execCycleNum(cyc) >= 3)
	{
		WalkToBallState::StateParams sp;
		sp.gait_type = 2;
		sc->goToState(NewStateInstance<WalkToBallState>(sc, sp));
		return PROCEED_NEXT_STATE;
	}

	// Return value
	return HOLD_THIS_STATE;
}

// Deactivate callback
void LookForBallState::deactivate(cycle_t cyc, bool wasExecuted)
{
	// Display callback text
	DISPLAY(cout << "Deactivating LookForBallState" << (wasExecuted ? "" : " (was not executed)") << endl);

	// Increment appropriate callback execution count
	sc->g_sc[LOOK_FOR_BALL][2]++;
}

//
// WalkToBallState class
//

// Activate callback
void WalkToBallState::activate(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "Activating   WalkToBallState" << endl);

	// Increment appropriate callback execution count
	sc->g_sc[WALK_TO_BALL][0]++;
}

// Execute callback
action_t WalkToBallState::execute(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "Executing    WalkToBallState" << endl);

	// Increment appropriate callback execution count
	sc->g_sc[WALK_TO_BALL][1]++;

	// Decide whether to change state
	if(execCycleNum(cyc) >= (cycle_t) sp.gait_type)
		return sc->goToState(NewStateInstance<LineUpGoalState>(sc)); // The short and efficient way of using goToState() for simple next-state logic

	// Return value
	return HOLD_THIS_STATE;
}

// Deactivate callback
void WalkToBallState::deactivate(cycle_t cyc, bool wasExecuted)
{
	// Display callback text
	DISPLAY(cout << "Deactivating WalkToBallState" << (wasExecuted ? "" : " (was not executed)") << endl);

	// Increment appropriate callback execution count
	sc->g_sc[WALK_TO_BALL][2]++;
}

//
// LineUpGoalState class
//

// Activate callback
void LineUpGoalState::activate(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "Activating   LineUpGoalState" << endl);

	// Increment appropriate callback execution count
	sc->g_sc[LINE_UP_GOAL][0]++;
}

// Execute callback
action_t LineUpGoalState::execute(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "Executing    LineUpGoalState" << endl);

	// Increment appropriate callback execution count
	sc->g_sc[LINE_UP_GOAL][1]++;

	// Decide whether to change state
	cycle_t elapsed = execCycleNum(cyc);
	if(elapsed == 4)
	{
		// Populate the queue with states that should go unused (they shouldn't activate, execute or deactivate)
		WalkToBallState::StateParams sp;
		sp.gait_type = 324; // This value should never be used as the following states should never be used
		Queue()->append(NewStateInstance<LookForBallState>(sc));
		Queue()->append(NewStateInstance<WalkToBallState>(sc, sp));
		Queue()->append(NewStateInstance<LineUpGoalState>(sc));
		
		// Terminate the state controller
		DISPLAY(cout << "--> Requesting termination of DemoSC controller" << endl);
		return sc->terminate(); // Termination of the state controller as the required tasks have been completed
	}
	else if(elapsed >= 8)
	{
		// Proceed to next state with empty queue => Terminate in the next step... BUT this should never happen if the terminate() function above works correctly
		return PROCEED_NEXT_STATE;
	}

	// Return value
	return HOLD_THIS_STATE;
}

// Deactivate callback
void LineUpGoalState::deactivate(cycle_t cyc, bool wasExecuted)
{
	// Display callback text
	DISPLAY(cout << "Deactivating LineUpGoalState" << (wasExecuted ? "" : " (was not executed)") << endl);

	// Increment appropriate callback execution count
	sc->g_sc[LINE_UP_GOAL][2]++;
}

//
// KickBallState class
//

// Execute callback
action_t KickBallState::execute(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "Executing KickBallState" << endl);

	// Record that execution happened
	sc->g_state_num_exec += 5;

	// Return value
	return PROCEED_NEXT_STATE;
}

// Deactivate callback
void KickBallState::deactivate(cycle_t cyc, bool wasExecuted)
{
	// Display callback text
	DISPLAY(cout << "Deactivating KickBallState" << (wasExecuted ? "" : " (was not executed)") << endl);

	// Record that deactivation happened
	if(wasExecuted)
		sc->g_state_num_deact += 7;
	else
		sc->g_state_num_deact += 13;
	
}

//
// TestState class
//

// Activate callback
void TestState::activate(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "TestState: In activate()" << endl);

	// Test trying to call callback-protected functions (this is a callback...)
	StatePtr snew = NewStateInstance<KickBallState>(sc, 0);
	EXPECT_EQ(SCR_BAD_CALLBACK, sc->init(snew));
	EXPECT_EQ(SCR_BAD_CALLBACK, sc->forceState(snew));
	EXPECT_EQ(SCR_BAD_CALLBACK, sc->step());
	EXPECT_EQ(SCR_BAD_CALLBACK, sc->loop());
	snew.reset();
	
	// Refuse to activate this state by calling terminate() (...at least the first time this function is called)
	if(!sc->g_tested_term)
	{
		DISPLAY(cout << "TestState: Calling terminate() from inside activate()" << endl);
		sc->g_tested_term = true;
		sc->terminate();
		return; // Just here for good coding standard after a terminate()
	}
}

// Execute callback
action_t TestState::execute(cycle_t cyc)
{
	// Display callback text
	DISPLAY(cout << "TestState: In execute()" << endl);

	// Record that execution (hopefully didn't) happen
	sc->g_test_exec = true;

	// Return value
	return HOLD_THIS_STATE;
}

// Deactivate callback
void TestState::deactivate(cycle_t cyc, bool wasExecuted)
{
	// Display callback text
	DISPLAY(cout << "TestState: In deactivate()" << (wasExecuted ? "" : " (state was never executed)") << endl);

	// Verify wasExecuted
	EXPECT_FALSE(wasExecuted);

	// Record that deactivation happened
	sc->g_test_deact = true;
}

//
// Unit Tests
//

// Function declarations
void PrintQueue(StateQueue& Q);

// Test State class
TEST(StateControllerTest, test_State)
{
	// Declare start of testing
	DISPLAY_NEWLINE;
	DISPLAY(FText(Attr::GREEN) << "Starting test of State class...");

	// Create the required state instances
	DemoSC dsc;
	DemoSC *sc = &dsc;
	boost::shared_ptr<KickBallState> k1 = NewStateInstance<KickBallState>(sc, 0); // Special case: Needed in order to access testProtected()
	StatePtr s0 = NewStateInstance<LookForBallState>(sc);
	StatePtr s1 = k1;

	// Test state properties
	EXPECT_EQ(LOOK_FOR_BALL, s0->id);
	EXPECT_TRUE(s0->name.compare("LOOK_FOR_BALL") == 0);
	EXPECT_TRUE(s0->scname.compare("DemoSC") == 0);
	EXPECT_EQ(KICK_BALL, s1->id);
	EXPECT_TRUE(s1->name.compare("KICK_BALL") == 0);
	EXPECT_TRUE(s1->scname.compare("DemoSC") == 0);

	// Test isNull
	EXPECT_TRUE(State::isNull(nullStatePtr));

	// Test protected functions
	k1->testProtected(); // Kick ball state

	// Declare end of testing
	DISPLAY(FText(Attr::GREEN) << "Completed test of State class!\n");
}

// Protected member test function inside KickBallState
void KickBallState::testProtected()
{
	// Verify that the cycle number was correctly initialised by the constructor
	EXPECT_EQ(0, firstExecCycle());
	EXPECT_EQ(1, execCycleNum(0));
	EXPECT_EQ(8, execCycleNum(7));

	// Enter a "safe" zone (doesn't protect against segmentation faults and the like though)
	try
	{
		// Test protected state properties
		statecontroller::StateController* sctmp = scref();
		ASSERT_TRUE(dynamic_cast<DemoSC*>(sctmp) == sc);

		// Test access to state queue
		StateQueue* Q = Queue();
		Q->clear();
		Q->append(nullStatePtr);
		EXPECT_EQ(1, Q->length());
		StatePtr stmp = NewStateInstance<KickBallState>(sc, 0);
		ASSERT_EQ(SCR_OK, sctmp->init(stmp)); // Clears the queue belonging to sctmp as a side-effect
		EXPECT_EQ(0, Q->length()); // Should have cleared our queue Q (one and the same object hopefully)

		// Test callActivate and callExecute (called from StateController::step() as they are private)
		int tmp_d = sc->g_state_num_deact;
		int tmp_e = sc->g_state_num_exec;
		ASSERT_EQ(SCR_OK, sctmp->step()); // Should activate and execute stmp
		EXPECT_EQ(tmp_e+5, sc->g_state_num_exec);

		// Test callDeactivate
		ASSERT_EQ(SCR_TERMINATED, sctmp->step()); // Should remove reference to stmp in state controller
		EXPECT_EQ(tmp_e+5, sc->g_state_num_exec);
		EXPECT_EQ(tmp_d+7, sc->g_state_num_deact);

		// Test cycle counting
		EXPECT_EQ(2, sctmp->getCycle());
	}
	catch(...)
	{
		// Rethrow any exceptions from within the block, but make gtest catch them
		EXPECT_NO_THROW(throw "Look inside the try block above here in code!");
	}
}

// Test StateQueue class
TEST(StateControllerTest, test_StateQueue)
{
	// Declare start of testing
	DISPLAY_NEWLINE;
	DISPLAY(FText(Attr::GREEN) << "Starting test of StateQueue class...");

	// Create a StateQueue object
	StateQueue Q;

	// Create a few State objects to be placed in the queue
	DemoSC sc;
	WalkToBallState::StateParams sp;
	StatePtr s0 = NewStateInstance<LookForBallState>(&sc);
	StatePtr s1 = NewStateInstance<WalkToBallState>(&sc, sp);
	StatePtr s2 = NewStateInstance<LineUpGoalState>(&sc);
	StatePtr s3 = NewStateInstance<KickBallState>(&sc, 0);

	// Test the reset function (should have been called from constructor)
	EXPECT_TRUE(Q.isEmpty());

	// Test reserveCapacity
	EXPECT_NO_THROW(Q.reserveCapacity(nullIndex));

	// Test element insertion and removal functions
	Q.prepend(s0);
	EXPECT_FALSE(Q.isEmpty());
	Q.append(s1);
	Q.insertAfter(0, s2);
	Q.insertBefore(3, s3);
	EXPECT_EQ(4, Q.length());
	EXPECT_NO_THROW(Q.insertAfter(4, s2));  // Should have no effect as index out of range...
	EXPECT_NO_THROW(Q.insertBefore(5, s2)); // Should have no effect as index out of range...
	EXPECT_EQ(4, Q.length());
	Q.prepend(s1);
	Q.remove(3);
	
	// Test get, set, headItem and tailItem
	Q.set(3, s1);
	EXPECT_EQ(s1, Q.get(3));
	EXPECT_EQ(s1, Q.headItem());
	EXPECT_EQ(s1, Q.tailItem());
	EXPECT_NO_THROW(Q.set(4, s3));
	EXPECT_NO_THROW(Q.get(4));

	// Display the state of the queue
	PrintQueue(Q);
	DISPLAY_NEWLINE;
	
	// Test that the state of the queue is as expected
	EXPECT_EQ(s1, Q.get(0));
	EXPECT_EQ(s0, Q.get(1));
	EXPECT_EQ(s2, Q.get(2));
	EXPECT_EQ(s1, Q.get(3));

	// Test validIndex, headIndex, tailIndex
	EXPECT_TRUE(Q.validIndex(1));
	EXPECT_FALSE(Q.validIndex(4));
	EXPECT_EQ(0, Q.headIndex());
	EXPECT_EQ(3, Q.tailIndex());

	//Test containsID, containsState
	EXPECT_TRUE(Q.containsID(LINE_UP_GOAL));
	EXPECT_FALSE(Q.containsID(KICK_BALL));
	EXPECT_TRUE(Q.containsState(s2));
	EXPECT_FALSE(Q.containsState(s3));

	// Test stateOf
	EXPECT_EQ(s1, Q.stateOf(WALK_TO_BALL));
	EXPECT_EQ(s1, Q.stateOf(WALK_TO_BALL, 1));
	EXPECT_EQ(nullStatePtr, Q.stateOf(WALK_TO_BALL, 4));
	EXPECT_EQ(s0, Q.stateOf(LOOK_FOR_BALL));
	EXPECT_EQ(nullStatePtr, Q.stateOf(LOOK_FOR_BALL, 2));

	// Test indexOf
	EXPECT_EQ(0, Q.indexOf(WALK_TO_BALL));
	EXPECT_EQ(3, Q.indexOf(WALK_TO_BALL, 1));
	EXPECT_EQ(nullIndex, Q.indexOf(WALK_TO_BALL, 4));
	EXPECT_EQ(1, Q.indexOf(LOOK_FOR_BALL));
	EXPECT_EQ(nullIndex, Q.indexOf(KICK_BALL));
	EXPECT_EQ(0, Q.indexOf(s1));
	EXPECT_EQ(3, Q.indexOf(s1, 1));
	EXPECT_EQ(nullIndex, Q.indexOf(s1, 4));
	EXPECT_EQ(1, Q.indexOf(s0));
	EXPECT_EQ(nullIndex, Q.indexOf(s3));
	EXPECT_EQ(nullIndex, Q.indexOf(nullStatePtr));

	// Test advance
	EXPECT_EQ(4, Q.length());
	EXPECT_EQ(s1, Q.advance());
	EXPECT_EQ(3, Q.length());
	EXPECT_EQ(s0, Q.advance());
	EXPECT_EQ(2, Q.length());

	// Display the state of the queue
	PrintQueue(Q);
	DISPLAY_NEWLINE;

	// Test that the state of the queue is as expected
	EXPECT_EQ(s2, Q.get(0));
	EXPECT_EQ(s1, Q.get(1));

	// Test setNextState, getNextState
	Q.setNextState(s3);
	EXPECT_EQ(1, Q.length());
	EXPECT_EQ(s3, Q.getNextState());
	
	// Test clear
	EXPECT_FALSE(Q.isEmpty());
	Q.clear();
	EXPECT_TRUE(Q.isEmpty());

	// Test headIndex, tailIndex, advance, headItem, tailItem, getNextState on empty queue
	EXPECT_EQ(nullIndex, Q.headIndex());
	EXPECT_EQ(nullIndex, Q.tailIndex());
	EXPECT_EQ(nullStatePtr, Q.advance());
	EXPECT_EQ(nullStatePtr, Q.headItem());
	EXPECT_EQ(nullStatePtr, Q.tailItem());
	EXPECT_EQ(nullStatePtr, Q.getNextState());

	// Display the state of the queue
	PrintQueue(Q);
	
	// Declare end of testing
	DISPLAY(FText(Attr::GREEN) << "Completed test of StateQueue class!\n");
}

// Test StateController class
TEST(StateControllerTest, test_StateController)
{
	// Declare variables
	int i, j;
	ret_t ret;
	
	// Declare start of testing
	DISPLAY_NEWLINE;
	DISPLAY(FText(Attr::GREEN) << "Starting test of StateController class...");

	// Create instances of the DemoSC class
	boost::shared_ptr<DemoSC> sc1 = boost::make_shared<DemoSC>();
	boost::shared_ptr<DemoSC> sc2 = boost::make_shared<DemoSC>();
	boost::shared_ptr<DemoSC> sc3 = boost::make_shared<DemoSC>();

	// Call to step() or loop() in uninitialised state controller should terminate state execution immediately ("cleanly")
	DISPLAY(cout << "Testing execution of empty state controller..." << endl);
	EXPECT_FALSE(sc1->isActive());
	EXPECT_FALSE(sc2->isActive());
	EXPECT_EQ(SCR_TERMINATED, sc1->step());
	EXPECT_EQ(SCR_OK, sc2->loop());
	EXPECT_EQ(1, sc1->getCycle());
	EXPECT_EQ(1, sc2->getCycle());
	EXPECT_FALSE(sc1->isActive());
	EXPECT_FALSE(sc2->isActive());

	// Test forceState() (not really much you can test)
	sc1->forceState(NewStateInstance<LookForBallState>(sc1.get()));
	ASSERT_FALSE(State::isNull(sc1->getCurState()));
	EXPECT_EQ(LOOK_FOR_BALL, sc1->getCurState()->id);
	EXPECT_EQ(1, sc1->getCycle());
	EXPECT_TRUE(sc1->isActive());

	// Test init() and forceState() with null states and states of different owners
	EXPECT_EQ(SCR_NULL_STATE, sc1->init(nullStatePtr));
	EXPECT_EQ(SCR_NOT_MY_STATE, sc1->init(NewStateInstance<LookForBallState>(sc2.get())));
	EXPECT_EQ(SCR_NULL_STATE, sc1->forceState(nullStatePtr));
	EXPECT_EQ(SCR_NOT_MY_STATE, sc1->forceState(NewStateInstance<LookForBallState>(sc2.get())));
	EXPECT_EQ(1, sc1->getCycle());
	EXPECT_TRUE(sc1->isActive());

	// Destroy the first two state controllers
	sc1.reset();
	sc2.reset();
	
	// Initialise the callback execution counts and let the user know you're doing it
	DISPLAY_NEWLINE;
	DISPLAY(cout << "Initialising all callback execution counts to zero..." << endl);
	for(i = 0;i < 3;i++)
		for(j = 0;j < 3;j++)
			sc3->g_sc[i][j] = 0;
	
	// Initialise state controller with a state
	EXPECT_FALSE(sc3->isActive());
	ret = sc3->init(NewStateInstance<LookForBallState>(sc3.get()));
	EXPECT_EQ(SCR_OK, ret);
	EXPECT_TRUE(sc3->isActive());
	
	// Keep executing state controller until completion
	DISPLAY_NEWLINE;
	sc3->g_test_sc_callbacks = true;
	for(i = 1;i <= 9;i++)
	{
		DISPLAY(FText(Attr::CYAN) << "Step " << i << ":");
		ret = sc3->step();
		EXPECT_EQ(i, sc3->getCycle());
		if(ret != SCR_OK) break;
	}

	// Check the return values and number of steps
	EXPECT_NE(SCR_OK, ret); // Checks that break was called (state controller finished voluntarily)
	EXPECT_EQ(SCR_TERMINATED, ret); // Checks that the break was called because of a clean finish
	EXPECT_EQ(9, i); // Checks that an early termination didn't occur
	EXPECT_EQ(9, sc3->getCycle());

	// Verify that the state controller is no longer active
	EXPECT_FALSE(sc3->isActive());
	
	// Display the g_sc results for the user and check that they are correct
	DISPLAY_NEWLINE;
	DISPLAY(cout << "Execution summary:" << endl);
	const char * const names[3] = {"LookForBallState", "WalkToBallState", "LineUpGoalState"};
	const char * const namesCB[6] = {"preStepCallback", "preActivateCallback", "preExecuteCallback", "postExecuteCallback", "onTerminateCallback", "postStepCallback"};
	const int numExp[3] = {3, 2, 4};
	int numExpCB[6] = {9, 9, 9, 9, 1, 8};
	for(i = 0;i < 3;i++)
	{
		DISPLAY(cout << left << setw(19) << names[i] << " was activated   " << sc3->g_sc[i][0] << " times" << endl);
		DISPLAY(cout << left << setw(19) << names[i] << " was executed    " << sc3->g_sc[i][1] << " times" << endl);
		DISPLAY(cout << left << setw(19) << names[i] << " was deactivated " << sc3->g_sc[i][2] << " times" << endl);
		DISPLAY_NO_PAD(cout << right);
		EXPECT_TRUE(names != 0); // Prevents -Wunused-variable when verbosity is turned off
		EXPECT_EQ(1, sc3->g_sc[i][0]);
		EXPECT_EQ(numExp[i], sc3->g_sc[i][1]);
		EXPECT_EQ(1, sc3->g_sc[i][2]);
	}
	for(i = 0;i < 6;i++)
	{
		DISPLAY(cout << left << setw(19) << namesCB[i] << " was called      " << sc3->g_scc[i] << " times" << endl);
		DISPLAY_NO_PAD(cout << right);
		EXPECT_TRUE(namesCB != 0); // Prevents -Wunused-variable when verbosity is turned off
		EXPECT_EQ(numExpCB[i], sc3->g_scc[i]);
	}

	// Test what happens if you try to step once more
	DISPLAY_NEWLINE;
	DISPLAY(cout << "Try to step one more time despite termination of state controller..." << endl);
	ret = sc3->step();
	EXPECT_EQ(SCR_TERMINATED, ret);
	EXPECT_EQ(10, sc3->getCycle()); // Cycle count still increments even though it was a null cycle
	for(i = 0;i < 3;i++) // Nothing should have been activated, executed or deactivated
	{
		EXPECT_EQ(1, sc3->g_sc[i][0]);
		EXPECT_EQ(numExp[i], sc3->g_sc[i][1]);
		EXPECT_EQ(1, sc3->g_sc[i][2]);
	}
	numExpCB[4]++; // Only thing that should have happened is one more call to onTerminateCallback...
	for(i = 0;i < 6;i++)
		EXPECT_EQ(numExpCB[i], sc3->g_scc[i]);
	EXPECT_FALSE(sc3->isActive());
	sc3->g_test_sc_callbacks = false;

	// Test reinitialisation with another state
	DISPLAY_NEWLINE;
	DISPLAY(cout << "Reinitialising state controller with TestState..." << endl);
	ret = sc3->init(NewStateInstance<TestState>(sc3.get()));
	EXPECT_EQ(SCR_OK, ret);
	EXPECT_TRUE(sc3->isActive());
	EXPECT_EQ(0, sc3->getCycle());

	// Perform a single step of the state controller
	EXPECT_FALSE(sc3->g_tested_term);
	EXPECT_FALSE(sc3->g_test_exec);
	EXPECT_FALSE(sc3->g_test_deact);
	ret = sc3->step();
	EXPECT_EQ(SCR_TERMINATED, ret);
	EXPECT_FALSE(sc3->isActive());
	EXPECT_EQ(1, sc3->getCycle());
	EXPECT_TRUE(sc3->g_tested_term);
	EXPECT_FALSE(sc3->g_test_exec);
	EXPECT_TRUE(sc3->g_test_deact);
	
	// Declare end of testing
	DISPLAY(FText(Attr::GREEN) << "Completed test of StateController class!\n");
}

// Print a queue to the screen
void PrintQueue(StateQueue& Q)
{
	// Print the required state names to cout
	StatePtr tmp;
	if(Q.isEmpty())
	{
		DISPLAY(cout << "Queue[0] = < Empty Queue >" << endl);
	}
	else
	{
		for(index_t i = 0;i < Q.length();i++)
		{
			tmp = Q.get(i);
			DISPLAY(cout << "Queue[" << i << "] = " << tmp->name << " (ID " << tmp->id << ")" << endl);
		} // No state parameters
	}
}

//
// Main function
//
int main(int argc, char **argv)
{
	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF