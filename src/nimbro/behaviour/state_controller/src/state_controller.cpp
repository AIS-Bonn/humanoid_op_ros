// State Controller Library
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <state_controller/state_controller.h>

// Namespaces
using namespace statecontroller;

//
// State class
//

// State callback wrappers
void State::callActivate(cycle_t cyc)
{
	// Activate the state if it hasn't already been
	if(!m_wasActivated)
	{
		activate(cyc);
		m_wasActivated = true;
	}
}
action_t State::callExecute(cycle_t cyc)
{
	// Declare variables
	action_t hold = HOLD_THIS_STATE; // Trying to execute an inactive state traps the execution (should never occur as only State and StateController can access this method, and they know what they're doing)
	
	// Execute the state as long as it is currently active
	if(m_wasActivated && !m_wasDeactivated)
	{
		if(!m_wasExecuted) m_firstExecCycle = cyc;
		hold = execute(cyc);
		m_wasExecuted = true;
	}

	// Return value
	return hold;
}
void State::callDeactivate(cycle_t cyc)
{
	// Deactivate the state if it hasn't been already
	if(m_wasActivated && !m_wasDeactivated)
	{
		deactivate(cyc, m_wasExecuted);
		m_wasDeactivated = true;
	}
}

//
// StateQueue class
//

// Reset function
void StateQueue::reset()
{
	clear_hard();
	reserveCapacity(DEF_QUEUE_CAPACITY);
}

// Queue advance function (pops and returns the state sitting at the head of the queue)
StatePtr StateQueue::advance() // Returns nullStatePtr if queue is empty
{
	StatePtr tmp;
	if(QueueVec.empty())
		tmp = nullStatePtr;
	else
	{
		tmp = QueueVec.at(0);
		QueueVec.erase(QueueVec.begin());
	}
	return tmp;
}

// Queue state and index retrieval functions
StatePtr StateQueue::stateOf(int id, index_t start) const // Returns nullStatePtr if the given ID is not found
{
	StatePtr tmp;
	for(index_t ind = start;ind < QueueVec.size();ind++)
	{
		tmp = QueueVec.at(ind);
		if(!State::isNull(tmp))
			if(tmp->id == id) return tmp;
	}
	return nullStatePtr;
}
index_t StateQueue::indexOf(int id, index_t start) const // Returns nullIndex if the given ID is not found
{
	StatePtr tmp;
	for(index_t ind = start;ind < QueueVec.size();ind++)
	{
		tmp = QueueVec.at(ind);
		if(!State::isNull(tmp))
			if(tmp->id == id) return ind;
	}
	return nullIndex;
}
index_t StateQueue::indexOf(StatePtr state, index_t start) const // Returns nullIndex if the given state is not found
{
	for(index_t ind = start;ind < QueueVec.size();ind++)
		if(QueueVec.at(ind) == state) return ind;
	return nullIndex;
}

//
// StateController class
//

// Function: StateController::goToState()
/**
* The transition itself occurs in the next step. This function sets @p state as the next and
* only element in the state queue. If @p state is null then this function is equivalent to
* calling `terminate()`. This function ignores calls that do not originate from a state callback.
*
* @param state An class instance of the new state to enter.
* @return Always returns @c PROCEED_NEXT_STATE so that the following usage has the desired effect.
* @code return sc->goToState(NewStateInstance<MyStateClass>(sc, args)); // In an execute() callback... @endcode
**/
action_t StateController::goToState(StatePtr state)
{
	// Allow call only from a callback
	if(!m_inCallback) return PROCEED_NEXT_STATE;
	
	// If state is null then terminate immediately (would have happened in the next step anyway)
	if(State::isNull(state))
		return terminate();

	// Set the next state as required
	Q.setNextState(state);
	
	// Tell the state controller to proceed to the next state
	return PROCEED_NEXT_STATE;
}

// Function: StateController::terminate()
/**
* Termination occurs in the calling step, and irrespective of the state of the queue and the
* action that is possibly subsequently returned from the `execute()` callback. This function
* only has an effect when used from within either the `activate()` or `execute()` callbacks
* of a state.
* 
* @return Always returns @c PROCEED_NEXT_STATE so that the following usage has the desired effect.
* @code return sc->terminate(); // In an execute() callback... @endcode
**/
action_t StateController::terminate()
{
	// Allow call only from a callback
	if(!m_inCallback) return PROCEED_NEXT_STATE; // This check is theoretically not actually needed, as the non-callback rejection happens implicitly in step()
	
	// Force termination of the state controller (within the same step)
	m_terminate = true; // Refer to step() to see what effect this flag has

	// Return value (the exact return value actually doesn't matter as termination happens in any case)
	return PROCEED_NEXT_STATE;
}

// Function: StateController::init()
/**
* The `init()` function can be called at any time and be guaranteed to bring the state controller
* to a well-defined initial state, while releasing all the states and resources that it owned. It
* is recommended to always use this function with an inline @c NewStateInstance call, as opposed
* to creating a state, assigning it to a variable in the calling function and then passing this
* variable as the argument to the `init()` function. For example:
* @code
* ret_t ret;
* MyStateController sc;
* ret = sc.init(NewStateInstance<MyState>(&sc, arg2, ...)); // Pass the newly created state directly to init() function (recommended)
* @endcode
*
* @param state The state instance to initialise the state controller with.
* @return A @c ret_t return ID specifying whether the function was successful and performed the
* required action. Calling this function from a callback (@c SCR_BAD_CALLBACK), passing a null
* state (@c SCR_NULL_STATE), and/or passing a state that belongs to another state controller
* (@c SCR_NOT_MY_STATE) results in return values other than @c SCR_OK.
**/
ret_t StateController::init(StatePtr state)
{
	// Protect from being called by a callback
	if(m_inCallback)         return SCR_BAD_CALLBACK;
	
	// Do nothing if state is invalid
	if(State::isNull(state)) return SCR_NULL_STATE;
	if(!belongsToMe(state))  return SCR_NOT_MY_STATE;

	// Reset the state controller
	reset();

	// Initialise the current state
	setCurState(state, HOLD_THIS_STATE);

	// Return value
	return SCR_OK;
}

// Function: StateController::forceState()
/**
* In current implementation, the only difference between the `forceState()` and `init()` functions
* is that `forceState()` leaves the internal cycle count of the state controller untouched. In
* particular, the current state queue is cleared, and all contained states are deactivated,
* including the current state.
*
* This function should almost never be required. Perform all state and queue changes within the
* states as a general rule - only call `forceState()` if special circumstances mandate an external
* invasion on the state of the state controller. Use this function with care!
*
* @param state The state to transition the state controller into.
* @return A @c ret_t return ID specifying whether the function was successful and performed the
* required action. Calling this function from a callback (@c SCR_BAD_CALLBACK), passing a null
* state (@c SCR_NULL_STATE), and/or passing a state that belongs to another state controller
* (@c SCR_NOT_MY_STATE) results in return values other than @c SCR_OK.
**/
ret_t StateController::forceState(StatePtr state)  // 
{
	// Protect from being called by a callback
	if(m_inCallback)         return SCR_BAD_CALLBACK;
	
	// Do nothing if state is invalid
	if(State::isNull(state)) return SCR_NULL_STATE;
	if(!belongsToMe(state))  return SCR_NOT_MY_STATE;

	// Free up all the current resources (including clearing of the queue)
	freeResources();
	m_terminate = false;

	// Modify the current state as required
	setCurState(state, HOLD_THIS_STATE);

	// Return value
	return SCR_OK;
}

// Function: StateController::step()
/**
* @return A @c ret_t return ID indicating the success of the function and/or the resulting
* state of the state controller (refer to the @c statecontroller::SCReturnID enumeration).
**/
ret_t StateController::step()
{
	// Protect from being called by a callback
	if(m_inCallback) return SCR_BAD_CALLBACK;

	// Declare variables
	ret_t ret;

	// Enter callback-protected mode
	m_inCallback = true;

	// Update the cycle count
	m_cycle++;

	// Call the pre-step callback
	if(!State::isNull(m_curState))
	{
		if(preStepCallback()) // Returns a boolean specifying whether to force a transition
			m_stateAction = PROCEED_NEXT_STATE;
	}

	// Pop the next state from the queue
	if(m_stateAction == PROCEED_NEXT_STATE)
		setCurState(Q.advance());

	// Reset flags
	m_stateAction = PROCEED_NEXT_STATE;
	m_terminate = false;

	// Activate and execute the new current state
	if(State::isNull(m_curState))
		ret = SCR_TERMINATED; // Encountering a null current state is considered to mean that the state controller execution is complete (clean finish)
	else if(!belongsToMe(m_curState))
		ret = SCR_NOT_MY_STATE;
	else
	{
		preActivateCallback(!m_curState->m_wasActivated);
		if(!m_curState->m_wasActivated) m_curState->callActivate(m_cycle);
		if(!m_terminate)
		{
			preExecuteCallback();
			m_stateAction = m_curState->callExecute(m_cycle); // Gives the execute() function the chance to set m_stateAction to HOLD_THIS_STATE
			if(postExecuteCallback()) m_stateAction = PROCEED_NEXT_STATE;
		}
		if(m_terminate)
			ret = SCR_TERMINATED;
		else
			ret = SCR_OK;
	}

	// Clean up our resources if the state controller just terminated
	if(ret == SCR_TERMINATED)
	{
		onTerminateCallback();
		freeResources();
	}

	// Call the post-step callback
	if(!State::isNull(m_curState))
		postStepCallback();

	// Exit callback-protected mode
	m_inCallback = false;

	// Return value
	return ret;
}
// Function: StateController::loop()
/**
* @return A @c ret_t return ID indicating the success of the function. If the state controller
* terminated cleanly then the function returns @c SCR_OK. Otherwise, this function echos the
* return code received from the offending internal call to `step()`. For more information refer
* to the @c statecontroller::SCReturnID enumeration.
**/
ret_t StateController::loop()
{
	// Declare variables
	ret_t ret;
	
	// Keep running the finite state machine until it terminates or errors
	while((ret = step()) == SCR_OK) {}

	// Successful return value of step() in case of clean termination is SCR_TERMINATED - mask this as SCR_OK for return from this function
	if(ret == SCR_TERMINATED) ret = SCR_OK;

	// Return value
	return ret;
}

// Internal functions
ret_t StateController::reset()
{
	// Protect from being called by a callback
	if(m_inCallback) return SCR_BAD_CALLBACK;

	// Reset the internal variables
	m_cycle = 0;
	m_terminate = false;

	// Free any resources we have at the moment
	freeResources();

	// Return value
	return SCR_OK;
}
void StateController::freeResources()
{
	// Enter callback-protected mode
	m_inCallback = true;
	
	// Deactivate the current state
	setCurState(nullStatePtr, HOLD_THIS_STATE);

	// Deactivate any remaining states in the queue
	StatePtr tmp;
	for(index_t i = 0;i < Q.length();i++)
	{
		tmp = Q.get(i);
		if(!State::isNull(tmp))
			tmp->callDeactivate(m_cycle);
	}

	// Explicitly release the queue resources (called AFTER the deactivate() callbacks as they still have access to the queue!)
	Q.reset();

	// Exit callback-protected mode
	m_inCallback = false;
}
void StateController::setCurState(StatePtr state, action_t hold)
{
	// Deactivate the old current state (entering callback-protected mode if necessary)
	if(!State::isNull(m_curState))
	{
		bool oldInCallback = m_inCallback;
		m_inCallback = true;
		m_curState->callDeactivate(m_cycle);
		m_inCallback = oldInCallback;
	}

	// Set the new current state
	m_curState = state;
	m_stateAction = hold;
}
// EOF