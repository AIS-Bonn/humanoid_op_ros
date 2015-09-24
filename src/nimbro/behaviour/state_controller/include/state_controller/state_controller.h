// State Controller Library
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file state_controller.h
* @brief Implements the %State Controller Library.
**/

/**
* @defgroup StateControllerLibrary State Controller Library
*
* @author Philipp Allgeuer (<pallgeuer@ais.uni-bonn.de>)
* @date November 21, 2014
* @version 1.2.2
*
* @section sclsec1 Overview
* The %State Controller Library is a generic platform-independent C++ namespace that allows finite state machines
* and multi-action planning generalisations thereof to be realised. The structure and implementation of this
* library focuses on applications of finite state machines in control loops, but can reasonably be adapted
* for virtually any other application. An emphasis has been placed on having very low overhead so as not to
* hurt overall system performance no matter where this library is used.
*
* Aside from implementing standard finite state machines and multi-action planning state machines, this library
* can also be used to implement hierarchical state controllers or any hybrid of the three. The hierarchical case can
* be achieved by creating a state controller within the state of another state controller, and stepping the child
* state controller within the @link statecontroller::State::execute `execute()` @endlink callback of the parent state
* controller. Minor variations of this approach are also possible to adjust the visibility of certain state variables
* and shared variables. An overview of the State Controller Library architecture is shown in the following diagram.
*
* \image html state_controller/scl_architecture.png Figure 1: A block diagram of the State Controller Library architecture showing the states, state instances, state queue and state controller objects.
*
* The State Controller Library was developed as a simple and state-based alternative to the more complex but
* powerful <a href="http://sourceforge.net/projects/behaviourcontrol/">Behaviour Control Framework</a>.
* 
* @section sclsec1a Academic Sources
* The State Controller Library and the Behaviour Control Framework are detailed in the following paper.
*
* > P. Allgeuer and S. Behnke, "Hierarchical and State-based Architectures for Robot Behavior
* > Planning and Control," in _Proceedings of the 8th Workshop on Humanoid Soccer Robots,
* > IEEE-RAS Int. Conference on Humanoid Robots_, Atlanta, USA, 2013.
*
* You are kindly asked to cite this paper if you use this framework for academic work.
*
@verbatim
@INPROCEEDINGS{Allgeuer2013,
  author = {Philipp Allgeuer and Sven Behnke},
  title = {Hierarchical and State-based Architectures for Robot Behavior Planning and Control},
  booktitle = {Proceedings of the 8th Workshop on Humanoid Soccer Robots, IEEE-RAS Int. Conference on Humanoid Robots},
  year = {2013},
  address = {Atlanta, USA}
}
@endverbatim
* 
* @section sclsec2 Dependencies
* This library depends on the following external libraries (to avoid requiring C++11):
*
* - Boost <a href="http://www.boost.org/doc/libs/release/libs/smart_ptr/">Smart Ptr</a>
* - Boost <a href="http://www.boost.org/doc/libs/release/libs/static_assert/">Static Assert</a>
* - Boost <a href="http://www.boost.org/doc/libs/release/libs/type_traits/">Type Traits</a>
*
* For more information, or to download the Boost Libraries, please refer to http://www.boost.org/.
*
* @section sclsec3 Library Implementation Notes
* The following notes describe how to use the library, and outline some of the finer points.
*
* - A new instance of a state class is created for every time the state controller is in that particular state.
*   This instantiation occurs (incidentally, by the user of this library) when the state is placed into the state
*   queue, which is stored internally in the state controller. This means that each state (i.e. derived `State` class)
*   will have many state instances throughout the lifetime of the state controller, not all of which will necessarily
*   be executed however (e.g. if a state is enqueued but then the queue is cleared and re-populated with other states
*   before it had a chance to execute). Boost shared pointers are used (and should be used by the user) to ensure
*   that the state instances get properly destroyed when no longer required (refer to the @c NewStateInstance macro).
*   As such, state instances are one-use only. Do not try to feed an executed state instance back into the
*   queue/state controller.
*
*   Example of creating a state:
*   @code
*   StatePtr s = NewStateInstance<MyState>(arg1, arg2, ...); // arg1, arg2, ... are the parameters to pass to the MyState constructor
*   ...
*   ret_t ret;
*   MyStateController sc;
*   ret = sc.init(NewStateInstance<MyState>(&sc, arg2, ...)); // Pass the newly created state directly to init() function (recommended)
*   @endcode
*
* - There are three main kinds of variables that can be used within a state controller and its states:
*
*     - <b>%State parameters:</b> Stored in the state class, used as input parameters for a state instance, generally
*                         remain constant throughout the execution of that state instance, an example is a target
*                         theta value for an @c AdjustActuatorPosition state, state parameters differentiate
*                         themselves from state variables as they appear (and are set) in the state constructor by
*                         definition.
*
*     - <b>%State variables:</b> Stored in the state class, used for storing data locally between calls to `execute()`
*                         for a single state instance.
*
*     - <b>Shared variables:</b> Stored in the state controller class, used as a form of 'global' storage as the
*                         values are retained between different instances of the same state, also used to share
*                         data between states, example is a pointer to a class with robot information.
*                         Shared variables are further sub-classified as <b>output shared variables</b> or <b>internal
*                         shared variables</b>. The former are the public shared variables that are used to pass
*                         information out of the state controller, while the latter are the private/protected shared
*                         variables that are used internally for data sharing between states.
*
* - Never declare a member of a state class as static in order to share it between separate instances of a state,
*   use shared variables for this purpose instead. The former may result in unexpected behaviour when more than one
*   instance of the owning state controller is instantiated at a time (even if this situation is not intended
*   to occur in your application).
*
* - User-defined states can either inherit from the @link statecontroller::State `State` @endlink class directly, or
*   indirectly through @link statecontroller::GenState `GenState` @endlink (with template parameter @c SCClass). The intended
*   advantage of using `GenState` is that is keeps an internal pointer to the owning state controller (of class `SCClass`),
*   so the user does not need to do this themselves in order to access the state instance independent ('global') variables
*   stored in the state controller. `GenState` also provides a default implementation for the `scname` and `scref()`
*   members. For obvious reasons, the @c SCClass template parameter of the `GenState` class must derive from the
*   @link statecontroller::StateController `StateController` @endlink class. A compile time error is issued if this is not
*   the case.
*
* - States with only a few state parameters can pass them directly as parameters to the constructor, making use of
*   the initialisation list to copy these to internal members. States with many state parameters should define a
*   @c StateParams struct inside the class and pass this to the constructor instead. This struct should then contain
*   all the required state parameters.
*
*   Example constructor that accepts a @c StateParams struct:
*   @code
*   class MyState : public GenState<MySC>
*   {
*   public:
*   	// State parameter data structure type
*   	struct StateParams
*   	{
*   		float myParam;
*   		... // Remaining parameters
*   	};
*
*   	// Constructor
*   	WalkToBallState(MySC *sc, const StateParams& sp) : GenState(sc, WALK_TO_BALL, "WALK_TO_BALL"), sp(sp) {}
*
*   	... // Rest of class
*
*   protected:
*   	// State parameters
*   	StateParams sp;
*   };
*   @endcode
*
* - %State instances are added to a state controller object by enqueueing them in the state controller's internal
*   @link statecontroller::StateQueue `StateQueue` @endlink. No static/compile time association needs to be made between
*   the states and the state controller, other than to define a pointer to the correct state controller class type inside
*   each of the state classes, and populating this member via the state constructor. This is automatically handled if
*   using @link statecontroller::GenState `GenState` @endlink.
*
* - A single step of the state controller is executed using the @link statecontroller::StateController::step `step()`
*   @endlink function. This function is useful for embedding state controllers inside timed loops or other higher
*   level main loops. To execute a state controller until completion (i.e. termination or error) the
*   @link statecontroller::StateController::loop `loop()` @endlink function can be used. This repeatedly calls the
*   `step()` function as fast as execution allows.
*
* - A constant reference to the current state of a state controller can be retrieved using the
*   @link statecontroller::StateController::getCurState `getCurState()` @endlink function.
*
* - Each state has three so-called <b>state callbacks</b>, namely @link statecontroller::State::activate `activate()`
*   @endlink, @link statecontroller::State::execute `execute()` @endlink and @link statecontroller::State::deactivate
*   `deactivate()` @endlink. The state callbacks should be overridden as required by classes derived (either directly
*   or indirectly) from `State`, but never called by them. The `activate()` and `deactivate()` callbacks have default
*   null implementations as they may frequently not be required by the user, so they can be omitted from the derived
*   class implementation. Each of the callbacks must accept a cycle count parameter @p cyc, which is used to explicitly
*   pass the current cycle count of the owning state controller. This could equivalently (but slightly less robustly in
*   certain situations) be retrieved directly from the @link statecontroller::StateController::getCycle `getCycle()`
*   @endlink function. The `deactivate()` function must also accept a boolean parameter that specifies whether the
*   `execute()` callback of the state was ever called after the state was activated. This could change what needs to be
*   done (e.g. what memory needs to be freed) in the `deactivate()` callback. There are only rare situations where the
*   boolean parameter will ever be false, such as when the @link statecontroller::StateController::terminate `terminate()`
*   @endlink function was used from the `activate()` callback.
*
* - Six user-overridable <b>step callbacks</b> exist in the @link statecontroller::StateController `StateController`
*   @endlink class. Namely, @link statecontroller::StateController::preStepCallback `preStepCallback` @endlink,
*   @link statecontroller::StateController::preActivateCallback `preActivateCallback` @endlink,
*   @link statecontroller::StateController::preExecuteCallback `preExecuteCallback` @endlink,
*   @link statecontroller::StateController::postExecuteCallback `postExecuteCallback` @endlink,
*   @link statecontroller::StateController::onTerminateCallback `onTerminateCallback` @endlink and
*   @link statecontroller::StateController::postStepCallback `postStepCallback` @endlink. All of these virtual
*   functions get called during the `step()` function. Note that `loop()` internally calls `step()`, so the callbacks
*   also get called if the `loop()` function is used to execute the state controller unto termination (or error). Exact
*   descriptions of when each of these callbacks are executed are provided in the individual function descriptions.
*
* - A state is activated by the state controller immediately before it is executed for the first time, but then never
*   again - a new state instance has to be created in order to enter a particular state again later in time, after
*   other states have been executed in the meantime. The `execute()` callback is then called in every state controller
*   step that this state remains the current state. Transitions occur by enqueueing element(s) into the state queue
*   (retrieve a pointer to the queue using the @link statecontroller::State::Queue `Queue()` @endlink function) from
*   within the `execute()` callback and then returning @c PROCEED_NEXT_STATE instead of @c HOLD_THIS_STATE, or by using
*   the @link statecontroller::StateController::goToState `StateController::goToState()` @endlink function. The
*   @link statecontroller::StateController::terminate `StateController::terminate()` @endlink function can be used to
*   leave the current state and terminate the state controller. Every state that gets activated at some point also gets
*   deactivated (this is a certainty that can be relied upon). This almost always occurs immediately when the state
*   controller no longer needs the state, but if for whatever special reason this is not the case (shouldn't
*   happen in normal use of the library), deactivation occurs at latest in the `State` destructor. Such a call is made
*   with a default cycle count of 0. Any cycle count-specific code in the deactivate function should be able to deal with
*   this possible count discontinuity.
*
* - The cycle count that is passed to the state callbacks can be used to add a notion of timing to the state controller.
*   That is, a state can be made to terminate after a set number of steps based on this count, which in a real-time
*   control loop would approximately correspond to terminating after a fixed amount of time. The important functions are
*   @link statecontroller::State::firstExecCycle `firstExecCycle()` @endlink and @link statecontroller::State::execCycleNum
*   `execCycleNum()` @endlink. The former returns the cycle number of the first call to `execute()` for that particular
*   state instance, while the latter returns the number of times `execute()` has been called, including any current call.
*   `firstExecCycle()` returns zero if `execute()` has never been called, and is guaranteed to be non-zero if `execute()`
*   has been called at least once. The `execCycleNum()` function for example can be used as follows:
*   @code
*   // MyState execute callback
*   action_t MyState::execute(cycle_t cyc)
*   {
*   	... // Perform the required state tasks
*
*   	// Change state after 5 steps (for a total of exactly 5 calls to this callback)
*   	if(execCycleNum(cyc) >= 5)
*   		return sc->goToState(NewStateInstance<MyNewState>(sc)); // The short and efficient way of using goToState() for simple next-state logic
*
*   	// Keep state controller in MyState until the termination condition above is satisfied
*   	return HOLD_THIS_STATE;
*   }
*   @endcode
*
* - The `id`, `name`, `scname` and `scref()` state properties need to be defined for each state (noting however
*   that `scname` and `scref()` are taken care of already if deriving from `GenState`). All but `scref()` are
*   defined using constructor arguments.
*
*     - @link statecontroller::State::id `id` @endlink: A unique numeric ID for each state. e.g. @c MY_STATE_NAME,
*               defined via an enumeration:
*               @code
*               enum MySCStateID
*               {
*               	... // Other state IDs
*               	MY_STATE_NAME,
*               	... // Other state IDs
*               };
*               @endcode
*
*     - @link statecontroller::State::name `name` @endlink: A `std::string` with a useful name for the state.
*               e.g. @c "MY_STATE_NAME", @c "performing_this_action", @c "waiting_for_input", etc...
*
*     - @link statecontroller::State::scname `scname` @endlink: A `std::string` with a useful name for the owning
*               state controller. This is generally retrieved directly from @link statecontroller::StateController::name
*               `StateController::name` @endlink.
*
*     - @link statecontroller::State::scref `scref()` @endlink: A pointer to the owning state controller, used to verify
*               which `StateController` derived class instance the state belongs to. This property is protected, unlike
*               the other state properties, and is generally implemented as a @c static_cast of the internal state controller
*               pointer (normally called @c sc).
*
* - Be careful never to dereference the return value of `scref()` or call `Queue()` when the owning state controller
*   object has already been destroyed, or a segmentation fault will result. Use of these functions in the state
*   callbacks is completely safe (as long as the user doesn't find a devious way of calling the callbacks from outside
*   the state controller, which with enough abuse of the library is actually possible to do incidentally [but I won't
*   say how!], and then specifically dereferences one of these two functions just to break the program). In short, it
*   will never happen in normal usage or by accident.
*
* - It is not recommended to keep a local reference to any states that you pass to @link statecontroller::StateController::init
*   `StateController::init()` @endlink and @link statecontroller::StateController::forceState `StateController::forceState()`
*   @endlink. A local reference should never be needed anyway, so the temptation should be limited. Refer to the
*   documentation for these functions for more information, as well as the code example of creating a state further
*   above on this page.
*
* - Although many of the state controller member functions are public by requirement, it is not recommended to call
*   them from within a state callback. Callback protection has been implemented to ensure that such function calls
*   are ignored, and as a result have no effect.
*
* - If you want to initialise a state controller with a particular queue of states, the correct way of approaching
*   this is to create an initialisation state that sets up the queue as desired and then immediately returns
*   @c PROCEED_NEXT_STATE. Avoid trying to set up the initial queue by calling queue functions externally (this has
*   intentionally been made near-impossible to do).
*
* @section sclsec4 Code Example
* A (relatively) minimal example of how to use this library is shown below.
*
* @code
* //
* // File: MyCode.h
* //
*
* // Includes
* #include <state_controller.h>
*
* // Using directives
* using namespace statecontroller;
*
* // State IDs
* enum MySCStateID
* {
* 	MY_STATE,
* 	...
* };
*
* // State controller
* class MySC : public StateController
* {
* public:
* 	// Constructors
* 	MySC() : StateController("MySC") {}
*
* 	// Shared variables
* 	...
* };
*
* // Sample state
* class MyState : public GenState<MySC>
* {
* public:
* 	// Constructors
* 	MyState(MySC *sc, int param) : GenState(sc, MY_STATE, "MY_STATE"), param(param) {}
*
* protected:
* 	// State callbacks
* 	action_t execute(cycle_t cyc)
* 	{
* 		// Execute ten times then terminate state controller
* 		if(execCycleNum(cyc) >= 10)
* 			return sc->terminate();
*
* 		// Stay in this state until the above condition is satisfied
* 		return HOLD_THIS_STATE;
* 	}
*
* 	// State parameters
* 	int param;
*
* 	// State variables
* 	...
* };
* @endcode
*
* @code
* //
* // File: MyCode.cpp
* //
*
* // Includes
* #include "MyCode.h"
*
* // Main function
* int main(int argc, char **argv)
* {
* 	// Run the state controller
* 	MySC sc;
* 	sc.init(NewStateInstance<MyState>(&sc, 4));
* 	sc.loop();
*
* 	// Return value
* 	return 0;
* }
* @endcode
*
* See `test/test_state_controller.h` and `test/test_state_controller.cpp` for more examples of how to define states
* and state controllers and how to run them.
*
* See `state_controller.h` and `state_controller.cpp` for the library source code.
*
* @see @link statecontroller statecontroller Namespace @endlink
* @see @link statecontroller::StateController StateController Class @endlink
* @see @link statecontroller::StateQueue StateQueue Class @endlink
* @see @link statecontroller::State State Class @endlink
**/

// Ensure header is only included once
#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

// Includes
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/static_assert.hpp>
#include <cstddef> // For std::size_t
#include <vector>  // For std::vector
#include <string> // For std::string

// Macro to create a new state instance
// Use like this: StatePtr s = NewStateInstance<MyState>(arg1, arg2, ...);
// Note: MyState should be a class derived either directly or indirectly from the State class.
//       arg1, arg2, ... is the parameter list to pass to the MyState constructor. arg1 will
//       generally be 'this' (when used from inside a state controller) or 'sc' (when used from
//       within another state instance), as it is generally used by the constructor to pass a
//       pointer to the owning state controller object.
#define NewStateInstance boost::make_shared

/**
* @namespace statecontroller
*
* @ingroup StateControllerLibrary
*
* @brief This namespace defines everything that is required for the @ref StateControllerLibrary "State Controller Library".
**/
namespace statecontroller
{
	// Class declarations
	class State;
	template <class SCClass> class GenState;
	class StateQueue;
	class StateController;

	// Enumerations
	enum SCReturnID       //! Used to specify return values and error codes of state controller functions
	{                     //  Programming hint: If foo() returns a ret_t then if(foo()) {...} executes when the return was *not* SCR_OK
		SCR_OK = 0,       //!< Signals successful execution of function
		SCR_TERMINATED,   //!< Signals a clean termination of the state controller
		SCR_NULL_STATE,   //!< Signals that a null state was encountered (where it wasn't expected)
		SCR_NOT_MY_STATE, //!< Signals that a state was encountered that belongs to another state controller instance (or none at all)
		SCR_BAD_CALLBACK  //!< Signals that a callback tried to call a function in StateController that it isn't supposed to
	};

	// Type definitions
	typedef boost::shared_ptr<State> StatePtr;            //!< Used to represent a Boost shared pointer to a State object
	typedef boost::shared_ptr<State const> StateConstPtr; //!< Used to represent a Boost shared pointer to a constant State object
	typedef std::size_t index_t;                          //!< Used to represent an array/vector/queue index (this MUST be an unsigned type)
	typedef unsigned int cycle_t;                         //!< Used to count the number of executed state controller cycles
	typedef bool action_t;                                //!< Used to represent the state transition actions (@c HOLD_THIS_STATE and @c PROCEED_NEXT_STATE)
	typedef int ret_t;                                    //!< Used to represent the return codes from state controller functions

	// Constants
	const StatePtr nullStatePtr = boost::shared_ptr<State>(); //!< Null @c StatePtr
	const index_t nullIndex = (index_t) -1;                   //!< Null @c index_t (@c nullIndex is guaranteed to exceed `std::vector::max_size()`, so there is no loss here)
	const action_t HOLD_THIS_STATE = true;                    //!< Specifies that the state controller should remain in its current state in the next step, instead of advancing the queue
	const action_t PROCEED_NEXT_STATE = false;                //!< Specifies that the state controller should advance the queue and retrieve a new State in the next step

	/**
	* @class State
	*
	* @brief Implements a state that a state controller can be in.
	*
	* This class is the base class of all states of a state controller.
	**/
	class State
	{
	public:
		// Constructors
		State(int id, const std::string& name, const std::string& scname)
			: id(id)
			, name(name)
			, scname(scname)
			, m_wasActivated(false)
			, m_wasExecuted(false)
			, m_wasDeactivated(false)
			, m_firstExecCycle(0)
		{}                                      //!< @brief Default constructor
		virtual ~State() { callDeactivate(0); } //!< @brief Destructor

		// State properties
		const int id;             //!< @brief The unique numeric ID of the state type
		const std::string name;   //!< @brief The human-friendly name of the state type
		const std::string scname; //!< @brief The human-friendly name of the owning state controller

		// Helper functions
		static bool isNull(StateConstPtr state) { return !((bool) state); } //!< @brief Return whether a @c StatePtr is null

	protected:
		// Friend classes
		friend class StateController;

		// Protected state properties
		virtual StateController* scref() const = 0;               //!< @brief Return pointer to owning state controller

		// State callbacks
		virtual void activate(cycle_t cyc) {}                     //!< @brief State activation callback
		virtual action_t execute(cycle_t cyc) = 0;                //!< @brief State execution callback
		virtual void deactivate(cycle_t cyc, bool wasExecuted) {} //!< @brief State deactivation callback

		// Get functions
		inline StateQueue* Queue() const;                                              //!< @brief Return pointer to the StateQueue of the owning state controller
		cycle_t firstExecCycle() const          { return m_firstExecCycle; }           //!< @brief The cycle ID of the first call to execute() for this state instance
		cycle_t execCycleNum(cycle_t cyc) const { return cyc - m_firstExecCycle + 1; } //!< @brief The number of cycles since the first call to execute(), including the initial call cycle and the current call cycle if the function is being used (as intended) from within execute(). \n Example: `if(execCycleNum(cyc) >= 10) return PROCEED_NEXT_STATE;`

	private:
		// State callback wrappers
		void callActivate(cycle_t cyc);    //!< @brief Wrapper of the activate() callback for internal use
		action_t callExecute(cycle_t cyc); //!< @brief Wrapper of the execute() callback for internal use
		void callDeactivate(cycle_t cyc);  //!< @brief Wrapper of the deactivate() callback for internal use

		// Callback execution flags
		bool m_wasActivated;
		bool m_wasExecuted;
		bool m_wasDeactivated;

		// Cycle of first call to execute()
		cycle_t m_firstExecCycle; // Equals zero before the first call to execute()
	};

	/**
	* @class GenState
	*
	* @brief Specialises the @c State class using templates to take care of the most common overloads.
	*
	* It is highly recommended to derive from this class instead of @c State directly. The @p SCClass
	* template parameter must be a derived class of @c StateController.
	**/
	template <class SCClass>
	class GenState : public State
	{
	public:
		// Ensure SCClass is a derived class of StateController
		typedef boost::is_base_of<StateController, SCClass> Assert;
		BOOST_STATIC_ASSERT_MSG(Assert::value, "SCClass template parameter must be a derived class of statecontroller::StateController");

		// Constructors
		GenState(SCClass* sc, int id, const std::string& name) : State(id, name, sc->name), sc(sc) {} //!< @brief Default constructor that takes a pointer to the owning state controller class (@c SCClass must be a derived class of StateController)
		virtual ~GenState() {} //!< @brief Destructor

	protected:
		// Protected state properties
		virtual StateController* scref() const { return static_cast<StateController*>(sc); }

		// State controller pointer
		SCClass* const sc; //!< @brief Pointer to the owning state controller (required in order to be able to access the StateQueue and many other vital functions)
	};

	/**
	* @class StateQueue
	*
	* @brief Implements a dynamic State queue.
	*
	* This class is used by the @c StateController class to schedule states into the future for execution.
	**/
	class StateQueue
	{
	public:
		// Constants
		static const index_t DEF_QUEUE_CAPACITY = 16; //!< @brief Default queue capacity to reserve in memory on reset

		// Constructors
		StateQueue()          { reset(); }      //!< @brief Default constructor
		virtual ~StateQueue() { clear_hard(); } //!< @brief Destructor

		// Reset function
		void reset(); //!< @brief Reinitialise the StateQueue object

		//
		// Queue manipulation functions
		//

		// Clearing
		void clear()                                   { QueueVec.resize(0); }                                                            //!< @brief For use by state controller states (guaranteed not to cause vector reallocation)
		void clear_hard()                              { QueueVec.clear(); }                                                              //!< @brief For use during reset/initialisation (may trigger vector reallocation)
		void reserveCapacity(index_t n)                { if(n <= QueueVec.max_size()) QueueVec.reserve(n); }                              //!< @brief Increase the queue capacity so that it is at least @p n (does nothing if the capacity is already at least @p n, or @p n is @c nullIndex, and throws @c std::bad_alloc if the allocation fails because @p n is too large)

		// Get and set
		StatePtr get(index_t ind) const                { return (ind < QueueVec.size() ? QueueVec.at(ind) : nullStatePtr); }              //!< @brief Returns @c nullStatePtr if @p ind is out of range or @c nullIndex
		void     set(index_t ind, StatePtr state)      { if(ind < QueueVec.size()) QueueVec.at(ind) = state; }                            //!< @brief Does nothing if @p ind is out of range or @c nullIndex

		// Element insertion and removal
		void     setNextState(StatePtr state)          { QueueVec.assign(1,state); }                                                      //!< @brief Clears the entire queue and sets the next state to the given state instance (use only with simple next state logic!)
		StatePtr getNextState() const                  { return (QueueVec.size() > 0 ? QueueVec.at(0) : nullStatePtr); }                  //!< @brief Returns @c nullStatePtr if queue is empty
		void append (StatePtr state)                   { QueueVec.push_back(state); }                                                     //!< @brief Always successful
		void prepend(StatePtr state)                   { QueueVec.insert(QueueVec.begin(), state); }                                      //!< @brief Always successful
		void insertBefore(index_t ind, StatePtr state) { if(ind <= QueueVec.size()) QueueVec.insert(QueueVec.begin() + ind, state); }     //!< @brief Does nothing if @p ind is out of range or @c nullIndex (in this case the allowed range is up to `QueueVec.size() = Queue.length()` inclusive)
		void insertAfter (index_t ind, StatePtr state) { if(ind <  QueueVec.size()) QueueVec.insert(QueueVec.begin() + ind + 1, state); } //!< @brief Does nothing if @p ind is out of range or @c nullIndex
		void remove(index_t ind)                       { if(ind <  QueueVec.size()) QueueVec.erase (QueueVec.begin() + ind); }            //!< @brief Does nothing if @p ind is out of range or @c nullIndex

		// Queue information
		index_t length() const                         { return QueueVec.size(); }                                                        //!< @brief Always successful
		bool    isEmpty() const                        { return QueueVec.empty(); }                                                       //!< @brief Always successful
		bool    validIndex(index_t ind) const          { return ind < QueueVec.size(); }                                                  //!< @brief Always successful (function returns false for @c nullIndex)

		// Head and tail items
		index_t headIndex() const                      { return (QueueVec.size() > 0 ? 0 : nullIndex); }                                  //!< @brief Returns @c nullIndex if queue is empty
		index_t tailIndex() const                      { return (QueueVec.size() > 0 ? QueueVec.size()-1 : nullIndex); }                  //!< @brief Returns @c nullIndex if queue is empty
		StatePtr headItem() const                      { return (QueueVec.size() > 0 ? QueueVec.at(0) : nullStatePtr); }                  //!< @brief Returns @c nullStatePtr if queue is empty
		StatePtr tailItem() const                      { return (QueueVec.size() > 0 ? QueueVec.at(QueueVec.size()-1) : nullStatePtr); }  //!< @brief Returns @c nullStatePtr if queue is empty

		// Queue advance function (pops and returns the state sitting at the head of the queue)
		StatePtr advance();                                                                                                               //!< @brief Returns @c nullStatePtr if queue is empty

		// Queue state and index search functions
		bool containsID(int id) const                  { return (indexOf(id) != nullIndex); }                                             //!< @brief Always successful
		bool containsState(StatePtr state) const       { return (indexOf(state) != nullIndex); }                                          //!< @brief Always successful

		// Queue state and index retrieval functions
		StatePtr stateOf(int id, index_t start = 0) const;                                                                                //!< @brief Returns @c nullStatePtr if the given ID is not found
		index_t  indexOf(int id, index_t start = 0) const;                                                                                //!< @brief Returns @c nullIndex if the given ID is not found
		index_t  indexOf(StatePtr state, index_t start = 0) const;                                                                        //!< @brief Returns @c nullIndex if the given state is not found

	private:
		// Internal queue vector object
		std::vector<StatePtr> QueueVec; //!< @brief The internal State queue vector object
	};

	/**
	* @class StateController
	*
	* @brief Base class for all state controllers.
	*
	* All state controllers implemented using this library need to derive from this abstract
	* base class, which provides the underlying functionality.
	**/
	class StateController
	{
	public:
		//
		// Constructors
		//

		// Constructors and destructors
		explicit StateController(const std::string& name) : name(name), m_inCallback(false) { reset(); } //!< @brief Default constructor
		virtual ~StateController() { freeResources(); } //!< @brief Destructor

		// State controller properties
		const std::string name; //!< @brief The human-friendly name of the state controller

		//
		// Public functions (permitted in callbacks)
		//

		// State control functions
		action_t goToState(StatePtr state); //!< @brief Transition to the specified next state (for callback)
		action_t terminate();               //!< @brief Terminate the execution of the state controller (for callback)

		// Helper functions
		bool belongsToMe(StateConstPtr state) const { return (state->scref() == this); }   //!< @brief Check whether a state is owned by the calling state controller
		bool isActive() const                       { return !State::isNull(m_curState); } //!< @brief Return whether the state controller is active or not
		cycle_t getCycle() const                    { return m_cycle; }                    //!< @brief Return the current cycle count (zero after reset)
		StateConstPtr getCurState() const           { return m_curState; }                 //!< @brief Return a constant reference to the current state

		//
		// Public functions (callback-protected)
		//

		// Initialisation and state change functions
		ret_t init(StatePtr state);       //!< @brief Reset and initialise the state controller with a particular state (callback-protected)
		ret_t forceState(StatePtr state); //!< @brief Force a change of state of the state controller (callback-protected)

		// Step functions
		ret_t step();                     //!< @brief Execute a single step of the state controller (callback-protected)
		ret_t loop();                     //!< @brief Execute state controller steps until either an error or clean termination occurs (callback-protected)

	protected:
		//
		// Callbacks
		//

		// Step callbacks
		virtual bool preStepCallback() { return false; }           //!< @brief A callback that is executed at the very beginning of the `step()` function, right after the cycle count is incremented, whenever the current state is not null @return Return `true` if you want to force a state transition to the state at the head of the queue (which you can modify using the `goToState()` function for example), otherwise return `false`.
		virtual void preActivateCallback(bool willCallActivate) {} //!< @brief A callback that is executed in the `step()` function immediately prior to the activation of a state (called with @p willCallActivate true if there is a new state to activate)
		virtual void preExecuteCallback() {}                       //!< @brief A callback that is executed in the `step()` function immediately prior to the execution of the current state (called without exception whenever there is a state to be executed)
		virtual bool postExecuteCallback() { return false; }       //!< @brief A callback that is executed in the `step()` function immediately after the execution of the current state (called without exception whenever there is a state that was just executed) @return Return `true` if you want to force a state transition to the state at the head of the queue (which you can modify using the `goToState()` function for example), otherwise return `false`.
		virtual void onTerminateCallback() {}                      //!< @brief A callback that is executed in the `step()` function (before the resources are freed) when a state controller termination condition is encountered
		virtual void postStepCallback() {}                         //!< @brief A callback that is executed at the very end of the `step()` function whenever the current state is not null (in particular, it is NOT called after `onTerminateCallback()` in the case of a termination condition, as termination resets the current state)

	private:
		//
		// Private members
		//

		// Friend classes
		friend class State; // However, all derived classes of State are NOT friends for privacy reasons!

		// Internal functions
		ret_t reset();
		void freeResources(); // This function clears m_curState, m_stateAction, m_inCallback and Q, but not anything else. In particular, m_cycle retains its value, so getCycle() can still be used. Calling step() again after the resources have been freed returns SCR_TERMINATED (but still increments m_cycle). The user needs to call init() again (or forceState()) in order to be able to step() again.
		void setCurState(StatePtr state, action_t hold = PROCEED_NEXT_STATE);

		// Internal variables
		bool m_terminate;
		bool m_inCallback;
		action_t m_stateAction;
		cycle_t m_cycle;

		// Current state
		StatePtr m_curState;

		// State queue
		StateQueue Q;
	};

	//
	// State class
	//

	// Get functions
	inline StateQueue* State::Queue() const // Defined here as inline (but cannot be defined before StateController class)
	{
		// Return the required reference
		return &(scref()->Q);
	}
}

#endif /* STATE_CONTROLLER_H */
// EOF