// behaviour_exercise_template.cpp - Template by Philipp Allgeuer - 19/07/13
// Template and starting point for the behaviours exercise

// Includes
#include "behaviour_exercise/behaviour_exercise_template.h"

// Defines
#define TINC 0.4

// Namespaces
using namespace std;
using namespace behaviourexercise;

//
// RobotSC class
//

// Constructor function
void RobotSC::onConstruct()
{
	// Identify which behaviour controller is running
	ROS_WARN("Using solution by ENTER YOUR NAME (ENTER DATE)");

	// TODO: Add any code here that needs to be called from the RobotSC constructor
}

// Reset function
void RobotSC::reset() // Called when 'run' is checked by the user
{
	// TODO: Add any reset code that your state controller requires here
}

// State controller callbacks
bool RobotSC::preStepCallbackUser()
{
	// TODO: Add any code here that needs to be called at the start of every step

	// Stop the behaviours if the run flag is unset
	if(!m_be_run() && (getCurState()->id != IDLE))
	{
		ROS_INFO_STREAM_THROTTLE(TINC, "Run stop detected - Transferring to idle state...");
		sendGaitCommand(true, 0.0, 0.0, 0.0);
		goToState(NewStateInstance<IdleState>(this));
		return true; // Force a state transition
	}

	// Return that we do not wish to force a state transition
	return false;
}
void RobotSC::postStepCallbackUser()
{
	// TODO: Add any code here that needs to be called at the end of every step
}

//
// IdleState class
//

// Execute callback
action_t IdleState::execute(cycle_t cyc)
{
	// Display current state
	ROS_INFO_STREAM_THROTTLE(TINC, "In state: " << name);

	// Stop the robot from walking
	sc->sendGaitCommand(false);
	
	// Start the behaviours if the run flag is set
	if(sc->m_be_run() && (cyc > 1))
	{
		// TODO: Uncomment: ROS_INFO_STREAM_THROTTLE(TINC, "Run start detected - Initialising field...");
		// TODO: Uncomment: sc->initField(); // This sets up the field, including placing the robot, ball, obstacle, etc...
		// TODO: Uncomment: sc->reset(); // This calls the user-defined reset function
		// TODO: Uncomment: return sc->goToState(NewStateInstance<SearchForBallState>(sc));)
	}
	
	// Stay in the current state
	return HOLD_THIS_STATE;
}

//
// TODO: Define all other required state class function members here (e.g. action_t SearchForBallState::execute(cycle_t cyc))
//

// EOF