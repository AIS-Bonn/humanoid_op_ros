// Finite state machine for motion demonstration on a standing robot
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Includes
#include <standing_demo/standing_demo_fsm.h>
#include <standing_demo/standing_demo_actions.h>
#include <standing_demo/standing_demo.h>
#include <ros/console.h>

// Namespaces
using namespace standing_demo;
using namespace limb_control;

//
// StandingDemoSC class
//

// Constructor
StandingDemoSC::StandingDemoSC(StandingDemo* SD)
: StateController("StandingDemoSC")
, m_SD(SD)
, m_headIdleEnabled(false)
{
}

// State controller callbacks
bool StandingDemoSC::preStepCallback()
{
	// Update the current step time
	m_curStepTime = m_SD->curStepTime();
	
	// We do no wish to force a state transition
	return false;
}
void StandingDemoSC::preActivateCallback(bool willCallActivate)
{
	// If a new state is being activated then let the world know about it
	if(willCallActivate)
	{
		// Retrieve the state that is being activated
		StateConstPtr newState = getCurState();
		cycle_t cyc = getCycle();
		
		// Publish the state being activated over ROS
		demo_msgs::DemoFSMState msg;
		msg.name  = newState->name;
		msg.id    = (demo_msgs::DemoFSMState::_id_type) newState->id;
		msg.cycle = (demo_msgs::DemoFSMState::_cycle_type) cyc;
		m_SD->publishFSMState(msg);
		
		// Display a console message for the new state
		if(newState->id == PLAN_DEMO_ACTION)
		{
			ROS_INFO(" ");
			ROS_WARN("Cycle %u => Entering state %s", (unsigned) cyc, newState->name.c_str());
		}
		else
		{
			ROS_INFO("Cycle %u => Entering state %s", (unsigned) cyc, newState->name.c_str());
		}
	}
}
void StandingDemoSC::preExecuteCallback()
{
	// Handle the background motions
	if(m_headIdleEnabled) handleHeadIdle();
}


void StandingDemoSC::enableHeadIdle()
{
	// Enable head idling if it is not already enabled
	if(!m_headIdleEnabled)
	{
		// Set the enabled flag
		m_headIdleEnabled = true;
	}
}
void StandingDemoSC::disableHeadIdle()
{
	// Disable head idling if it is not already disabled
	if(m_headIdleEnabled)
	{
		// Reset the enabled flag
		m_headIdleEnabled = false;
	}
}


void StandingDemoSC::handleHeadIdle()
{
	// TODO: Handle background head idling
}

//
// InitDemoState class
//

// State callbacks
action_t InitDemoState::execute(cycle_t cyc)
{
	// Clear the current queue
	Queue()->clear();
	
	// Set up the initial state of the queue
	Queue()->append(NewStateInstance<WaitForStandingState>(sc));
	Queue()->append(NewStateInstance<WaitForTimeState>(sc, INIT_DELAY));
	Queue()->append(NewStateInstance<PlanDemoActionState>(sc));
	
	// Proceed to the next enqueued state
	return PROCEED_NEXT_STATE;
}

//
// WaitForStandingState class
//

// State callbacks
action_t WaitForStandingState::execute(cycle_t cyc)
{
	// Hold this state until the robot state changes to standing
	if(sc->m_SD->robotIsStanding())
		return PROCEED_NEXT_STATE;
	else
		return HOLD_THIS_STATE;
}

//
// WaitForTimeState class
//

// Constants
const ros::Time WaitForTimeState::NullROSTime = ros::Time(0, 0);

// State callbacks
void WaitForTimeState::activate(cycle_t cyc)
{
	// Decide on the ROS time at which our wait is over
	if(m_endTime.isZero())
	{
		if(m_waitTime < 0.0) m_waitTime = 0.0;
		m_endTime = sc->now() + ros::Duration(m_waitTime);
	}
}
action_t WaitForTimeState::execute(cycle_t cyc)
{
	// If the given ROS time has passed then our wait is over...
	if(sc->now() >= m_endTime)
		return PROCEED_NEXT_STATE;
	else
		return HOLD_THIS_STATE;
}

//
// WaitForMotionPlayerState class
//

// State callbacks
action_t WaitForMotionPlayerState::execute(cycle_t cyc)
{
	// Hold this state until the robot state changes back to standing (as this is what should happen when the currently playing motion finishes)
	if(sc->m_SD->robotIsStanding())
		return PROCEED_NEXT_STATE;
	else
		return HOLD_THIS_STATE;
}

//
// WaitForLimbControlState
//

// State callbacks
action_t WaitForLimbControlState::execute(cycle_t cyc)
{
	// If we know that the limb control shouldn't be finished yet, then just hold this state
	if(sc->now() < sc->m_SD->limbControlFinishTime())
		return HOLD_THIS_STATE;
	
	// Query the limb control with a null service call to find out how much time is left
		double timeleft = sc->m_SD->sendLimbCommands(PCmdReqNull);
		
		// Proceed to the next state if no time is left
		if(timeleft <= 0.0)
			return PROCEED_NEXT_STATE;
		else
			return HOLD_THIS_STATE;
}

//
// PlanDemoActionState class
//

// State callbacks
action_t PlanDemoActionState::execute(cycle_t cyc)
{
	// Clear the current queue
	Queue()->clear();
	
	// Add the required prefix states to the queue
	Queue()->append(NewStateInstance<WaitForMotionPlayerState>(sc));
	Queue()->append(NewStateInstance<WaitForLimbControlState>(sc));
	Queue()->append(NewStateInstance<WaitForTimeState>(sc, PRE_ACTION_DELAY));
	
	// Pick a random action
	int action = (rand() % NUM_ACTIONS);
	
	// Just to be extra sure
	if(action < 0 || action >= NUM_ACTIONS) action = ACT_DO_NOTHING;

	// TODO: TEMP Disable head idling
	if(action == ACT_HEAD_IDLE) action = ACT_PLAY_MOTIONS;

	// Let the user know which action was picked
	ROS_WARN("Performing action %s:", DemoActionName[action].c_str());
	
	// Publish the action being started over ROS
	demo_msgs::DemoFSMState msg;
	msg.name  = DemoActionName[action];
	msg.id    = (demo_msgs::DemoFSMState::_id_type) action;
	msg.cycle = (demo_msgs::DemoFSMState::_cycle_type) cyc;
	sc->m_SD->publishAction(msg);

	// Perform the selected action
	StateQueue* Q = Queue();
	switch(action)
	{
		default:
		case ACT_DO_NOTHING:           StandingDemoAction::ActDoNothing(sc, Q); break;
		case ACT_HEAD_IDLE:            StandingDemoAction::ActHeadIdle(sc, Q); break;
		case ACT_PLAY_MOTIONS:         StandingDemoAction::ActPlayMotions(sc, Q); break;
		case ACT_PLAY_MOTIONS_HI:      StandingDemoAction::ActPlayMotionsHI(sc, Q); break;
	}

	// Add the required suffix states to the queue
	Queue()->append(NewStateInstance<ReturnToStandingState>(sc));
	Queue()->append(NewStateInstance<PlanDemoActionState>(sc));

	// Proceed to the next enqueued state
	return PROCEED_NEXT_STATE;
}

//
// ReturnToStandingState class
//

// State callbacks
void ReturnToStandingState::activate(cycle_t cyc)
{
	// Declare variables
	LimbCommand Cmd;
	
	// Clear the command array
	PCmdReq.commands.clear();
	
	// Configure the base limb command for each joint
	Cmd.type = TYPE_SETPOINT_VEL;
	Cmd.effort = RETURN_STANDING_EFFORT;
	Cmd.timeref = 1;
	Cmd.period = 0.0;
	Cmd.velocity = RETURN_STANDING_VELOCITY;
	
	// Set all joints that we have potentially modified to go back to their default standing positions with constant velocity
	Cmd.joint = NAME_HEAD_PITCH;
	Cmd.position = DSP_HEAD_PITCH;
	PCmdReq.commands.push_back(Cmd);
	
	Cmd.joint = NAME_HEAD_YAW;
	Cmd.position = DSP_HEAD_YAW;
	PCmdReq.commands.push_back(Cmd);
	
	Cmd.joint = NAME_LEFT_SHOULDER_PITCH;
	Cmd.position = DSP_LEFT_SHOULDER_PITCH;
	PCmdReq.commands.push_back(Cmd);
	
	Cmd.joint = NAME_LEFT_SHOULDER_ROLL;
	Cmd.position = DSP_LEFT_SHOULDER_ROLL;
	PCmdReq.commands.push_back(Cmd);
	
	Cmd.joint = NAME_LEFT_ELBOW_PITCH;
	Cmd.position = DSP_LEFT_ELBOW_PITCH;
	PCmdReq.commands.push_back(Cmd);
	
	Cmd.joint = NAME_RIGHT_SHOULDER_PITCH;
	Cmd.position = DSP_RIGHT_SHOULDER_PITCH;
	PCmdReq.commands.push_back(Cmd);
	
	Cmd.joint = NAME_RIGHT_SHOULDER_ROLL;
	Cmd.position = DSP_RIGHT_SHOULDER_ROLL;
	PCmdReq.commands.push_back(Cmd);
	
	Cmd.joint = NAME_RIGHT_ELBOW_PITCH;
	Cmd.position = DSP_RIGHT_ELBOW_PITCH;
	PCmdReq.commands.push_back(Cmd);
	
	// Send the limb commands
	sc->m_SD->sendLimbCommands(PCmdReq);
}
action_t ReturnToStandingState::execute(cycle_t cyc)
{
	// Wait for limb control to finish
	Queue()->prepend(NewStateInstance<WaitForLimbControlState>(sc));
	
	// Proceed to the next enqueued state
	return PROCEED_NEXT_STATE;
}

//
// StartMotionPlaybackState class
//

// State callbacks
void StartMotionPlaybackState::activate(cycle_t cyc)
{
	// Trigger the required motion
	sc->m_SD->playMotion(m_motion);
}
action_t StartMotionPlaybackState::execute(cycle_t cyc)
{
	// Proceed to the next enqueued state
	return PROCEED_NEXT_STATE;
}

//
// StopMotionPlaybackState class
//

// State callbacks
action_t StopMotionPlaybackState::execute(cycle_t cyc)
{
	// Note: As we can't (and don't wish to) abort motion playback by the motion player, we are forced to just wait until the motion is over by itself.
	
	// Hold this state until the robot state changes back to standing (as this is what should happen when the currently playing motion finishes)
	if(sc->m_SD->robotIsStanding())
		return PROCEED_NEXT_STATE;
	else
		return HOLD_THIS_STATE;
}


//
// StartHeadIdlingState class
//

// State callbacks
void StartHeadIdlingState::activate(cycle_t cyc)
{
	// TODO: Implement!
}
action_t StartHeadIdlingState::execute(cycle_t cyc)
{
	// Proceed to the next enqueued state
	return PROCEED_NEXT_STATE;
}

//
// StopHeadIdlingState class
//

// State callbacks
void StopHeadIdlingState::activate(cycle_t cyc)
{
	// TODO: Implement!
}
action_t StopHeadIdlingState::execute(cycle_t cyc)
{
	// Proceed to the next enqueued state
	return PROCEED_NEXT_STATE;
}