// Motion module for individual control of robot limbs
// Author: Philipp Allgeuer

// Includes
#include <limb_control/limb_control.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <cmath>

// Defines - Misc
#define MIN_EFFORT          0.000 // Units: Unit interval
#define MAX_EFFORT          1.000 // Units: Unit interval
#define MIN_PERIOD_SPEC     0.020 // Units: s
#define MIN_ANG_VEL_SPEC    0.020 // Units: rad/s
#define MAX_ANG_VEL_SPEC    4.000 // Units: rad/s
#define MIN_SINE_CYCLES     0.020 // Units: cycles
#define MIN_SINE_AMPLITUDE  0.000 // Units: rad

// Namespaces
using namespace limb_control;
using namespace robotcontrol;

// Typedefs
typedef std::list<LimbCmd>::iterator LCAIt;
typedef std::list<LimbCmd>::const_iterator LCAConstIt;

//
// LimbControl class
//

// Constructor
LimbControl::LimbControl()
 : CONFIG_PARAM_PATH("/limbControl/")
 , m_model(NULL)
 , m_limbControlEnabled(CONFIG_PARAM_PATH + "limbControlEnabled", true)
 , m_motionStartTime(0.0)
 , m_curTimeRef(INVALID_TIME_REF)
 , m_numMotions(0)
{
	// Create node handle
	ros::NodeHandle nh("~");

	// Advertise ROS services
	m_srv_playCommands = nh.advertiseService("/limb_control/playCommands", &LimbControl::handlePlayCommands, this);

	// Initialise variables
	m_lastEnabledState = m_limbControlEnabled();
	m_LCA.clear();
}

// Initialisation function
bool LimbControl::init(RobotModel* model)
{
	// Save the reference to the owning RobotModel
	m_model = model;
	
	// Initialise the motion module
	MotionModule::init(m_model);
  
	// Return success
	return true;
}

// Trigger function
bool LimbControl::isTriggered()
{
	// Reset the limb control state if the enabled flag changes
	if(m_limbControlEnabled() != m_lastEnabledState)
	{
		m_LCA.clear();
		m_curTimeRef = INVALID_TIME_REF;
		m_numMotions = 0;
		m_lastEnabledState = m_limbControlEnabled();
	}

	// Return whether the step function of this motion module should be called
	return m_limbControlEnabled() && !m_LCA.empty();
}

// Step function
void LimbControl::step()
{
	// Nothing to do if our limb command array is empty
	if(m_LCA.empty())
		return;

	// Save the current step time
	const ros::Time curTime = ros::Time::now();

	// Retrieve the robotcontrol time step value
	const double dT = m_model->timerDuration();

	// Declare variables
	LCAConstIt itEnd, itc;

	// If we are not currently executing a motion then retrieve the next one from the array
	if(m_curTimeRef == INVALID_TIME_REF)
	{
		// Declare variables
		uint8_t timeRef = INVALID_TIME_REF;
		LimbCmd* LCmd = NULL;

		// Retrieve the first non-null command from the LCA array
		while(true)
		{
			if(m_LCA.empty()) return;
			LCmd = &(m_LCA.front());
			timeRef = LCmd->LC.timeref;
			if(timeRef == INVALID_TIME_REF)
				m_LCA.pop_front();
			else break;
		}

		// Find and count all subsequent motions with the same timeref (guaranteed to be at least one given the construction of timeRef)
		m_numMotions = 0;
		for(itc = m_LCA.begin(), itEnd = m_LCA.end(); itc != itEnd; itc++)
		{
			if(itc->LC.timeref != timeRef) break;
			m_numMotions++;
		}

		// Start the motion playing
		m_motionStartTime = curTime;
		m_curTimeRef = timeRef; // Guaranteed not to be INVALID_TIME_REF!
	}

	// The following assumptions apply for the following code:
	// - m_curTimeRef is not invalid
	// - The first m_numMotions commands in m_LCA all have the timeref m_curTimeRef and refer to unique joints
	// - m_motionStartTime is a ROS time in the past

	// Declare variables
	double t, motionTime, jointCmd;
	bool stillGoing;
	size_t i;
	LCAIt it;

	// Calculate the motion time
	motionTime = (curTime - m_motionStartTime).toSec();
	if(motionTime < 0) motionTime = 0;

	// Loop through the current commands and update each joint
	stillGoing = false;
	for(i = 0, it = m_LCA.begin(), itEnd = m_LCA.end(); (i < m_numMotions) && (it != itEnd); i++, it++)
	{
		// Calculate the time index
		if(motionTime >= it->duration)
			t = it->duration;
		else
		{
			t = motionTime;
			stillGoing = true;
		}

		// Calculate the desired joint command
		switch(it->LC.type)
		{
			case TYPE_SETPOINT_TIME:
			case TYPE_SETPOINT_VEL:
				if(it->LC.position >= it->initPos)
					jointCmd = it->initPos + it->LC.velocity * t;
				else
					jointCmd = it->initPos - it->LC.velocity * t;
				break;
			case TYPE_SINE_WAVE:
				jointCmd = it->LC.position + it->LC.amplitude * std::sin(M_2PI * (t / it->LC.period) + it->LC.phase);
				break;
			default:
				jointCmd = it->initPos;
				break;
		}

		// Coerce the desired joint position to [-pi,pi]
		if(jointCmd < -M_PI) jointCmd = -M_PI;
		if(jointCmd >  M_PI) jointCmd =  M_PI;

		// Write the required joint command
		it->joint->cmd.setFromPos(dT, jointCmd);
		it->joint->cmd.raw = false;
		it->joint->cmd.effort = it->LC.effort;
	}

	// Check whether all the current motions have finished playing and react appropriately if so
	if(!stillGoing)
	{
		ROS_INFO("A limb control motion with time ref %u, controlling %u joints, just finished.", (unsigned) m_curTimeRef, (unsigned) m_numMotions);
		if(m_numMotions >= m_LCA.size())
			m_LCA.clear();
		else
		{
			for(i = 0; i < m_numMotions; i++)
				m_LCA.pop_front();
		}
		m_curTimeRef = INVALID_TIME_REF;
		m_numMotions = 0;
	}
}

// Service handler for the play commands service
bool LimbControl::handlePlayCommands(limb_control::PlayCommands::Request& req, limb_control::PlayCommands::Response& res)
{
	// Ignore the message if limb control is disabled
	if(!m_limbControlEnabled())
	{
		ROS_WARN("Limb commands received but ignored as limb control is disabled!");
		res.timeleft = 0.0;
		return false;
	}

	// Declare variables
	size_t numCmds = req.commands.size();
	size_t numValid = 0;
	uint8_t lastTimeRef;
	LimbCmd LCmd;

	// Retrieve the current last timeref in the internal LCA array
	if(m_LCA.empty())
		lastTimeRef = INVALID_TIME_REF;
	else
		lastTimeRef = m_LCA.back().LC.timeref;

	// Transcribe the limb commands to the internal LCA array
	for(size_t i = 0; i < numCmds; i++)
	{
		// Retrieve a pointer to the limb command and the corresponding joint
		const LimbCommand* LC = &(req.commands[i]);
		Joint::Ptr joint = m_model->getJoint(LC->joint);

		// Process and check the command if it's not just an INVALID_TIME_REF spacer anyway
		if(LC->timeref == INVALID_TIME_REF)
			LCmd.LC = *LC;
		else
		{
			// Check whether the limb command is valid
			// Note: LC->timeref is not checked for INVALID_TIME_REF so that such null commands can hypothetically be used to splice apart separate motions
			if(!joint || (LC->type == TYPE_INVALID || LC->type >= NUM_TYPES) || (LC->effort < MIN_EFFORT || LC->effort > MAX_EFFORT) ||
			(LC->type != TYPE_SETPOINT_VEL && (LC->period < MIN_PERIOD_SPEC)) || (LC->position < -M_PI || LC->position > M_PI) ||
			(LC->type == TYPE_SETPOINT_VEL && (LC->velocity < MIN_ANG_VEL_SPEC)) ||
			(LC->type == TYPE_SINE_WAVE && (LC->amplitude < MIN_SINE_AMPLITUDE || LC->phase < -M_2PI || LC->phase > M_2PI || LC->cycles < MIN_SINE_CYCLES)))
			{
				ROS_WARN("Ignoring invalid limb command #%u", (unsigned) i);
				continue;
			}

			// Transcribe the current LimbCommand into our (local) LimbCmd object
			LCmd.LC = *LC;

			// Save the pointer to the required joint
			LCmd.joint = joint;

			// Save the true initial/current position of the joint
			LCmd.initPos = joint->lastCmd.pos;

			// Processing for types: TYPE_SETPOINT_TIME, TYPE_SETPOINT_VEL
			if(LCmd.LC.type == TYPE_SETPOINT_TIME || LCmd.LC.type == TYPE_SETPOINT_VEL)
			{
				if(LCmd.LC.type == TYPE_SETPOINT_TIME)
				{
					LCmd.LC.velocity = fabs((LCmd.LC.position - LCmd.initPos) / LCmd.LC.period);
					LCmd.LC.type = TYPE_SETPOINT_VEL;
				}
				if(LCmd.LC.velocity < MIN_ANG_VEL_SPEC)
					LCmd.LC.velocity = MIN_ANG_VEL_SPEC;
				if(LCmd.LC.velocity > MAX_ANG_VEL_SPEC)
					LCmd.LC.velocity = MAX_ANG_VEL_SPEC;
				LCmd.duration = fabs((LCmd.LC.position - LCmd.initPos) / LCmd.LC.velocity);
			}

			// Processing for type: TYPE_SINE_WAVE
			if(LCmd.LC.type == TYPE_SINE_WAVE)
			{
				LCmd.duration = LCmd.LC.cycles * LCmd.LC.period;
			}

			// Insert a null timeref command if required to separate the new motions from existing motions in the local limb command array
			if(numValid == 0 && LCmd.LC.timeref == lastTimeRef && lastTimeRef != INVALID_TIME_REF)
			{
				LimbCmd LCmdTmp;
				LCmdTmp.LC.timeref = INVALID_TIME_REF;
				m_LCA.push_back(LCmdTmp);
				lastTimeRef = INVALID_TIME_REF;
			}
		}

		// Transcribe the processed limb command into our local command array
		m_LCA.push_back(LCmd);
		numValid++;
	}

	// Remove duplicate joints from m_LCA
	removeDuplicateJoints();

	// Return to the service caller an estimate of how long it will take (in sec) before all motions that are now enqueued in LCA will finish playing (including the ones that were already there from previous service calls)
	res.timeleft = (float) getRemainingTime();

	// Inform the user that limb commands were received
	if(numCmds > 0)
	{
		ROS_INFO("Received %u limb commands with %u/%u valid!", (unsigned) numCmds, (unsigned) numValid, (unsigned) numCmds);
		ROS_INFO("Total remaining limb control time is now %.3f seconds.", res.timeleft);
	}

	// Return success
	return true;
}

// Remove any duplicate joints from the LCA array
void LimbControl::removeDuplicateJoints()
{
	// Nothing to do if LCA is empty
	if(m_LCA.empty())
		return;
	
	// Declare variables
	LCAIt it;
	LCAConstIt itb, itc, itEnd;
	bool duplicateJoint;
	std::string name;
	uint8_t timeRef;

	// Initialise variables
	timeRef = INVALID_TIME_REF;

	// Remove any limb commands that specify the same joint as another limb command in the same motion
	it = m_LCA.begin();
	itb = it;
	while(it != m_LCA.end())
	{
		// Ignore all commands with invalid time references, but make sure they are still noted as motion separators
		if(it->LC.timeref == INVALID_TIME_REF)
		{
			it++;
			itb = it;
			timeRef = INVALID_TIME_REF;
			continue;
		}

		// If command has different timeref than we've last seen then it's part of a new motion
		if(it->LC.timeref != timeRef)
		{
			itb = it;
			timeRef = it->LC.timeref;
		}

		// Check whether the command refers to the same underlying joint as a previous command
		duplicateJoint = false;
		name = it->joint->name;
		for(itc = itb; itc != it; itc++)
		{
			if(name == itc->joint->name)
			{
				duplicateJoint = true;
				break;
			}
		}

		// Discard this command if it refers to the same underlying joint as a previous command
		if(duplicateJoint)
			it = m_LCA.erase(it);
		else
			it++;

		// Warn if a duplicate joint was detected
		if(duplicateJoint)
			ROS_WARN("Ignoring duplicate command (within same motion) for joint '%s'!", name.c_str());
	}
}

// Analyse the current internal LCA array and predict how much time will be needed from now to complete the remaining motions
double LimbControl::getRemainingTime()
{
	// Note: This assumes that there are no duplicate joints!

	// If not playing a motion then we have no time remaining
	if(!isTriggered() || m_LCA.empty())
		return 0.0;

	// Declare variables
	LCAConstIt it, itEnd;
	double maxDuration, totalDuration;
	uint8_t timeRef;

	// Initialise variables
	maxDuration = 0.0;
	totalDuration = 0.0;
	timeRef = INVALID_TIME_REF;

	// Loop through command by command, find the motion groups, and sum up their respective durations
	for(it = m_LCA.begin(), itEnd = m_LCA.end(); it != itEnd; it++)
	{
		// Ignore all commands with invalid time references
		if(it->LC.timeref == INVALID_TIME_REF || it->LC.timeref != timeRef)
		{
			totalDuration += maxDuration;
			maxDuration = 0.0;
			timeRef = it->LC.timeref;
			if(it->LC.timeref == INVALID_TIME_REF) continue;
		}

		// See if this command is the longest one of the motion
		if(it->duration > maxDuration)
			maxDuration = it->duration;
	}
	totalDuration += maxDuration;

	// Subtract the amount of time that we're already underway into the current motion
	totalDuration -= getCurrentMotionTime();
	if(totalDuration < 0.0) totalDuration = 0.0;

	// Return the given time left
	return totalDuration;
}

// Get the time that the current motion has been underway
double LimbControl::getCurrentMotionTime() const
{
	// Return the current motion time, and 0.0 if no motion is underway
	if(m_curTimeRef != INVALID_TIME_REF)
	{
		const ros::Time curTime = ros::Time::now();
		double timeRunning = (curTime - m_motionStartTime).toSec();
		return (timeRunning >= 0.0 ? timeRunning : 0.0);
	}
	else
	{
		return 0.0;
	}
}

PLUGINLIB_EXPORT_CLASS(limb_control::LimbControl, robotcontrol::MotionModule)
// EOF