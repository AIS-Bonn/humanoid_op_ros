// Interface for robotcontrol motion module plugins
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Includes
#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/robotmodel.h>

// Namespaces
using namespace robotcontrol;

// Initialisation function
bool MotionModule::init(RobotModel* model)
{
	// Save the reference to the robot model
	m_model = model;

	// Check for an invalid RobotModel pointer
	if(m_model == NULL)
	{
		ROS_ERROR("Null RobotModel pointer was passed => Failed to initialise motion module!");
		return false;
	}

	// Return that initialisation was successful
	return true;
}

// Trigger function
bool MotionModule::isTriggered()
{
	// Return that the motion module is always triggered by default
	return true;
}

// Set a joint command by position and effort only
void MotionModule::setJointCommand(int index, double pos, double effort, bool raw)
{
	// Set the joint command as required
	Joint::Ptr joint = m_model->joint(index);
	joint->cmd.setFromPos(m_model->timerDuration(), pos);
	joint->cmd.effort = effort;
	joint->cmd.raw = raw;
}

bool MotionModule::isSafeToFadeIn()
{
	return true;
}
