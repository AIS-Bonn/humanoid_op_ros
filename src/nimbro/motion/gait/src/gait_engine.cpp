// Base class for all gait engines
// File: gait_engine.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <gait/gait_engine.h>
#include <pluginlib/class_list_macros.h>

// Defines
#define DEFAULT_HALT_EFFORT_ARMS  0.25 // Default joint effort to use in the halt pose for all the arm joints
#define DEFAULT_HALT_EFFORT_LEGS  0.50 // Default joint effort to use in the halt pose for all the leg joints

// Namespaces
using namespace gait;

//
// GaitEngine class
//

// Step function
void GaitEngine::step()
{
	// Update the halt pose
	updateHaltPose();

	// By default just return the halt pose of the robot
	for(int i = 0; i < NUM_JOINTS; i++)
	{
		out.jointCmd[i] = haltJointCmd[i];
		out.jointEffort[i] = haltJointEffort[i];
	}
	out.useRawJointCmds = haltUseRawJointCmds;

	// Set the status flags
	out.walking = in.gaitCmd.walk;

	// Set the support coefficients
	out.supportCoeffLeftLeg = 0.5;
	out.supportCoeffRightLeg = 0.5;

	// Update the odometry
	updateOdometry();
}

// Update the robot's halt pose
void GaitEngine::updateHaltPose()
{
	// By default just set halt as the zero pose of the robot
	for(int i = ARMS_BEGIN; i <= ARMS_END; i++)
	{
		haltJointCmd[i] = 0.0;
		haltJointEffort[i] = DEFAULT_HALT_EFFORT_ARMS;
	}
	for(int i = LEGS_BEGIN; i <= LEGS_END; i++)
	{
		haltJointCmd[i] = 0.0;
		haltJointEffort[i] = DEFAULT_HALT_EFFORT_LEGS;
	}
	haltUseRawJointCmds = false;
}

// Set the robot's odometry
void GaitEngine::setOdometry(double posX, double posY, double rotZ)
{
	// Transcribe the values
	m_posX = posX;
	m_posY = posY;
	m_rotZ = rotZ;

	// Update the odometry data members
	updateOdometry();
}

// Update the robot's odometry
void GaitEngine::updateOdometry()
{
	// Update the position
	out.odomPosition[0] = m_posX;
	out.odomPosition[1] = m_posY;
	out.odomPosition[2] = 0.0;

	// Update the orientation
	out.odomOrientation[0] = cos(0.5*m_rotZ);
	out.odomOrientation[1] = 0.0;
	out.odomOrientation[2] = 0.0;
	out.odomOrientation[3] = sin(0.5*m_rotZ);
}

PLUGINLIB_EXPORT_CLASS(gait::GaitEngine, gait::GaitEngine)
// EOF