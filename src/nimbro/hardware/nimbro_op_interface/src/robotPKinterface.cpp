// NimbRo-OP robot hardware interface (parallel kinematics)
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <nimbro_op_interface/robotPKinterface.h>
#include <pluginlib/class_list_macros.h>
#include <boost/regex.hpp>

// Namespaces
using namespace nimbro_op_interface;

//
// RobotPKInterface class
//

// Parallel kinematics joint enumeration
const std::string RobotPKInterface::PKSet::PKJointName[COUNT] = {
	"thigh_pitch",
	"shank_pitch",
	"ankle_roll",
	"hip_pitch",
	"knee_pitch",
	"ankle_pitch",
	"ankle_roll"
};

// Initialise joint dependencies
bool RobotPKInterface::initJointDependencies()
{
	// Add mimic joint dependencies
	if(!addMimicJointDependencies())
		return false;

	// Add parallel kinematics joint dependencies
	if(!addPKJointDependencies())
		return false;

	// Return success
	return true;
}

// Add parallel kinematics joint dependencies
bool RobotPKInterface::addPKJointDependencies()
{
	// Regex to find the first parallel kinematics joint in a set
	boost::regex regexFirstPar("(parallel_(.*_))" + PKSet::PKJointName[PKSet::PAR_START]);
	boost::smatch matchFirstPar;

	// Find the required parallel kinematics sets and add the associated dependencies
	for(size_t i = 0; i < m_model->numJoints(); i++)
	{
		// Declare variables
		PKSet PK;

		// Get a pointer to the joint
		DXLJoint* jointFirstPar = dxlJoint(i);

		// See whether the joint name matches the first parallel joint regex
		if(!boost::regex_match(jointFirstPar->name, matchFirstPar, regexFirstPar)) continue;
		if(matchFirstPar.size() < 3) continue;
		std::string parallelPrefix = matchFirstPar[1].str();
		std::string serialPrefix = matchFirstPar[2].str();

		// Save the first parallel joint
		PK.joint[PKSet::PAR_START] = jointFirstPar;

		// Try to find matching remaining parallel joints
		for(int s = PKSet::PAR_START + 1; s < PKSet::PAR_END; s++)
			PK.joint[s] = dxlJointForName(parallelPrefix + PKSet::PKJointName[s]);

		// Try to find matching remaining serial joints
		for(int s = PKSet::SER_START; s < PKSet::SER_END; s++)
			PK.joint[s] = dxlJointForName(serialPrefix + PKSet::PKJointName[s]);

		// Check whether all joints were found
		bool validSet = true;
		for(int s = PKSet::START; s < PKSet::END; s++)
		{
			if(!PK.joint[s])
			{
				ROS_INFO("Failed to find matching '%s' joint for '%s'", PKSet::PKJointName[s].c_str(), jointFirstPar->name.c_str());
				validSet = false;
			}
		}

		// Continue if this is not a valid set
		if(!validSet)
		{
			ROS_WARN("Ignoring incomplete parallel kinematics joint set with base '%s'", jointFirstPar->name.c_str());
			continue;
		}

		// Add joint command dependencies (serial --> parallel)
		addJointCommandDependency(jointCmdEqual, &PK.joint[PKSet::PAR_TP]->cmd, &PK.joint[PKSet::SER_HP]->cmd);
		addJointCommandDependency(jointCmdSum, &PK.joint[PKSet::PAR_SP]->cmd, &PK.joint[PKSet::SER_HP]->cmd, &PK.joint[PKSet::SER_KP]->cmd);
		addJointCommandDependency(jointCmdEqual, &PK.joint[PKSet::PAR_AR]->cmd, &PK.joint[PKSet::SER_AR]->cmd);

		// Add joint torque dependencies (serial --> parallel)
		addJointTorqueDependency(jointTorqueEqual, &PK.joint[PKSet::PAR_TP]->feedback.modelTorque, &PK.joint[PKSet::SER_HP]->feedback.modelTorque);
		addJointTorqueDependency(jointTorqueSum, &PK.joint[PKSet::PAR_SP]->feedback.modelTorque, &PK.joint[PKSet::SER_HP]->feedback.modelTorque, &PK.joint[PKSet::SER_KP]->feedback.modelTorque);
		addJointTorqueDependency(jointTorqueEqual, &PK.joint[PKSet::PAR_AR]->feedback.modelTorque, &PK.joint[PKSet::SER_AR]->feedback.modelTorque);

		// Add joint feedback dependencies (parallel --> serial)
		addJointFeedbackDependency(jointFeedEqual, &PK.joint[PKSet::SER_HP]->feedback, &PK.joint[PKSet::PAR_TP]->feedback);
		addJointFeedbackDependency(jointFeedDiff, &PK.joint[PKSet::SER_KP]->feedback, &PK.joint[PKSet::PAR_SP]->feedback, &PK.joint[PKSet::PAR_TP]->feedback);
		addJointFeedbackDependency(jointFeedNegate, &PK.joint[PKSet::SER_AP]->feedback, &PK.joint[PKSet::PAR_SP]->feedback);
		addJointFeedbackDependency(jointFeedEqual, &PK.joint[PKSet::SER_AR]->feedback, &PK.joint[PKSet::PAR_AR]->feedback);
	}

	// Return success
	return true;
}

PLUGINLIB_EXPORT_CLASS(nimbro_op_interface::RobotPKInterface, robotcontrol::HardwareInterface);
// EOF
