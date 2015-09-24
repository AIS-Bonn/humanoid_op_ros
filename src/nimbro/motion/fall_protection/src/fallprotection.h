// Fall protection for the robot
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FALLPROTECTION_H
#define FALLPROTECTION_H

// Includes
#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/FadeTorqueAction.h>
#include <actionlib/client/simple_action_client.h>
#include <config_server/parameter.h>

// Fall protection namespace
namespace fall_protection
{
	/**
	* @class FallProtection
	*
	* @brief Motion module that provides fall protection for the robot.
	**/
	class FallProtection : public robotcontrol::MotionModule
	{
	public:
		// Constructor/destructor
		FallProtection();
		virtual ~FallProtection() {}

		// Motion module function overrides
		virtual bool init(robotcontrol::RobotModel* model);
		virtual bool isTriggered();
		virtual void step();

	private:
		// Constants
		static const std::string m_lyingPlayStatePrefix;

		// Config server parameters
		config_server::Parameter<bool>  m_fallProtectionEnabled;
		config_server::Parameter<bool>  m_sideGetupsEnabled;
		config_server::Parameter<float> m_fallTriggerAngle;
		config_server::Parameter<float> m_fallTriggerAngleKick;
		config_server::Parameter<float> m_sideGetupRollAngle;

		// Robot states
		robotcontrol::RobotModel::State m_state_relaxed;
		robotcontrol::RobotModel::State m_state_setting_pose;
		robotcontrol::RobotModel::State m_state_init;
		robotcontrol::RobotModel::State m_state_sitting;
		robotcontrol::RobotModel::State m_state_kicking;
		robotcontrol::RobotModel::State m_state_falling;
		robotcontrol::RobotModel::State m_state_getting_up;
		robotcontrol::RobotModel::State m_state_lying_prone;
		robotcontrol::RobotModel::State m_state_lying_supine;
		robotcontrol::RobotModel::State m_state_lying_side;
		robotcontrol::RobotModel::State m_state_lying_biased;
		robotcontrol::RobotModel::State m_state_lying_side_left_prone;
		robotcontrol::RobotModel::State m_state_lying_side_left_supine;
		robotcontrol::RobotModel::State m_state_lying_side_right_prone;
		robotcontrol::RobotModel::State m_state_lying_side_right_supine;

		// Fade torque action client
		actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction> m_fadeTorqueAction;

		// Internal variables
		bool m_enabled;
		bool m_fall_triggered;
		ros::Time m_triggerTime;
	};
}

#endif /* FALLPROTECTION_H */
// EOF