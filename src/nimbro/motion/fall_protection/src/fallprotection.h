// Fall protection for the robot
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Hafez Farazi <farazi@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FALLPROTECTION_H
#define FALLPROTECTION_H

// Includes
#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/FadeTorqueAction.h>
#include <actionlib/client/simple_action_client.h>
#include <config_server/parameter.h>
#include <plot_msgs/plot_manager.h>

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
		static const std::string m_walkingStatePrefix;

		// Config server parameters
		config_server::Parameter<bool>  m_fallProtectionEnabled;
		config_server::Parameter<bool>  m_sideGetupsEnabled;
		config_server::Parameter<bool>  m_landingEnabled;
		config_server::Parameter<bool>  m_landingProneEnabled;
		config_server::Parameter<bool>  m_landingSupineEnabled;
		config_server::Parameter<float> m_fallTriggerAngle;
		config_server::Parameter<float> m_fallTriggerAngleKick;
		config_server::Parameter<float> m_fallTriggerAngleRelaxed;
		config_server::Parameter<float> m_sideGetupRollAngle;
		config_server::Parameter<float> m_maxLandingDuration;
		config_server::Parameter<float> m_angleToRelaxAfterLanding;
		config_server::Parameter<float> m_maxRollForLanding;

		// Robot states
		robotcontrol::RobotModel::State m_state_relaxed;
		robotcontrol::RobotModel::State m_state_setting_pose;
		robotcontrol::RobotModel::State m_state_init;
		robotcontrol::RobotModel::State m_state_standing;
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
		robotcontrol::RobotModel::State m_state_landing_prone;
		robotcontrol::RobotModel::State m_state_landing_supine;

		// Fade torque action client
		actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction> m_fadeTorqueAction;

		// Falling state enumeration
		enum FallingState
		{
			FS_NONE = 0,
			FS_TRIGGERED,
			FS_LANDING_PRONE,
			FS_LANDING_SUPINE,
			FS_RELAXED,
			FS_COUNT
		};

		// Internal variables
		bool m_enabled;
		double m_currentAngle;
		bool m_relaxLock;
		bool m_fall_triggered;
		ros::Time m_triggerTime;
		bool m_useLanding;
		bool m_isLanding;
		FallingState m_fallingState;

		// Plot manager
		enum PMIds
		{
			PM_FALLING_STATE = 0,
			PM_COUNT
		};
		plot_msgs::PlotManagerFS m_PM;
	};
}

#endif /* FALLPROTECTION_H */
// EOF
