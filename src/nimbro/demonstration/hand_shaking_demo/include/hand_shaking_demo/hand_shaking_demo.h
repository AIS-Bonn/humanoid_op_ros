// Hand shaking demonstration on a standing robot
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef HAND_SHAKING_DEMO_H
#define HAND_SHAKING_DEMO_H

// Includes
#include <nimbro_op_interface/Button.h>
#include <config_server/parameter.h>
#include <robotcontrol/State.h>
#include <ros/service.h>
#include <ros/time.h>

// Hand shaking demo namespace
namespace hand_shaking_demo
{
	// HandShakingDemo class
	class HandShakingDemo
	{
	public:
		// Constructor/destructor
		HandShakingDemo();

		// Main loop step
		bool step();
		bool halt();

		// Get functions
		inline bool enabled() const { return m_handShakingDemoEnabled(); }

	private:
		// ROS services
		ros::ServiceClient m_srv_playMotion;
		
		// ROS topics
		ros::Subscriber m_sub_button;
		
		// Motions
		void playMotion(const std::string& motionName);
		
		// Button presses
		void handleButton(const nimbro_op_interface::ButtonConstPtr& msg);
		bool m_buttonPressed;

		// Robotcontrol robot state
		ros::Subscriber m_sub_robotState;
		void handleRobotState(const robotcontrol::StateConstPtr& msg);
		std::string m_robotStateName;
		bool m_robotIsStanding;
		bool m_robotIsShaking;

		// Configuration parameters
		config_server::Parameter<bool> m_handShakingDemoEnabled;
		void handleDemoEnabled();

		// Internal variables
		ros::Time m_curStepTime;
	};
}

#endif /* HAND_SHAKING_DEMO_H */
// EOF