// Motion demonstration on a standing robot
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Ensure header is only included once
#ifndef STANDING_DEMO_H
#define STANDING_DEMO_H

// Includes
#include <nimbro_op_interface/LEDCommand.h>
#include <standing_demo/standing_demo_fsm.h>
#include <limb_control/PlayCommandsSrv.h>
#include <nimbro_op_interface/Button.h>
#include <config_server/parameter.h>
#include <demo_msgs/DemoFSMState.h>
#include <robotcontrol/State.h>
#include <ros/service.h>
#include <ros/time.h>

// Standing demo namespace
namespace standing_demo
{
	// StandingDemo class
	class StandingDemo
	{
	public:
		// Constructor/destructor
		StandingDemo();
		
		// Initialisation
		void loadDemoMotions();
		
		// Main loop step
		bool step();
		bool halt();
		
		// Motion functions
		void playMotion(int motionID);
		double sendLimbCommands(const limb_control::PlayCommandsRequest& PCmdReq);
		
		// Utility functions
		void publishFSMState(const demo_msgs::DemoFSMState& state);
		void publishAction(const demo_msgs::DemoFSMState& action);
		
		// Get functions
		inline int numMotions() const { return (int) m_demoMotions.size(); }
		inline bool haveMotions() const { return m_haveMotions; }
		inline bool enabled() const { return m_enabled && m_standingDemoEnabled(); }
		inline bool robotIsStanding() const { return m_robotIsStanding; }
		inline const ros::Time& curStepTime() const { return m_curStepTime; }
		inline const ros::Time& limbControlFinishTime() const { return m_limbControlFinishTime; }
		
	private:
		// Demo state controller
		StandingDemoSC m_sc;
		
		// ROS topics
		ros::Publisher m_pub_FSMState;
		ros::Publisher m_pub_action;
		ros::Publisher m_pub_LEDCommand;
		
		// Robot LEDs
		void publishLEDCommand();
		nimbro_op_interface::LEDCommand m_LED;
		
		// ROS services
		ros::ServiceClient m_srv_playMotion;
		ros::ServiceClient m_srv_playCommands;
		
		// Robotcontrol robot state
		ros::Subscriber m_sub_robotState;
		void handleRobotState(const robotcontrol::StateConstPtr& msg);
		std::string m_robotStateName;
		bool m_robotIsStanding;
		
		// Button presses
		ros::Subscriber m_sub_button;
		void handleButton(const nimbro_op_interface::ButtonConstPtr& msg);
		bool m_buttonPressed;
		bool m_enabled;
		
		// Configuration parameters
		config_server::Parameter<bool> m_standingDemoEnabled;
		void handleDemoEnabled();
		
		// Motions
		std::vector<std::string> m_demoMotions;
		bool m_haveMotions;
		
		// Internal variables
		ros::Time m_curStepTime;
		ros::Time m_limbControlFinishTime;
	};
}

#endif