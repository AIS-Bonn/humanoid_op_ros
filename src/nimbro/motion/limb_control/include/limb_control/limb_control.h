// Motion module for individual control of robot limbs
// Author: Philipp Allgeuer

// Ensure header is only included once
#ifndef LIMB_CONTROL_H
#define LIMB_CONTROL_H

// Includes
#include <list>
#include <ros/time.h>
#include <ros/service.h>
#include <config_server/parameter.h>
#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/robotmodel.h>
#include <limb_control/PlayCommandsSrv.h>

// Limbcontrol namespace
namespace limb_control
{
	// LimbCmd struct
	struct LimbCmd
	{
		LimbCommand LC;
		robotcontrol::Joint::Ptr joint;
		double initPos;
		double duration;
	};

	// LimbControl class
	class LimbControl : public robotcontrol::MotionModule
	{
	public:
		// Constructor
		LimbControl();
		
		// Motion module overloads
		virtual bool init(robotcontrol::RobotModel* model);
		virtual void step();
		virtual bool isTriggered();

	private:
		// Constants
		const std::string CONFIG_PARAM_PATH;

		// RobotModel reference
		robotcontrol::RobotModel* m_model;

		// Config server parameters
		config_server::Parameter<bool> m_limbControlEnabled;
		bool m_lastEnabledState;

		// ROS services
		ros::ServiceServer m_srv_playCommands;
		bool handlePlayCommands(limb_control::PlayCommands::Request& req, limb_control::PlayCommands::Response& res);

		// Utility functions
		void removeDuplicateJoints();
		double getRemainingTime();
		double getCurrentMotionTime() const;

		// Limb command list
		std::list<LimbCmd> m_LCA;

		// Motion playing variables
		ros::Time m_motionStartTime;
		uint8_t m_curTimeRef;
		size_t m_numMotions;
	};
}

#endif /* LIMB_CONTROL_H */
// EOF