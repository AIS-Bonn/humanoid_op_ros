// Robot control node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_loader.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "model/joint.h"
#include "model/robotmodel.h"
#include "motionmodule.h"

#include <robotcontrol/Diagnostics.h>
#include <robotcontrol/FadeTorqueAction.h>
#include <robotcontrol/robotcontrolvis.h>

#include <config_server/parameter.h>

#include <tf/transform_broadcaster.h>

#include <plot_msgs/Plot.h>

namespace robotcontrol
{

class MotionModule;
class HardwareInterface;
typedef boost::shared_ptr<MotionModule> MotionModulePtr;

/**
 * @ingroup nodes
 * @brief Real-time robot control
 *
 * The robotcontrol node is responsible for hardware communication,
 * robot state estimation and motion command generation.
 * It does this in a hard real-time loop, which ensures that the core robot
 * control is always responsive.
 *
 * The robotcontrol node provides a set of static configuration variables on
 * the parameter server:
 *
 *
 * Name              | Type            | Meaning
 * ----------------- | --------------- | ------------------------------------------------------------
 * hw_interface      | string          | The hardware interface (e.g. robotcontrol::RobotInterface)
 * motion_modules    | list of strings | The motion modules to load (e.g. [Gait])
 * publish_tf        | bool            | Publish TF directly (default)
 **/
class RobotControl
{
public:
	RobotControl();
	virtual ~RobotControl();

	//! Initialization
	bool init();

	//! One step of the real-time loop
	void step();

	//!@name External subscriptions
	//@{
	void handleJointStateCommand(const sensor_msgs::JointStatePtr& cmd);
	void handleRawJointCommand(const sensor_msgs::JointStatePtr& cmd);
	//@}

	//! Send a diagnostics message including e.g. battery information
	void sendDiagnostics(const ros::TimerEvent&);

	void printTimeDiagnostics();
	void plotEvent(const std::string& event);

	void setTimerDuration(float sec) { m_robotModel.setTimerDuration(sec); }

	boost::shared_ptr<HardwareInterface> hardwareInterface()
	{ return m_hw; }
private:
	//! Helper for handling joint state messages
	void doHandleJSCommand(const sensor_msgs::JointStatePtr& cmd, bool raw = false);

	//! Initialize the motion modules
	bool initModules();

	//! Publish the measured joint states
	void publishJointStates();

	//! Publish the commanded joint states
	void publishJointCommands();

	//! Callback handlers for parameter updates
	void handleVelLimitUpdate(float value);
	void handleAccLimitUpdate(float value);

	//! Handle fade in/out requests
	void handleFadeTorque();

	ros::NodeHandle m_nh;

	// Plugin loader for the Hardware Interface
	pluginlib::ClassLoader<HardwareInterface> m_hwLoader;
	boost::shared_ptr<HardwareInterface> m_hw;

	// Plugin loader for Motion Modules
	pluginlib::ClassLoader<MotionModule> m_pluginLoader;
	std::vector<MotionModulePtr> m_modules;

	//! The URDF robot description
	boost::shared_ptr<urdf::Model> m_model;

	//! Our robot model
	RobotModel m_robotModel;

	// Publishers & Subscribers for joint state information
	ros::Publisher m_pub_js;
	ros::Publisher m_pub_js_cmd;
	unsigned int m_pub_js_counter;
	ros::Subscriber m_sub_js;
	ros::Subscriber m_sub_js_raw;

	// Diagnostics publishing
	ros::Publisher m_pub_diag;
	ros::Timer m_diagnosticsTimer;

	// Fading
	actionlib::SimpleActionServer<FadeTorqueAction> m_fadeTorqueServer;
	FadeTorqueGoalConstPtr m_fadeTorqueGoal;
	float m_fadeTorqueState;
	bool m_newFadeTorqueGoal;

	// Parameters
	config_server::Parameter<float> m_velLimit;
	config_server::Parameter<float> m_accLimit;
	config_server::Parameter<bool> m_publishCommand;
	config_server::Parameter<bool> m_plotRobotControlData;
	config_server::Parameter<float> m_fadeInMaxDelta;

	bool m_shouldShutdown;

	// Broadcaster for the /ego_map tf
	tf::TransformBroadcaster m_tf_broadcaster;

	RobotModel::State m_state_init;
	RobotModel::State m_state_relaxed;
	RobotModel::State m_state_sitting;

	ros::WallTime m_lastIterationTime;
	ros::WallDuration m_dur_tx;
	ros::WallDuration m_dur_rx;
	ros::WallDuration m_dur_motion;

	// Plotting
	enum PlotIDs
	{
		PM_TIM_PERIOD = 0,
		PM_TIM_MOTION,
		PM_TIM_TX,
		PM_TIM_RX,
		PM_COUNT
	};
	plot_msgs::Plot m_plot;
	ros::Publisher m_pub_plot;

	// Visualisations
	RCMarkerMan m_markers;
	
	// ROS services
	ros::ServiceServer m_srv_printJointCommands;
	bool handlePrintJointCommands(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) { printJointCommands(); return true; }
	void printJointCommands();

	bool m_publishTF;
};

}

#endif
// EOF
