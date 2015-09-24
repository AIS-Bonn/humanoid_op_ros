// Interface to ros_control controllers
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/hw/roscontrolinterface.h>
#include <robotcontrol/model/robotmodel.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <boost/make_shared.hpp>
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.h>

namespace robotcontrol
{

ROSControlInterface::ROSJoint::ROSJoint(const std::string& name)
 : velocityMode("joints/" + name + "/velocityMode", false)
 , p_gain("rosControlInterface/joints/" + name + "/PGain", 0.0, 5.0, 1000.0, 100.0)
 , i_gain("rosControlInterface/joints/" + name + "/IGain", 0.0, 0.05, 10.0, 0.0)
 , d_gain("rosControlInterface/joints/" + name + "/DGain", 0.0, 0.05, 10.0, 1.0)
{
	this->name = name;
}

void ROSControlInterface::ROSJoint::handleState(const control_msgs::JointControllerStateConstPtr& state)
{
	feedback.torque = state->command;
}

ROSControlInterface::ROSControlInterface()
 : m_nh("~")
 , m_mechDampingRatio("rosControlInterface/jointMechDampingRatio", 0.0, 0.01, 1.0, 0.25)
{
	m_nh.param("prefix", m_prefix, std::string(""));

	// Get the joint states here as fast as possible (disable Nagle's algorithm)
	m_sub_js = m_nh.subscribe(
		m_prefix + "/joint_states", 50,
		&ROSControlInterface::handleJointState, this,
		ros::TransportHints().tcpNoDelay()
	);
	
	m_lastReconfig.fromNSec(0);
	m_lastTorque = 0.0;
}

ROSControlInterface::~ROSControlInterface()
{
}

boost::shared_ptr<Joint> ROSControlInterface::createJoint(const std::string& name)
{
	boost::shared_ptr<ROSJoint> joint = boost::make_shared<ROSJoint>(name);

	std::string topic;
	std::string stateTopic;
	std::string pidSrv;
	if(joint->velocityMode())
	{
		topic = m_prefix + "/" + name + "_velocity_controller/command";
		stateTopic = m_prefix + "/" + name + "_velocity_controller/state";
		pidSrv = m_prefix + "/" + name + "_velocity_controller/pid/set_parameters";
	}
	else
	{
		topic = m_prefix + "/" + name + "_position_controller/command";
		stateTopic = m_prefix + "/" + name + "_position_controller/state";
		pidSrv = m_prefix + "/" + name + "_position_controller/pid/set_parameters";
	}

	joint->pub_cmd = m_nh.advertise<std_msgs::Float64>(topic, 1);
	joint->sub_state = m_nh.subscribe(stateTopic, 1, &ROSJoint::handleState, joint.get());
	joint->srv_set_pid = m_nh.serviceClient<dynamic_reconfigure::Reconfigure>(pidSrv, false);

	m_jointMap[name] = joint;

	return joint;
}

bool ROSControlInterface::init(RobotModel* model)
{
	m_model = model;
	
	initReconfigure();
	m_lastReconfig.fromNSec(0);
	m_lastTorque = 0.0;
	
	return true;
}

void ROSControlInterface::handleJointState(const sensor_msgs::JointStateConstPtr& js)
{
	m_last_js = js;
}

bool ROSControlInterface::readJointStates()
{
	ros::spinOnce();

	if(!m_last_js)
		return true;

	for(size_t i = 0; i < m_last_js->name.size(); ++i)
	{
		std::map<std::string, boost::shared_ptr<ROSJoint> >::iterator it = m_jointMap.find(m_last_js->name[i]);

		if(it == m_jointMap.end())
			continue;

		const boost::shared_ptr<ROSJoint>& joint = it->second;

		joint->feedback.pos = m_last_js->position[i];
		joint->feedback.stamp = m_last_js->header.stamp;
	}

	m_last_js.reset();

	return true;
}

bool ROSControlInterface::sendJointTargets()
{
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		ROSJoint* joint = rosJoint(i);

		std_msgs::Float64 msg;
		if(joint->velocityMode())
			msg.data = joint->cmd.vel;
		else
			msg.data = joint->cmd.pos;
		joint->pub_cmd.publish(msg);
	}

	return true;
}

void ROSControlInterface::initReconfigure()
{
	dynamic_reconfigure::DoubleParameter dparam;
	dparam.value = 0.0;
	m_reconfig_pid.request.config.doubles.assign(DRO_COUNT, dparam);
	m_reconfig_pid.request.config.doubles[DRO_P_GAIN].name = "p";
	m_reconfig_pid.request.config.doubles[DRO_I_GAIN].name = "i";
	m_reconfig_pid.request.config.doubles[DRO_D_GAIN].name = "d";
}

bool ROSControlInterface::setStiffness(float torque)
{
	// Display a throttled info message that fading is active
	ROS_INFO_THROTTLE(0.4, "Fading is active (%.3f)", torque);
	
	// Decide whether to actually send something
	ros::Time now = ros::Time::now();
	double elapsedTime = (now - m_lastReconfig).toSec();
	bool actuallySend = (((torque == 0 || torque == 1) && fabs(torque - m_lastTorque) > 0.001) || elapsedTime > 0.3);
	
	// Update the joint stiffnesses by updating the PID gains of the gazebo joints
	if(actuallySend)
	{
		m_lastReconfig = now;
		m_lastTorque = torque;
		std::map<std::string, boost::shared_ptr<ROSJoint> >::iterator it;
		for(it = m_jointMap.begin(); it != m_jointMap.end(); ++it)
		{
			boost::shared_ptr<ROSJoint>& joint = it->second;
			m_reconfig_pid.request.config.doubles[DRO_P_GAIN].value = joint->p_gain() * torque;
			m_reconfig_pid.request.config.doubles[DRO_I_GAIN].value = joint->i_gain() * torque;
			m_reconfig_pid.request.config.doubles[DRO_D_GAIN].value = joint->d_gain() * (torque + (1 - torque)*m_mechDampingRatio());
			joint->srv_set_pid.call(m_reconfig_pid);
		}
	}
	
	// Return that we successfully set the stiffness
	return true;
}

void ROSControlInterface::getDiagnostics(DiagnosticsPtr ptr)
{
}

}

PLUGINLIB_EXPORT_CLASS(robotcontrol::ROSControlInterface, robotcontrol::HardwareInterface)
