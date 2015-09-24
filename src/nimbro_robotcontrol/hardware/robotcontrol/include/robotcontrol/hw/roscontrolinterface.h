// Interface to ros_control controllers
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSCONTROLINTERFACE_H
#define ROSCONTROLINTERFACE_H

#include <robotcontrol/hw/hardwareinterface.h>
#include <robotcontrol/model/joint.h>
#include <model/robotmodel.h>
#include <ros/node_handle.h>
#include <config_server/parameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <control_msgs/JointControllerState.h>

namespace robotcontrol
{

class ROSControlInterface : public HardwareInterface
{
public:
	ROSControlInterface();
	virtual ~ROSControlInterface();

	virtual boost::shared_ptr< Joint > createJoint(const std::string& name);
	virtual bool init(RobotModel* model);

	virtual bool readJointStates();
	virtual bool sendJointTargets();

	virtual void getDiagnostics(robotcontrol::DiagnosticsPtr ptr);
	virtual bool setStiffness(float torque);

protected:
	struct ROSJoint : public Joint
	{
		ROSJoint(const std::string& name);

		void handleState(const control_msgs::JointControllerStateConstPtr& state);

		config_server::Parameter<bool> velocityMode;
		ros::Publisher pub_cmd;
		ros::Subscriber sub_state;

		ros::ServiceClient srv_set_pid;
		config_server::Parameter<float> p_gain;
		config_server::Parameter<float> i_gain;
		config_server::Parameter<float> d_gain;
	};

	inline ROSJoint* rosJoint(size_t index) { return (ROSJoint*)m_model->joint(index).get(); }

	void handleJointState(const sensor_msgs::JointStateConstPtr& js);

	RobotModel* m_model;
	ros::NodeHandle m_nh;
	std::string m_prefix;
	ros::Subscriber m_sub_js;
	std::map<std::string, boost::shared_ptr<ROSJoint> > m_jointMap;
	sensor_msgs::JointStateConstPtr m_last_js;

	void initReconfigure();
	config_server::Parameter<float> m_mechDampingRatio;
	dynamic_reconfigure::Reconfigure m_reconfig_pid;
	ros::Time m_lastReconfig;
	double m_lastTorque;
	
	enum DynRecOrder
	{
		DRO_P_GAIN = 0,
		DRO_I_GAIN,
		DRO_D_GAIN,
		DRO_COUNT
	};
};

}

#endif
