// Wrapper for a version of the NimbRo classic gait code
// Author: Sebastian Schüller

#ifndef INDEP_CPG_GAIT_H
#define INDEP_CPG_GAIT_H

#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/robotmodel.h>

#include <config_server/parameter.h>

#include <gait_msgs/GaitCommand.h>
#include <nimbro_op_interface/Button.h>

#include "gaitcode/RobotControl/RobotControl.h"
#include "gaitcode/RobotControl/Percept.h"
#include "gaitcode/RobotControl/Action.h"
#include "configclient/configclient.h"

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <gait_msgs/SetOdom.h>
#include <plot_msgs/Plot.h>

#include <boost/circular_buffer.hpp>

#include <Eigen/Core>

namespace indep_cpg_gait
{

class IndepCPGGait : public robotcontrol::MotionModule
{
public:
	IndepCPGGait();

	virtual bool init(robotcontrol::RobotModel* model);
	virtual bool isTriggered();

	virtual void publishTransforms();

	virtual void step();

private:
	enum JointId
	{
		ARMS_BEGIN,
		L_SHOULDER_PITCH = ARMS_BEGIN,
		R_SHOULDER_PITCH,
		L_SHOULDER_ROLL,
		R_SHOULDER_ROLL,
		L_ELBOW_PITCH,
		R_ELBOW_PITCH,
		ARMS_END = R_ELBOW_PITCH,

		LEFT_LEG_BEGIN,
		L_HIP_PITCH = LEFT_LEG_BEGIN,
		L_HIP_ROLL,
		L_HIP_YAW,
		L_KNEE,
		L_ANKLE_PITCH,
		L_ANKLE_ROLL,
		LEFT_LEG_END = L_ANKLE_ROLL,

		RIGHT_LEG_BEGIN,
		R_HIP_PITCH = RIGHT_LEG_BEGIN,
		R_HIP_ROLL,
		R_HIP_YAW,
		R_KNEE,
		R_ANKLE_PITCH,
		R_ANKLE_ROLL,
		RIGHT_LEG_END = R_ANKLE_ROLL,

		MAX_NUM
	};

	struct PerformanceData
	{
		PerformanceData()
		 : maxFusedAngle(0)
		 , maxTorque(0)
		{}

		double maxFusedAngle;
		double maxTorque;
	};

	robotcontrol::RobotModel::State m_state_idle;
	robotcontrol::RobotModel::State m_state_walking;

	indep_cpg_gait::RobotControl m_robotControl;
	indep_cpg_gait::Percept m_percept;
	indep_cpg_gait::Action m_action;

	double m_dT;
	robotcontrol::RobotModel* m_model;
	std::vector<int> m_jointMap;

	config_client::ConfigClient m_gaitClient;
	config_server::Parameter<float> m_effort;
	config_server::Parameter<float> m_supportEffort;
	config_server::Parameter<float> m_ankleEffort;
	config_server::Parameter<bool> m_joy_enabled;
	config_server::Parameter<bool> m_buttonEnabled;

	ros::Subscriber m_sub_joy;
	bool m_joy_pressed;

	ros::Subscriber m_sub_button;

	boost::shared_ptr<const urdf::Link> m_rightFoot;
	boost::shared_ptr<const urdf::Link> m_leftFoot;
	boost::shared_ptr<const urdf::Link> m_trunk;

	ros::Publisher m_pub_plot;

	PerformanceData m_currentPerformance;
	boost::circular_buffer<PerformanceData> m_perfBuf;
	ros::Publisher m_pub_perf;

	Eigen::VectorXd m_commandVector;

	bool m_fadeIn;

	bool m_lock;

	ros::Time m_lastWalking;

	bool m_req_walking;

	void interpretJointList();

	void interpretAction();

	ros::Subscriber m_sub_cmd;

	tf::TransformBroadcaster m_tf_pub;
	std::vector<tf::StampedTransform> m_tf_buf;

	void mapJointList();

	inline const boost::shared_ptr<robotcontrol::Joint>& joint(JointId id)
	{ return (*m_model)[m_jointMap[id]]; }

	void handleButton(const nimbro_op_interface::ButtonConstPtr& msg);
	void handleJoystickData(const sensor_msgs::JoyConstPtr& joy);
	void handleGaitCommand(const gait_msgs::GaitCommandConstPtr& cmd);

	void evaluatePerformance();

	void startFading();
	void handleFade();

	void resetDerivs();

	config_server::Parameter<bool> m_use_sim_odom;
	config_server::Parameter<float> m_odom_offset_x;
	config_server::Parameter<float> m_odom_scale_x;
	config_server::Parameter<float> m_odom_scale_y;
	config_server::Parameter<float> m_odom_scale_z;
	config_server::Parameter<float> m_gcv_offset_x;
	config_server::Parameter<float> m_gcv_offset_y;
	config_server::Parameter<float> m_gcv_offset_z;
	config_server::Parameter<float> m_balance_gain;

	ros::ServiceServer m_srv_resetOdom;
	ros::ServiceServer m_srv_setOdom;
	bool resetOdometry(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
	bool setOdometry(gait_msgs::SetOdomRequest &req, gait_msgs::SetOdomResponse &res);
};

}

#endif /* INDEP_CPG_GAIT_H */
// EOF
