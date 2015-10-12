// Wrapper for a version of the NimbRo classic gait code
// Author: Sebastian Schüller

#include "indep_cpg_gait.h"

#include <gait/gait_common.h>
#include <gait_msgs/GaitPerformance.h>

#include <pluginlib/class_list_macros.h>
#include <robotcontrol/model/joint.h>
#include <nimbro_op_model/dimensions.h>

#include <ros/package.h>
#include <std_msgs/Empty.h>

using namespace indep_cpg_gait;
using namespace robotcontrol;

IndepCPGGait::IndepCPGGait()
 : m_effort("/indep_cpg_gait/effort", 0, 0.05, 1, 0.2)
 , m_supportEffort("/indep_cpg_gait/supportEffort", 0, 0.05, 1, 0.2)
 , m_ankleEffort("/indep_cpg_gait/ankle_effort", 0, 0.05, 1, 0.2)
 , m_joy_enabled("/indep_cpg_gait/joystick_enabled", true)
 , m_buttonEnabled("/indep_cpg_gait/button_enabled", false)
 , m_joy_pressed(false)
 , m_perfBuf(10)
 , m_commandVector(MAX_NUM)
 , m_fadeIn(true)
 , m_req_walking(false)
 , m_use_sim_odom("/indep_cpg_gait/odometry/use_simulation_odometry", false)
 , m_odom_offset_x("/indep_cpg_gait/odometry/x_offset", -2.0, 0.001, 2.0, -0.7)
 , m_odom_scale_x("/indep_cpg_gait/odometry/scale/x", -2.0, 0.001, 2.0, 0.2)
 , m_odom_scale_y("/indep_cpg_gait/odometry/scale/y", -2.0, 0.001, 2.0, 0.2)
 , m_odom_scale_z("/indep_cpg_gait/odometry/scale/z", -2.0, 0.001, 2.0, 0.35)
 , m_gcv_offset_x("/indep_cpg_gait/gcv_offset/x", -2.0, 0.001, 2.0, -0.23)
 , m_gcv_offset_y("/indep_cpg_gait/gcv_offset/y", -2.0, 0.001, 2.0, 0.0)
 , m_gcv_offset_z("/indep_cpg_gait/gcv_offset/z", -2.0, 0.001, 2.0, 0.0)
 , m_balance_gain("/indep_cpg_gait/balance_gain", -2.0, 0.001, 2.0, 0.0)
{
	m_tf_buf.resize(2);
	tf::StampedTransform* ego_floor = &m_tf_buf[0];
	tf::StampedTransform* odom = &m_tf_buf[1];

	ego_floor->frame_id_ = "/ego_floor";
	ego_floor->child_frame_id_ = "/ego_rot";
	ego_floor->setIdentity();

	odom->frame_id_ = gait::gaitOdomFrame;
	odom->child_frame_id_ = "/ego_floor";
	odom->setIdentity();
}

bool IndepCPGGait::init(robotcontrol::RobotModel* model)
{
	if(!MotionModule::init(model)) return false;
	m_model = model;
	
	m_dT = m_model->timerDuration();

	m_state_idle = m_model->registerState("standing");
	m_state_walking = m_model->registerState("walking");

	ros::NodeHandle nh("~");

	m_gaitClient.init();
	m_gaitClient.setCallback(boost::bind(&IndepCPGGait::resetDerivs, this));
	mapJointList();

	m_sub_joy = nh.subscribe("/joy", 1, &IndepCPGGait::handleJoystickData, this);
	m_sub_button = nh.subscribe("/button", 1, &IndepCPGGait::handleButton, this);
	m_pub_plot = nh.advertise<plot_msgs::Plot>("/plot", 1);
	m_pub_perf = nh.advertise<gait_msgs::GaitPerformance>("/indep_cpg_gait/performance", 1);
	m_sub_cmd = nh.subscribe("/gaitCommand", 1, &IndepCPGGait::handleGaitCommand, this);

	indep_cpg_gait::command.GCV.x = 0;
	indep_cpg_gait::command.GCV.y = 0;
	indep_cpg_gait::command.GCV.z = 0;
	indep_cpg_gait::command.walk = false;

	m_leftFoot = m_model->urdf()->getLink("left_foot_plane_link");
	m_rightFoot = m_model->urdf()->getLink("right_foot_plane_link");
	m_trunk = m_model->urdf()->getLink("trunk_link");

	m_srv_resetOdom = nh.advertiseService("/indep_cpg_gait/resetOdom", &IndepCPGGait::resetOdometry, this);
	m_srv_setOdom   = nh.advertiseService("/indep_cpg_gait/setOdom", &IndepCPGGait::setOdometry, this);

	indep_cpg_gait::config.systemIterationTime = m_dT; // Note: Here we override the value read from the config server parameter and replace it with our value from RobotModel

	m_lock = false;

	return true;
}

void IndepCPGGait::resetDerivs()
{
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		robotcontrol::Joint::Ptr joint = m_model->joint(i);
		joint->cmd.resetDerivs();
	}
}

bool IndepCPGGait::isTriggered()
{
	if(model()->state() == m_state_idle && m_req_walking)
	{
		// Setup joints
		for(size_t i = 0; i < m_model->numJoints(); ++i)
		{
			robotcontrol::Joint::Ptr joint = m_model->joint(i);

			joint->cmd.raw = false;
		}

		model()->setState(m_state_walking);
		indep_cpg_gait::command.walk = false;
		startFading();
		return true;
	}

	if(model()->state() == m_state_walking)
	{
		if(m_req_walking || (ros::Time::now() - m_lastWalking) < ros::Duration(1.0))
		{
			m_lock = true;
			return true;
		}
		else
		{
			model()->setState(m_state_idle);
			return m_lock;
		}
	}

	if(model()->state() != m_state_idle)
		m_lock = false;
	indep_cpg_gait::command.walk = false;
	return m_lock;
}

void IndepCPGGait::step()
{
	m_dT = m_model->timerDuration();

	interpretJointList();

	if(!m_fadeIn)
	{
		if(indep_cpg_gait::command.walk != m_req_walking)
		{
			ROS_INFO("Setting walk cmd: %d (old %d)", m_req_walking, indep_cpg_gait::command.walk);
		}

		indep_cpg_gait::command.walk = m_req_walking;
	}

	if(m_robotControl.motionLayer.walking)
		m_lastWalking = ros::Time::now();

	double fusedPitch = m_model->robotFPitchPR();
	double fusedRoll = m_model->robotFRollPR();

	Vec2f newFusedAngle(fusedRoll, fusedPitch);
	Vec2f delta = newFusedAngle - m_percept.fusedAngle;

	m_percept.fusedAngle = newFusedAngle;
	m_percept.DfusedAngle = delta / m_dT;

	m_robotControl.motionLayer.gait.fusedAngle_sag = fusedPitch;

	m_robotControl.sense(m_percept);
	m_action.pose = m_robotControl.act();

	interpretAction();

	// Update support leg in robot model
	double leftCoeff = m_robotControl.motionLayer.gait.supportCoefficient(indep_cpg_gait::DynamicGait::RIGHT_LEG);
	double rightCoeff = m_robotControl.motionLayer.gait.supportCoefficient(indep_cpg_gait::DynamicGait::LEFT_LEG);
	double trunkCoeff = 0;

	if(leftCoeff + rightCoeff < 1.0)
		trunkCoeff = 1.0 - leftCoeff - rightCoeff;

	m_model->resetSupport();
	m_model->setSupportCoefficient(m_rightFoot, leftCoeff);
	m_model->setSupportCoefficient(m_leftFoot, rightCoeff);
	m_model->setSupportCoefficient(m_trunk, trunkCoeff);

	evaluatePerformance();

	// Update odometry transform
	tf::StampedTransform* odom = &m_tf_buf[1];

	double dt = indep_cpg_gait::config.systemIterationTime;
	Vec3f smoothed = m_robotControl.motionLayer.gait.gcv;
	tf::Vector3 velVec;
	double zstep;
	if(m_use_sim_odom())
	{
		velVec = tf::Vector3(0.15 * (smoothed.x), 0.15*smoothed.y, 0);
		zstep = dt * 0.35 * 1.5 *smoothed.z;
	}
	else
	{
		velVec = tf::Vector3(m_odom_scale_x() * (smoothed.x - m_odom_offset_x()), m_odom_scale_y()*smoothed.y, 0);
		zstep = dt * 0.35 * m_odom_scale_z()*smoothed.z;
	}
	tf::Transform stepTransform(
		tf::createQuaternionFromYaw(zstep),
		dt * velVec
	);

	odom->setData((*odom) * stepTransform);
}

bool IndepCPGGait::resetOdometry(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	tf::StampedTransform* odom = &m_tf_buf[1];
	odom->setIdentity();
	return true;
}
bool IndepCPGGait::setOdometry(gait_msgs::SetOdomRequest &req, gait_msgs::SetOdomResponse &res)
{
	tf::StampedTransform* odom = &m_tf_buf[1];
	odom->setIdentity();
	tf::Vector3 tmp(req.posX, req.posY, 0.0);
	odom->setOrigin(tmp);
	return true;
}

void IndepCPGGait::publishTransforms()
{
	ros::Time now = ros::Time::now();

	tf::StampedTransform* ego_floor = &m_tf_buf[0];
	tf::StampedTransform* odom = &m_tf_buf[1];

	ego_floor->setOrigin(tf::Vector3(0, 0, 0.1374 + 0.44 - indep_cpg_gait::config.footOffsetZ + nimbro_op_model::ANKLE_Z_HEIGHT));
	ego_floor->stamp_ = now;

	odom->stamp_ = now;

	m_tf_pub.sendTransform(m_tf_buf);
}


void IndepCPGGait::evaluatePerformance()
{
	double fx = fabs(m_percept.fusedAngle.x);
	double fy = fabs(m_percept.fusedAngle.y);

	if(fx > m_currentPerformance.maxFusedAngle)
		m_currentPerformance.maxFusedAngle = fx;

	if(fy > m_currentPerformance.maxFusedAngle)
		m_currentPerformance.maxFusedAngle = fy;

	double torque = 0;
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		torque += fabs(m_model->joint(i)->feedback.modelTorque);
	}

	if(torque > m_currentPerformance.maxTorque)
		m_currentPerformance.maxTorque = torque;

	if(m_robotControl.motionLayer.gait.gaitPhase > 0 && m_robotControl.motionLayer.gait.lastGaitPhase <= 0)
	{
		m_perfBuf.push_back(m_currentPerformance);
		m_currentPerformance.maxFusedAngle = 0;
		m_currentPerformance.maxTorque = 0;

		PerformanceData avg;
		for(size_t i = 0; i < m_perfBuf.size(); ++i)
		{
			avg.maxFusedAngle += m_perfBuf[i].maxFusedAngle;
			avg.maxTorque += m_perfBuf[i].maxTorque;
		}
		avg.maxFusedAngle /= m_perfBuf.size();
		avg.maxTorque /= m_perfBuf.size();

		gait_msgs::GaitPerformance perf;
		perf.header.stamp = ros::Time::now();
		perf.maxAngle = avg.maxFusedAngle;
		perf.torque = avg.maxTorque;
		m_pub_perf.publish(perf);
	}
}

void IndepCPGGait::interpretJointList()
{
	m_percept.pose.rightArmPose.shoulder.x = joint(R_SHOULDER_ROLL)->feedback.pos;
	m_percept.pose.rightArmPose.shoulder.y = joint(R_SHOULDER_PITCH)->feedback.pos;

	m_percept.pose.rightArmPose.elbow.y = joint(R_ELBOW_PITCH)->feedback.pos;

	m_percept.pose.rightLegPose.ankle.x = joint(R_ANKLE_ROLL)->feedback.pos;
	m_percept.pose.rightLegPose.ankle.y = joint(R_ANKLE_PITCH)->feedback.pos;

	m_percept.pose.rightLegPose.knee.y = joint(R_KNEE)->feedback.pos;

	m_percept.pose.rightLegPose.hip.x = joint(R_HIP_ROLL)->feedback.pos;
	m_percept.pose.rightLegPose.hip.y = joint(R_HIP_PITCH)->feedback.pos;
	m_percept.pose.rightLegPose.hip.z = joint(R_HIP_YAW)->feedback.pos;


	m_percept.pose.leftArmPose.shoulder.x = joint(L_SHOULDER_ROLL)->feedback.pos;
	m_percept.pose.leftArmPose.shoulder.y = joint(L_SHOULDER_PITCH)->feedback.pos;

	m_percept.pose.leftArmPose.elbow.y = joint(L_ELBOW_PITCH)->feedback.pos;

	m_percept.pose.leftLegPose.ankle.x = joint(L_ANKLE_ROLL)->feedback.pos;
	m_percept.pose.leftLegPose.ankle.y = joint(L_ANKLE_PITCH)->feedback.pos;

	m_percept.pose.leftLegPose.knee.y = joint(L_KNEE)->feedback.pos;

	m_percept.pose.leftLegPose.hip.x = joint(L_HIP_ROLL)->feedback.pos;
	m_percept.pose.leftLegPose.hip.y = joint(L_HIP_PITCH)->feedback.pos;
	m_percept.pose.leftLegPose.hip.z = joint(L_HIP_YAW)->feedback.pos;
}

void IndepCPGGait::interpretAction()
{
	ros::Time now = ros::Time::now();

	const indep_cpg_gait::ArmPose& ra = m_action.pose.rightArmPose;
	const indep_cpg_gait::ArmPose& la = m_action.pose.leftArmPose;
	const indep_cpg_gait::LegPose& rl = m_action.pose.rightLegPose;
	const indep_cpg_gait::LegPose& ll = m_action.pose.leftLegPose;

	double armEffort = m_effort();

	double leftCoeff = m_robotControl.motionLayer.gait.supportCoefficient(indep_cpg_gait::DynamicGait::LEFT_LEG);
	double leftEffort = m_effort() + leftCoeff * (m_supportEffort() - m_effort());

	double rightCoeff = m_robotControl.motionLayer.gait.supportCoefficient(indep_cpg_gait::DynamicGait::RIGHT_LEG);
	double rightEffort = m_effort() + rightCoeff * (m_supportEffort() - m_effort());

	double balance_fb = m_balance_gain() * m_model->robotAngularVelocity().y() / 100.0;

	m_commandVector[R_SHOULDER_ROLL] = ra.shoulder.x;
	m_commandVector[R_SHOULDER_PITCH] = ra.shoulder.y;
	m_commandVector[R_ELBOW_PITCH] = ra.elbow.y;
	m_commandVector[R_ANKLE_ROLL] = rl.ankle.x;
	m_commandVector[R_ANKLE_PITCH] = rl.ankle.y + balance_fb;
	m_commandVector[R_KNEE] = rl.knee.y;
	m_commandVector[R_HIP_ROLL] = rl.hip.x;
	m_commandVector[R_HIP_PITCH] = rl.hip.y;
	m_commandVector[R_HIP_YAW] = rl.hip.z;

	m_commandVector[L_SHOULDER_ROLL] = la.shoulder.x;
	m_commandVector[L_SHOULDER_PITCH] = la.shoulder.y;
	m_commandVector[L_ELBOW_PITCH] = la.elbow.y;
	m_commandVector[L_ANKLE_ROLL] = ll.ankle.x;
	m_commandVector[L_ANKLE_PITCH] = ll.ankle.y + balance_fb;
	m_commandVector[L_KNEE] = ll.knee.y;
	m_commandVector[L_HIP_ROLL] = ll.hip.x;
	m_commandVector[L_HIP_PITCH] = ll.hip.y;
	m_commandVector[L_HIP_YAW] = ll.hip.z;

	if(m_fadeIn)
		handleFade();
	else
	{
		for(int i = ARMS_BEGIN; i <= ARMS_END; ++i)
			setJointCommand(m_jointMap[i], m_commandVector[i], armEffort, false);

		for(int i = LEFT_LEG_BEGIN; i <= LEFT_LEG_END; ++i)
			setJointCommand(m_jointMap[i], m_commandVector[i], leftEffort, false);

		for(int i = RIGHT_LEG_BEGIN; i <= RIGHT_LEG_END; ++i)
			setJointCommand(m_jointMap[i], m_commandVector[i], rightEffort, false);
	}
}

void IndepCPGGait::mapJointList()
{
	m_jointMap.resize(MAX_NUM);

	m_jointMap[L_SHOULDER_PITCH] = m_model->jointIndex("left_shoulder_pitch");
	m_jointMap[R_SHOULDER_PITCH] = m_model->jointIndex("right_shoulder_pitch");
	m_jointMap[L_SHOULDER_ROLL]  = m_model->jointIndex("left_shoulder_roll");
	m_jointMap[R_SHOULDER_ROLL]  = m_model->jointIndex("right_shoulder_roll");

	m_jointMap[L_ELBOW_PITCH] = m_model->jointIndex("left_elbow_pitch");
	m_jointMap[R_ELBOW_PITCH] = m_model->jointIndex("right_elbow_pitch");

	m_jointMap[L_HIP_PITCH] = m_model->jointIndex("left_hip_pitch");
	m_jointMap[R_HIP_PITCH] = m_model->jointIndex("right_hip_pitch");
	m_jointMap[L_HIP_ROLL]  = m_model->jointIndex("left_hip_roll");
	m_jointMap[R_HIP_ROLL]  = m_model->jointIndex("right_hip_roll");
	m_jointMap[L_HIP_YAW]   = m_model->jointIndex("left_hip_yaw");
	m_jointMap[R_HIP_YAW]   = m_model->jointIndex("right_hip_yaw");

	m_jointMap[L_KNEE] = m_model->jointIndex("left_knee_pitch");
	m_jointMap[R_KNEE] = m_model->jointIndex("right_knee_pitch");

	m_jointMap[L_ANKLE_PITCH] = m_model->jointIndex("left_ankle_pitch");
	m_jointMap[R_ANKLE_PITCH] = m_model->jointIndex("right_ankle_pitch");
	m_jointMap[L_ANKLE_ROLL]  = m_model->jointIndex("left_ankle_roll");
	m_jointMap[R_ANKLE_ROLL]  = m_model->jointIndex("right_ankle_roll");
}

void IndepCPGGait::handleJoystickData(const sensor_msgs::JoyConstPtr& joy)
{
	if(!m_joy_enabled())
		return;

	if(joy->axes.size() < 3 || joy->buttons.size() < 1)
		return;

	indep_cpg_gait::command.GCV.x = joy->axes[1];
	indep_cpg_gait::command.GCV.y = joy->axes[0];
	indep_cpg_gait::command.GCV.z = joy->axes[2];

	if(m_joy_pressed && !joy->buttons[0])
	{
		m_req_walking = !m_req_walking;
	}
	m_joy_pressed = joy->buttons[0];
}

void IndepCPGGait::handleGaitCommand(const gait_msgs::GaitCommandConstPtr& cmd)
{
	indep_cpg_gait::command.GCV.x = cmd->gcvX;
	indep_cpg_gait::command.GCV.y = cmd->gcvY;
	indep_cpg_gait::command.GCV.z = cmd->gcvZ;
	m_req_walking = cmd->walk;

	// Bias the gait command vector to make the robot walk on the spot (TODO: Experimental)
	if(!m_use_sim_odom())
	{
		indep_cpg_gait::command.GCV.x += m_gcv_offset_x();
		indep_cpg_gait::command.GCV.y += m_gcv_offset_y();
		indep_cpg_gait::command.GCV.z += m_gcv_offset_z();
	}
}

void IndepCPGGait::handleButton(const nimbro_op_interface::ButtonConstPtr& msg)
{
	if(m_buttonEnabled())
	{
		if(msg->button == 1)
		{
			indep_cpg_gait::command.GCV.x = 0.0;
			indep_cpg_gait::command.GCV.y = 0.0;
			indep_cpg_gait::command.GCV.z = 0.0;
			m_req_walking = !m_req_walking;
		}
	}
}

void IndepCPGGait::startFading()
{
	m_fadeIn = true;
	ROS_INFO("Motion fade to the gait halt position is starting.");
}

void IndepCPGGait::handleFade()
{
	const double FADE_SPEED = 10.0 * M_PI / 180.0 * m_dT;

	if(indep_cpg_gait::command.walk)
	{
		ROS_WARN_THROTTLE(0.1, "Fading in while walking, this is not good at all!");
	}

	double max_delta = 0.0;
	for(int i = 0; i < MAX_NUM; i++)
	{
		int idx = m_jointMap[i];
		robotcontrol::Joint::Ptr joint = model()->joint(idx);
		double delta = m_commandVector[i] - joint->lastCmd.pos;

		if(fabs(delta) > max_delta)
			max_delta = fabs(delta);

		if(delta > FADE_SPEED)
			delta = FADE_SPEED;
		if(delta < -FADE_SPEED)
			delta = -FADE_SPEED;

		joint->cmd.setFromPos(m_dT, joint->lastCmd.pos + delta);
	}

	if(max_delta < 0.05 * M_PI / 180.0)
	{
		ROS_INFO("Motion fade to the gait halt position has finished.");
		m_fadeIn = false;
	}
}

PLUGINLIB_EXPORT_CLASS(indep_cpg_gait::IndepCPGGait, robotcontrol::MotionModule)
// EOF