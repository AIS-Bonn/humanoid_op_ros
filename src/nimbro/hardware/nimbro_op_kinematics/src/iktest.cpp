// IK test motion module
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "iktest.h"
#include <robotcontrol/model/robotmodel.h>

#include <pluginlib/class_list_macros.h>

namespace nimbro_op_kinematics
{

IKTest::IKTest()
 : m_ik(0)
 , m_goal_x("/iktest/x", -2, 0.01, 2, 0.0)
 , m_goal_y("/iktest/y", -2, 0.01, 2, 0.0)
 , m_goal_z("/iktest/z", -2, 0.01, 2, -0.5)
 , m_goal_roll("/iktest/roll", -M_PI, 0.01, M_PI, 0.0)
 , m_goal_pitch("/iktest/pitch", -M_PI, 0.01, M_PI, 0.0)
 , m_goal_yaw("/iktest/yaw", -M_PI, 0.01, M_PI, 0.0)
{
}

IKTest::~IKTest()
{
	if(m_ik)
		delete m_ik;
}

bool IKTest::init(robotcontrol::RobotModel* model)
{
	robotcontrol::MotionModule::init(model);

	m_ik = new LegIK(model->supportModel("trunk_link"), "left_foot_plane_link");

	return true;
}

void IKTest::step()
{
	Eigen::Vector3d goal;
	goal << m_goal_x(), m_goal_y(), m_goal_z();

	Eigen::Matrix3d rot;
	rot =
		  Eigen::AngleAxisd(m_goal_yaw(), Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(m_goal_roll(), Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(m_goal_pitch(), Eigen::Vector3d::UnitY())
	;

	if(!m_ik->sendTargetsFor(goal, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), rot))
	{
		ROS_ERROR("IK fail!");
	}
	else
		ROS_INFO_THROTTLE(1.0, "IK success");
}

bool IKTest::isTriggered()
{
	return true;
}


}

PLUGINLIB_EXPORT_CLASS(nimbro_op_kinematics::IKTest, robotcontrol::MotionModule)
