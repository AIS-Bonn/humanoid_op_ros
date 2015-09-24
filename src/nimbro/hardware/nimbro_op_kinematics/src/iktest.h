// IK test motion module
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef IKTEST_H
#define IKTEST_H

#include <robotcontrol/motionmodule.h>

#include <nimbro_op_kinematics/leg_ik.h>

#include <config_server/parameter.h>

namespace nimbro_op_kinematics
{

class IKTest : public robotcontrol::MotionModule
{
public:
	IKTest();
	virtual ~IKTest();

	virtual bool init(robotcontrol::RobotModel* model);
	virtual void step();

	virtual bool isTriggered();
private:
	LegIK* m_ik;

	config_server::Parameter<float> m_goal_x;
	config_server::Parameter<float> m_goal_y;
	config_server::Parameter<float> m_goal_z;

	config_server::Parameter<float> m_goal_roll;
	config_server::Parameter<float> m_goal_pitch;
	config_server::Parameter<float> m_goal_yaw;
};

}

#endif
