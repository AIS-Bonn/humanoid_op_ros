// Kinematic model of NimbRo-OP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_op_kinematics/kinematic_model.h>

#include <nimbro_op_model/dimensions.h>

namespace nimbro_op_kinematics
{

KinematicModel::KinematicModel(robotcontrol::RobotModel* robotModel)
 : m_robotModel(robotModel)
 , m_leftLeg(robotModel, "left_foot_plane_link", "right_foot_plane_link", 1.0)
 , m_rightLeg(robotModel, "right_foot_plane_link", "left_foot_plane_link", -1.0)
{
	m_trunkModel = robotModel->supportModel("trunk_link");
}

KinematicModel::~KinematicModel()
{
}

void KinematicModel::updateKinematics()
{
	m_leftLeg.computeForwardKinematics();
	m_rightLeg.computeForwardKinematics();
}

}
