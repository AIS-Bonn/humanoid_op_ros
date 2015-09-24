// Leg representation
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_op_kinematics/leg.h>

#include <nimbro_op_model/dimensions.h>

namespace nimbro_op_kinematics
{

Leg::Leg(robotcontrol::RobotModel* robotModel, const std::string& footLink, const std::string& otherFootLink, double sign)
 : m_ik(robotModel->supportModel("trunk_link"), footLink)
{
	m_trunk_SSM = robotModel->supportModel("trunk_link");

	m_foot_id_in_trunkModel = m_trunk_SSM->GetBodyId(footLink.c_str());
	if(m_foot_id_in_trunkModel >= m_trunk_SSM->fixed_body_discriminator)
	{
		unsigned int fb = m_foot_id_in_trunkModel - m_trunk_SSM->fixed_body_discriminator;
		m_fixedTransform = m_trunk_SSM->mFixedBodies[fb].mParentTransform;
		m_foot_id_in_trunkModel = m_trunk_SSM->mFixedBodies[fb].mMovableParent;
	}

	// Setup foot polygon in _foot_link coordinates
	double w = nimbro_op_model::FOOT_WIDTH;
	double l = nimbro_op_model::FOOT_LENGTH;

	m_footPolygon.points[0] << +l/2.0, +w/2.0, 0.0;
	m_footPolygon.points[1] << -l/2.0, +w/2.0, 0.0;
	m_footPolygon.points[2] << -l/2.0, -w/2.0, 0.0;
	m_footPolygon.points[3] << +l/2.0, -w/2.0, 0.0;
}

void Leg::moveFoot(const Vector3& goalInTrunk, const Matrix3& rotInTrunk)
{
	m_ik.sendTargetsFor(goalInTrunk, rotInTrunk);
}

void Leg::computeForwardKinematics()
{
	const Math::SpatialTransform& trans = m_trunk_SSM->X_base[m_foot_id_in_trunkModel];

	m_trunk_SSM->updateRBDLJointPos(robotcontrol::SingleSupportModel::CommandData);
	m_cmd_posInTrunk = trans.r + trans.E.transpose() * m_fixedTransform.r;
	m_cmd_rotInTrunk = trans.E.transpose() * m_fixedTransform.E.transpose();

	m_trunk_SSM->updateRBDLJointPos(robotcontrol::SingleSupportModel::MeasurementData);
	m_act_posInTrunk = trans.r + trans.E.transpose() * m_fixedTransform.r;
	m_act_rotInTrunk = trans.E.transpose() * m_fixedTransform.E.transpose();
}

Leg::FootPolygon Leg::footPolygonInTrunk() const
{
	FootPolygon ret;

	for(int i = 0; i < 4; ++i)
		ret.points[i] = m_act_rotInTrunk * m_footPolygon.points[i] + m_act_posInTrunk;

	return ret;
}

}
