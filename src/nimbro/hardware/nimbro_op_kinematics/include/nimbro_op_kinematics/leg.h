// Leg representation
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LEG_H
#define LEG_H

#include <robotcontrol/model/robotmodel.h>

#include <nimbro_op_kinematics/leg_ik.h>
#include <nimbro_op_kinematics/types.h>

namespace nimbro_op_kinematics
{

class Leg
{
public:
	struct FootPolygon
	{
		inline void rotate(const Matrix3& rot)
		{
			for(int i = 0; i < 4; ++i)
				points[i] = rot * points[i];
		}

		Vector3 points[4];
	};

	Leg(robotcontrol::RobotModel* robotModel, const std::string& footLink, const std::string& otherFootLink, double sign);

	void moveFoot(const Vector3& goalInTrunk, const Matrix3& rotInTrunk);

	void computeForwardKinematics();

	inline const Vector3& commandedFootPositionInTrunk() const
	{ return m_cmd_posInTrunk; }

	inline const Matrix3& commandedRotation() const
	{ return m_cmd_rotInTrunk; }

	inline const Vector3& measuredPosition() const
	{ return m_act_posInTrunk; }

	inline const Matrix3& measuredRotation() const
	{ return m_act_rotInTrunk; }

	inline Vector3 transformFootToTrunk(const Vector3& local) const
	{ return commandedFootPositionInTrunk() + commandedRotation() * local; }

	inline Vector3 transformTrunkToFoot(const Vector3& inTrunk) const
	{ return commandedRotation().transpose() * (inTrunk - commandedFootPositionInTrunk()); }

	inline Vector3 transformMeasuredTrunkToFoot(const Vector3& inTrunk) const
	{ return measuredRotation().transpose() * (inTrunk - measuredPosition()); }

	//! Return the foot polygon in trunk coordinates
	FootPolygon footPolygonInTrunk() const;
private:
	//! Support model for trunk
	boost::shared_ptr<robotcontrol::SingleSupportModel> m_trunk_SSM;

	//! Inverse IK for our foot
	nimbro_op_kinematics::LegIK m_ik;

	//! @name Body IDs in all support models
	//@{
	unsigned int m_foot_id_in_trunkModel;
	//@}

	//! Fixed transform to first movable parent
	Math::SpatialTransform m_fixedTransform;

	//! @name Foot plane polygon (in ankle frame)
	FootPolygon m_footPolygon;

	//! @name State
	//@{
	Vector3 m_cmd_posInTrunk;
	Matrix3 m_cmd_rotInTrunk;
	Vector3 m_act_posInTrunk;
	Matrix3 m_act_rotInTrunk;
	//@}
};

}

#endif
