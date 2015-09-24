// Analytical Inverse Kinematics for robot legs
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LEG_IK_H
#define LEG_IK_H

#include <rbdl/Model.h>

#include <robotcontrol/model/singlesupportmodel.h>

#include <ros/publisher.h>

namespace nimbro_op_kinematics
{

class LegIK
{
public:
	typedef Eigen::Matrix<double, 6, 1> JointVec;
	typedef Eigen::Vector3d Vector3;
	typedef Eigen::Matrix3d Matrix3;

	/**
	 * Construct a LegIK instance for single support model @b model, with
	 * foot link @b tip
	 **/
	LegIK(const boost::shared_ptr<robotcontrol::SingleSupportModel>& model, const std::string& tip);
	virtual ~LegIK();

	//! @name Joint space access
	//@{
	//! Get current commanded joint positions of the leg
	JointVec currentCommandedJointVec() const;

	//! Set commanded joint positions of the leg
	void sendJointVec(const JointVec& cmd);

	//! Set commanded joint positions, velocities and accelerations of the leg
	void sendJointVec(const JointVec& q, const JointVec& v, const JointVec& a);
	//@}

	//! @name Inverse kinematics
	//@{
	//! Calculate joint positions @b q from foot location and rotation
	bool doIK(JointVec* q, const Vector3& footLocation, const Matrix3& footRot = Matrix3::Identity());

	//! Calculate joint positions, velocities and accelerations
	bool doIK(JointVec* q, JointVec* v, JointVec* a, const Vector3& footLocation, const Vector3& footVel, const Vector3& footAcc, const Matrix3& footRot = Matrix3::Identity());
	//@}

	//! @name Convenience methods
	//@{
	//! Calculate and set joint positions for goal foot location and rotation
	bool sendTargetsFor(const Vector3& footLocation, const Matrix3& footRot = Matrix3::Identity());

	//! Calculate and set joint positions, velocities and accelerations
	bool sendTargetsFor(const Vector3& footLocation, const Vector3& footVel, const Vector3& footAcc, const Matrix3& footRot = Matrix3::Identity());
	//@}
private:
	boost::shared_ptr<robotcontrol::SingleSupportModel> m_model;
	std::string m_tip;
	int m_sign;

	std::vector<unsigned int> m_links;

	Math::SpatialTransform m_hipTransform;
	Math::SpatialTransform m_footTransform;
	double m_shankLength;
	double m_shankLength2;
	double m_thighLength;
	double m_thighLength2;

	double m_lengthDiv;

	ros::Publisher m_pub_markers;

	Eigen::Vector3d m_hip_yaw_pitch_offset;
};

}

#endif
