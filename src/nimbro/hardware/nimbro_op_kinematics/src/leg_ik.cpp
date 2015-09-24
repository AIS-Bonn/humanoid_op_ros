// Closed-form Inverse Kinematics for robot legs
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_op_kinematics/leg_ik.h>

#include <nimbro_op_model/dimensions.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

#include <rbdl/Kinematics.h>

#include <eigen_conversions/eigen_msg.h>

#include <exception>

#define PUBLISH_MARKERS 0

namespace nimbro_op_kinematics
{

LegIK::LegIK(const boost::shared_ptr<robotcontrol::SingleSupportModel>& model, const std::string& tip)
 : m_model(model)
 , m_tip(tip)
{
	unsigned int id = model->GetBodyId(tip.c_str());
	if(id == (unsigned int)-1)
		throw std::logic_error((std::string("LegIK: unknown link: ") + tip).c_str());

	Math::VectorNd Q = Math::VectorNd::Constant(model->dof_count, 0);
	RigidBodyDynamics::UpdateKinematicsCustom(*model, &Q, 0, 0);

	// The first transform is probably a fixed one
	m_footTransform.E.setIdentity();
	m_footTransform.r.setZero();

	if(id >= model->fixed_body_discriminator)
	{
		unsigned int fbody_id = id - model->fixed_body_discriminator;

		m_footTransform = model->mFixedBodies[fbody_id].mParentTransform;

		ROS_INFO_STREAM("LegIK(" << tip << "): foot offset: " << m_footTransform.r.transpose());

		// Continue at the next non-fixed body
		id = model->mFixedBodies[fbody_id].mMovableParent;
	}

	// Build a chain of joints to the trunk link
	for(int pid = id; pid != 0; pid = model->lambda[pid])
	{
		ROS_INFO("LegIK for %s: link %lu: %s", tip.c_str(), m_links.size(), model->GetBodyName(pid).c_str());
		m_links.push_back(pid);
	}

	/* Links:
	 * link 1: left_ankle_link
	 * link 2: left_shank_link
	 * link 3: left_thigh_link
	 * link 4: left_hip_roll_link
	 * link 5: left_hip_yaw_link
	 */

	// Sanity checks
	if(m_links.size() != 6)
		throw std::logic_error("LegIK: We need exactly 6 joints between the foot frame and trunk.");

	for(size_t i = 0; i < m_links.size(); ++i)
	{
		const Math::SpatialTransform& X_lambda = model->X_lambda[m_links[i]];

		if(!X_lambda.E.isIdentity())
		{
			ROS_ERROR("LegIK(%s): The rotation from %s to %s is not zero.",
				tip.c_str(), model->GetBodyName(model->lambda[m_links[i]]).c_str(),
				model->GetBodyName(m_links[i]).c_str()
			);
// 			ROS_ERROR_STREAM("E: " << X_lambda.E);
		}
	}

	const Math::SpatialTransform trans_foot_ankle = model->X_lambda[m_links[0]];
	if(!trans_foot_ankle.r.isZero())
	{
		ROS_ERROR("LegIK(%s): the foot and ankle frames are not equal. Please check the URDF model!", tip.c_str());
		ROS_ERROR_STREAM("r: " << trans_foot_ankle.r.transpose());
	}

	const Math::SpatialTransform& trans_ankle_knee = model->X_lambda[m_links[1]];
	m_shankLength = trans_ankle_knee.r.norm();
	ROS_INFO("LegIK for %s: shank length: %8.5lfm", tip.c_str(), m_shankLength);

// 	if(!trans_ankle_knee.E.isIdentity())
// 	{
// 		ROS_ERROR("LegIK(%s): The rotation from knee to hip is not zero.", tip.c_str());
// 		ROS_ERROR_STREAM("E: " << trans_ankle_knee.E);
// 	}

	const Math::SpatialTransform& trans_knee_hip = model->X_lambda[m_links[2]];
	m_thighLength = trans_knee_hip.r.norm();
	ROS_INFO("LegIK for %s: thigh length: %8.5lfm", tip.c_str(), m_thighLength);

// 	const Math::SpatialTransform trans_hip_roll_yaw = model->X_lambda[m_links[4]];
// 	if(!trans_hip_roll_yaw.r.isZero())
// 	{
// 		ROS_ERROR("LegIK(%s): the hip roll and yaw frames are not equal. Please check the URDF model!", tip.c_str());
// 		ROS_ERROR_STREAM("r: " << trans_hip_roll_yaw.r.transpose());
// 	}

	m_hipTransform = model->X_base[m_links[5]];
	ROS_INFO_STREAM("LegIK(" << tip << "): hip offset: " << m_hipTransform.r.transpose());

	m_hipTransform.r.z() += model->X_lambda[m_links[4]].r.z() + model->X_lambda[m_links[3]].r.z();

	ROS_INFO_STREAM("LegIK(" << tip << "): hip offset: " << m_hipTransform.r.transpose());

	m_hip_yaw_pitch_offset << m_model->X_lambda[m_links[4]].r.x() + m_model->X_lambda[m_links[3]].r.x(), 0.0, 0.0;

	// Some pre-calculated stuff for the IK
	m_shankLength2 = m_shankLength * m_shankLength;
	m_thighLength2 = m_thighLength * m_thighLength;
	m_lengthDiv = 2.0 * m_shankLength * m_thighLength;

	if(fabs(m_thighLength - m_shankLength) > 1e-5)
	{
		throw std::logic_error("LegIK: FIXME: Thigh and shank links need to have the same length at the moment");
	}

	if(tip == "left_foot_link")
		m_sign = 1;
	else
		m_sign = -1;

#if PUBLISH_MARKERS
	m_pub_markers = ros::NodeHandle("~").advertise<visualization_msgs::MarkerArray>("legik/markers", 1);
#endif
}

LegIK::~LegIK()
{

}

inline void drawArrow(visualization_msgs::MarkerArray* array, const std::string& ns, const std::string& frame, const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
	visualization_msgs::Marker marker;
	marker.ns = ns;
	marker.id = array->markers.size();
	marker.type = visualization_msgs::Marker::ARROW;
	marker.points.resize(2);
	tf::pointEigenToMsg(from, marker.points[0]);
	tf::pointEigenToMsg(to, marker.points[1]);
	marker.scale.x = 0.005;
	marker.scale.y = 0.025;
	marker.scale.z = 0;

	marker.action = visualization_msgs::Marker::ADD;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time::now();

	array->markers.push_back(marker);
}

bool LegIK::doIK(JointVec* q, const Vector3& footLocation, const Eigen::Matrix3d& footRot)
{
	// Remove the fixed hip transform (assume no rotation)
	Vector3 goal = footLocation - m_hipTransform.r;

	// Remove the fixed foot transform (assume no rotation)
	goal = goal - footRot * m_footTransform.r;

	// Rotation semantics: All rotations convert from local coordinates to
	// global coordinates.

	// The general idea is to treat the shank-knee-thigh assembly as a prismatic
	// joint. The knee angle can then be calculated in a last step from the
	// distance between ankle and hip.

	// 1) Determine the roll angle at the foot
	Vector3 goal_foot = footRot.transpose() * (-goal);

	if(goal_foot.z() == 0)
		return false;

	double roll_angle = atan(goal_foot.y() / goal_foot.z());
	Eigen::Matrix3d rot_ankle_roll;
	rot_ankle_roll = Eigen::AngleAxisd(roll_angle, Vector3::UnitX());

	// Eliminate the roll from the following equations
	goal_foot = rot_ankle_roll * goal_foot;

	// Okay, the foot is in a fixed pose and the ankle roll angle is already
	// determined. This means our hip can move on a plane defined by
	// the ankle pitch axis (as the normal).

	// In particular, the hip roll axis needs to lie completely inside the
	// plane. We can use that to calculate the hip yaw.

	// The plane normal (in trunk coordinate system)
	Vector3 normal = footRot * rot_ankle_roll.transpose() * Vector3::UnitY();

	// We are only interested in the direction of the intersection (and we know
	// one exists as we rotated the plane in step 1) to that effect)
	Vector3 intersection = normal.cross(Vector3::UnitZ());

	if(intersection.x() == 0)
		return false;

	// We need to rotate the X axis onto the intersection.
	double yaw_angle = atan2(intersection.y(), intersection.x());

	Eigen::Matrix3d rot_yaw;
	rot_yaw = Eigen::AngleAxisd(yaw_angle, Vector3::UnitZ());

	// Determine the location of the intersection hip_roll/pitch in foot coordinates
	Vector3 pitch_goal_foot = rot_ankle_roll * footRot.transpose() * (-goal + rot_yaw * m_hip_yaw_pitch_offset);

	if(pitch_goal_foot.z() == 0)
		return false;

	double pitch_angle = atan2(-pitch_goal_foot.x(), pitch_goal_foot.z());

	Eigen::Matrix3d rot_ankle_pitch;
	rot_ankle_pitch = Eigen::AngleAxisd(pitch_angle, Vector3::UnitY());

	// Determine missing angles in the hip
	// This rotation matrix describes the rotation in the roll and pitch joints
	// of the hip.
	Eigen::Matrix3d shank_to_yawed_hip = rot_yaw.transpose() * footRot * rot_ankle_roll.transpose() * rot_ankle_pitch.transpose();

	// As the rotation matrix only consists of two rotations, it we can recover
	// sin/cos values of both rotations from the product.
	double hip_roll_angle = atan2(shank_to_yawed_hip(2,1), shank_to_yawed_hip(1,1));
	double hip_pitch_angle = atan2(shank_to_yawed_hip(0,2), shank_to_yawed_hip(0,0));

	// Now we can replace the prismatic joint with the real knee
	double length = pitch_goal_foot.norm();

	// Calculate the angle between the shank and thigh links (Law of cosines)
	double knee_len = (m_shankLength2 + m_thighLength2 - length*length) / m_lengthDiv;
	double knee_angle = acos(knee_len);
	if(isnan(knee_angle))
		return false; // Goal is too far away

#if PUBLISH_MARKERS
	visualization_msgs::MarkerArray markers;
	drawArrow(&markers, m_tip, "/trunk_link", Vector3::Zero(), footLocation);
	drawArrow(&markers, m_tip, "/trunk_link", footLocation, footLocation + footRot * Vector3(0.2, 0.0, 0.0));
	drawArrow(&markers, m_tip, "/trunk_link", m_hipTransform.r, m_hipTransform.r + m_hip_yaw_pitch_offset);

	Vector3 boxLoc(nimbro_op_model::FOOT_CENTER_OFFSET_X, m_sign * nimbro_op_model::FOOT_CENTER_OFFSET_Y_LEFT, -nimbro_op_model::ANKLE_Z_HEIGHT/2.0);
	boxLoc = footLocation + footRot * boxLoc;

	visualization_msgs::Marker box;
	box.ns = m_tip;
	box.id = markers.markers.size();
	box.type = visualization_msgs::Marker::CUBE;
	box.scale.x = nimbro_op_model::FOOT_LENGTH;
	box.scale.y = nimbro_op_model::FOOT_WIDTH;
	box.scale.z = nimbro_op_model::ANKLE_Z_HEIGHT;

	tf::pointEigenToMsg(boxLoc, box.pose.position);
	tf::quaternionEigenToMsg(Eigen::Quaterniond(footRot), box.pose.orientation);

	box.action = visualization_msgs::Marker::ADD;
	box.color.r = 0.0;
	box.color.g = 0.0;
	box.color.b = 1.0;
	box.color.a = 0.5;

	box.header.frame_id = "/trunk_link";
	box.header.stamp = ros::Time::now();
	markers.markers.push_back(box);


	m_pub_markers.publish(markers);
#endif



	// FIXME: The correction for the knee angle in the pitch joints below
	//  is only correct if m_tighLength == m_shankLength.
	//  If you fix this, please remove the check in the constructor above.

	// Ankle roll
	(*q)[0] = roll_angle;

	// Ankle pitch
	(*q)[1] = pitch_angle - asin(sin(knee_angle) / length * m_shankLength);

	// Knee pitch
	(*q)[2] = M_PI - knee_angle;

	// Hip pitch
	(*q)[3] = hip_pitch_angle - asin(sin(knee_angle) / length * m_thighLength);

	// Hip roll
	(*q)[4] = hip_roll_angle;

	// Hip yaw
	(*q)[5] = yaw_angle;

	return true;
}

bool LegIK::doIK(LegIK::JointVec* q, LegIK::JointVec* v, LegIK::JointVec* a,
	const LegIK::Vector3& footLocation, const LegIK::Vector3& footVel,
	const LegIK::Vector3& footAcc, const LegIK::Matrix3& footRot)
{
	JointVec q0, q1, q2;

	if(!doIK(&q0, footLocation, footRot))
		return false;

	// Estimate joint accelerations and velocities by extrapolating
	// into the future and running inverse kinematics again.
	// FIXME: This does not take angular velocity into account!
	double h = 0.0001;
	Vector3 in_1step = footLocation + footVel * h + 0.5 * footAcc * h*h;
	Vector3 in_2step = footLocation + footVel * 2.0 * h + 0.5 * footAcc * (2.0*h)*(2.0*h);
	if(!doIK(&q1, in_1step, footRot))
		return false;
	if(!doIK(&q2, in_2step, footRot))
		return false;

	JointVec q_vel, q_acc;

	*q = q0;
	*v = (q1 - q0) / h;
	*a = (q2 - 2.0 * q1 + q0) / (h*h);

	return true;
}

LegIK::JointVec LegIK::currentCommandedJointVec() const
{
	JointVec ret;

	for(int i = 0; i < 6; ++i)
		ret[i] = m_model->joint(m_links[i]-1)->cmd.pos;

	return ret;
}

void LegIK::sendJointVec(const LegIK::JointVec& cmd)
{
	for(int i = 0; i < 6; ++i)
		m_model->setJointCmd(m_links[i]-1, cmd[i]);
}

void LegIK::sendJointVec(const LegIK::JointVec& q, const LegIK::JointVec& v, const LegIK::JointVec& a)
{
	ros::Time now = ros::Time::now();
	for(int i = 0; i < 6; ++i)
		m_model->joint(m_links[i]-1)->cmd.setFromPosVelAcc(q[i], v[i], a[i]);
}

bool LegIK::sendTargetsFor(const Vector3& footLocation, const Vector3& footVel, const Vector3& footAcc, const Eigen::Matrix3d& footRot)
{
	JointVec q, v, a;
	if(!doIK(&q, &v, &a, footLocation, footVel, footAcc, footRot))
		return false;

	sendJointVec(q, v, a);

	return true;
}

bool LegIK::sendTargetsFor(const Vector3& footLocation, const Eigen::Matrix3d& footRot)
{
	JointVec q;
	if(!doIK(&q, footLocation, footRot))
		return false;

	sendJointVec(q);

	return true;
}

}
