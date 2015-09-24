// Gait utility class that provides inverse pose representation functionality
// File: gait_inverse_pose.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <gait/util/gait_joint_pose.h>
#include <gait/util/gait_inverse_pose.h>
#include <gait/util/gait_abstract_pose.h>
#include <nimbro_utils/math_funcs.h>
#include <stdio.h> // TODO: TEMP => For the "not implemented yet" messages...

// Namespaces
using namespace gait;
using namespace nimbro_utils;

//
// InverseLegPose
//

// Set the inverse leg pose to a given joint leg pose
void InverseLegPose::setFromJointPose(const JointLegPose& pose)
{
	cld = pose.cld;
	fromJointAngles(pose.hipYaw, pose.hipRoll, pose.hipPitch, pose.kneePitch, pose.anklePitch, pose.ankleRoll);
}

// Set the inverse leg pose to a given abstract leg pose
void InverseLegPose::setFromAbstractPose(const AbstractLegPose& pose)
{
	cld = pose.cld;
	double hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll;
	pose.getJointAngles(hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll);
	fromJointAngles(hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll);
}

// Set the pose of the leg to the pose defined by the given joint angles
void InverseLegPose::fromJointAngles(double hipYaw, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll)
{
	// Construct quaternions for the three major joint rotations
	Eigen::Quaterniond hipRot   = Eigen::AngleAxisd(hipYaw,     Eigen::Vector3d::UnitZ())
	                            * Eigen::AngleAxisd(hipRoll,    Eigen::Vector3d::UnitX())
	                            * Eigen::AngleAxisd(hipPitch,   Eigen::Vector3d::UnitY());
	Eigen::Quaterniond kneeRot   (Eigen::AngleAxisd(kneePitch,  Eigen::Vector3d::UnitY()));
	Eigen::Quaterniond ankleRot = Eigen::AngleAxisd(anklePitch, Eigen::Vector3d::UnitY())
	                            * Eigen::AngleAxisd(ankleRoll,  Eigen::Vector3d::UnitX());

	// Calculate the foot rotation
	Eigen::Quaterniond shinRot = hipRot * kneeRot;
	footRot = shinRot * ankleRot;

	// Calculate the foot position
	footPos = -cld.linkLength * (hipRot * Eigen::Vector3d::UnitZ() + shinRot * Eigen::Vector3d::UnitZ()); // = hipRot*(0,0,-L) + shinRot*(0,0,-L)
	footPos.z() += 2.0*cld.linkLength;
}

// Calculate the joint pose corresponding to the current inverse leg pose
void InverseLegPose::getJointAngles(double& hipYaw, double& hipRoll, double& hipPitch, double& kneePitch, double& anklePitch, double& ankleRoll) const
{
	// Notes:
	// - The hip origin is the fixed point of intersection between all three hip joint axes.
	// - The ankle origin is the point of intersection between the two ankle joint axes.
	// - We assume that the lengths of the upper and lower legs (thigh/shank), defined as the perpendicular distances
	//   between the hip and knee, and knee and ankle pitch axes respectively, are equal. This common length is denoted
	//   L and is given by the class member cld.linkLength.
	// - The zero foot position, defined to be the vector displacement from the hip origin to the ankle origin in
	//   body-fixed coordinates when the robot is in its zero position, is given by (0,0,-2L).
	// - The displacement from the hip origin to the ankle origin in body-fixed coordinates is given by the sum of
	//   footPos and the zero foot position, as footPos is defined relative to the zero foot position.
	// - The rotation from the body-fixed frame to the frame attached to the foot is given by footRot.

	// TODO: Definitions for getJointAngles IK
	// - Define {H} to be a frame located at the hip origin, aligned with the body-fixed frame.
	// - Define {F} to be a frame located at the ankle origin, aligned with the local coordinate system of the foot.
	//   In the zero pose this frame should have zero rotation relative to {H}.

	// TODO: Case where hip origin and ankle origin coincide? Case where y=z=0 already problematic enough? Multiple solutions!
	// TODO: This function needs to be reworked!

	//
	// Preliminary calculations
	//

	// Ensure that the foot rotation is a unit quaternion
	Eigen::Quaterniond footRotation = footRot.normalized();

	// Calculate the coordinates of the ankle origin in terms of the hip frame {H}
	double nominalLegLength = 2.0*cld.linkLength; // = 2L
	Eigen::Vector3d ankleInHipFrame = footPos;
	ankleInHipFrame.z() -= nominalLegLength; // Add (0,0,-2L) to footPos

	// Calculate the coordinates of the hip origin in terms of the foot frame {F}
	Eigen::Vector3d hipInAnkleFrame = footRotation.conjugate() * (-ankleInHipFrame);

	// Calculate the leg length as the distance between the hip and ankle origins
	double legLength = hipInAnkleFrame.norm();
	if(legLength > nominalLegLength)
	{
		hipInAnkleFrame *= nominalLegLength / legLength;
		legLength = nominalLegLength;
	}

	//
	// Joint angle calculations
	//

	// Calculate the knee pitch based on the hip origin to ankle origin separation
	double alpha = acos(coerce(legLength/nominalLegLength, 0.0, 1.0));
	kneePitch = 2.0*alpha;

	// Calculate the ankle roll (singularity with multiple solutions if hipInAnkleFrame y = z = 0)
	ankleRoll = atan2(hipInAnkleFrame.y(), hipInAnkleFrame.z());

	// Calculate the ankle pitch (singularity with multiple solutions if legLength = 0) // TODO: And if legLength = 0?
	double legAxisPitch = asin(coerce(-hipInAnkleFrame.x()/legLength, -1.0, 1.0));
	anklePitch = legAxisPitch - alpha;

	// Calculate the hip yaw
	Eigen::Vector3d normal = (footRotation * Eigen::AngleAxisd(-ankleRoll, Eigen::Vector3d::UnitX())) * Eigen::Vector3d::UnitY(); // y-axis of the {A} frame (ankle roll removed) in body-fixed coordinates
	Eigen::Vector3d dir = normal.cross(Eigen::Vector3d::UnitZ());
	hipYaw = atan2(dir.y(), dir.x());

	// Transform the hip vector into the hip frame and eliminate the hip yaw angle
	hipInAnkleFrame = (Eigen::AngleAxisd(-hipYaw, Eigen::Vector3d::UnitZ())
	                * footRotation
	                * Eigen::AngleAxisd(-ankleRoll, Eigen::Vector3d::UnitX())
	                * Eigen::AngleAxisd(-legAxisPitch, Eigen::Vector3d::UnitY())) * Eigen::Vector3d::UnitZ();

	// Calculate the hip roll and pitch
	hipRoll = atan2(-hipInAnkleFrame.y(), hipInAnkleFrame.z());
	hipPitch = asin(coerce(hipInAnkleFrame.x()/hipInAnkleFrame.norm(), -1.0, 1.0)) - alpha;
}

//
// InverseArmPose
//

// Set the inverse arm pose to a given joint arm pose
void InverseArmPose::setFromJointPose(const JointArmPose& pose) // TODO: Write this!
{
	printf("WARNING: The function '%s' hasn't been implemented yet (NO INVERSE ARM POSE YET)!\n", __PRETTY_FUNCTION__);
}

// Set the inverse arm pose to a given abstract arm pose
void InverseArmPose::setFromAbstractPose(const AbstractArmPose& pose) // TODO: Write this!
{
	printf("WARNING: The function '%s' hasn't been implemented yet (NO INVERSE ARM POSE YET)!\n", __PRETTY_FUNCTION__);
}

// Set the pose of the arm to the pose defined by the given joint angles
void InverseArmPose::fromJointAngles(double shoulderPitch, double shoulderRoll, double elbowPitch) // TODO: Write this!
{
	printf("WARNING: The function '%s' hasn't been implemented yet (NO INVERSE ARM POSE YET)!\n", __PRETTY_FUNCTION__);
}

// Calculate the joint angles corresponding to the current inverse arm pose
void InverseArmPose::getJointAngles(double& shoulderPitch, double& shoulderRoll, double& elbowPitch) const // TODO: Write this!
{
	printf("WARNING: The function '%s' hasn't been implemented yet (NO INVERSE ARM POSE YET)!\n", __PRETTY_FUNCTION__);
}

//
// InversePose
//

// Set the inverse pose to a given joint pose
void InversePose::setFromJointPose(const JointPose& pose)
{
	leftLeg.setFromJointPose(pose.leftLeg);
	rightLeg.setFromJointPose(pose.rightLeg);
	leftArm.setFromJointPose(pose.leftArm);
	rightArm.setFromJointPose(pose.rightArm);
}

// Set the inverse pose to a given abstract pose
void InversePose::setFromAbstractPose(const AbstractPose& pose)
{
	leftLeg.setFromAbstractPose(pose.leftLeg);
	rightLeg.setFromAbstractPose(pose.rightLeg);
	leftArm.setFromAbstractPose(pose.leftArm);
	rightArm.setFromAbstractPose(pose.rightArm);
}
// EOF