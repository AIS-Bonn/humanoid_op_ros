// Gait utility class that provides inverse pose representation functionality
// File: gait_inverse_pose.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_INVERSE_POSE_H
#define GAIT_INVERSE_POSE_H

// Includes
#include <gait/util/gait_common_pose.h>
#include <Eigen/Geometry>

// Gait namespace
namespace gait
{
	// Class forward declarations
	class JointPose;
	class JointLegPose;
	class JointArmPose;
	class InversePose;
	class InverseLegPose;
	class InverseArmPose;
	class AbstractPose;
	class AbstractLegPose;
	class AbstractArmPose;

	/**
	* @struct InverseLegPose
	*
	* @brief Data struct that encompasses the inverse representation of a leg pose.
	*
	* The assumed joint order is `hip yaw` &rarr; `hip roll` &rarr; `hip pitch` &rarr; `knee pitch` &rarr; `ankle pitch` &rarr; `ankle roll`.
	* The upper and lower leg links are assumed to be of the same length. The correct value of the link length must be set only if the pose
	* conversion functions are used.
	**/
	struct InverseLegPose
	{
		//
		// Constructor
		//

		//! Default constructor
		explicit InverseLegPose(bool left = true) { reset(left); }

		//! Reset function
		inline void reset(bool left = true)
		{
			cld.reset(left);
			setPose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
		}

		//
		// Set functions
		//

		//! Set the inverse pose (directly set the inverse pose parameters)
		inline void setPose(const Eigen::Vector3d& footPosition, const Eigen::Quaterniond& footRotation = Eigen::Quaterniond::Identity())
		{
			footPos = footPosition;
			footRot = footRotation;
		}

		//! Set the inverse pose in mirror mode (directly set the inverse pose parameters if this is the left leg, or mirror the pose if this is the right leg)
		inline void setPoseMirrored(const Eigen::Vector3d& footPosition, const Eigen::Quaterniond& footRotation = Eigen::Quaterniond::Identity())
		{
			footPos = footPosition;
			footRot = footRotation;
			if(!cld.isLeft)
			{
				footPos.y() = -footPos.y(); // Flip the position about the XZ plane
				footRot.x() = -footRot.x(); // Mirror the orientation about the XZ plane
				footRot.z() = -footRot.z(); // ...
			}
		}

		//! Set the link length used for pose conversions
		inline void setLinkLength(double length)
		{
			cld.linkLength = length;
		}

		//
		// Pose conversion functions
		//

		//! Set the inverse leg pose to a given joint leg pose
		void setFromJointPose(const JointLegPose& pose);

		//! Set the inverse leg pose to a given abstract leg pose
		void setFromAbstractPose(const AbstractLegPose& pose);

		//! Set the pose of the leg to the pose defined by the given joint angles
		void fromJointAngles(double hipYaw, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll);

		//! Calculate the joint angles corresponding to the current inverse leg pose
		void getJointAngles(double& hipYaw, double& hipRoll, double& hipPitch, double& kneePitch, double& anklePitch, double& ankleRoll) const;

		//
		// Data members
		//

		// Common leg data
		CommonLegData cld;          //!< Data that is shared by all leg pose representations

		// Leg pose
		Eigen::Vector3d footPos;    //!< Position of the foot relative to its zero position (in body-fixed coordinates)
		Eigen::Quaterniond footRot; //!< Rotation of the foot relative to its zero position (relative to the body-fixed frame)
	};

	/**
	* @struct InverseArmPose
	*
	* @brief Data struct that encompasses the inverse representation of an arm pose.
	*
	* The assumed joint order is `shoulder pitch` &rarr; `shoulder roll` &rarr; `elbow pitch`. The upper and lower arm links are assumed to be of
	* the same length (i.e. `cad.linkLength`). The correct value of the link length must be set only if the pose conversion functions are used.
	**/
	struct InverseArmPose
	{
		//
		// Constructor
		//

		//! Default constructor
		explicit InverseArmPose(bool left = true) { reset(left); }

		//! Reset function
		inline void reset(bool left = true)
		{
			cad.reset(left);
			setPose(/* TODO */);
		}

		//
		// Set functions
		//

		//! Set the inverse pose (directly set the inverse pose parameters)
		inline void setPose(/* TODO */)
		{
		}

		//! Set the inverse pose in mirror mode (directly set the inverse pose parameters if this is the left arm, or mirror the pose if this is the right arm)
		inline void setPoseMirrored(/* TODO */)
		{
		}

		//! Set the link length used for pose conversions
		inline void setLinkLength(double length)
		{
			cad.linkLength = length;
		}

		//
		// Pose conversion functions
		//

		//! Set the inverse arm pose to a given joint arm pose
		void setFromJointPose(const JointArmPose& pose);

		//! Set the inverse arm pose to a given abstract arm pose
		void setFromAbstractPose(const AbstractArmPose& pose);

		//! Set the pose of the arm to the pose defined by the given joint angles
		void fromJointAngles(double shoulderPitch, double shoulderRoll, double elbowPitch);

		//! Calculate the joint angles corresponding to the current inverse arm pose
		void getJointAngles(double& shoulderPitch, double& shoulderRoll, double& elbowPitch) const;

		//
		// Data members
		//

		// Common arm data
		CommonArmData cad; //!< Data that is shared by all arm pose representations

		// Arm pose
		/* TODO */
	};

	/**
	* @struct InversePose
	*
	* @brief Data struct that encompasses the inverse representation of a robot pose.
	*
	* The correct value of the link lengths must be set only if the pose conversion functions are used.
	**/
	struct InversePose
	{
		//
		// Constructor
		//

		//! Default constructor
		InversePose() { reset(); }

		//! Reset function
		inline void reset()
		{
			leftLeg.reset(true);
			rightLeg.reset(false);
			leftArm.reset(true);
			rightArm.reset(false);
		}

		//
		// Set functions
		//

		//! Set the arm and leg link lengths used for pose conversions
		inline void setLinkLengths(double legLinkLength, double armLinkLength)
		{
			leftLeg.cld.setLinkLength(legLinkLength);
			rightLeg.cld.setLinkLength(legLinkLength);
			leftArm.cad.setLinkLength(armLinkLength);
			rightArm.cad.setLinkLength(armLinkLength);
		}

		//
		// Pose conversion functions
		//

		//! Set the inverse pose to a given joint pose
		void setFromJointPose(const JointPose& pose);

		//! Set the inverse pose to a given abstract pose
		void setFromAbstractPose(const AbstractPose& pose);

		//! Set the inverse leg poses to given joint poses
		inline void setLegsFromJointPose(const JointLegPose& left, const JointLegPose& right)
		{
			leftLeg.setFromJointPose(left);
			rightLeg.setFromJointPose(right);
		}

		//! Set the inverse leg poses to given abstract poses
		inline void setLegsFromAbstractPose(const AbstractLegPose& left, const AbstractLegPose& right)
		{
			leftLeg.setFromAbstractPose(left);
			rightLeg.setFromAbstractPose(right);
		}

		//! Set the inverse arm poses to given joint poses
		inline void setArmsFromJointPose(const JointArmPose& left, const JointArmPose& right)
		{
			leftArm.setFromJointPose(left);
			rightArm.setFromJointPose(right);
		}

		//! Set the inverse arm poses to given abstract poses
		inline void setArmsFromAbstractPose(const AbstractArmPose& left, const AbstractArmPose& right)
		{
			leftArm.setFromAbstractPose(left);
			rightArm.setFromAbstractPose(right);
		}

		//
		// Data members
		//

		// Inverse limb pose structs
		InverseLegPose leftLeg;  //!< Inverse pose of the left leg
		InverseLegPose rightLeg; //!< Inverse pose of the right leg
		InverseArmPose leftArm;  //!< Inverse pose of the left arm
		InverseArmPose rightArm; //!< Inverse pose of the right arm
	};
}

#endif /* GAIT_INVERSE_POSE_H */
// EOF