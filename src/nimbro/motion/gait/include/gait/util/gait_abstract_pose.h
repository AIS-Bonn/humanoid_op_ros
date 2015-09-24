// Gait utility class that provides abstract pose representation functionality
// File: gait_abstract_pose.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_ABSTRACT_POSE_H
#define GAIT_ABSTRACT_POSE_H

// Includes
#include <gait/util/gait_common_pose.h>

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
	* @struct AbstractLegPose
	*
	* @brief Data struct that encompasses the abstract representation of a leg pose.
	*
	* The assumed joint order is `hip yaw` &rarr; `hip roll` &rarr; `hip pitch` &rarr; `knee pitch` &rarr; `ankle pitch` &rarr; `ankle roll`.
	* The upper and lower leg links are assumed to be of the same length. The correct value of the link length must be set only if
	* `setFromInversePose()` is used.
	**/
	struct AbstractLegPose
	{
		//
		// Constructor
		//

		//! Default constructor
		explicit AbstractLegPose(bool left = true) { reset(left); }

		//! Reset function
		inline void reset(bool left = true)
		{
			cld.reset(left);
			setPose(0.0, 0.0, 0.0, 0.0);
			setFootPose(0.0, 0.0);
		}

		//
		// Set functions
		//

		//! Set the abstract pose (directly set the abstract pose parameters)
		inline void setPose(double ext, double angX, double angY, double angZ)
		{
			extension = ext;
			angleX = angX;
			angleY = angY;
			angleZ = angZ;
		}

		//! Set the abstract pose in mirror mode (directly set the abstract pose parameters if this is the left leg, or mirror the pose if this is the right leg)
		inline void setPoseMirrored(double ext, double angX, double angY, double angZ)
		{
			extension = ext;
			angleX = (cld.isLeft ? angX : -angX);
			angleY = angY;
			angleZ = (cld.isLeft ? angZ : -angZ);
		}

		//! Set the abstract foot pose (directly set the abstract foot pose parameters)
		inline void setFootPose(double footAngX, double footAngY)
		{
			footAngleX = footAngX;
			footAngleY = footAngY;
		}

		//! Set the abstract foot pose in mirror mode (directly set the abstract foot pose parameters if this is the left leg, or mirror the pose if this is the right leg)
		inline void setFootPoseMirrored(double footAngX, double footAngY)
		{
			footAngleX = (cld.isLeft ? footAngX : -footAngX);
			footAngleY = footAngY;
		}

		//! Set the link length used for pose conversions
		inline void setLinkLength(double length)
		{
			cld.linkLength = length;
		}

		//
		// Pose conversion functions
		//

		//! Set the abstract leg pose to a given joint leg pose
		void setFromJointPose(const JointLegPose& pose);

		//! Set the abstract leg pose to a given inverse leg pose
		void setFromInversePose(const InverseLegPose& pose);

		//! Set the pose of the leg to the pose defined by the given joint angles
		void fromJointAngles(double hipYaw, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll);

		//! Calculate the joint angles corresponding to the current abstract leg pose
		void getJointAngles(double& hipYaw, double& hipRoll, double& hipPitch, double& kneePitch, double& anklePitch, double& ankleRoll) const;

		//
		// Data members
		//

		// Common leg data
		CommonLegData cld; //!< Data that is shared by all leg pose representations

		// Leg pose
		double extension;  //!< Extension of the leg (in the range `[0,1]`, 0 = Fully extended, 1 = Fully contracted)
		double angleX;     //!< Orientation angle of the leg axis about the positive x axis (2nd ZXY Euler angle)
		double angleY;     //!< Orientation angle of the leg axis about the positive y axis (3rd ZXY Euler angle)
		double angleZ;     //!< Orientation angle of the leg axis about the positive z axis (1st ZXY Euler angle)

		// Foot pose
		double footAngleX; //!< Global orientation angle of the foot about the positive x axis (more precisely, if all non-roll servo positions are zeroed then footAngleX is the pure roll rotation from the global frame to the foot frame)
		double footAngleY; //!< Global orientation angle of the foot about the positive y axis (more precisely, if all non-pitch servo positions are zeroed then footAngleY is the pure pitch rotation from the global frame to the foot frame)
	};

	/**
	* @struct AbstractArmPose
	*
	* @brief Data struct that encompasses the abstract representation of an arm pose.
	*
	* The assumed joint order is `shoulder pitch` &rarr; `shoulder roll` &rarr; `elbow pitch`. The upper and lower arm links are assumed to be of
	* the same length (i.e. `cad.linkLength`). The correct value of the link length must be set only if `setFromInversePose()` is used.
	**/
	struct AbstractArmPose
	{
		//
		// Constructor
		//

		//! Default constructor
		explicit AbstractArmPose(bool left = true) { reset(left); }

		//! Reset function
		inline void reset(bool left = true)
		{
			cad.reset(left);
			setPose(0.0, 0.0, 0.0);
		}

		//
		// Set functions
		//

		//! Set the abstract pose (directly set the abstract pose parameters)
		inline void setPose(double ext, double angX, double angY)
		{
			extension = ext;
			angleX = angX;
			angleY = angY;
		}

		//! Set the abstract pose in mirror mode (directly set the abstract pose parameters if this is the left arm, or mirror the pose if this is the right arm)
		inline void setPoseMirrored(double ext, double angX, double angY)
		{
			extension = ext;
			angleX = (cad.isLeft ? angX : -angX);
			angleY = angY;
		}

		//! Set the link length used for pose conversions
		inline void setLinkLength(double length)
		{
			cad.linkLength = length;
		}

		//
		// Pose conversion functions
		//

		//! Set the abstract arm pose to a given joint arm pose
		void setFromJointPose(const JointArmPose& pose);

		//! Set the abstract arm pose to a given inverse arm pose
		void setFromInversePose(const InverseArmPose& pose);

		//! Set the pose of the arm to the pose defined by the given joint angles
		void fromJointAngles(double shoulderPitch, double shoulderRoll, double elbowPitch);

		//! Calculate the joint angles corresponding to the current abstract arm pose
		void getJointAngles(double& shoulderPitch, double& shoulderRoll, double& elbowPitch) const;

		//
		// Data members
		//

		// Common arm data
		CommonArmData cad; //!< Data that is shared by all arm pose representations

		// Arm pose
		double extension;  //!< Extension of the arm (in the range `[0,1]`, 0 = Fully extended, 1 = Fully contracted)
		double angleX;     //!< Orientation angle of the arm axis about the positive x axis (3rd ZYX Euler angle, angleZ is zero)
		double angleY;     //!< Orientation angle of the arm axis about the positive y axis (2nd ZYX Euler angle, angleZ is zero)
	};

	/**
	* @struct AbstractPose
	*
	* @brief Data struct that encompasses the abstract representation of a robot pose.
	*
	* The correct value of the link lengths must be set only if a pose conversion function involving the inverse pose representation is used.
	**/
	struct AbstractPose
	{
		//
		// Constructor
		//

		//! Default constructor
		AbstractPose() { reset(); }

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

		//! Set the abstract pose to a given joint pose
		void setFromJointPose(const JointPose& pose);

		//! Set the abstract pose to a given inverse pose
		void setFromInversePose(const InversePose& pose);

		//! Set the abstract leg poses to given joint poses
		inline void setLegsFromJointPose(const JointLegPose& left, const JointLegPose& right)
		{
			leftLeg.setFromJointPose(left);
			rightLeg.setFromJointPose(right);
		}

		//! Set the abstract leg poses to given inverse poses
		inline void setLegsFromInversePose(const InverseLegPose& left, const InverseLegPose& right)
		{
			leftLeg.setFromInversePose(left);
			rightLeg.setFromInversePose(right);
		}

		//! Set the abstract arm poses to given joint poses
		inline void setArmsFromJointPose(const JointArmPose& left, const JointArmPose& right)
		{
			leftArm.setFromJointPose(left);
			rightArm.setFromJointPose(right);
		}

		//! Set the abstract arm poses to given inverse poses
		inline void setArmsFromInversePose(const InverseArmPose& left, const InverseArmPose& right)
		{
			leftArm.setFromInversePose(left);
			rightArm.setFromInversePose(right);
		}

		//
		// Data members
		//

		// Abstract limb pose structs
		AbstractLegPose leftLeg;  //!< Abstract pose of the left leg
		AbstractLegPose rightLeg; //!< Abstract pose of the right leg
		AbstractArmPose leftArm;  //!< Abstract pose of the left arm
		AbstractArmPose rightArm; //!< Abstract pose of the right arm
	};
}

#endif /* GAIT_ABSTRACT_POSE_H */
// EOF