// Gait utility class that provides joint pose representation functionality
// File: gait_joint_pose.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_JOINT_POSE_H
#define GAIT_JOINT_POSE_H

// Includes
#include <gait/util/gait_common_pose.h>
#include <gait/gait_common.h>

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
	* @struct JointLegPose
	*
	* @brief Data struct that encompasses the joint representation of a leg pose.
	*
	* The assumed joint order is `hip yaw` &rarr; `hip roll` &rarr; `hip pitch` &rarr; `knee pitch` &rarr; `ankle pitch` &rarr; `ankle roll`.
	* The upper and lower leg links are assumed to be of the same length. The correct value of the link length must be set only if
	* `setFromInversePose()` is used.
	**/
	struct JointLegPose
	{
		//
		// Constructor
		//

		//! Default constructor
		explicit JointLegPose(bool left = true) { reset(left); }

		//! Reset function
		inline void reset(bool left = true)
		{
			cld.reset(left);
			setPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		}

		//
		// Set functions
		//

		//! Set the joint pose (directly set the joint pose parameters)
		inline void setPose(double hipYaw, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll)
		{
			this->hipYaw = hipYaw;
			this->hipRoll = hipRoll;
			this->hipPitch = hipPitch;
			this->kneePitch = kneePitch;
			this->anklePitch = anklePitch;
			this->ankleRoll = ankleRoll;
		}

		//! Set the joint pose in mirror mode (directly set the joint pose parameters if this is the left leg, or mirror the pose if this is the right leg)
		inline void setPoseMirrored(double hipYaw, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll)
		{
			this->hipYaw = (cld.isLeft ? hipYaw : -hipYaw);
			this->hipRoll = (cld.isLeft ? hipRoll : -hipRoll);
			this->hipPitch = hipPitch;
			this->kneePitch = kneePitch;
			this->anklePitch = anklePitch;
			this->ankleRoll = (cld.isLeft ? ankleRoll : -ankleRoll);
		}

		//! Set the link length used for pose conversions
		inline void setLinkLength(double length)
		{
			cld.linkLength = length;
		}

		//
		// Pose conversion functions
		//

		//! Set the joint leg pose to a given inverse leg pose
		void setFromInversePose(const InverseLegPose& pose);

		//! Set the joint leg pose to a given abstract leg pose
		void setFromAbstractPose(const AbstractLegPose& pose);

		//! Set the pose of the leg to the pose defined by the given joint angles
		inline void fromJointAngles(double hipYaw, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll)
		{
			this->hipYaw = hipYaw;
			this->hipRoll = hipRoll;
			this->hipPitch = hipPitch;
			this->kneePitch = kneePitch;
			this->anklePitch = anklePitch;
			this->ankleRoll = ankleRoll;
		}

		//! Retrieve the joint angles corresponding to the current joint leg pose
		inline void getJointAngles(double& hipYaw, double& hipRoll, double& hipPitch, double& kneePitch, double& anklePitch, double& ankleRoll) const
		{
			hipYaw = this->hipYaw;
			hipRoll = this->hipRoll;
			hipPitch = this->hipPitch;
			kneePitch = this->kneePitch;
			anklePitch = this->anklePitch;
			ankleRoll = this->ankleRoll;
		}

		//
		// Other functions
		//

		//! Blend this joint pose towards another one by a given blending factor (@p b: 0 = Pose remains unchanged, 1 = Pose becomes @p other, (0,1) = Blending by linear interpolation between `this` and @p other)
		void blendTowards(const JointLegPose& other, double b);

		//
		// Data members
		//

		// Common leg data
		CommonLegData cld; //!< Data that is shared by all leg pose representations

		// Leg pose
		double hipYaw;     //!< The hip yaw joint position
		double hipRoll;    //!< The hip roll joint position
		double hipPitch;   //!< The hip pitch joint position
		double kneePitch;  //!< The knee pitch joint position
		double anklePitch; //!< The ankle pitch joint position
		double ankleRoll;  //!< The ankle roll joint position
	};

	/**
	* @struct JointArmPose
	*
	* @brief Data struct that encompasses the joint representation of an arm pose.
	*
	* The assumed joint order is `shoulder pitch` &rarr; `shoulder roll` &rarr; `elbow pitch`. The upper and lower arm links are assumed to be of
	* the same length (i.e. `cad.linkLength`). The correct value of the link length must be set only if `setFromInversePose()` is used.
	**/
	struct JointArmPose
	{
		//
		// Constructor
		//

		//! Default constructor
		explicit JointArmPose(bool left = true) { reset(left); }

		//! Reset function
		inline void reset(bool left = true)
		{
			cad.reset(left);
			setPose(0.0, 0.0, 0.0);
		}

		//
		// Set functions
		//

		//! Set the joint pose (directly set the joint pose parameters)
		inline void setPose(double shoulderPitch, double shoulderRoll, double elbowPitch)
		{
			this->shoulderPitch = shoulderPitch;
			this->shoulderRoll = shoulderRoll;
			this->elbowPitch = elbowPitch;
		}

		//! Set the joint pose in mirror mode (directly set the joint pose parameters if this is the left arm, or mirror the pose if this is the right arm)
		inline void setPoseMirrored(double shoulderPitch, double shoulderRoll, double elbowPitch)
		{
			this->shoulderPitch = shoulderPitch;
			this->shoulderRoll = (cad.isLeft ? shoulderRoll : -shoulderRoll);
			this->elbowPitch = elbowPitch;
		}

		//! Set the link length used for pose conversions
		inline void setLinkLength(double length)
		{
			cad.linkLength = length;
		}

		//
		// Pose conversion functions
		//

		//! Set the joint arm pose to a given inverse arm pose
		void setFromInversePose(const InverseArmPose& pose);

		//! Set the joint arm pose to a given abstract arm pose
		void setFromAbstractPose(const AbstractArmPose& pose);

		//! Set the pose of the arm to the pose defined by the given joint angles
		inline void fromJointAngles(double shoulderPitch, double shoulderRoll, double elbowPitch)
		{
			this->shoulderPitch = shoulderPitch;
			this->shoulderRoll = shoulderRoll;
			this->elbowPitch = elbowPitch;
		}

		//! Retrieve the joint angles corresponding to the current joint arm pose
		inline void getJointAngles(double& shoulderPitch, double& shoulderRoll, double& elbowPitch) const
		{
			shoulderPitch = this->shoulderPitch;
			shoulderRoll = this->shoulderRoll;
			elbowPitch = this->elbowPitch;
		}

		//
		// Other functions
		//

		//! Blend this joint pose towards another one by a given blending factor (@p b: 0 = Pose remains unchanged, 1 = Pose becomes @p other, (0,1) = Blending by linear interpolation between `this` and @p other)
		void blendTowards(const JointArmPose& other, double b);

		//
		// Data members
		//

		// Common arm data
		CommonArmData cad;    //!< Data that is shared by all arm pose representations

		// Arm pose
		double shoulderPitch; //!< The shoulder pitch joint position
		double shoulderRoll;  //!< The shoulder roll joint position
		double elbowPitch;    //!< The elbow pitch joint position
	};

	/**
	* @struct JointPose
	*
	* @brief Data struct that encompasses the joint representation of a robot pose.
	*
	* The correct value of the link lengths must be set only if a pose conversion function involving the inverse pose representation is used.
	**/
	struct JointPose
	{
		//
		// Constructor
		//

		//! Default constructor
		JointPose() { reset(); }

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

		//! Set the joint pose to a given inverse pose
		void setFromInversePose(const InversePose& pose);

		//! Set the joint pose to a given abstract pose
		void setFromAbstractPose(const AbstractPose& pose);

		//! Set the joint leg poses to given inverse poses
		inline void setLegsFromInversePose(const InverseLegPose& left, const InverseLegPose& right)
		{
			leftLeg.setFromInversePose(left);
			rightLeg.setFromInversePose(right);
		}

		//! Set the joint leg poses to given abstract poses
		inline void setLegsFromAbstractPose(const AbstractLegPose& left, const AbstractLegPose& right)
		{
			leftLeg.setFromAbstractPose(left);
			rightLeg.setFromAbstractPose(right);
		}

		//! Set the joint arm poses to given inverse poses
		inline void setArmsFromInversePose(const InverseArmPose& left, const InverseArmPose& right)
		{
			leftArm.setFromInversePose(left);
			rightArm.setFromInversePose(right);
		}

		//! Set the joint arm poses to given abstract poses
		inline void setArmsFromAbstractPose(const AbstractArmPose& left, const AbstractArmPose& right)
		{
			leftArm.setFromAbstractPose(left);
			rightArm.setFromAbstractPose(right);
		}

		//
		// Data interface functions
		//

		//! Set the joint pose to the values specified by a given joint position array
		void readJointPosArray(const double (&pos)[NUM_JOINTS]);

		//! Set the joint efforts to the values specified by a given joint effort array
		void readJointEffortArray(const double (&effort)[NUM_JOINTS]);

		//! Transcribe the stored joint pose to a joint position array
		void writeJointPosArray(double (&pos)[NUM_JOINTS]) const;

		//! Transcribe the stored joint efforts to a joint effort array
		void writeJointEffortArray(double (&effort)[NUM_JOINTS]) const;

		//
		// Other functions
		//

		//! Blend this joint pose towards another one by a given blending factor (@p b: 0 = Pose remains unchanged, 1 = Pose becomes @p other, (0,1) = Blending by linear interpolation between `this` and @p other)
		void blendTowards(const JointPose& other, double b);

		//
		// Data members
		//

		// Joint limb pose structs
		JointLegPose leftLeg;  //!< Joint pose of the left leg
		JointLegPose rightLeg; //!< Joint pose of the right leg
		JointArmPose leftArm;  //!< Joint pose of the left arm
		JointArmPose rightArm; //!< Joint pose of the right arm
	};
}

#endif /* GAIT_JOINT_POSE_H */
// EOF