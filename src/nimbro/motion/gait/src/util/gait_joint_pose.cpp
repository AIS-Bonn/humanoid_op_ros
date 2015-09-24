// Gait utility class that provides joint pose representation functionality
// File: gait_joint_pose.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <gait/util/gait_joint_pose.h>
#include <gait/util/gait_inverse_pose.h>
#include <gait/util/gait_abstract_pose.h>

// Namespaces
using namespace gait;

//
// JointLegPose
//

// Set the joint leg pose to a given inverse leg pose
void JointLegPose::setFromInversePose(const InverseLegPose& pose)
{
	cld = pose.cld;
	pose.getJointAngles(hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll);
}

// Set the joint leg pose to a given abstract leg pose
void JointLegPose::setFromAbstractPose(const AbstractLegPose& pose)
{
	cld = pose.cld;
	pose.getJointAngles(hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll);
}

// Blend this joint pose towards another one by a given blending factor
void JointLegPose::blendTowards(const JointLegPose& other, double b) // b: 0 => No change, 1 => Replace by other
{
	double invb = 1.0 - b;
	hipYaw = invb*hipYaw + b*other.hipYaw;
	hipRoll = invb*hipRoll + b*other.hipRoll;
	hipPitch = invb*hipPitch + b*other.hipPitch;
	kneePitch = invb*kneePitch + b*other.kneePitch;
	anklePitch = invb*anklePitch + b*other.anklePitch;
	ankleRoll = invb*ankleRoll + b*other.ankleRoll;
	cld.effortHipYaw = invb*cld.effortHipYaw + b*other.cld.effortHipYaw;
	cld.effortHipRoll = invb*cld.effortHipRoll + b*other.cld.effortHipRoll;
	cld.effortHipPitch = invb*cld.effortHipPitch + b*other.cld.effortHipPitch;
	cld.effortKneePitch = invb*cld.effortKneePitch + b*other.cld.effortKneePitch;
	cld.effortAnklePitch = invb*cld.effortAnklePitch + b*other.cld.effortAnklePitch;
	cld.effortAnkleRoll = invb*cld.effortAnkleRoll + b*other.cld.effortAnkleRoll;
	cld.supportCoeff = invb*cld.supportCoeff + b*other.cld.supportCoeff;
}

//
// JointArmPose
//

// Set the joint arm pose to a given inverse arm pose
void JointArmPose::setFromInversePose(const InverseArmPose& pose)
{
	cad = pose.cad;
	pose.getJointAngles(shoulderPitch, shoulderRoll, elbowPitch);
}

// Set the joint arm pose to a given abstract arm pose
void JointArmPose::setFromAbstractPose(const AbstractArmPose& pose)
{
	cad = pose.cad;
	pose.getJointAngles(shoulderPitch, shoulderRoll, elbowPitch);
}

// Blend this joint pose towards another one by a given blending factor
void JointArmPose::blendTowards(const JointArmPose& other, double b) // b: 0 => No change, 1 => Replace by other
{
	double invb = 1.0 - b;
	shoulderPitch = invb*shoulderPitch + b*other.shoulderPitch;
	shoulderRoll = invb*shoulderRoll + b*other.shoulderRoll;
	elbowPitch = invb*elbowPitch + b*other.elbowPitch;
	cad.effortShoulderPitch = invb*cad.effortShoulderPitch + b*other.cad.effortShoulderPitch;
	cad.effortShoulderRoll = invb*cad.effortShoulderRoll + b*other.cad.effortShoulderRoll;
	cad.effortElbowPitch = invb*cad.effortElbowPitch + b*other.cad.effortElbowPitch;
}

//
// JointPose
//

// Set the joint pose to a given inverse pose
void JointPose::setFromInversePose(const InversePose& pose)
{
	leftLeg.setFromInversePose(pose.leftLeg);
	rightLeg.setFromInversePose(pose.rightLeg);
	leftArm.setFromInversePose(pose.leftArm);
	rightArm.setFromInversePose(pose.rightArm);
}

// Set the joint pose to a given abstract pose
void JointPose::setFromAbstractPose(const AbstractPose& pose)
{
	leftLeg.setFromAbstractPose(pose.leftLeg);
	rightLeg.setFromAbstractPose(pose.rightLeg);
	leftArm.setFromAbstractPose(pose.leftArm);
	rightArm.setFromAbstractPose(pose.rightArm);
}

// Set the joint pose to the values specified by a given joint position array
void JointPose::readJointPosArray(const double (&pos)[NUM_JOINTS])
{
	// Transcribe values for the left leg
	leftLeg.hipYaw = pos[L_HIP_YAW];
	leftLeg.hipRoll = pos[L_HIP_ROLL];
	leftLeg.hipPitch = pos[L_HIP_PITCH];
	leftLeg.kneePitch = pos[L_KNEE_PITCH];
	leftLeg.anklePitch = pos[L_ANKLE_PITCH];
	leftLeg.ankleRoll = pos[L_ANKLE_ROLL];
	
	// Transcribe values for the right leg
	rightLeg.hipYaw = pos[R_HIP_YAW];
	rightLeg.hipRoll = pos[R_HIP_ROLL];
	rightLeg.hipPitch = pos[R_HIP_PITCH];
	rightLeg.kneePitch = pos[R_KNEE_PITCH];
	rightLeg.anklePitch = pos[R_ANKLE_PITCH];
	rightLeg.ankleRoll = pos[R_ANKLE_ROLL];

	// Transcribe values for the left arm
	leftArm.shoulderPitch = pos[L_SHOULDER_PITCH];
	leftArm.shoulderRoll = pos[L_SHOULDER_ROLL];
	leftArm.elbowPitch = pos[L_ELBOW_PITCH];

	// Transcribe values for the right arm
	rightArm.shoulderPitch = pos[R_SHOULDER_PITCH];
	rightArm.shoulderRoll = pos[R_SHOULDER_ROLL];
	rightArm.elbowPitch = pos[R_ELBOW_PITCH];
}

// Set the joint efforts to the values specified by a given joint effort array
void JointPose::readJointEffortArray(const double (&effort)[NUM_JOINTS])
{
	// Transcribe values for the left leg
	leftLeg.cld.effortHipYaw = effort[L_HIP_YAW];
	leftLeg.cld.effortHipRoll = effort[L_HIP_ROLL];
	leftLeg.cld.effortHipPitch = effort[L_HIP_PITCH];
	leftLeg.cld.effortKneePitch = effort[L_KNEE_PITCH];
	leftLeg.cld.effortAnklePitch = effort[L_ANKLE_PITCH];
	leftLeg.cld.effortAnkleRoll = effort[L_ANKLE_ROLL];

	// Transcribe values for the right leg
	rightLeg.cld.effortHipYaw = effort[R_HIP_YAW];
	rightLeg.cld.effortHipRoll = effort[R_HIP_ROLL];
	rightLeg.cld.effortHipPitch = effort[R_HIP_PITCH];
	rightLeg.cld.effortKneePitch = effort[R_KNEE_PITCH];
	rightLeg.cld.effortAnklePitch = effort[R_ANKLE_PITCH];
	rightLeg.cld.effortAnkleRoll = effort[R_ANKLE_ROLL];

	// Transcribe values for the left arm
	leftArm.cad.effortShoulderPitch = effort[L_SHOULDER_PITCH];
	leftArm.cad.effortShoulderRoll = effort[L_SHOULDER_ROLL];
	leftArm.cad.effortElbowPitch = effort[L_ELBOW_PITCH];

	// Transcribe values for the right arm
	rightArm.cad.effortShoulderPitch = effort[R_SHOULDER_PITCH];
	rightArm.cad.effortShoulderRoll = effort[R_SHOULDER_ROLL];
	rightArm.cad.effortElbowPitch = effort[R_ELBOW_PITCH];
}

// Transcribe the stored joint pose to a joint position array
void JointPose::writeJointPosArray(double (&pos)[NUM_JOINTS]) const
{
	// Transcribe values for the left leg
	pos[L_HIP_YAW] = leftLeg.hipYaw;
	pos[L_HIP_ROLL] = leftLeg.hipRoll;
	pos[L_HIP_PITCH] = leftLeg.hipPitch;
	pos[L_KNEE_PITCH] = leftLeg.kneePitch;
	pos[L_ANKLE_PITCH] = leftLeg.anklePitch;
	pos[L_ANKLE_ROLL] = leftLeg.ankleRoll;

	// Transcribe values for the right leg
	pos[R_HIP_YAW] = rightLeg.hipYaw;
	pos[R_HIP_ROLL] = rightLeg.hipRoll;
	pos[R_HIP_PITCH] = rightLeg.hipPitch;
	pos[R_KNEE_PITCH] = rightLeg.kneePitch;
	pos[R_ANKLE_PITCH] = rightLeg.anklePitch;
	pos[R_ANKLE_ROLL] = rightLeg.ankleRoll;

	// Transcribe values for the left arm
	pos[L_SHOULDER_PITCH] = leftArm.shoulderPitch;
	pos[L_SHOULDER_ROLL] = leftArm.shoulderRoll;
	pos[L_ELBOW_PITCH] = leftArm.elbowPitch;

	// Transcribe values for the right arm
	pos[R_SHOULDER_PITCH] = rightArm.shoulderPitch;
	pos[R_SHOULDER_ROLL] = rightArm.shoulderRoll;
	pos[R_ELBOW_PITCH] = rightArm.elbowPitch;
}

// Transcribe the stored joint efforts to a joint effort array
void JointPose::writeJointEffortArray(double (&effort)[NUM_JOINTS]) const
{
	// Transcribe values for the left leg
	effort[L_HIP_YAW] = leftLeg.cld.effortHipYaw;
	effort[L_HIP_ROLL] = leftLeg.cld.effortHipRoll;
	effort[L_HIP_PITCH] = leftLeg.cld.effortHipPitch;
	effort[L_KNEE_PITCH] = leftLeg.cld.effortKneePitch;
	effort[L_ANKLE_PITCH] = leftLeg.cld.effortAnklePitch;
	effort[L_ANKLE_ROLL] = leftLeg.cld.effortAnkleRoll;

	// Transcribe values for the right leg
	effort[R_HIP_YAW] = rightLeg.cld.effortHipYaw;
	effort[R_HIP_ROLL] = rightLeg.cld.effortHipRoll;
	effort[R_HIP_PITCH] = rightLeg.cld.effortHipPitch;
	effort[R_KNEE_PITCH] = rightLeg.cld.effortKneePitch;
	effort[R_ANKLE_PITCH] = rightLeg.cld.effortAnklePitch;
	effort[R_ANKLE_ROLL] = rightLeg.cld.effortAnkleRoll;

	// Transcribe values for the left arm
	effort[L_SHOULDER_PITCH] = leftArm.cad.effortShoulderPitch;
	effort[L_SHOULDER_ROLL] = leftArm.cad.effortShoulderRoll;
	effort[L_ELBOW_PITCH] = leftArm.cad.effortElbowPitch;

	// Transcribe values for the right arm
	effort[R_SHOULDER_PITCH] = rightArm.cad.effortShoulderPitch;
	effort[R_SHOULDER_ROLL] = rightArm.cad.effortShoulderRoll;
	effort[R_ELBOW_PITCH] = rightArm.cad.effortElbowPitch;
}

// Blend this joint pose towards another one by a given blending factor
void JointPose::blendTowards(const JointPose& other, double b) // b: 0 => No change, 1 => Replace by other
{
	leftLeg.blendTowards(other.leftLeg, b);
	rightLeg.blendTowards(other.rightLeg, b);
	leftArm.blendTowards(other.leftArm, b);
	rightArm.blendTowards(other.rightArm, b);
}
// EOF