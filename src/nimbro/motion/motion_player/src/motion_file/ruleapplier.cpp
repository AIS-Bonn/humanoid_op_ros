// Class to apply rule parts to keyframe
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <math.h>

#include <motion_file/ruleapplier.h>
#include <motion_file/motionfile.h>
#include <motion_file/poseconverter.h>

namespace motionfile
{

RuleApplier::RuleApplier()
{
	m_inverse_pose.setLinkLengths(0.2, 0.2);
	m_temp_inverse_pose.setLinkLengths(0.2, 0.2);
}

bool RuleApplier::applyRulePart(motionfile::KeyframePtr frame, const std::vector<std::string>& joint_list, const motionfile::RulePart& part, double delta, bool apply, bool limit_inverse, double epsilon)
{
	if(part.type == "joint")
		return applyJoint(frame, joint_list, part, delta, apply);
	else if(part.type == "abstract")
		return applyAbstract(frame, joint_list, part, delta, apply);
	else if(part.type == "inverse")
		return applyInverse(frame, joint_list, part, delta, apply, limit_inverse, epsilon);
	
	return false;
}

bool RuleApplier::applyJoint(motionfile::KeyframePtr frame, const std::vector<std::string>& joint_list, const motionfile::RulePart& part, double delta, bool apply)
{
	int index = motionfile::Motion::nameToIndex(joint_list, part.jointName);
	
	// Check if joint exists
	if(index == -1)
	{
		ROS_ERROR("Joint %s was not found in motion. Check your rule!", part.jointName.c_str());
		return false;
	}
	
	double new_value = frame->joints[index].position + delta * part.scale;
	
	// Check limits
	if(new_value > M_PI || new_value < -M_PI)
	{
		ROS_ERROR("Joint %s was going to have position %f which is out of limits!", part.jointName.c_str(), new_value);
		return false;
	}
	
	if(apply)
		frame->joints[index].position = new_value;
	
	return true;
}

bool RuleApplier::applyAbstract(motionfile::KeyframePtr frame, const std::vector<std::string>& joint_list, const motionfile::RulePart& part, double delta, bool apply)
{
	motionfile::PoseConverter p;
	
	gait::AbstractPose temp_abstract_pose;
	temp_abstract_pose.setFromJointPose(p.getJointPose(frame, joint_list));
	
	double currentPos = getPos(temp_abstract_pose, part.jointName);
	
	// Check if joint exists
	if(currentPos == -999)
	{
		ROS_ERROR("Joint %s was not found in motion. Check your rule!", part.jointName.c_str());
		return false;
	}
	
	double new_value = currentPos + delta * part.scale;
	
	// Check limits
	if(!inAbstractLimits(part.jointName, new_value))
	{
		ROS_ERROR("Joint %s was going to have position %f which is out of limits!", part.jointName.c_str(), new_value);
		return false;
	}
	
	if(apply)
	{
		setPos(temp_abstract_pose, part.jointName, new_value);
		p.updateFrame(temp_abstract_pose, frame, joint_list);
	}
	
	return true;
}

bool RuleApplier::inAbstractLimits(const std::string& joint_name, const double value)
{
	if(joint_name == "left_arm_extension" ||
		joint_name == "right_arm_extension" ||
		joint_name == "left_leg_extension" ||
		joint_name == "right_leg_extension")
	{
		return (value <= 1 && value >= 0);
	}
	
	return (value < M_PI && value > -M_PI);
}

bool RuleApplier::inInverseLimits(const std::string& joint_name, const double value)
{
	return (value < M_PI && value > -M_PI);
}

double RuleApplier::getPos(const gait::AbstractPose& abstract_pose, const std::string& joint_name)
{
	// Left arm
	if(joint_name == "left_shoulder_pitch")
		return abstract_pose.leftArm.angleY;
	else if(joint_name == "left_shoulder_roll")
		return abstract_pose.leftArm.angleX;
	else if(joint_name == "left_arm_extension")
		return abstract_pose.leftArm.extension;
	
	// Right arm
	else if(joint_name == "right_shoulder_pitch")
		return abstract_pose.rightArm.angleY;
	else if(joint_name == "right_shoulder_roll")
		return abstract_pose.rightArm.angleX;
	else if(joint_name == "right_arm_extension")
		return abstract_pose.rightArm.extension;
	
	// Left left leg
	else if(joint_name == "left_hip_yaw")
		return abstract_pose.leftLeg.angleZ;
	else if(joint_name == "left_hip_roll")
		return abstract_pose.leftLeg.angleX;
	else if(joint_name == "left_hip_pitch")
		return abstract_pose.leftLeg.angleY;
	else if(joint_name == "left_leg_extension")
		return abstract_pose.leftLeg.extension;
	else if(joint_name == "left_ankle_pitch")
		return abstract_pose.leftLeg.footAngleY;
	else if(joint_name == "left_ankle_roll")
		return abstract_pose.leftLeg.footAngleX;

	// Right leg
	else if(joint_name == "right_hip_yaw")
		return abstract_pose.rightLeg.angleZ;
	else if(joint_name == "right_hip_roll")
		return abstract_pose.rightLeg.angleX;
	else if(joint_name == "right_hip_pitch")
		return abstract_pose.rightLeg.angleY;
	else if(joint_name == "right_leg_extension")
		return abstract_pose.rightLeg.extension;
	else if(joint_name == "right_ankle_pitch")
		return abstract_pose.rightLeg.footAngleY;
	else if(joint_name == "right_ankle_roll")
		return abstract_pose.rightLeg.footAngleX;
	
	return -999;
}

void RuleApplier::setPos(gait::AbstractPose& abstract_pose, const std::string& joint_name, const double value)
{
	// Left arm
	if(joint_name == "left_shoulder_pitch")
		abstract_pose.leftArm.angleY = value;
	else if(joint_name == "left_shoulder_roll")
		abstract_pose.leftArm.angleX = value;
	else if(joint_name == "left_arm_extension")
		abstract_pose.leftArm.extension = value;
	
	// Right arm
	else if(joint_name == "right_shoulder_pitch")
		abstract_pose.rightArm.angleY = value;
	else if(joint_name == "right_shoulder_roll")
		abstract_pose.rightArm.angleX = value;
	else if(joint_name == "right_arm_extension")
		abstract_pose.rightArm.extension = value;
	
	// Left left leg
	else if(joint_name == "left_hip_yaw")
		abstract_pose.leftLeg.angleZ = value;
	else if(joint_name == "left_hip_roll")
		abstract_pose.leftLeg.angleX = value;
	else if(joint_name == "left_hip_pitch")
		abstract_pose.leftLeg.angleY = value;
	else if(joint_name == "left_leg_extension")
		abstract_pose.leftLeg.extension = value;
	else if(joint_name == "left_ankle_pitch")
		abstract_pose.leftLeg.footAngleY = value;
	else if(joint_name == "left_ankle_roll")
		abstract_pose.leftLeg.footAngleX = value;

	// Right leg
	else if(joint_name == "right_hip_yaw")
		abstract_pose.rightLeg.angleZ = value;
	else if(joint_name == "right_hip_roll")
		abstract_pose.rightLeg.angleX = value;
	else if(joint_name == "right_hip_pitch")
		abstract_pose.rightLeg.angleY = value;
	else if(joint_name == "right_leg_extension")
		abstract_pose.rightLeg.extension = value;
	else if(joint_name == "right_ankle_pitch")
		abstract_pose.rightLeg.footAngleY = value;
	else if(joint_name == "right_ankle_roll")
		abstract_pose.rightLeg.footAngleX = value;
}

bool RuleApplier::applyInverse(motionfile::KeyframePtr frame, const std::vector<std::string>& joint_list, const motionfile::RulePart& part, double delta, bool apply, bool limit_inverse, double epsilon)
{
	motionfile::PoseConverter p;
	
	gait::JointPose jointPose = p.getJointPose(frame, joint_list);
	
	m_inverse_pose.leftLeg.setFromJointPose(jointPose.leftLeg);
	m_inverse_pose.rightLeg.setFromJointPose(jointPose.rightLeg);
	
	double currentPos = getPos(m_inverse_pose, part.jointName);
	
	// Check if joint exists
	if(currentPos == -999)
	{
		ROS_ERROR("Joint %s was not found in motion. Check your rule!", part.jointName.c_str());
		return false;
	}
	
	double new_value = currentPos + delta * part.scale;
	
	// Check limits
	if(!inInverseLimits(part.jointName, new_value))
	{
		ROS_ERROR("Joint %s was going to have position %f which is out of limits!", part.jointName.c_str(), new_value);
		return false;
	}
	
	if(limit_inverse) // Change 1 DOF only
	{
		// Some preliminary conversions
		gait::JointPose joint_pose = p.getJointPose(frame, joint_list);
	
		m_temp_inverse_pose.leftLeg.setFromJointPose(joint_pose.leftLeg);
		m_temp_inverse_pose.rightLeg.setFromJointPose(joint_pose.rightLeg);
		
		setPos(m_temp_inverse_pose, part.jointName, new_value);
		
		joint_pose.leftLeg.setFromInversePose(m_temp_inverse_pose.leftLeg);
		joint_pose.rightLeg.setFromInversePose(m_temp_inverse_pose.rightLeg);
		
		m_temp_inverse_pose.leftLeg.setFromJointPose(joint_pose.leftLeg);
		m_temp_inverse_pose.rightLeg.setFromJointPose(joint_pose.rightLeg);
		
		// Check if this change affects joints other than 'joint_name'
		if(!motionfile::PoseConverter::equalExcept(m_inverse_pose, m_temp_inverse_pose, part.jointName, epsilon))
		{
			ROS_ERROR("More than 1 joint was changed for more than epsilon(%f). Increase the epsilon, or turn off this feature", epsilon);
			return false;
		}
	}
	
	if(apply)
	{
		setPos(m_inverse_pose, part.jointName, new_value);
		p.updateFrame(m_inverse_pose, frame, joint_list);
	}
	
	return true;
}

double RuleApplier::getPos(const gait::InversePose& inverse_pose, const std::string& joint_name)
{
	if(joint_name == "left_leg_x")
		return inverse_pose.leftLeg.footPos.x();
	else if(joint_name == "left_leg_y")
		return inverse_pose.leftLeg.footPos.y();
	else if(joint_name == "left_leg_z")
		return inverse_pose.leftLeg.footPos.z();
	
	else if(joint_name == "right_leg_x")
		return inverse_pose.rightLeg.footPos.x();
	else if(joint_name == "right_leg_y")
		return inverse_pose.rightLeg.footPos.y();
	else if(joint_name == "right_leg_z")
		return inverse_pose.rightLeg.footPos.z();
	
	else if(joint_name == "left_foot_yaw" || joint_name == "left_foot_pitch" || joint_name == "left_foot_roll")
	{
		motionfile::PoseConverter::Angles left_angles = motionfile::PoseConverter::anglesFromQuaternion(inverse_pose.leftLeg.footRot);
		
		if(joint_name == "left_foot_yaw")
			return left_angles.yaw;
		else if(joint_name == "left_foot_pitch")
			return left_angles.pitch;
		else if(joint_name == "left_foot_roll")
			return left_angles.roll;
	}
	else if(joint_name == "right_foot_yaw" || joint_name == "right_foot_pitch" || joint_name == "right_foot_roll")
	{
		motionfile::PoseConverter::Angles right_angles = motionfile::PoseConverter::anglesFromQuaternion(inverse_pose.rightLeg.footRot);
		
		if(joint_name == "right_foot_yaw")
			return right_angles.yaw;
		else if(joint_name == "right_foot_pitch")
			return right_angles.pitch;
		else if(joint_name == "right_foot_roll")
			return right_angles.roll;
	}
	
	return -999;
}

void RuleApplier::setPos(gait::InversePose& inverse_pose, const std::string& joint_name, const double value)
{
	if(joint_name == "left_leg_x")
		inverse_pose.leftLeg.footPos.x() = value;
	else if(joint_name == "left_leg_y")
		inverse_pose.leftLeg.footPos.y() = value;
	else if(joint_name == "left_leg_z")
		inverse_pose.leftLeg.footPos.z() = value;
	
	else if(joint_name == "right_leg_x")
		inverse_pose.rightLeg.footPos.x() = value;
	else if(joint_name == "right_leg_y")
		inverse_pose.rightLeg.footPos.y() = value;
	else if(joint_name == "right_leg_z")
		inverse_pose.rightLeg.footPos.z() = value;
	
	else if(joint_name == "left_foot_yaw" || joint_name == "left_foot_pitch" || joint_name == "left_foot_roll")
	{
		motionfile::PoseConverter::Angles left_angles = motionfile::PoseConverter::anglesFromQuaternion(inverse_pose.leftLeg.footRot);
		
		if(joint_name == "left_foot_yaw")
			left_angles.yaw = value;
		else if(joint_name == "left_foot_pitch")
			left_angles.pitch = value;
		else if(joint_name == "left_foot_roll")
			left_angles.roll = value;
		
		inverse_pose.leftLeg.footRot = motionfile::PoseConverter::quaternionFromAngles(left_angles);
	}
	else if(joint_name == "right_foot_yaw" || joint_name == "right_foot_pitch" || joint_name == "right_foot_roll")
	{
		motionfile::PoseConverter::Angles right_angles = motionfile::PoseConverter::anglesFromQuaternion(inverse_pose.rightLeg.footRot);
		
		if(joint_name == "right_foot_yaw")
			right_angles.yaw = value;
		else if(joint_name == "right_foot_pitch")
			right_angles.pitch = value;
		else if(joint_name == "right_foot_roll")
			right_angles.roll = value;
		
		inverse_pose.rightLeg.footRot = motionfile::PoseConverter::quaternionFromAngles(right_angles);
	}
}

RuleApplier::~RuleApplier()
{

}

}