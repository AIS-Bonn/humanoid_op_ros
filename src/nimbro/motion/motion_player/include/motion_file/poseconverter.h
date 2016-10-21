// Handle conversions between poses
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef POSECONVERTER_H
#define POSECONVERTER_H

#include <motion_file/motionfile.h>

#include <gait/util/gait_joint_pose.h>
#include <gait/util/gait_abstract_pose.h>
#include <gait/util/gait_inverse_pose.h>

namespace motionfile
{

class PoseConverter
{
public:	
	struct Angles
	{
		double yaw;
		double roll;
		double pitch;
	};
	
	gait::JointPose getJointPose(motionfile::KeyframePtr frame, const std::vector<std::string> &jointList);
	
	void updateFrame(gait::AbstractPose &abstractPose, motionfile::KeyframePtr frame, const std::vector<std::string> &jointList);
	void updateFrame(gait::InversePose &inversePose, motionfile::KeyframePtr frame, const std::vector<std::string> &jointList);
	
	// Returns true if all joints except 'joint_name' are the same in two given poses
	static bool equalExcept(gait::InversePose &pose_one, gait::InversePose &pose_two, const std::string &joint_name, const double epsilon);
	
	static PoseConverter::Angles anglesFromQuaternion(Eigen::Quaterniond q);
	static Eigen::Quaterniond quaternionFromAngles(PoseConverter::Angles angles);
private:
	// Get position of joint namejointName
	double getPos(motionfile::KeyframePtr frame, const std::vector<std::string> &jointList, std::string jointName);
	
	// Set position of joint namejointName to 'pos'
	void setPos(motionfile::KeyframePtr frame, const std::vector<std::string> &jointList, std::string jointName, double pos);
	
	void updateFrame(gait::JointPose &pose, motionfile::KeyframePtr frame, const std::vector<std::string> &jointList, bool fromInverse);
};

}

#endif // POSECONVERTER_H