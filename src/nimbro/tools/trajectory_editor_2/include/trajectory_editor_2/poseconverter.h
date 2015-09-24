#ifndef POSECONVERTER_H
#define POSECONVERTER_H

// Handle conversions between poses
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <boost/shared_ptr.hpp>
#include <motion_file/motionfile.h>

#include <gait/util/gait_joint_pose.h>
#include <gait/util/gait_abstract_pose.h>
#include <gait/util/gait_inverse_pose.h>

using namespace std;
using namespace gait;

class PoseConverter
{
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	PoseConverter();
	~PoseConverter();
	
	static JointPose getJointPose(KeyframePtr frame, vector<string> &jointList);
	
	static void updateFrame(AbstractPose &abstractPose, KeyframePtr frame, vector<string> &jointList);
	static void updateFrame(InversePose &inversePose, KeyframePtr frame, vector<string> &jointList);
	
private:
	// Get position of joint namejointName
	static double getPos(KeyframePtr frame, vector<string> &jointList, string jointName);
	
	// Set position of joint namejointName to 'pos'
	static void setPos(KeyframePtr frame, vector<string> &jointList, string jointName, double pos);
	
	static void updateFrame(JointPose &pose, KeyframePtr frame, vector<string> &jointList, bool fromInverse);
};

#endif // POSECONVERTER_H