#include <motion_file/poseconverter.h>

using namespace std;
using namespace gait;
using namespace motionfile;

JointPose PoseConverter::getJointPose(KeyframePtr frame, const vector<string> &jointList)
{
	JointPose jointPose;
	jointPose.setLinkLengths(1, 1); // TODO get correct link lenght!
	
	// Set left arm
	jointPose.leftArm.shoulderPitch = getPos(frame, jointList, "left_shoulder_pitch");
	jointPose.leftArm.shoulderRoll  = getPos(frame, jointList, "left_shoulder_roll");
	jointPose.leftArm.elbowPitch    = getPos(frame, jointList, "left_elbow_pitch");
	
	// Set right arm
	jointPose.rightArm.shoulderPitch = getPos(frame, jointList, "right_shoulder_pitch");
	jointPose.rightArm.shoulderRoll  = getPos(frame, jointList, "right_shoulder_roll");
	jointPose.rightArm.elbowPitch    = getPos(frame, jointList, "right_elbow_pitch");
	
	// Set left leg
	jointPose.leftLeg.anklePitch = getPos(frame, jointList, "left_ankle_pitch");
	jointPose.leftLeg.ankleRoll  = getPos(frame, jointList, "left_ankle_roll");
	jointPose.leftLeg.hipPitch   = getPos(frame, jointList, "left_hip_pitch");
	jointPose.leftLeg.hipRoll    = getPos(frame, jointList, "left_hip_roll");
	jointPose.leftLeg.hipYaw     = getPos(frame, jointList, "left_hip_yaw");
	jointPose.leftLeg.kneePitch  = getPos(frame, jointList, "left_knee_pitch");
	
	// Set right leg
	jointPose.rightLeg.anklePitch = getPos(frame, jointList, "right_ankle_pitch");
	jointPose.rightLeg.ankleRoll  = getPos(frame, jointList, "right_ankle_roll");
	jointPose.rightLeg.hipPitch   = getPos(frame, jointList, "right_hip_pitch");
	jointPose.rightLeg.hipRoll    = getPos(frame, jointList, "right_hip_roll");
	jointPose.rightLeg.hipYaw     = getPos(frame, jointList, "right_hip_yaw");
	jointPose.rightLeg.kneePitch  = getPos(frame, jointList, "right_knee_pitch");
	
	return jointPose;
}

void PoseConverter::updateFrame(AbstractPose &abstractPose, motionfile::KeyframePtr frame, const vector<string> &jointList)
{
	JointPose jointPose;
	jointPose.setFromAbstractPose(abstractPose);
	
	updateFrame(jointPose, frame, jointList, false);
}

void PoseConverter::updateFrame(InversePose &inversePose, motionfile::KeyframePtr frame, const vector<string> &jointList)
{
	JointPose jointPose;
	
	jointPose.leftLeg.setFromInversePose(inversePose.leftLeg);
	jointPose.rightLeg.setFromInversePose(inversePose.rightLeg);
	
	updateFrame(jointPose, frame, jointList, true);
}

void PoseConverter::updateFrame(JointPose &pose, motionfile::KeyframePtr frame, const vector<string> &jointList, bool fromInverse)
{
	if(!fromInverse)
	{
		// Left arm
		setPos(frame, jointList, "left_shoulder_pitch", pose.leftArm.shoulderPitch);
		setPos(frame, jointList, "left_shoulder_roll", pose.leftArm.shoulderRoll);
		setPos(frame, jointList, "left_elbow_pitch", pose.leftArm.elbowPitch);
		
		// Right arm
		setPos(frame, jointList, "right_shoulder_pitch", pose.rightArm.shoulderPitch);
		setPos(frame, jointList, "right_shoulder_roll", pose.rightArm.shoulderRoll);
		setPos(frame, jointList, "right_elbow_pitch", pose.rightArm.elbowPitch);
	}
	
	// Left leg
	setPos(frame, jointList, "left_ankle_pitch", pose.leftLeg.anklePitch);
	setPos(frame, jointList, "left_ankle_roll", pose.leftLeg.ankleRoll);
	setPos(frame, jointList, "left_hip_pitch", pose.leftLeg.hipPitch);
	setPos(frame, jointList, "left_hip_roll", pose.leftLeg.hipRoll);
	setPos(frame, jointList, "left_hip_yaw", pose.leftLeg.hipYaw);
	setPos(frame, jointList, "left_knee_pitch", pose.leftLeg.kneePitch);
	
	// Right leg
	setPos(frame, jointList, "right_ankle_pitch", pose.rightLeg.anklePitch);
	setPos(frame, jointList, "right_ankle_roll", pose.rightLeg.ankleRoll);
	setPos(frame, jointList, "right_hip_pitch", pose.rightLeg.hipPitch);
	setPos(frame, jointList, "right_hip_roll", pose.rightLeg.hipRoll);
	setPos(frame, jointList, "right_hip_yaw", pose.rightLeg.hipYaw);
	setPos(frame, jointList, "right_knee_pitch", pose.rightLeg.kneePitch);
}

bool PoseConverter::equalExcept(InversePose& pose_one, InversePose& pose_two, const string& joint_name, const double epsilon)
{
	std::vector<std::string> not_equal_joints;
	double e = fabs(epsilon);
	
	// Check legs positions
	if(fabs(pose_one.leftLeg.footPos.x() - pose_two.leftLeg.footPos.x()) > e)
		not_equal_joints.push_back("left_leg_x");
	if(fabs(pose_one.leftLeg.footPos.y() - pose_two.leftLeg.footPos.y()) > e)
		not_equal_joints.push_back("left_leg_y");
	if(fabs(pose_one.leftLeg.footPos.z() - pose_two.leftLeg.footPos.z()) > e)
		not_equal_joints.push_back("left_leg_z");
	
	if(fabs(pose_one.rightLeg.footPos.x() - pose_two.rightLeg.footPos.x()) > e)
		not_equal_joints.push_back("right_leg_x");
	if(fabs(pose_one.rightLeg.footPos.y() - pose_two.rightLeg.footPos.y()) > e)
		not_equal_joints.push_back("right_leg_y");
	if(fabs(pose_one.rightLeg.footPos.z() - pose_two.rightLeg.footPos.z()) > e)
		not_equal_joints.push_back("right_leg_z");
	
	// Check legs rotations
	Angles left_angles_one = anglesFromQuaternion(pose_one.leftLeg.footRot);
	Angles right_angles_one = anglesFromQuaternion(pose_one.rightLeg.footRot);
	
	Angles left_angles_two = anglesFromQuaternion(pose_two.leftLeg.footRot);
	Angles right_angles_two = anglesFromQuaternion(pose_two.rightLeg.footRot);
	
	if(fabs(left_angles_one.yaw - left_angles_two.yaw) > e)
		not_equal_joints.push_back("left_foot_yaw");
	if(fabs(left_angles_one.roll - left_angles_two.roll) > e)
		not_equal_joints.push_back("left_foot_roll");
	if(fabs(left_angles_one.pitch - left_angles_two.pitch) > e)
		not_equal_joints.push_back("left_foot_pitch");
	
	if(fabs(right_angles_one.yaw - right_angles_two.yaw) > e)
		not_equal_joints.push_back("left_foot_yaw");
	if(fabs(right_angles_one.roll - right_angles_two.roll) > e)
		not_equal_joints.push_back("left_foot_roll");
	if(fabs(right_angles_one.pitch - right_angles_two.pitch) > e)
		not_equal_joints.push_back("left_foot_pitch");
	
	bool result = true;
	
	/*for(size_t i = 0; i < not_equal_joints.size(); i++)
		printf("Not equal: %s\n", not_equal_joints[i].c_str());
	printf("\n");*/
	
	for(size_t i = 0; i < not_equal_joints.size(); i++)
	{
		if(not_equal_joints[i] != joint_name)
		{
			result = false;
			break;
		}
	}
	
	return result;
}

PoseConverter::Angles PoseConverter::anglesFromQuaternion(Eigen::Quaterniond q)
{
	// Calculate pitch
	// wxyz
	double stheta = 2.0*(q.w()*q.y() - q.z()*q.x());
	stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
	double pitch = asin(stheta);

	// Calculate yaw and roll
	double ysq  = q.y()*q.y();
	double yaw  = atan2(q.w()*q.z()+q.x()*q.y(), 0.5-(ysq+q.z()*q.z()));
	double roll = atan2(q.w()*q.x()+q.y()*q.z(), 0.5-(ysq+q.x()*q.x()));
	
	PoseConverter::Angles angles;
	angles.pitch = pitch;
	angles.yaw   = yaw;
	angles.roll  = roll;
	
	return angles;
}

Eigen::Quaterniond PoseConverter::quaternionFromAngles(PoseConverter::Angles angles)
{
	Eigen::Quaterniond q;
	double cpsi, spsi, cth, sth, cphi, sphi;

	// Halve the yaw, pitch and roll values (for calculation purposes only)
	angles.yaw   *= 0.5;
	angles.pitch *= 0.5;
	angles.roll  *= 0.5;

	// Precalculate the required sin and cos values
	cpsi = cos(angles.yaw);
	spsi = sin(angles.yaw);
	cth  = cos(angles.pitch);
	sth  = sin(angles.pitch);
	cphi = cos(angles.roll);
	sphi = sin(angles.roll);

	// Calculate the required quaternion components
	q.w() = cpsi*cth*cphi + spsi*sth*sphi;
	q.x() = cpsi*cth*sphi - spsi*sth*cphi;
	q.y() = cpsi*sth*cphi + spsi*cth*sphi;
	q.z() = spsi*cth*cphi - cpsi*sth*sphi;
	
	return q;
}

void PoseConverter::setPos(motionfile::KeyframePtr frame, const vector<string> &jointList, string jointName, double pos)
{
	// Find index of jointName
	int index = 0;
	for (unsigned i = 0; i < jointList.size(); i++)
	{
		if (jointList[i] == jointName)
		{
			index = i;
			break;
		}
		else
			index = -1;
	}
	
	if (index < 0)
		return;
	else
		frame->joints[index].position = pos;
}

double PoseConverter::getPos(motionfile::KeyframePtr frame, const vector<string> &jointList, string jointName)
{
	// Find index of jointName
	int index = 0;
	for (unsigned i = 0; i < jointList.size(); i++)
	{
		if (jointList[i] == jointName)
		{
			index = i;
			break;
		}
		else
			index = -1;
	}
	
	if (index < 0) // If valid index was not found -> return 0
		return 0;
	else
		return frame->joints[index].position;
}