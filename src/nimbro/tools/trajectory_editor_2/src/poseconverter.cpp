#include <trajectory_editor_2/poseconverter.h>

#include <ros/package.h>
#include <ros/console.h>

PoseConverter::PoseConverter()
{
	
}

PoseConverter::~PoseConverter()
{
	
}

JointPose PoseConverter::getJointPose(KeyframePtr frame, vector<string> &jointList)
{
	JointPose jointPose;
	jointPose.setLinkLengths(1,1);
	
	// Set left arm
	jointPose.leftArm.shoulderPitch = getPos(frame, jointList, "left_shoulder_pitch");
	jointPose.leftArm.shoulderRoll =  getPos(frame, jointList, "left_shoulder_roll");
	jointPose.leftArm.elbowPitch =    getPos(frame, jointList, "left_elbow_pitch");
	
	// Set right arm
	jointPose.rightArm.shoulderPitch = getPos(frame, jointList, "right_shoulder_pitch");
	jointPose.rightArm.shoulderRoll =  getPos(frame, jointList, "right_shoulder_roll");
	jointPose.rightArm.elbowPitch =    getPos(frame, jointList, "right_elbow_pitch");
	
	// Set left leg
	jointPose.leftLeg.anklePitch = getPos(frame, jointList, "left_ankle_pitch");
	jointPose.leftLeg.ankleRoll =  getPos(frame, jointList, "left_ankle_roll");
	jointPose.leftLeg.hipPitch =   getPos(frame, jointList, "left_hip_pitch");
	jointPose.leftLeg.hipRoll =    getPos(frame, jointList, "left_hip_roll");
	jointPose.leftLeg.hipYaw =     getPos(frame, jointList, "left_hip_yaw");
	jointPose.leftLeg.kneePitch =  getPos(frame, jointList, "left_knee_pitch");
	
	// Set right leg
	jointPose.rightLeg.anklePitch = getPos(frame, jointList, "right_ankle_pitch");
	jointPose.rightLeg.ankleRoll =  getPos(frame, jointList, "right_ankle_roll");
	jointPose.rightLeg.hipPitch =   getPos(frame, jointList, "right_hip_pitch");
	jointPose.rightLeg.hipRoll =    getPos(frame, jointList, "right_hip_roll");
	jointPose.rightLeg.hipYaw =     getPos(frame, jointList, "right_hip_yaw");
	jointPose.rightLeg.kneePitch =  getPos(frame, jointList, "right_knee_pitch");
	
	return jointPose;
}

void PoseConverter::updateFrame(AbstractPose &abstractPose, KeyframePtr frame, vector<string> &jointList)
{
	JointPose jointPose;
	jointPose.setFromAbstractPose(abstractPose);
	
	updateFrame(jointPose, frame, jointList, false);
}

void PoseConverter::updateFrame(InversePose &inversePose, KeyframePtr frame, vector<string> &jointList)
{
	JointPose jointPose;
	
	jointPose.leftLeg.setFromInversePose(inversePose.leftLeg);
	jointPose.rightLeg.setFromInversePose(inversePose.rightLeg);
	
	updateFrame(jointPose, frame, jointList, true);
}

void PoseConverter::updateFrame(JointPose &pose, KeyframePtr frame, vector<string> &jointList, bool fromInverse)
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

void PoseConverter::setPos(KeyframePtr frame, vector<string> &jointList, string jointName, double pos)
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

double PoseConverter::getPos(KeyframePtr frame, vector<string> &jointList, string jointName)
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
	
	if (index < 0) // If valid index was not found -> return -100
		return -100;
	else
		return frame->joints[index].position;
}