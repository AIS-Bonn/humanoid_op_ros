// Unit testing of the gait utility pose representations
// File: test_gait_util_pose.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <string>
#include <iostream>
#include <gtest/gtest.h>
#include <gait/util/gait_joint_pose.h>
#include <gait/util/gait_inverse_pose.h>
#include <gait/util/gait_abstract_pose.h>

// TODO: A lot of this code is just quick hacked and brittle!

// Namespaces
using namespace std;
using namespace gait;

// Print a joint pose
void printJointPose(const JointPose& JP, const std::string& name = "")
{
	// Print the required joint pose
	if(!name.empty()) cout << "Joint pose: " << name << endl;
	cout << "Left arm:  " << JP.leftArm.shoulderPitch << " " << JP.leftArm.shoulderRoll << " " << JP.leftArm.elbowPitch << endl;
	cout << "Right arm: " << JP.rightArm.shoulderPitch << " " << JP.rightArm.shoulderRoll << " " << JP.rightArm.elbowPitch << endl;
	cout << "Left leg:  " << JP.leftLeg.hipYaw << " " << JP.leftLeg.hipRoll << " " << JP.leftLeg.hipPitch << " " << JP.leftLeg.kneePitch << " " << JP.leftLeg.anklePitch << " " << JP.leftLeg.ankleRoll << endl;
	cout << "Right leg: " << JP.rightLeg.hipYaw << " " << JP.rightLeg.hipRoll << " " << JP.rightLeg.hipPitch << " " << JP.rightLeg.kneePitch << " " << JP.rightLeg.anklePitch << " " << JP.rightLeg.ankleRoll << endl;
	cout << endl;
}

// Print an inverse pose
void printInversePose(const InversePose& IP, const std::string& name = "")
{
	// Print the required inverse pose
	if(!name.empty()) cout << "Inverse pose: " << name << endl;
	cout << "Left arm:  " << "<none>" << endl;
	cout << "Right arm: " << "<none>" << endl;
	cout << "Left leg:  " << "Pos = " << IP.leftLeg.footPos.x() << " " << IP.leftLeg.footPos.y() << " " << IP.leftLeg.footPos.z() << " | Rot = " << IP.leftLeg.footRot.w() << " " << IP.leftLeg.footRot.x() << " " << IP.leftLeg.footRot.y() << " " << IP.leftLeg.footRot.z() << endl;
	cout << "Right leg: " << "Pos = " << IP.rightLeg.footPos.x() << " " << IP.rightLeg.footPos.y() << " " << IP.rightLeg.footPos.z() << " | Rot = " << IP.rightLeg.footRot.w() << " " << IP.rightLeg.footRot.x() << " " << IP.rightLeg.footRot.y() << " " << IP.rightLeg.footRot.z() << endl;
	cout << endl;
}

// Print an abstract pose
void printAbstractPose(const AbstractPose& AP, const std::string& name = "")
{
	// Print the required abstract pose
	if(!name.empty()) cout << "Abstract pose: " << name << endl;
	cout << "Left arm:  " << AP.leftArm.extension << " " << AP.leftArm.angleX << " " << AP.leftArm.angleY << endl;
	cout << "Right arm: " << AP.rightArm.extension << " " << AP.rightArm.angleX << " " << AP.rightArm.angleY << endl;
	cout << "Left leg:  " << AP.leftLeg.extension << " " << AP.leftLeg.angleX << " " << AP.leftLeg.angleY << " " << AP.leftLeg.angleZ << " " << AP.leftLeg.footAngleX << " " << AP.leftLeg.footAngleY << endl;
	cout << "Right leg:  " << AP.rightLeg.extension << " " << AP.rightLeg.angleX << " " << AP.rightLeg.angleY << " " << AP.rightLeg.angleZ << " " << AP.rightLeg.footAngleX << " " << AP.rightLeg.footAngleY << endl;
	cout << endl;
}

// Test: Pose conversions
void foo() // TEST(GaitPoseTest, test_pose_conversions) // TODO: This testing must be made much better
{
	// Print header
	cout << "Running test: Pose conversions" << endl << endl;

	// Poses
	double randJointPoseA[NUM_JOINTS] = {
		0.1, 0.2, -0.3, // Left arm
		-0.1, -0.2, -0.2, // Right arm
		0.05, 0.08, 0.12, 0.03, -0.1, -0.2, // Left leg
		-0.1, -0.05, -0.07, 0.03, 0.15, 0.2 // Right leg
	};
	double randJointPoseB[NUM_JOINTS] = {
		0.2, -0.1, -0.15, // Left arm
		-0.14, 0.05, -0.18, // Right arm
		0.15, -0.13, 0.07, 0.09, 0.23, -0.16, // Left leg
		-0.14, 0.04, -0.12, 0.07, 0.13, -0.11 // Right leg
	};
	double randJointPoseC[NUM_JOINTS] = {
		0.15, -0.3, -0.24, // Left arm
		-0.18, 0.25, -0.08, // Right arm
		0.08, -0.07, -0.13, 0.01, 0.17, 0.16, // Left leg
		-0.22, -0.04, 0.17, 0.06, 0.11, -0.12 // Right leg
	};

	// Joint representations
	JointPose JP, JP2;
	AbstractPose AP;
	InversePose IP;

	// Conversion joint to abstract and back
	cout << "Conversion joint --> abstract --> joint" << endl;
	cout << "---------------------------------------" << endl;
	JP.readJointPosArray(randJointPoseA);
	printJointPose(JP, "Input");
	AP.setFromJointPose(JP);
	printAbstractPose(AP, "Output");
	JP2.setFromAbstractPose(AP);
	printJointPose(JP2, "Check against input");

	// Conversion joint to abstract and back
	cout << "Conversion joint --> abstract --> joint" << endl;
	cout << "---------------------------------------" << endl;
	JP.readJointPosArray(randJointPoseB);
	printJointPose(JP, "Input");
	AP.setFromJointPose(JP);
	printAbstractPose(AP, "Output");
	JP2.setFromAbstractPose(AP);
	printJointPose(JP2, "Check against input");

	// Conversion joint to abstract and back
	cout << "Conversion joint --> abstract --> joint" << endl;
	cout << "---------------------------------------" << endl;
	JP.readJointPosArray(randJointPoseC);
	printJointPose(JP, "Input");
	AP.setFromJointPose(JP);
	printAbstractPose(AP, "Output");
	JP2.setFromAbstractPose(AP);
	printJointPose(JP2, "Check against input");

	// Conversion joint to inverse and back
	cout << "Conversion joint --> inverse --> joint" << endl;
	cout << "--------------------------------------" << endl;
	JP.readJointPosArray(randJointPoseA);
	printJointPose(JP, "Input");
	IP.setLegsFromJointPose(JP.leftLeg, JP.rightLeg);
	printInversePose(IP, "Output");
	JP2.setLegsFromInversePose(IP.leftLeg, IP.rightLeg);
	JP2.leftArm = JP.leftArm;
	JP2.rightArm = JP.rightArm;
	printJointPose(JP2, "Check against input");

	// Conversion joint to inverse and back
	cout << "Conversion joint --> inverse --> joint" << endl;
	cout << "--------------------------------------" << endl;
	JP.readJointPosArray(randJointPoseB);
	printJointPose(JP, "Input");
	IP.setLegsFromJointPose(JP.leftLeg, JP.rightLeg);
	printInversePose(IP, "Output");
	JP2.setLegsFromInversePose(IP.leftLeg, IP.rightLeg);
	JP2.leftArm = JP.leftArm;
	JP2.rightArm = JP.rightArm;
	printJointPose(JP2, "Check against input");

	// Conversion joint to inverse and back
	cout << "Conversion joint --> inverse --> joint" << endl;
	cout << "--------------------------------------" << endl;
	JP.readJointPosArray(randJointPoseC);
	printJointPose(JP, "Input");
	IP.setLegsFromJointPose(JP.leftLeg, JP.rightLeg);
	printInversePose(IP, "Output");
	JP2.setLegsFromInversePose(IP.leftLeg, IP.rightLeg);
	JP2.leftArm = JP.leftArm;
	JP2.rightArm = JP.rightArm;
	printJointPose(JP2, "Check against input");
}

//
// Main function
//
int main(int argc, char **argv)
{
	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF