// Common definitions for the generic gait motion module and underlying gait engines
// File: gait_common.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_COMMON_H
#define GAIT_COMMON_H

// Includes
#include <string>

// Gait namespace
namespace gait
{
	//! ID list of joints used by the gait motion module
	enum JointID
	{
		ARMS_BEGIN,
		LEFT_ARM_BEGIN = ARMS_BEGIN,
		L_SHOULDER_PITCH = LEFT_ARM_BEGIN,
		L_SHOULDER_ROLL,
		L_ELBOW_PITCH,
		LEFT_ARM_END = L_ELBOW_PITCH,

		RIGHT_ARM_BEGIN,
		R_SHOULDER_PITCH = RIGHT_ARM_BEGIN,
		R_SHOULDER_ROLL,
		R_ELBOW_PITCH,
		RIGHT_ARM_END = R_ELBOW_PITCH,
		ARMS_END = RIGHT_ARM_END,

		LEGS_BEGIN,
		LEFT_LEG_BEGIN = LEGS_BEGIN,
		L_HIP_YAW = LEFT_LEG_BEGIN,
		L_HIP_ROLL,
		L_HIP_PITCH,
		L_KNEE_PITCH,
		L_ANKLE_PITCH,
		L_ANKLE_ROLL,
		LEFT_LEG_END = L_ANKLE_ROLL,

		RIGHT_LEG_BEGIN,
		R_HIP_YAW = RIGHT_LEG_BEGIN,
		R_HIP_ROLL,
		R_HIP_PITCH,
		R_KNEE_PITCH,
		R_ANKLE_PITCH,
		R_ANKLE_ROLL,
		RIGHT_LEG_END = R_ANKLE_ROLL,
		LEGS_END = RIGHT_LEG_END,

		NUM_JOINTS
	};

	//! List of names of the joints that are required by the gait motion module (indexed by the `JointID` enum)
	const std::string jointName[NUM_JOINTS] = {
		"left_shoulder_pitch",  // <-- LEFT_ARM_BEGIN
		"left_shoulder_roll",   //          ...
		"left_elbow_pitch",     // <-- LEFT_ARM_END
		
		"right_shoulder_pitch", // <-- RIGHT_ARM_BEGIN
		"right_shoulder_roll",  //          ...
		"right_elbow_pitch",    // <-- RIGHT_ARM_END
		
		"left_hip_yaw",         // <-- LEFT_LEG_BEGIN
		"left_hip_roll",        //          ...
		"left_hip_pitch",       //          ...
		"left_knee_pitch",      //          ...
		"left_ankle_pitch",     //          ...
		"left_ankle_roll",      // <-- LEFT_LEG_END
		
		"right_hip_yaw",        // <-- RIGHT_LEG_BEGIN
		"right_hip_roll",       //          ...
		"right_hip_pitch",      //          ...
		"right_knee_pitch",     //          ...
		"right_ankle_pitch",    //          ...
		"right_ankle_roll"      // <-- RIGHT_LEG_END
	};

	//! Name of the tf frame that is used for publishing the gait odometry
	const std::string gaitOdomFrame = "/odom_gait";

	//! Enumeration of motion IDs that can be used in the GaitCommand::motion field to trigger motions through the gait
	enum MotionID
	{
		MID_NONE,
		MID_KICK_LEFT,
		MID_KICK_RIGHT,
		MID_COUNT
	};

	//! List of names corresponding to the gait motion IDs
	static const std::string motionName[MID_COUNT] = {
		"",
		"kick_left",
		"kick_right"
	};
	
	//! Enumeration of motion stances that can be commanded to a gait engine
	enum MotionStance
	{
		STANCE_DEFAULT,
		STANCE_KICK,
		STANCE_COUNT
	};
}

#endif
// EOF