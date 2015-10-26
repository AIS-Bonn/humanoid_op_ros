// Encapsulates the model of a robot for gait purposes.
// File: RobotModel.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CAP_ROBOTMODEL_H
#define CAP_ROBOTMODEL_H

// Includes
#include <QGLViewer/frame.h>
#include <cap_gait/contrib/Action.h>
#include <cap_gait/cap_gait_config.h>

// Margait contrib namespace
namespace margait_contrib
{

// Namespaces
using namespace qglviewer;

// RobotModel class
class RobotModel
{
public:
	// Constructor
	explicit RobotModel(cap_gait::CapConfig* capConfig);
	
	// Configuration variables
	const cap_gait::CapConfig* getConfig() const { return config; }

	// Reset function
	void reset(bool resetModel = true);
	
	// Robot odometry
	void resetOdom(); // Resets the CoM odometry to a zero position in the world frame
	void setOdom(double comX, double comY, double fYaw); // Set the CoM odometry to a particular position and fused yaw in the world frame

	// Update the robot model with new pose and fused angle data
	void update(const Pose& pose, double fusedX, double fusedY);

	// Sets the support leg sign of the model and updates the required associated frames
	void setSupportLeg(int supportLegSign);

	// Robot model CoM vector (vector from footstep frame to CoM in footstep frame coordinates)
	Vec suppComVector() const;    // Returns the vector pointing from the support footstep frame to the CoM in support footstep frame coordinates
	Vec freeComVector() const;    // Returns the vector pointing from the free footstep frame to the CoM in free footstep frame coordinates
	Vec leftComVector() const;    // Returns the vector pointing from the left footstep frame to the CoM in left footstep frame coordinates
	Vec rightComVector() const;   // Returns the vector pointing from the right footstep frame to the CoM in right footstep frame coordinates

	// Robot model step vector (vector from footstep frame to other footstep frame in footstep frame coordinates)
	Vec suppStepVector() const;   // Returns the vector pointing from the support footstep frame to the free footstep frame in support footstep frame coordinates
	Vec freeStepVector() const;   // Returns the vector pointing from the free footstep frame to the support footstep frame in free footstep frame coordinates
	Vec leftStepVector() const;   // Returns the vector pointing from the left footstep frame to the right footstep frame in left footstep frame coordinates
	Vec rightStepVector() const;  // Returns the vector pointing from the right footstep frame to the left footstep frame in right footstep frame coordinates

	// Robot model swing vector (vector from CoM to other footstep frame in footstep frame coordinates)
	Vec suppSwingVector() const;  // Returns the vector pointing from the CoM to the free footstep frame in support foot coordinates
	Vec freeSwingVector() const;  // Returns the vector pointing from the CoM to the support footstep frame in free foot coordinates
	Vec leftSwingVector() const;  // Returns the vector pointing from the CoM to the right footstep frame in left foot coordinates
	Vec rightSwingVector() const; // Returns the vector pointing from the CoM to the left footstep frame in right foot coordinates

	// Robot model step yaw (difference in fused yaw from footstep frame to other footstep frame in the global coordinate frame)
	double suppStepYaw() const;   // Returns the difference in fused yaw from the support footstep frame to the free footstep frame in the global coordinate frame
	double freeStepYaw() const;   // Returns the difference in fused yaw from the free footstep frame to the support footstep frame in the global coordinate frame
	double leftStepYaw() const;   // Returns the difference in fused yaw from the left footstep frame to the right footstep frame in the global coordinate frame
	double rightStepYaw() const;  // Returns the difference in fused yaw from the right footstep frame to the left footstep frame in the global coordinate frame

	// Static conversion functions
	static double fusedYaw(const Quaternion& q); // Returns the fused yaw of a quaternion orientation in the range (-pi,pi]
	static Quaternion quatFromFusedPR(double fusedX, double fusedY); // Converts a pure fused pitch/roll rotation to a quaternion
	
protected:
	// Initialisation of kinematic model
	void initKinematicModel();        // Initialises the entire kinematic chain
	void initKinematicHierarchy();    // Initialises the reference frame hierarchy of the kinematic chain
	void initKinematicTranslations(); // Initialises the translations of the kinematic chain
	void initKinematicRotations();    // Initialises the rotations of the kinematic chain

	// Update functions
	void updateLeftRightFootstep();   // Updates the left/right footstep frames based on the left/right foot floor point frames
	void updateSuppFreeFootstep();    // Updates the support and free footstep frames as being the left or right footstep frames, based on the current support leg sign

	// Set robot pose functions
	void setPose(const Pose& pose) { this->pose = pose; applyPose(); } // Set the kinematic pose of the modelled robot and apply it
	void applyPose();                                                  // Applies the currently set joint angles to the kinematic model
	void alignModel(double fusedX, double fusedY);                     // Set the fused pitch/roll orientation of the model (fused yaw is part of the odometry)

	// Helper functions
	double getStepYaw(const Frame& fromFootstep, const Frame& toFootstep) const;

	// Configuration variables
	cap_gait::CapConfig* config;
	void robotSpecCallback() { initKinematicTranslations(); }

	// Robot pose (encapsulates the currently set joint angles, applied to the kinematic frames by the applyPose() function)
	Pose pose;

public:
	// Root frame
	Frame base;            // Global frame placed at the centre between the two hip joints (the nominal "base" reference point of the robot)
	
	// Footstep frames
	Frame leftFootstep;    // Global frame placed at the left foot floor point, with only the yaw component of the rotation of the left foot (left footstep frame)
	Frame rightFootstep;   // Global frame placed at the right foot floor point, with only the yaw component of the rotation of the right foot (right footstep frame)
	Frame suppFootstep;    // Frame placed at either the left or right footstep frame depending on the current support leg (support footstep frame)
	Frame freeFootstep;    // Frame placed at either the left or right footstep frame depending on the current free leg (free footstep frame)
	
	// Trunk frames
	Frame com;             // Frame placed at the CoM of the robot
	Frame trunkLink;       // Frame placed at the trunk link of the robot as per the tf (i.e. as defined by the URDF model)
	
	// Head frames
	Frame neck;            // Frame placed at the neck joint, aligned with the head
	Frame head;            // Frame placed at the centre of the head, aligned with the head
	
	// Left arm frames
	Frame lShoulder;       // Frame placed at the left shoulder joint, aligned with the upper arm
	Frame lElbow;          // Frame placed at the left elbow joint, aligned with the lower arm
	Frame lHand;           // Frame placed at the left hand, aligned with the lower arm
	
	// Right arm frames
	Frame rShoulder;       // Frame placed at the right shoulder joint, aligned with the upper arm
	Frame rElbow;          // Frame placed at the right elbow joint, aligned with the lower arm
	Frame rHand;           // Frame placed at the right hand, aligned with the lower arm
	
	// Left leg frames
	Frame lHip;            // Frame placed at the left hip joint, aligned with the upper leg
	Frame lKnee;           // Frame placed at the left knee joint, aligned with the lower leg
	Frame lAnkle;          // Frame placed at the left ankle joint, aligned with the foot
	Frame lFootFloorPoint; // Frame placed at the point on the bottom of the left foot that is assumed to be in contact with the ground when left is the support foot, aligned with the foot
	
	// Right leg frames
	Frame rHip;            // Frame placed at the right hip joint, aligned with the upper leg
	Frame rKnee;           // Frame placed at the right knee joint, aligned with the lower leg
	Frame rAnkle;          // Frame placed at the right ankle joint, aligned with the foot
	Frame rFootFloorPoint; // Frame placed at the point on the bottom of the right foot that is assumed to be in contact with the ground when right is the support foot, aligned with the foot
	
	// TODO: A current shortcoming of the robot model is that x-offsets from the hip midpoint to the shoulder line/arm axis/head axis aren't considered.

	// Support leg information
	enum
	{
		LEFT_LEG = -1,
		RIGHT_LEG = 1
	};
	int  supportLegSign;      // Sign of the current support leg (1 = Right leg, -1 = Left leg)
	bool supportExchange;     // Flag whether a support exchange happened during a call to update()
	bool supportExchangeLock; // Flag whether a support exchange is currently disallowed (used to enforce a hysteresis)
};

}

#endif
// EOF