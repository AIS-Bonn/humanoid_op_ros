// RobotModel.h
//
// The RobotModel is an important object in the framework that describes a humanoid
// robot model and offers interfaces to manipulate and query the pose and meta
// information of the robot. For kinematics, three levels of abstraction are
// supported. A low level interface based on joint angles (setPose()) allows to
// manipulate the kinematic chain directly with joint angles. A medium level,
// abstract kinematic interface allows manipulation on a more abstract level using
// leg extension, leg angle and foot angle as parameters and calculating the
// coordination of single joints internally. The highest level consists of an inverse
// kinematic interface that can be used to provide a pose based on end effector
// positions. The kinematic interface methods maintain consistency of all three levels
// at all times and produce an Action object that contains all kinematic related
// information.
//
// Aside from the kinematic interface, a set of methods supports feature extraction and
// visualization. These are based on a model constructed from Frame objects of the
// QGLViewer library. To complement the stateless kinematic interface, a "walk" method
// is provided, that is supposed to be used in combination with continuous motion
// trajectories in joint angle space. The walk interface does not generate a walk.
// It takes a walk as a parameter and maintains the pose of the robot model as well as
// meta information, such as the current support leg and a fixed support foot location
// on the floor.
//
// The main functions of the robot model are
// - description of the kinematic structure and masses
// - loading a calibration file that defines signs, offsets and limits for the joints.
// - a "walk" interface for continuous motion trajectories including trunk attitude information.
//   that also takes care of support foot tracking
// - a stateless interface to set the model in a pose using joint targets, the abstract kinematic
//   interface and/or the inverse kinematic interface
// - meta information interface for CoM location, support sole angle, step vector and such.
// - openGL visualization code in the draw() method.

// Ensure header is only included once
#ifndef ROBOTMODEL_H_
#define ROBOTMODEL_H_

// Includes
#include <QString>
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

	// Reset function
	void reset(bool resetPose = true);
	
	// Configuration variables
	const cap_gait::CapConfig* getConfig() const { return config; }
	
	// Robot odometry
	void resetOdom(); // Resets the robot model to a 0 position in the world frame (initial footstep is zero)
	void setOdom(double comX, double comY, double fYaw); // Set the CoM odometry to a particular position and fused yaw

	// Simulates a walk. It assumes to be fed with continuous position and fused angle data.
	// It sets the model into the given pose, it determines the current support foot and it
	// rotates the model around the support foot such that the trunk angle matches the fused
	// angle. It also sets the support exchange boolean flag. The stepVector() makes most
	// sense when this method is used and the support exchange indicates true.
	void update(const Pose& pose, Vec fusedAngle);

	// Sets the support leg sign and updates the footStep frame.
	void setSupportLeg(int supportLegSign);

	// RobotModel information
	Vec stepVector() const;    // Returns the vector pointing from the footstep frame to the "other" foot in footStep coordinates (x,y), and the difference in fused yaw from the footStep frame to the "other" foot (z)
	Vec supportVector() const; // Returns the vector pointing from the footstep frame to the com in footStep coordinates
	Vec swingVector() const;   // Returns the vector pointing from the CoM to the current swing foot floor point frame in footStep coordinates

	// Static conversion functions
	static double fusedYaw(const Quaternion& q); // Returns the fused yaw of a quaternion orientation in the range (-pi,pi]
	static Quaternion quatFromFusedPR(double fusedX, double fusedY); // Converts a pure fused pitch/roll rotation to a quaternion
	
protected:
	// Initialisation of kinematic model
	void initKinematicModel();
	void initKinematicHierarchy();
	void initKinematicTranslations();
	void initKinematicRotations();

	// Set robot pose functions
	void setPose(const Pose& pose) { this->pose = pose; applyPose(); } // Set the kinematic pose of the modelled robot
	void applyPose(); // Applies the currently set joint angles to the kinematic model

	// Rotates the whole model to the given fused pitch and roll, and adds yaw so that the support foot
	// is aligned with the footStep frame in terms of fused yaw. The model is then translated so that
	// the positions of the support foot and footStep frames coincide.
	void alignModel(Vec fusedAngle);
	
	// Configuration variables
	cap_gait::CapConfig* config;
	void robotSpecCallback() { initKinematicTranslations(); }

	// Robot pose
	Pose pose;

public:
	// Root frames
	Frame base;     // Global frame placed at the centre between the two hip joints (i.e. at the assumed CoM)
	Frame footStep; // Global frame placed at the current estimated support foot location (the orientation of footStep is taken to be just the fused yaw component of the corresponding FootFloorPoint frame)
	Frame freeFoot; // TODO: Global frame placed at the current estimated free foot location (the orientation of freeFoot is taken to be just the fused yaw component of the corresponding FootFloorPoint frame)
	
	// Head frames
	Frame neck;
	Frame head;
	
	// Left arm frames
	Frame lShoulder;
	Frame lElbow;
	Frame lHand;
	
	// Right arm frames
	Frame rShoulder;
	Frame rElbow;
	Frame rHand;
	
	// Left leg frames
	Frame lHip;
	Frame lKnee;
	Frame lAnkle;
	Frame lFootFloorPoint;
	
	// Right leg frames
	Frame rHip;
	Frame rKnee;
	Frame rAnkle;
	Frame rFootFloorPoint;

	// Support leg information
	int  supportLegSign;
	bool supportExchange;
	bool supportExchangeLock;
};

}

#endif
// EOF