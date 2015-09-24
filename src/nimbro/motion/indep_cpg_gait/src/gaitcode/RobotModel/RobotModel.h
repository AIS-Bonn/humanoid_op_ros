#ifndef ROBOTMODEL_H_
#define ROBOTMODEL_H_

#include <QGLViewer/frame.h>
using namespace qglviewer;

#include "RobotControl/Action.h"
#include "util/Vec2f.h"
#include "util/StopWatch.h"

namespace indep_cpg_gait
{
	class RobotModel
	{

	public:

		// Model definition parameters.

		double hipWidth; // Distance between the left and right hip joint.
		double footOffsetX; // Forward offset between the foot center and the ankle joint.
		double footWidth; // Width of the foot plate.
		double footLength; // Length of the foot plate.
		double footHeight; // Height of the foot plate.
		double jointRadius; // Radius of every joint. Joints are modeled as spheres.
		double legSegmentLength; // Length of the leg segments (thigh and shank) not including the joint radius.
		double legThickness; // Thickness of the leg hull. Only for collision shape and visuals.
		double hipDiscHeight; // Height of the hip disc for the hip Z rotation.
		double trunkHeight; // The height of the trunk.
		double headRadius; // Radius of the head.
		double armThickness; // Thickness of the arm hull. Only for collision shape and visuals.
		double armSegmentLength; // Length of the arm segments upper arm and lower arm not including the joint radius.

		double armLength;
		double legLength;

		double footMass;
		double legSegmentMass;
		double armSegmentMass;
		double trunkMass;
		double headMass;
		double hipDiscMass;

		Pose signs; // Separate structures for the calib. The Pose itself gets copied a lot, so it would suck if it contained the constant calibs.
		Pose offsets;
		Pose upperLimits;
		Pose lowerLimits;

		QString name;

		bool supportExchange;
		int supportLegSign;



		// The root frame.
		Frame base;

		// The joints of the robot.
		Frame neck;
		Frame lShoulder;
		Frame lElbow;
		Frame rShoulder;
		Frame rElbow;
		Frame lHip;
		Frame lKnee;
		Frame lAnkle;
		Frame rHip;
		Frame rKnee;
		Frame rAnkle;

		// Special body parts.
		Frame rFootFloorPoint;
		Frame lFootFloorPoint;
		Frame lHand;
		Frame rHand;
		Frame imu;
		Frame camera;

		// The fixed footstep aligned with the floor.
		Frame footStep;

		// Kinematic data structures.
		Pose pose;
		AbstractPose abstractPose;
		InversePose inversePose;

	private:

		StopWatch stopWatch;
		double lastSupportChangeTime;

	public:
		RobotModel();
	~RobotModel(){};

		// Resets the robot model to a 0 pose and 0 position in the world frame.
		void reset();

		// Resets the robot model to a 0 position in the world frame.
		void resetPosition();

	// Simulates a walk. It assumes to be fed with continuous position and fused angle data.
		// It sets the model into the given pose, it determines the current support foot and it
		// rotates the model around the support foot such that the trunk angle matches the fused
		// angle. It also sets the support exchange boolean flag. The stepVector() makes most
		// sense when this method is used and the support exchange indicates true.
		void walk(const Pose& pose, Vec2f fusedAngle);

		// Sets the kinematic model in a pose defined by single joint angles.
		void setPose(const Pose& pose);

		// Adds a joint level pose to the current pose.
		void addPose(const Pose& pose);

		// Sets the kinematic model in an abstract pose defined by kinematic interface parameters.
		void setAbstractPose(const AbstractPose& pose);

		// Adds a kinematic pose to the current one.
		void addAbstractPose(const AbstractPose& pose);

		// Sets the kinematic model in an pose using inverse kinematics.
		void setInversePose(const InversePose& pose);

		// Adds pose difference to the current one defined on the inverse kinematics level.
		void addInversePose(const InversePose& pose);

		// Sets the support leg signs and updates the footStep frame.
		void setSupportLeg(int supportLegSign);

		// Aligns the model such that the support foot matches the current frame.
		void alignWithFootStep();

		// Rotates the kinematic model around the current support foot such that the trunk angle matches the given fused angle.
		void applyFusedAngle(Vec2f fusedAngle);

		void alignModel(Vec2f fusedAngle);

		// Returns the trunk rotation angles in the world reference frame.
		Vec trunkAngle();

		// Returns the sole rotation angles in the world reference frame.
		Vec soleAngle();

		// Returns the vector pointing from the support foot to the swing foot in the base coordinate frame.
		Vec stepVector();

		// Returns the vector from the support foot to the base in the base coordinate frame.
		Vec comVector();

		// Returns a complete action struct with pose and abstract pose.
		Action getAction();

		// Draws the kinematic model into the current GL environment.
		virtual void draw(float r=0.5, float g=0.5, float b=0.8, float a=0.6);


		// Kinematic interface. Body interface? Motion interface?
		ArmPose armInterfaceAbstractToJoint(const AbstractArmPose&);
		AbstractArmPose armInterfaceJointToAbstract(const ArmPose&);
		AbstractArmPose armInterfaceInverseToAbstract(const InverseArmPose&);
		InverseArmPose armInterfaceAbstractToInverse(const AbstractArmPose&);
		LegPose legInterfaceAbstractToJoint(const AbstractLegPose&);
		AbstractLegPose legInterfaceJointToAbstract(const LegPose&);
		AbstractLegPose legInterfaceInverseToAbstract(const InverseLegPose&);
		InverseLegPose legInterfaceAbstractToInverse(const AbstractLegPose&);

		// Returns the orthogonal extrinsic rotation angles of the frame with respect to the world coordinate frame.
		// Orthogonal angles are not Euler angles. They are measured with respect to a fixed coordinate frame.
		// The angle around the x-axis is measured by projecting the rotated z-axis (or y) into the plane that has
		// the world x-axis as normal. Then, the angle is calculated between the projected vector and the world
		// z-axis (or y). This is repeated for each axes respectively.
		Vec getRotationAngles(const Frame& frame);

		// Actually applies the joint angles to the model. This is used internally to avoid repetitions.
		void applyPose(const Pose& pose);

	private:

		// Loads the calib file.
		void loadCalib();

		static void drawJoint(double temperature=0);
	};
}
#endif // ROBOTMODEL_H_
