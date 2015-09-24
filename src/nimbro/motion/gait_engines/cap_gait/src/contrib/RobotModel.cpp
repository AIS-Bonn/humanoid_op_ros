// RobotModel.cpp
// Encapsulates the model of a robot for gait purposes.

// Includes
#include <cap_gait/contrib/RobotModel.h>
#include <cap_gait/contrib/Globals.h>
#include <cap_gait/contrib/Vec2f.h>
#include <cap_gait/contrib/Vec3f.h>
#include <cmath>

// Namespaces
using namespace margait_contrib;
using namespace qglviewer;

//
// RobotModel class
//

// Default constructor
RobotModel::RobotModel(cap_gait::CapConfig* capConfig) : config(capConfig)
{
	// Set up the robot spec callback
	config->addRobotSpecCallback(boost::bind(&RobotModel::robotSpecCallback, this));
	
	// Reset the robot model object
	reset(true);
}

// Reset function
void RobotModel::reset(bool resetPose)
{
	// Reset the pose if required
	if(resetPose)
	{
		// Initialise the support conditions
		supportLegSign = 1;
		supportExchange = false;
		supportExchangeLock = true;

		// Intialise the kinematic model
		initKinematicModel();
		
		// Initialise to the default pose
		setPose(Pose());
	}
	
	// Update the robot specifications
	robotSpecCallback();

	// Reset the odometry
	resetOdom();
}

//
// RobotModel initialisation
//

// Completely initialise the kinematic chain
void RobotModel::initKinematicModel()
{
	// Initialise the base frame
	base.setReferenceFrame(NULL);
	base.setTranslation(0, 0.5*config->hipWidth(), 2.0*config->legLinkLength() + config->footOffsetZ());
	base.setRotation(0, 0, 0, 1);
	
	// Initialise the footstep frame
	footStep.setReferenceFrame(NULL);
	footStep.setTranslation(0, 0, 0);
	footStep.setRotation(0, 0, 0, 1);
	
	// Initialise the kinematic chain hierarchy
	initKinematicHierarchy();

	// Initialise the kinematic chain translations
	initKinematicTranslations();

	// Initialise the kinematic chain rotations
	initKinematicRotations();
}

// Initialise the hierarchy of frames in the kinematic chain
void RobotModel::initKinematicHierarchy()
{
	// Head
	neck.setReferenceFrame(&base);
	head.setReferenceFrame(&neck);
	
	// Left arm
	lShoulder.setReferenceFrame(&base);
	lElbow.setReferenceFrame(&lShoulder);
	lHand.setReferenceFrame(&lElbow);
	
	// Right arm
	rShoulder.setReferenceFrame(&base);
	rElbow.setReferenceFrame(&rShoulder);
	rHand.setReferenceFrame(&rElbow);
	
	// Left leg
	lHip.setReferenceFrame(&base);
	lKnee.setReferenceFrame(&lHip);
	lAnkle.setReferenceFrame(&lKnee);
	lFootFloorPoint.setReferenceFrame(&lAnkle);
	
	// Right leg
	rHip.setReferenceFrame(&base);
	rKnee.setReferenceFrame(&rHip);
	rAnkle.setReferenceFrame(&rKnee);
	rFootFloorPoint.setReferenceFrame(&rAnkle);
}

// Initialise the (fixed) translations of the frames in the kinematic chain
void RobotModel::initKinematicTranslations()
{
	// Head
	neck.setTranslation(0, 0, config->trunkHeight() + config->neckHeight());
	head.setTranslation(config->headOffsetX(), 0, config->headOffsetZ());
	
	// Left arm
	lShoulder.setTranslation(0, 0.5*config->shoulderWidth(), config->trunkHeight());
	lElbow.setTranslation(0, 0, -config->armLinkLength());
	lHand.setTranslation(0, 0, -config->armLinkLength());
	
	// Right arm
	rShoulder.setTranslation(0, -0.5*config->shoulderWidth(), config->trunkHeight());
	rElbow.setTranslation(0, 0, -config->armLinkLength());
	rHand.setTranslation(0, 0, -config->armLinkLength());
	
	// Left leg
	lHip.setTranslation(0, 0.5*config->hipWidth(), 0);
	lKnee.setTranslation(0, 0, -config->legLinkLength());
	lAnkle.setTranslation(0, 0, -config->legLinkLength());
	lFootFloorPoint.setTranslation(0, 0, -config->footOffsetZ());
	
	// Right leg
	rHip.setTranslation(0, -0.5*config->hipWidth(), 0);
	rKnee.setTranslation(0, 0, -config->legLinkLength());
	rAnkle.setTranslation(0, 0, -config->legLinkLength());
	rFootFloorPoint.setTranslation(0, 0, -config->footOffsetZ());
}

// Initialise the rotations of the frames in the kinematic chain
void RobotModel::initKinematicRotations()
{
	// Head
	neck.setRotation(0, 0, 0, 1);
	head.setRotation(0, 0, 0, 1);
	
	// Left arm
	lShoulder.setRotation(0, 0, 0, 1);
	lElbow.setRotation(0, 0, 0, 1);
	lHand.setRotation(0, 0, 0, 1);
	
	// Right arm
	rShoulder.setRotation(0, 0, 0, 1);
	rElbow.setRotation(0, 0, 0, 1);
	rHand.setRotation(0, 0, 0, 1);
	
	// Left leg
	lHip.setRotation(0, 0, 0, 1);
	lKnee.setRotation(0, 0, 0, 1);
	lAnkle.setRotation(0, 0, 0, 1);
	lFootFloorPoint.setRotation(0, 0, 0, 1);
	
	// Right leg
	rHip.setRotation(0, 0, 0, 1);
	rKnee.setRotation(0, 0, 0, 1);
	rAnkle.setRotation(0, 0, 0, 1);
	rFootFloorPoint.setRotation(0, 0, 0, 1);
}

//
// RobotModel odometry
//

// Reset the internal RobotModel odometry so that the footStep frame coincides with the global frame (default output of initKinematicModel())
void RobotModel::resetOdom()
{
	// Update the base and footStep transforms, moving the footStep frame to the origin with identity orientation
	base.setTranslation(supportVector());
	base.setRotation(footStep.rotation().inverse() * base.rotation());
	footStep.setTranslation(0, 0, 0);
	footStep.setRotation(0, 0, 0, 1);
}

// Sets the internal CoM odometry to a particular position and heading
void RobotModel::setOdom(double comX, double comY, double fYaw) // Note: fYaw is interpreted as the fused yaw of the robot
{
	// Retrieve the current base and footStep translations and rotations
	Vec basePos = base.translation();
	Vec footStepPos = footStep.translation();
	Quaternion baseRot = base.rotation();
	Quaternion footStepRot = footStep.rotation();

	// Work out the yaw rotation required so that the fused yaw of the base frame matches fYaw
	Quaternion rot(Vec(0,0,1), fYaw - fusedYaw(baseRot));

	// Update the footStep frame
	Vec v = rot * (footStepPos - basePos);
	footStep.setTranslation(comX + v.x, comY + v.y, footStepPos.z);
	footStep.setRotation(rot * footStepRot);

	// Update the base frame
	base.setTranslation(comX, comY, basePos.z);
	base.setRotation(rot * baseRot);
}

//
// RobotModel updates
//

// Simulates a walk. It assumes to be fed with continuous joint angle and fused angle data.
// It sets the model into the given pose and rotates the model such that the trunk angle
// matches the fused angle. While applying the motion sequence, a fixed location of the
// support foot on the floor is maintained so that the CoM trajectory can be used as motion
// model for the particle filter. The sign of the support foot is flipped when the
// swing foot z coordinate is less than the current support foot z coordinate. It also sets
// the support exchange boolean flag. The stepVector() makes most sense when this method is
// used and the support exchange indicates true.
void RobotModel::update(const Pose& pose, Vec fusedAngle)
{
	// Update the robot model with the given sensor data
	setPose(pose); // Applies the supplied joint angles to the kinematic hierarchy (i.e. this updates the coordinate frames from neck/shoulder/hip onwards)
	alignModel(fusedAngle); // Updates the base transform (global position and orientation of the robot CoM) based on the current support foot location and measured fused angle

	// Perform support exchange if needed.
	// This will set the footStep frame to the location of the new support foot.
	supportExchange = false;
	int lowerFoot = (rFootFloorPoint.position().z < lFootFloorPoint.position().z ? 1 : -1);
	if(lowerFoot != supportLegSign && !supportExchangeLock)
	{
		supportExchangeLock = true;
		supportExchange = true;
		setSupportLeg(lowerFoot);
	}

	// Hysteresis
	if(supportExchangeLock && (qAbs(lFootFloorPoint.position().z - rFootFloorPoint.position().z) > 0.005))
		supportExchangeLock = false;
}

// Rotates the whole model to the given fused pitch and roll, and adds yaw so that the support foot
// is aligned with the footStep frame in terms of fused yaw. The model is then translated so that
// the positions of the support foot and footStep frames coincide.
void RobotModel::alignModel(Vec fusedAngle)
{
	// Get the support foot frame
	Frame& footFloorPoint = (supportLegSign == 1 ? rFootFloorPoint : lFootFloorPoint);

	// Apply the given fused pitch/roll to the model
	base.setRotation(quatFromFusedPR(fusedAngle.x, fusedAngle.y));

	// Calculate the difference in fused yaw (CCW) from the foot floor point frame to the required footStep frame, and yaw the base frame by this value
	double yawDelta = fusedYaw(footStep.orientation()) - fusedYaw(footFloorPoint.orientation());
	base.setRotation(Quaternion(0.0, 0.0, sin(0.5*yawDelta), cos(0.5*yawDelta)) * base.rotation());

	// Translate the model (base frame) so that the foot floor point and footStep frames coincide in terms of position
	base.translate(footStep.position() - footFloorPoint.position());
}

// Sets the support leg sign and updates the footStep frame.
void RobotModel::setSupportLeg(int sls)
{
	// Update the support foot
	supportLegSign = sls;
	Frame& footFloorPoint = (supportLegSign == 1 ? rFootFloorPoint : lFootFloorPoint);
	Vec footPos = footFloorPoint.position();

	// Update the footStep frame to be on the floor exactly under the currently nominated support foot (same x,y,fyaw)
	footStep.setTranslation(footPos.x, footPos.y, 0.0);
	footStep.setRotation(Quaternion(Vec(0,0,1), fusedYaw(footFloorPoint.orientation())));

	// Shift the model vertically so that the support foot is on the ground
	base.translate(0.0, 0.0, -footPos.z);
}

// Applies the joint angles to the robot model. This sets the coordinate frames of the robot into the right
// positions and orientations, so that the model can be visualized and features can be extracted.
void RobotModel::applyPose()
{
	// The order in which rotations are applied to the joints is important.
	// We are following the mechanical servo order of NimbRo robots.
	// Shoulder: y -> x -> z
	// Hip:      z -> x -> y
	// Ankle:    y -> x
	// Neck:     z -> y

	// Head
	neck.setRotation(Quaternion(Vec(0, 0, 1), pose.headPose.neck.z)
	               * Quaternion(Vec(0, 1, 0), pose.headPose.neck.y));
	
	// Left arm
	lShoulder.setRotation(Quaternion(Vec(0, 1, 0), pose.leftArmPose.shoulder.y)
	                    * Quaternion(Vec(1, 0, 0), pose.leftArmPose.shoulder.x)
	                    * Quaternion(Vec(0, 0, 1), pose.leftArmPose.shoulder.z));
	lElbow.setRotation(Quaternion(Vec(0, 1, 0), pose.leftArmPose.elbow.y));
	
	// Right arm
	rShoulder.setRotation(Quaternion(Vec(0, 1, 0), pose.rightArmPose.shoulder.y)
	                    * Quaternion(Vec(1, 0, 0), pose.rightArmPose.shoulder.x)
	                    * Quaternion(Vec(0, 0, 1), pose.rightArmPose.shoulder.z));
	rElbow.setRotation(Quaternion(Vec(0, 1, 0), pose.rightArmPose.elbow.y));
	
	// Left leg
	lHip.setRotation(Quaternion(Vec(0, 0, 1), pose.leftLegPose.hip.z)
	               * Quaternion(Vec(1, 0, 0), pose.leftLegPose.hip.x)
	               * Quaternion(Vec(0, 1, 0), pose.leftLegPose.hip.y));
	lKnee.setRotation(Quaternion(Vec(0, 1, 0), pose.leftLegPose.knee.y));
	lAnkle.setRotation(Quaternion(Vec(0, 1, 0), pose.leftLegPose.ankle.y)
	                 * Quaternion(Vec(1, 0, 0), pose.leftLegPose.ankle.x));
	
	// Right leg
	rHip.setRotation(Quaternion(Vec(0, 0, 1), pose.rightLegPose.hip.z)
	               * Quaternion(Vec(1, 0, 0), pose.rightLegPose.hip.x)
	               * Quaternion(Vec(0, 1, 0), pose.rightLegPose.hip.y));
	rKnee.setRotation(Quaternion(Vec(0, 1, 0), pose.rightLegPose.knee.y));
	rAnkle.setRotation(Quaternion(Vec(0, 1, 0), pose.rightLegPose.ankle.y)
	                 * Quaternion(Vec(1, 0, 0), pose.rightLegPose.ankle.x));
}

//
// RobotModel information
//

// Returns the vector pointing from the footstep frame to the "other" foot in footStep coordinates (x,y), and the difference in fused yaw from the footStep frame to the "other" foot (z)
Vec RobotModel::stepVector() const
{
	// Calculate and return the required vector
	const Frame& otherFootPt = (supportLegSign != 1 ? rFootFloorPoint : lFootFloorPoint);
	Vec stepVec = footStep.coordinatesOf(otherFootPt.position());
	stepVec.z = fusedYaw(otherFootPt.orientation()) - fusedYaw(footStep.orientation()); // Result is in [-2*pi,2*pi]
	if(stepVec.z > M_PI) stepVec.z -= 2.0*M_PI;   // Value is now in [-2*pi,pi]
	if(stepVec.z <= -M_PI) stepVec.z += 2.0*M_PI; // Value is now in (-pi,pi]
	return stepVec;
}

// Returns the vector pointing from the footstep frame to the com in footStep coordinates
Vec RobotModel::supportVector() const
{
	// Calculate and return the required vector
	return footStep.coordinatesOf(base.position());
}

// Returns the vector pointing from the com to the current swing foot floor point frame in footStep coordinates
Vec RobotModel::swingVector() const
{
	// Calculate and return the required vector
	const Frame& otherFootPt = (supportLegSign != 1 ? rFootFloorPoint : lFootFloorPoint);
	return footStep.coordinatesOf(otherFootPt.position()) - footStep.coordinatesOf(base.position());
}

//
// Static functions
//

// Returns the fused yaw of a quaternion orientation in the range (-pi,pi]
double RobotModel::fusedYaw(const Quaternion& q)
{
	// Calculate the fused yaw
	double fyaw = 2.0*atan2(q[2], q[3]); // Output of atan2(z,w) is [-pi,pi], so this expression is in [-2*pi,2*pi]
	if(fyaw > M_PI) fyaw -= 2.0*M_PI;    // Fyaw is now in [-2*pi,pi]
	if(fyaw <= -M_PI) fyaw += 2.0*M_PI;  // Fyaw is now in (-pi,pi]
	return fyaw;
}

// Converts a pure fused pitch/roll rotation to a quaternion
Quaternion RobotModel::quatFromFusedPR(double fusedX, double fusedY) // Note: It is assumed that the missing fusedZ = 0 and hemi = 1
{
	// Precalculate the sin values
	double sth  = sin(fusedY);
	double sphi = sin(fusedX);

	// Calculate the sine sum criterion
	double crit = sth*sth + sphi*sphi;

	// Calculate the tilt angle alpha
	double alpha = (crit >= 1.0 ? M_PI_2 : acos(sqrt(1.0 - crit)));
	double halpha  = 0.5*alpha;
	double chalpha = cos(halpha);
	double shalpha = sin(halpha);

	// Calculate the tilt axis gamma
	double gamma = atan2(sth,sphi);
	double cgamma = cos(gamma);
	double sgamma = sin(gamma);

	// Return the required quaternion orientation (a rotation about (cgamma, sgamma, 0) by angle alpha)
	return Quaternion(cgamma*shalpha, sgamma*shalpha, 0.0, chalpha); // Order: x, y, z, w!
}
// EOF