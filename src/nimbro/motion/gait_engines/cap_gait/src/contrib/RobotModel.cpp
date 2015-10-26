// Encapsulates the model of a robot for gait purposes.
// File: RobotModel.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

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

// Reset function (if resetModel is true this resets everything, otherwise it just resets the odometry)
void RobotModel::reset(bool resetModel)
{
	// Reset the entire kinematic model if required
	if(resetModel)
		initKinematicModel();

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
	// Initialise the support conditions
	supportLegSign = RIGHT_LEG;
	supportExchange = false;
	supportExchangeLock = false;

	// Initialise the base frame
	base.setReferenceFrame(NULL);
	base.setTranslation(0, 0, 0); // Updated further down in this function
	base.setRotation(0, 0, 0, 1);

	// Initialise the left and right footstep frames
	leftFootstep.setReferenceFrame(NULL);
	leftFootstep.setTranslation(0, 0, 0); // Updated further down in this function
	leftFootstep.setRotation(0, 0, 0, 1); // Updated further down in this function
	rightFootstep.setReferenceFrame(NULL);
	rightFootstep.setTranslation(0, 0, 0); // Updated further down in this function
	rightFootstep.setRotation(0, 0, 0, 1); // Updated further down in this function

	// Initialise the support and free footstep frames
	suppFootstep.setTranslation(0, 0, 0);
	suppFootstep.setRotation(0, 0, 0, 1);
	freeFootstep.setTranslation(0, 0, 0);
	freeFootstep.setRotation(0, 0, 0, 1);
	updateSuppFreeFootstep(); // Sets the reference frames of the support and free footstep frames

	// Initialise the kinematic chain hierarchy (reference frames)
	initKinematicHierarchy();

	// Initialise the kinematic chain translations (translations)
	initKinematicTranslations();

	// Initialise the kinematic chain rotations (rotations)
	initKinematicRotations();

	// Initialise the kinematic model to the default pose (rotations)
	setPose(Pose());

	// Set the translation of the base frame so that the CoM is at (0,0), and the support foot floor point is on the floor (z = 0)
	Vec comPos = com.position();
	Vec suppFootFloorPointPos = (supportLegSign == RIGHT_LEG ? rFootFloorPoint.position() : lFootFloorPoint.position());
	base.setTranslation(-comPos.x, -comPos.y, -suppFootFloorPointPos.z);

	// Update the left and right footstep frames
	updateLeftRightFootstep();
}

// Initialise the hierarchy of frames in the kinematic chain
void RobotModel::initKinematicHierarchy()
{
	// Trunk
	com.setReferenceFrame(&base);
	trunkLink.setReferenceFrame(&base);
	
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
	// Trunk
	com.setTranslation(config->comOffsetX(), 0, config->comOffsetZ());
	trunkLink.setTranslation(config->trunkLinkOffsetX(), config->trunkLinkOffsetY(), config->trunkLinkOffsetZ());
	
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
	// Trunk
	com.setRotation(0, 0, 0, 1);
	trunkLink.setRotation(0, 0, 0, 1);
	
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
// RobotModel update functions
//

// Update the left and right footstep frames
void RobotModel::updateLeftRightFootstep()
{
	// Set the translations of the left and right footstep frames
	leftFootstep.setTranslation(lFootFloorPoint.position());
	leftFootstep.setRotation(Quaternion(Vec(0,0,1), fusedYaw(lFootFloorPoint.orientation())));
	rightFootstep.setTranslation(rFootFloorPoint.position());
	rightFootstep.setRotation(Quaternion(Vec(0,0,1), fusedYaw(rFootFloorPoint.orientation())));
}

// Update the support and free footstep frames
void RobotModel::updateSuppFreeFootstep()
{
	// Update the reference frames of the support and free footstep frames
	if(supportLegSign == RIGHT_LEG)
	{
		suppFootstep.setReferenceFrame(&rightFootstep);
		freeFootstep.setReferenceFrame(&leftFootstep);
	}
	else
	{
		suppFootstep.setReferenceFrame(&leftFootstep);
		freeFootstep.setReferenceFrame(&rightFootstep);
	}
}

//
// RobotModel odometry
//

// Reset the internal CoM odometry so that the CoM frame is at (0,0) with a fused yaw of zero
void RobotModel::resetOdom()
{
	// Set the odometry to zero
	setOdom(0.0, 0.0, 0.0);
}

// Sets the internal CoM odometry to a particular position and fused yaw heading
void RobotModel::setOdom(double comX, double comY, double fYaw)
{
	// Retrieve the current CoM frame position and orientation
	Vec comPos = com.position();
	Quaternion comOrient = com.orientation();

	// Retrieve the translations of the global frames (same as their global positions by definition)
	Vec baseTrans = base.translation();
	Vec leftFootstepTrans = leftFootstep.translation();
	Vec rightFootstepTrans = rightFootstep.translation();

	// Work out the pure yaw quaternion rotation required so that the fused yaw of the CoM frame matches fYaw
	Quaternion rot(Vec(0,0,1), fYaw - fusedYaw(comOrient));

	// Rotate all the global frames by the required yaw rotation
	base.setRotation(rot * base.rotation());
	leftFootstep.setRotation(rot * leftFootstep.rotation());
	rightFootstep.setRotation(rot * rightFootstep.rotation());

	// Update the translations of all the global frames
	Vec baseV = rot * (baseTrans - comPos);
	Vec leftFootstepV = rot * (leftFootstepTrans - comPos);
	Vec rightFootstepV = rot * (rightFootstepTrans - comPos);
	base.setTranslation(comX + baseV.x, comY + baseV.y, baseTrans.z);
	leftFootstep.setTranslation(comX + leftFootstepV.x, comY + leftFootstepV.y, leftFootstepTrans.z);
	rightFootstep.setTranslation(comX + rightFootstepV.x, comY + rightFootstepV.y, rightFootstepTrans.z);
}

//
// RobotModel pose update functions
//

// Update the robot model with a new pose and fused roll/pitch
void RobotModel::update(const Pose& pose, double fusedX, double fusedY)
{
	// Update the robot model with the new data
	setPose(pose);              // Applies the measured joint angles to the kinematic chain
	alignModel(fusedX, fusedY); // Updates the base transform based on the current support foot location and the measured fused angles

	// Calculate the lower of the two feet
	double footHeightDiff = rFootFloorPoint.position().z - lFootFloorPoint.position().z; // Positive if the right foot is higher than the left
	int lowestFoot = (footHeightDiff > 0 ? LEFT_LEG : RIGHT_LEG);

	// Allow a support exchange if the vertical foot separation has exceeded a configured threshold
	if(supportExchangeLock && (fabs(footHeightDiff) >= config->footHeightHysteresis()))
		supportExchangeLock = false;

	// Perform a support exchange if needed
	supportExchange = false;
	if(lowestFoot != supportLegSign && !supportExchangeLock)
	{
		supportExchangeLock = true;
		supportExchange = true;
		setSupportLeg(lowestFoot);
	}
}

// Apply the stored pose (joint angles) to the kinematic model
void RobotModel::applyPose()
{
	// The assumed orders of rotation of the robot joints are as follows:
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

// Align the model to the given fused roll/pitch, add yaw to ensure the fused yaw of the support foot hasn't changed, and translate the model to ensure that the support foot floor point is preserved
void RobotModel::alignModel(double fusedX, double fusedY)
{
	// Get pointers to the required frames
	Frame *suppFloorPoint, *otherFloorPoint, *supportFootstep, *otherFootstep; // It is assumed that the footstep frames are global
	if(supportLegSign == RIGHT_LEG)
	{
		suppFloorPoint = &rFootFloorPoint;
		otherFloorPoint = &lFootFloorPoint;
		supportFootstep = &rightFootstep;
		otherFootstep = &leftFootstep;
	}
	else
	{
		suppFloorPoint = &lFootFloorPoint;
		otherFloorPoint = &rFootFloorPoint;
		supportFootstep = &leftFootstep;
		otherFootstep = &rightFootstep;
	}

	// Apply the given fused roll/pitch to the model (temporarily annuls the fused yaw of the robot)
	base.setRotation(quatFromFusedPR(fusedX, fusedY));

	// Calculate the difference in fused yaw (CCW) from the support foot floor point frame to the current support footstep frame, and yaw the base frame by this value
	double yawDelta = fusedYaw(supportFootstep->rotation()) - fusedYaw(suppFloorPoint->orientation());
	base.setRotation(Quaternion(0.0, 0.0, sin(0.5*yawDelta), cos(0.5*yawDelta)) * base.rotation());

	// Translate the model (base frame) so that the support foot floor point and support footstep frames coincide in terms of position
	base.translate(supportFootstep->translation() - suppFloorPoint->position());

	// Update the free footstep frame (the support footstep frame by construction has not changed)
	otherFootstep->setTranslation(otherFloorPoint->position());
	otherFootstep->setRotation(Quaternion(Vec(0,0,1), fusedYaw(otherFloorPoint->orientation())));
}

// Sets the support leg sign of the model and updates the required associated frames
void RobotModel::setSupportLeg(int sls)
{
	// Update the support leg sign
	supportLegSign = sls;
	updateSuppFreeFootstep();

	// Retrieve the position of the new support foot
	Vec footPos = (supportLegSign == RIGHT_LEG ? rFootFloorPoint.position() : lFootFloorPoint.position());

	// Shift the model vertically so that the support foot is exactly on the ground
	base.translate(0.0, 0.0, -footPos.z);
	leftFootstep.translate(0.0, 0.0, -footPos.z);
	rightFootstep.translate(0.0, 0.0, -footPos.z);
}

//
// RobotModel information
//

// Support footstep CoM vector
Vec RobotModel::suppComVector() const
{
	// Return the required vector
	if(supportLegSign == RIGHT_LEG)
		return rightFootstep.coordinatesOf(com.position());
	else
		return leftFootstep.coordinatesOf(com.position());
}

// Free footstep CoM vector
Vec RobotModel::freeComVector() const
{
	// Return the required vector
	if(supportLegSign == RIGHT_LEG)
		return leftFootstep.coordinatesOf(com.position());
	else
		return rightFootstep.coordinatesOf(com.position());
}

// Left footstep CoM vector
Vec RobotModel::leftComVector() const
{
	// Return the required vector
	return leftFootstep.coordinatesOf(com.position());
}

// Right footstep CoM vector
Vec RobotModel::rightComVector() const
{
	// Return the required vector
	return rightFootstep.coordinatesOf(com.position());
}

// Support footstep step vector
Vec RobotModel::suppStepVector() const
{
	// Return the required vector
	if(supportLegSign == RIGHT_LEG)
		return rightFootstep.coordinatesOf(leftFootstep.translation());
	else
		return leftFootstep.coordinatesOf(rightFootstep.translation());
}

// Free footstep step vector
Vec RobotModel::freeStepVector() const
{
	// Return the required vector
	if(supportLegSign == RIGHT_LEG)
		return leftFootstep.coordinatesOf(rightFootstep.translation());
	else
		return rightFootstep.coordinatesOf(leftFootstep.translation());
}

// Left footstep step vector
Vec RobotModel::leftStepVector() const
{
	// Return the required vector
	return leftFootstep.coordinatesOf(rightFootstep.translation());
}

// Right footstep step vector
Vec RobotModel::rightStepVector() const
{
	// Return the required vector
	return rightFootstep.coordinatesOf(leftFootstep.translation());
}

// Support footstep swing vector
Vec RobotModel::suppSwingVector() const
{
	// Return the required vector
	if(supportLegSign == RIGHT_LEG)
		return rightFootstep.coordinatesOf(leftFootstep.translation()) - rightFootstep.coordinatesOf(com.position());
	else
		return leftFootstep.coordinatesOf(rightFootstep.translation()) - leftFootstep.coordinatesOf(com.position());
}

// Free footstep swing vector
Vec RobotModel::freeSwingVector() const
{
	// Return the required vector
	if(supportLegSign == RIGHT_LEG)
		return leftFootstep.coordinatesOf(rightFootstep.translation()) - leftFootstep.coordinatesOf(com.position());
	else
		return rightFootstep.coordinatesOf(leftFootstep.translation()) - rightFootstep.coordinatesOf(com.position());
}

// Left footstep swing vector
Vec RobotModel::leftSwingVector() const
{
	// Return the required vector
	return leftFootstep.coordinatesOf(rightFootstep.translation()) - leftFootstep.coordinatesOf(com.position());
}

// Right footstep swing vector
Vec RobotModel::rightSwingVector() const
{
	// Return the required vector
	return rightFootstep.coordinatesOf(leftFootstep.translation()) - rightFootstep.coordinatesOf(com.position());
}

// Support footstep step yaw
double RobotModel::suppStepYaw() const
{
	// Return the required yaw
	if(supportLegSign == RIGHT_LEG)
		return getStepYaw(rightFootstep, leftFootstep);
	else
		return getStepYaw(leftFootstep, rightFootstep);
}

// Free footstep step yaw
double RobotModel::freeStepYaw() const
{
	// Return the required yaw
	if(supportLegSign == RIGHT_LEG)
		return getStepYaw(leftFootstep, rightFootstep);
	else
		return getStepYaw(rightFootstep, leftFootstep);
}

// Left footstep step yaw
double RobotModel::leftStepYaw() const
{
	// Return the required yaw
	return getStepYaw(leftFootstep, rightFootstep);
}

// Right footstep step yaw
double RobotModel::rightStepYaw() const
{
	// Return the required yaw
	return getStepYaw(rightFootstep, leftFootstep);
}

// Calculate a particular step yaw (the passed frames must be global!)
double RobotModel::getStepYaw(const Frame& fromFootstep, const Frame& toFootstep) const
{
	// Calculate the fused yaw difference between the footstep frames
	double fyaw = fusedYaw(toFootstep.rotation()) - fusedYaw(fromFootstep.rotation()); // Result is in [-2*pi,2*pi]
	if(fyaw >   M_PI) fyaw -= 2.0*M_PI; // Value is now in [-2*pi,pi]
	if(fyaw <= -M_PI) fyaw += 2.0*M_PI; // Value is now in (-pi,pi]

	// Return the required fused yaw
	return fyaw;
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