// Feedback gait serial kinematics implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/kinematics/serial/feed_kinematics_impl_serial.h>

// Namespaces
using namespace feed_gait;

//
// FeedKinematics<serial::SerialKinematics> class
//

// Gait halt pose function: Pose vector
template<> void FeedKinematics<serial::SerialKinematics>::getHaltPose(PoseCommand::DblVec& pos) const
{
	// Construct the required joint halt pose
	JointPose JPHalt;
	getHaltPose(JPHalt);

	// Populate the output vector
	JPHalt.toVector(pos);
}

// Gait halt pose function: Pose efforts vector
template<> void FeedKinematics<serial::SerialKinematics>::getHaltPoseEffort(PoseCommand::DblVec& effort) const
{
	// Construct the required joint halt pose efforts
	JointEffort JEHalt;
	getHaltPoseEffort(JEHalt);

	// Populate the output vector
	JEHalt.toVector(effort);
}

// Gait halt pose function: Support coefficients
template<> void FeedKinematics<serial::SerialKinematics>::getHaltPoseSuppCoeff(PoseCommand::SuppCoeff& suppCoeff) const
{
	// Populate the output array
	suppCoeff[hk::LEFT] = 0.5;
	suppCoeff[hk::RIGHT] = 0.5;
}

// Gait halt pose function: JointPose
template<> void FeedKinematics<serial::SerialKinematics>::getHaltPose(JointPose& JP) const
{
	// Calculate the required gait halt pose
	AbsPose AP;
	getHaltPose(AP);
	K.JointFromAbs(AP, JP);
}

// Gait halt pose function: AbsPose
template<> void FeedKinematics<serial::SerialKinematics>::getHaltPose(AbsPose& AP) const
{
	// Calculate the required left leg halt pose
	AP.legL.angleX = konfig.haltLegAngleX();
	AP.legL.angleY = konfig.haltLegAngleY();
	AP.legL.angleZ = konfig.haltLegAngleZ();
	AP.legL.footAngleX = konfig.haltLegFootAngleX();
	AP.legL.footAngleY = konfig.haltLegFootAngleY();
	AP.legL.retraction = konfig.haltLegRetraction();

	// Calculate the required right leg halt pose
	AP.legR.angleX = -konfig.haltLegAngleX();
	AP.legR.angleY = konfig.haltLegAngleY();
	AP.legR.angleZ = -konfig.haltLegAngleZ();
	AP.legR.footAngleX = -konfig.haltLegFootAngleX();
	AP.legR.footAngleY = konfig.haltLegFootAngleY();
	AP.legR.retraction = konfig.haltLegRetraction();

	// Calculate the required left arm halt pose
	AP.armL.angleX = konfig.haltArmAngleX();
	AP.armL.angleY = konfig.haltArmAngleY();
	AP.armL.retraction = konfig.haltArmRetraction();

	// Calculate the required right arm halt pose
	AP.armR.angleX = -konfig.haltArmAngleX();
	AP.armR.angleY = konfig.haltArmAngleY();
	AP.armR.retraction = konfig.haltArmRetraction();

	// Calculate the required head halt pose
	AP.head.angleY = konfig.haltHeadAngleY();
	AP.head.angleZ = konfig.haltHeadAngleZ();
}

// Gait halt pose function: InvPose
template<> void FeedKinematics<serial::SerialKinematics>::getHaltPose(InvPose& IP) const
{
	// Calculate the required gait halt pose
	AbsPose AP;
	getHaltPose(AP);
	K.InvFromAbs(AP, IP);
}

// Gait halt pose function: JointEffort
template<> void FeedKinematics<serial::SerialKinematics>::getHaltPoseEffort(JointEffort& JE) const
{
	// Config aliases
	double effortArm = konfig.haltEffortArm();
	double effortHead = konfig.haltEffortHead();

	// Calculate the required left leg halt pose efforts
	JE.legL.hipYaw = konfig.haltEffortHipYaw();
	JE.legL.hipRoll = konfig.haltEffortHipRoll();
	JE.legL.hipPitch = konfig.haltEffortHipPitch();
	JE.legL.kneePitch = konfig.haltEffortKneePitch();
	JE.legL.anklePitch = konfig.haltEffortAnklePitch();
	JE.legL.ankleRoll = konfig.haltEffortAnkleRoll();

	// Calculate the required right leg halt pose efforts
	JE.legR.hipYaw = konfig.haltEffortHipYaw();
	JE.legR.hipRoll = konfig.haltEffortHipRoll();
	JE.legR.hipPitch = konfig.haltEffortHipPitch();
	JE.legR.kneePitch = konfig.haltEffortKneePitch();
	JE.legR.anklePitch = konfig.haltEffortAnklePitch();
	JE.legR.ankleRoll = konfig.haltEffortAnkleRoll();

	// Calculate the required left arm halt pose efforts
	JE.armL.shoulderPitch = effortArm;
	JE.armL.shoulderRoll = effortArm;
	JE.armL.elbowPitch = effortArm;

	// Calculate the required right arm halt pose efforts
	JE.armR.shoulderPitch = effortArm;
	JE.armR.shoulderRoll = effortArm;
	JE.armR.elbowPitch = effortArm;

	// Calculate the required head halt pose efforts
	JE.head.neckYaw = effortHead;
	JE.head.neckPitch = effortHead;
}

// Safe inverse kinematics function: Inverse --> Joint
template<> bool FeedKinematics<serial::SerialKinematics>::JointFromInvSafe(const InvLegPose& ILP, JointLegPose& JLP) const
{
	// Safely perform the inverse kinematics as required
	AbsLegPose ALP(ILP.limbIndex);
	return JointAbsFromInvSafe(ILP, JLP, ALP);
}

// Safe inverse kinematics function: Inverse --> Abstract
template<> bool FeedKinematics<serial::SerialKinematics>::AbsFromInvSafe(const InvLegPose& ILP, AbsLegPose& ALP) const
{
	// Initialise that the kinematics calculation was exact
	bool exactKin = true;

	// Create a working copy of the inverse pose
	InvLegPose ILPW = ILP;

	// Protect against excessive ankle point Z values
	double legScaleInv = RK.legScaleInv();
	double offsetZ = legScaleInv * konfig.limAnklePosZMax() - ILPW.anklePos.z();
	if(offsetZ < 0.0)
	{
		ILPW.anklePos.z() += offsetZ;
		exactKin = false;
	}

	// Perform inverse kinematics to get the corresponding abstract pose
	exactKin &= K.AbsFromInv(ILPW, ALP);

	// Protect against insufficient leg retractions
	if(ALP.retraction < konfig.limLegRetMin())
	{
		ALP.retraction = konfig.limLegRetMin();
		exactKin = false;
	}

	// Return whether the kinematics calculation was exact
	return exactKin;
}

// Safe inverse kinematics function: Inverse --> Joint and Abstract
template<> bool FeedKinematics<serial::SerialKinematics>::JointAbsFromInvSafe(const InvLegPose& ILP, JointLegPose& JLP, AbsLegPose& ALP) const
{
	// Safely perform the inverse kinematics as required
	bool exactKin = AbsFromInvSafe(ILP, ALP);
	K.JointFromAbs(ALP, JLP);
	return exactKin;
}

// CoM inverse kinematics function: Joint
template<> void FeedKinematics<serial::SerialKinematics>::JointFromCoMRay(const Vec3& CoMRay, const JointArmPose& JAPRef, JointArmPose& JAP) const
{
	// Calculate the joint CoM inverse kinematics as required
	K.JointFromCoMRay(CoMRay, JAPRef, konfig.limShoulderRollCoMMax(), konfig.limShoulderRollCoMBuf(), JAP);
}

// Pose bias function: AbsLegPose
template<> void FeedKinematics<serial::SerialKinematics>::biasAbsPose(AbsLegPose& ALP) const
{
	// Bias the leg and foot angle X as required
	ALP.angleX += konfig.biasLegAngleX();
	ALP.footAngleX += konfig.biasFootAngleX();

	// Bias the leg extension as required
	double extBias = konfig.biasLegRet();
	if(extBias >= 0.0 && ALP.isLeft)
		ALP.retraction += extBias;
	if(extBias < 0.0 && !ALP.isLeft)
		ALP.retraction -= extBias;
}

// Pose coercion function: JointArmPose
template<> bool FeedKinematics<serial::SerialKinematics>::JointCoerceSoft(JointArmPose& JAP) const
{
	// Declare variables
	bool coerced[3];

	// Retrieve the limb sign
	int ls = JAP.limbSign;

	// Soft-coerce the joint arm pose as required
	JAP.shoulderPitch = rc_utils::coerceSoft<double>(JAP.shoulderPitch, konfig.limShoulderPitchMin(), konfig.limShoulderPitchMax(), konfig.limShoulderPitchBuf(), coerced[0]);
	JAP.shoulderRoll = ls * rc_utils::coerceSoft<double>(JAP.shoulderRoll / ls, konfig.limShoulderRollMin(), konfig.limShoulderRollMax(), konfig.limShoulderRollBuf(), coerced[1]);
	JAP.elbowPitch = rc_utils::coerceSoft<double>(JAP.elbowPitch, -konfig.limElbowPitchMax(), -konfig.limElbowPitchMin(), konfig.limElbowPitchBuf(), coerced[2]);

	// Return whether any field of the joint arm pose was coerced
	return (coerced[0] || coerced[1] || coerced[2]);
}

// Pose coercion function: AbsLegPose
template<> bool FeedKinematics<serial::SerialKinematics>::AbsCoerceSoft(AbsLegPose& ALP) const
{
	// Declare variables
	bool coerced[6];

	// Retrieve the limb sign
	int ls = ALP.limbSign;

	// Soft-coerce the abstract leg pose as required
	ALP.angleX = ls * rc_utils::coerceSoft<double>(ALP.angleX / ls, konfig.limLegAngleXMin(), konfig.limLegAngleXMax(), konfig.limLegAngleXBuf(), coerced[0]);
	ALP.angleY = rc_utils::coerceSoft<double>(ALP.angleY, konfig.limLegAngleYMin(), konfig.limLegAngleYMax(), konfig.limLegAngleYBuf(), coerced[1]);
	ALP.angleZ = ls * rc_utils::coerceSoft<double>(ALP.angleZ / ls, konfig.limLegAngleZMin(), konfig.limLegAngleZMax(), konfig.limLegAngleZBuf(), coerced[2]);
	ALP.footAngleX = ls * rc_utils::coerceSoft<double>(ALP.footAngleX / ls, konfig.limFootAngleXMin(), konfig.limFootAngleXMax(), konfig.limFootAngleXBuf(), coerced[3]);
	ALP.footAngleY = rc_utils::coerceSoft<double>(ALP.footAngleY, konfig.limFootAngleYMin(), konfig.limFootAngleYMax(), konfig.limFootAngleYBuf(), coerced[4]);
	ALP.retraction = rc_utils::coerceSoft<double>(ALP.retraction, konfig.limLegRetMin(), konfig.limLegRetMax(), konfig.limLegRetBuf(), coerced[5]);

	// Return whether any field of the abstract leg pose was coerced
	return (coerced[0] || coerced[1] || coerced[2] || coerced[3] || coerced[4] || coerced[5]);
}
// EOF