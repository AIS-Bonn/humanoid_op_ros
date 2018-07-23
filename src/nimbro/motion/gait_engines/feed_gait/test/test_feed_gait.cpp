// Test the feedback gait (NOT a unit test)
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/feed_gait.h>
#include <feed_gait/kinematics/feed_kinematics.h>
#include <feed_gait/trajectory/feed_trajectory.h>
#include <feed_gait/odometry/feed_odometry.h>
#include <feed_gait/model/feed_model.h>

// Namespaces
using namespace std;
using namespace feed_gait;
using namespace rot_conv;
using namespace humanoid_kinematics;
using namespace humanoid_kinematics::serial;
using namespace humanoid_kinematics::serial_pose_classes;

// Temp function A
void tempA()
{
	LimbIndex limbIndex = RIGHT;
	LimbSign limbSign = LS_LEFT;
	LimbType limbType = LT_ARM;

	cout << limbIndex << limbType << " is " << limbSign << endl;

	const SerialKinematics SK;

	JointLegPose jlp(LEFT, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
	cout << jlp.limbIndex << endl;
	cout << jlp.hipRoll << " " << jlp.anklePitch << endl;
	AbsLegPose alp(LEFT);
	SK.AbsFromJoint(jlp, alp);
	cout << alp.angleY << " " << alp.footAngleX << endl;
	SK.AbsFromJoint({LEFT, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, alp);
	cout << alp.angleY << " " << alp.footAngleX << endl;
	cout << endl;

	JointLegPose jlpp(jlp);
	JointLegPose jlppp = jlp;
	JointLegPose jlpppp(RIGHT);
	jlpppp = jlp;
	cout << jlp.limbName() << endl;
	cout << jlpp.limbName() << endl;
	cout << jlppp.limbName() << endl;
	cout << jlpppp.limbName() << endl;
	cout << endl;

	JointLegPose jlpa(LEFT);
	JointLegPose jlpb(RIGHT);
	jlpa.anklePitch = 1.34;
	jlpb.anklePitch = 0.13;
	jlpa = jlpb;
	cout << jlpa.limbIndex << " " << jlpa.anklePitch << endl;

	cout << "-------------------------------------------------------------------" << endl;

	std::vector<JointInfo> info;

	jlpa.getJointInfo(info);
	for(std::size_t i = 0; i < info.size(); i++)
		cout << info[i] << endl;
	jlpb.getJointInfo(info);
	for(std::size_t i = 0; i < info.size(); i++)
		cout << info[i] << endl;

	JointArmPose japa(LEFT);
	JointArmPose japb(RIGHT);

	japa.getJointInfo(info);
	for(std::size_t i = 0; i < info.size(); i++)
		cout << info[i] << endl;
	japb.getJointInfo(info);
	for(std::size_t i = 0; i < info.size(); i++)
		cout << info[i] << endl;

	JointHeadPose jhp;

	jhp.getJointInfo(info);
	for(std::size_t i = 0; i < info.size(); i++)
		cout << info[i] << endl;

	cout << "-------------------------------------------------------------------" << endl;

	JointPose jp;

	jp.getJointInfo(info);
	for(std::size_t i = 0; i < info.size(); i++)
		cout << info[i] << endl;

	cout << "-------------------------------------------------------------------" << endl;

	double orig[JointPose::NUM_FIELDS];
	for(std::size_t i = 0; i < JointPose::NUM_FIELDS; i++)
		orig[i] = i*i;
	jp.fromArray(orig);

	cout << jp.legL.hipRoll << endl;
	cout << jp.legR.anklePitch << endl;
	cout << jp.armL.elbowPitch << endl;
	cout << jp.armR.shoulderPitch << endl;
	cout << jp.head.neckYaw << endl;

	double neww[JointPose::NUM_FIELDS];
	jp.toArray(neww);
	for(std::size_t i = 0; i < JointPose::NUM_FIELDS; i++)
		cout << "neww[" << i << "] = " << neww[i] << endl;

	cout << "-------------------------------------------------------------------" << endl;

	InvPose ip;

	double iorig[InvPose::NUM_FIELDS];
	for(std::size_t i = 0; i < InvPose::NUM_FIELDS; i++)
		iorig[i] = i - 3;
	ip.fromArray(iorig);

	cout << ip.legL.anklePos << endl;
	cout << ip.legR.footRot << endl;
	cout << ip.armL.handPos << endl;
	cout << ip.armR.handPos << endl;
	cout << ip.head.dirnVec << endl;

	Vec3 foo(2.3, 4.5, -1.2);
	cout << "Hi: " << foo << endl;

	double newi[InvPose::NUM_FIELDS];
	ip.toArray(newi);
	for(std::size_t i = 0; i < InvPose::NUM_FIELDS; i++)
		cout << "newi[" << i << "] = " << newi[i] << endl;

	cout << "-------------------------------------------------------------------" << endl;

	cout << jlpa.getFieldName(3) << endl;
	cout << jlpa.getFieldNameShort(3) << endl;
	cout << endl;

	std::vector<std::string> names;

	jlpa.getFieldNames(names);
	for(std::size_t i = 0; i < names.size(); i++)
		cout << names[i] << endl;

	jlpa.getFieldNamesShort(names);
	for(std::size_t i = 0; i < names.size(); i++)
		cout << names[i] << endl;

	cout << "-------------------------------------------------------------------" << endl;
}

// Temp function B
void tempB()
{
	KinematicsWrapperBasePtr KW = std::make_shared<KinematicsWrapper<SerialKinematics>>();

	std::vector<JointInfo> jointInfo;
	KW->getJointInfo(jointInfo);
	for(std::size_t i = 0; i < jointInfo.size(); i++)
		cout << "[Joint " << i << "] " << jointInfo[i] << endl;
	cout << endl;

	std::vector<std::string> names, namesShort;

	KW->getJointFields(names);
	KW->getJointFieldsShort(namesShort);
	for(std::size_t i = 0; i < names.size(); i++)
		cout << "[J" << i << "] " << namesShort[i] << " --> " << names[i] << endl;
	cout << endl;

	KW->getAbsFields(names);
	KW->getAbsFieldsShort(namesShort);
	for(std::size_t i = 0; i < names.size(); i++)
		cout << "[A" << i << "] " << namesShort[i] << " --> " << names[i] << endl;
	cout << endl;

	KW->getInvFields(names);
	KW->getInvFieldsShort(namesShort);
	for(std::size_t i = 0; i < names.size(); i++)
		cout << "[I" << i << "] " << namesShort[i] << " --> " << names[i] << endl;
	cout << endl;
}

// Temp function C
void tempC()
{
	const SerialKinematics SK;

	cout << "LL:    " << SK.sconfig.LL << endl;
	cout << "LLdbl: " << SK.sconfig.LLdbl << endl;
	cout << "LA:    " << SK.sconfig.LA << endl;
	cout << "LAdbl: " << SK.sconfig.LAdbl << endl;
	cout << "hx:    " << SK.sconfig.hipOffsetX() << endl;
	cout << "hy:    " << SK.sconfig.hipOffsetY() << endl;
	cout << "legScaleInv (2L): " << SK.legScaleInv() << endl;
	cout << "legScaleTip (F):  " << SK.legScaleTip() << endl;
	cout << endl;

	JointLegPose JLP(LEFT, 0.3, -0.2, 0.1, 0.5, 0.2, 0.15);
	AbsLegPose ALP(LEFT, 0.2, -0.1, 0.4, 0.2, -0.3, 0.15);
	InvLegPose ILP(LEFT, Vec3(0.05, -0.03, 0.1), QuatFromAxis(Z_AXIS, 0.1));

	JointArmPose JAP(LEFT, 0.4, 0.2, -0.1);
	AbsArmPose AAP(LEFT, 0.2, -0.4, 0.15);
	InvArmPose IAP(LEFT, Vec3(0.03, 0.02, 0.08));

	JointHeadPose JHP(1.9, -0.4);
	AbsHeadPose AHP = SK.AbsFromJoint(JHP);
	InvHeadPose IHP = SK.InvFromJoint(JHP);

	JointPose JP(JLP, JLP, JAP, JAP, JHP);
	AbsPose AP(ALP, ALP, AAP, AAP, AHP);
	InvPose IP(ILP, ILP, IAP, IAP, IHP);

	cout << "LEGS..." << endl;
	cout << endl;

	cout << JLP << " --> " << SK.AbsFromJoint(JLP) << endl;
	cout << JLP << " --> " << SK.InvFromJoint(JLP) << endl;
	cout << endl;

	cout << ALP << " --> " << SK.JointFromAbs(ALP) << endl;
	cout << ALP << " --> " << SK.InvFromAbs(ALP) << endl;
	cout << endl;

	cout << ILP << " --> " << SK.JointFromInv(ILP) << endl;
	cout << ILP << " --> " << SK.AbsFromInv(ILP) << endl;
	cout << endl;

	cout << "ARMS..." << endl;
	cout << endl;

	cout << JAP << " --> " << SK.AbsFromJoint(JAP) << endl;
	cout << JAP << " --> " << SK.InvFromJoint(JAP) << endl;
	cout << endl;

	cout << AAP << " --> " << SK.JointFromAbs(AAP) << endl;
	cout << AAP << " --> " << SK.InvFromAbs(AAP) << endl;
	cout << endl;

	cout << IAP << " --> " << SK.JointFromInv(IAP) << endl;
	cout << IAP << " --> " << SK.AbsFromInv(IAP) << endl;
	cout << endl;

	cout << "HEAD..." << endl;
	cout << endl;

	cout << JHP << " --> " << AHP << endl;
	cout << JHP << " --> " << IHP << endl;
	cout << endl;

	cout << AHP << " --> " << SK.JointFromAbs(AHP) << endl;
	cout << AHP << " --> " << SK.InvFromAbs(AHP) << endl;
	cout << endl;

	cout << IHP << " --> " << SK.JointFromInv(IHP) << endl;
	cout << IHP << " --> " << SK.AbsFromInv(IHP) << endl;
	cout << endl;

	cout << "ALL..." << endl;
	cout << endl;

	cout << JP << " --> " << SK.AbsFromJoint(JP) << endl;
	cout << JP << " --> " << SK.InvFromJoint(JP) << endl;
	cout << endl;

	cout << AP << " --> " << SK.JointFromAbs(AP) << endl;
	cout << AP << " --> " << SK.InvFromAbs(AP) << endl;
	cout << endl;

	cout << IP << " --> " << SK.JointFromInv(IP) << endl;
	cout << IP << " --> " << SK.AbsFromInv(IP) << endl;
	cout << endl;

	cout << "With zero head: " << SK.AbsFromJoint({JLP, JLP, JAP, JAP}) << endl;
	cout << "With zero head: " << SK.InvFromAbs({ALP, ALP, AAP, AAP}) << endl;
	cout << "With zero head: " << SK.JointFromInv({ILP, ILP, IAP, IAP}) << endl;
	cout << endl;

	cout << SK.AbsFromJoint(JointLegPose::ZeroL()) << " = " << AbsLegPose::ZeroL() << endl;
	cout << SK.AbsFromJoint(JointLegPose::ZeroR()) << " = " << AbsLegPose::ZeroR() << endl;
	cout << SK.InvFromJoint(JointLegPose::ZeroL()) << " = " << InvLegPose::ZeroL() << endl;
	cout << SK.InvFromJoint(JointLegPose::ZeroR()) << " = " << InvLegPose::ZeroR() << endl;
	cout << endl;

	cout << SK.JointFromAbs(AbsLegPose::ZeroL()) << " = " << JointLegPose::ZeroL() << endl;
	cout << SK.JointFromAbs(AbsLegPose::ZeroR()) << " = " << JointLegPose::ZeroR() << endl;
	cout << SK.InvFromAbs(AbsLegPose::ZeroL()) << " = " << InvLegPose::ZeroL() << endl;
	cout << SK.InvFromAbs(AbsLegPose::ZeroR()) << " = " << InvLegPose::ZeroR() << endl;
	cout << endl;

	cout << SK.JointFromInv(InvLegPose::ZeroL()) << " = " << JointLegPose::ZeroL() << endl;
	cout << SK.JointFromInv(InvLegPose::ZeroR()) << " = " << JointLegPose::ZeroR() << endl;
	cout << SK.AbsFromInv(InvLegPose::ZeroL()) << " = " << AbsLegPose::ZeroL() << endl;
	cout << SK.AbsFromInv(InvLegPose::ZeroR()) << " = " << AbsLegPose::ZeroR() << endl;
	cout << endl;

	cout << SK.AbsFromJoint(JointArmPose::ZeroL()) << " = " << AbsArmPose::ZeroL() << endl;
	cout << SK.AbsFromJoint(JointArmPose::ZeroR()) << " = " << AbsArmPose::ZeroR() << endl;
	cout << SK.InvFromJoint(JointArmPose::ZeroL()) << " = " << InvArmPose::ZeroL() << endl;
	cout << SK.InvFromJoint(JointArmPose::ZeroR()) << " = " << InvArmPose::ZeroR() << endl;
	cout << endl;

	cout << SK.JointFromAbs(AbsArmPose::ZeroL()) << " = " << JointArmPose::ZeroL() << endl;
	cout << SK.JointFromAbs(AbsArmPose::ZeroR()) << " = " << JointArmPose::ZeroR() << endl;
	cout << SK.InvFromAbs(AbsArmPose::ZeroL()) << " = " << InvArmPose::ZeroL() << endl;
	cout << SK.InvFromAbs(AbsArmPose::ZeroR()) << " = " << InvArmPose::ZeroR() << endl;
	cout << endl;

	cout << SK.JointFromInv(InvArmPose::ZeroL()) << " = " << JointArmPose::ZeroL() << endl;
	cout << SK.JointFromInv(InvArmPose::ZeroR()) << " = " << JointArmPose::ZeroR() << endl;
	cout << SK.AbsFromInv(InvArmPose::ZeroL()) << " = " << AbsArmPose::ZeroL() << endl;
	cout << SK.AbsFromInv(InvArmPose::ZeroR()) << " = " << AbsArmPose::ZeroR() << endl;
	cout << endl;

	cout << SK.AbsFromJoint(JointHeadPose::Zero()) << " = " << AbsHeadPose::Zero() << endl;
	cout << SK.InvFromJoint(JointHeadPose::Zero()) << " = " << InvHeadPose::Zero() << endl;
	cout << endl;

	cout << SK.JointFromAbs(AbsHeadPose::Zero()) << " = " << JointHeadPose::Zero() << endl;
	cout << SK.InvFromAbs(AbsHeadPose::Zero()) << " = " << InvHeadPose::Zero() << endl;
	cout << endl;

	cout << SK.JointFromInv(InvHeadPose::Zero()) << " = " << JointHeadPose::Zero() << endl;
	cout << SK.AbsFromInv(InvHeadPose::Zero()) << " = " << AbsHeadPose::Zero() << endl;
	cout << endl;

	cout << SK.AbsFromJoint(JointPose::Zero()) << " = " << endl << AbsPose::Zero() << endl;
	cout << SK.InvFromJoint(JointPose::Zero()) << " = " << endl << InvPose::Zero() << endl;
	cout << endl;

	cout << SK.JointFromAbs(AbsPose::Zero()) << " = " << endl << JointPose::Zero() << endl;
	cout << SK.InvFromAbs(AbsPose::Zero()) << " = " << endl << InvPose::Zero() << endl;
	cout << endl;

	cout << SK.JointFromInv(InvPose::Zero()) << " = " << endl << JointPose::Zero() << endl;
	cout << SK.AbsFromInv(InvPose::Zero()) << " = " << endl << AbsPose::Zero() << endl;
	cout << endl;
}

// Temp function D
void tempD()
{
	TipPose TP(LegTipPose(LEFT, Vec3(0.1, 0.2, 0.3), QuatFromAxis(X_AXIS, 0.4)),
	           LegTipPose(RIGHT, Vec3(0.3, 0.2, 0.1), QuatFromAxis(Z_AXIS, -0.2)),
	           ArmTipPose(LEFT, Vec3(0.15, -0.15, 0.2), Quat::Identity()),
	           ArmTipPose(RIGHT, Vec3(-0.25, 0.1, -0.1), QuatFromAxis(Y_AXIS, 0.3)),
	           HeadTipPose(Vec3(0.05, -0.05, 0.02), Quat::Identity()));

	double array[TipPose::NUM_FIELDS];
	TP.toArray(array);
	for(std::size_t i = 0; i < TipPose::NUM_FIELDS; i++)
		cout << "array[" << i << "] = " << array[i] << endl;
	cout << endl;

	TipPose TPout;
	TPout.fromArray(array);
	cout << TPout << endl;
	cout << endl;

	std::vector<std::string> names;
	TPout.getFieldNames(names);
	for(const std::string& name : names)
		cout << name << endl;
	cout << endl;
	TPout.getFieldNamesShort(names);
	for(const std::string& name : names)
		cout << name << endl;
	cout << endl;
}

// Temp function E
void tempE()
{
	const SerialKinematics SK;

	JointHeadPose JHP(0.2, 0.3);
	InvHeadPose IHP = SK.InvFromJoint(JHP);
	HeadTipPose HTPa = SK.TipFromInv(IHP);
	HeadTipPose HTPb = SK.TipFromJoint(JHP);

	cout << JHP << endl;
	cout << IHP << endl;
	cout << HTPa << endl;
	cout << HTPb << endl;
	cout << endl;

	InvHeadPose IHPa = SK.InvFromTip(HTPa);
	AbsHeadPose AHPa = SK.AbsFromInv(IHPa);
	JointHeadPose JHPa = SK.JointFromAbs(AHPa);

	cout << IHPa << endl;
	cout << AHPa << endl;
	cout << JHPa << endl;
}

// Temp function F
void tempF()
{
	const SerialKinematics SK;

	JointLegPose JLP(LEFT, 0.1, -0.2, 0.3, 0.15, -0.05, 0.1), JLPout(LEFT);
	AbsLegPose ALP(LEFT, 0.1, -0.2, 0.3, 0.15, -0.05, 0.1), ALPout(LEFT);
	InvLegPose ILP(LEFT, Vec3(0.03, -0.02, 0.1), QuatFromAxis(Vec3(1.0, -1.5, 0.5), 0.25)), ILPout(LEFT);
	LegTipPose LTP = SK.TipFromInv(ILP), LTPout(LEFT);

	JointLegPoseVel JLPV(LEFT, 2.0, 1.0, 3.0, -2.0, -1.0, -3.0), JLPVout(LEFT);
	AbsLegPoseVel ALPV(LEFT, 2.0, 1.0, 3.0, -2.0, -1.0, -3.0), ALPVout(LEFT);
	InvLegPoseVel ILPV(LEFT, Vec3(0.5, 0.9, -0.4), AngVel(1.0, -0.4, 0.4)), ILPVout(LEFT);
	LegTipPoseVel LTPV(LEFT, Vec3(0.5, 0.9, -0.4), AngVel(1.0, -0.4, 0.4)), LTPVout(LEFT);

	LegJacobian LJ, LJJ, LJA;
	LegJacobianSolver LJS;

	AbsLegPose ALPin = SK.AbsFromJoint(JLP);

	cout << "-------------------------------------------------------------------" << endl;

	cout << "Conversion: Joint --> Abstract" << endl;
	cout << "Input pos: " << JLP << endl;
	cout << "Input pos: " << ALPin << endl;
	cout << "Input vel: " << JLPV << endl;
	SK.AbsFromJointVel(JLPV, JLP, ALPVout);
	cout << "Output vel: " << ALPVout << endl;
	SK.AbsFromJointVel(JLPV, ALPin, ALPVout);
	cout << "Output vel: " << ALPVout << endl;
	SK.AbsFromJointJacob(JLP, LJ);
	cout << "Output Jacobian:" << endl << LJ << endl;
	SK.AbsFromJointJacob(ALPin, LJ);
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	cout << "Conversion: Abstract --> Joint" << endl;
	cout << "Input pos: " << JLP << endl;
	cout << "Input pos: " << ALPin << endl;
	cout << "Input vel: " << ALPV << endl;
	SK.JointFromAbsVel(ALPV, JLP, JLPVout);
	cout << "Output vel: " << JLPVout << endl;
	SK.JointFromAbsVel(ALPV, ALPin, JLPVout);
	cout << "Output vel: " << JLPVout << endl;
	SK.JointFromAbsJacob(JLP, LJ);
	cout << "Output Jacobian:" << endl << LJ << endl;
	SK.JointFromAbsJacob(ALPin, LJ);
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	cout << "-------------------------------------------------------------------" << endl;

	SK.InvFromAbsVel(ALPV, ALP, ILPVout, LJ, ILPout);
	cout << "Conversion: Abstract --> Inverse" << endl;
	cout << "Input pos: " << ALP << endl;
	cout << "Input vel: " << ALPV << endl;
	cout << "Output pos: " << ILPout << endl;
	cout << "Output vel: " << ILPVout << endl;
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	SK.AbsFromInvVel(ILPV, ALP, ALPVout, LJS, ILPout);
	LJ = LegJacobFromSolver(LJS);
	cout << "Conversion: Inverse --> Abstract" << endl;
	cout << "Input pos: " << ALP << endl;
	cout << "Input vel: " << ILPV << endl;
	cout << "Output pos: " << ILPout << endl;
	cout << "Output vel: " << ALPVout << endl;
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	cout << "-------------------------------------------------------------------" << endl;

	SK.InvFromJointVel(JLPV, JLP, ILPVout, LJ, ILPout);
	cout << "Conversion: Joint --> Inverse" << endl;
	cout << "Input pos: " << JLP << endl;
	cout << "Input vel: " << JLPV << endl;
	cout << "Output pos: " << ILPout << endl;
	cout << "Output vel: " << ILPVout << endl;
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	SK.JointFromInvVel(ILPV, JLP, JLPVout, LJS, ILPout);
	LJ = LegJacobFromSolver(LJS);
	cout << "Conversion: Inverse --> Joint" << endl;
	cout << "Input pos: " << JLP << endl;
	cout << "Input vel: " << ILPV << endl;
	cout << "Output pos: " << ILPout << endl;
	cout << "Output vel: " << JLPVout << endl;
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	cout << "-------------------------------------------------------------------" << endl;

	SK.TipFromAbsVel(ALPV, ALP, LTPVout, LJ, LTPout);
	cout << "Conversion: Abstract --> Tip" << endl;
	cout << "Input pos: " << ALP << endl;
	cout << "Input vel: " << ALPV << endl;
	cout << "Output pos: " << LTPout << endl;
	cout << "Output vel: " << LTPVout << endl;
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	SK.AbsFromTipVel(LTPV, ALP, ALPVout, LJS, LTPout);
	LJ = LegJacobFromSolver(LJS);
	cout << "Conversion: Tip --> Abstract" << endl;
	cout << "Input pos: " << ALP << endl;
	cout << "Input vel: " << LTPV << endl;
	cout << "Output pos: " << LTPout << endl;
	cout << "Output vel: " << ALPVout << endl;
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	cout << "-------------------------------------------------------------------" << endl;

	SK.TipFromJointVel(JLPV, JLP, LTPVout, LJ, LTPout);
	cout << "Conversion: Joint --> Tip" << endl;
	cout << "Input pos: " << JLP << endl;
	cout << "Input vel: " << JLPV << endl;
	cout << "Output pos: " << LTPout << endl;
	cout << "Output vel: " << LTPVout << endl;
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	SK.JointFromTipVel(LTPV, JLP, JLPVout, LJS, LTPout);
	LJ = LegJacobFromSolver(LJS);
	cout << "Conversion: Tip --> Joint" << endl;
	cout << "Input pos: " << JLP << endl;
	cout << "Input vel: " << LTPV << endl;
	cout << "Output pos: " << LTPout << endl;
	cout << "Output vel: " << JLPVout << endl;
	cout << "Output Jacobian:" << endl << LJ << endl;
	cout << endl;

	cout << "-------------------------------------------------------------------" << endl;

	cout << "Conversion: Inverse --> Tip" << endl;
	cout << "Input pos: " << ILP << endl;
	cout << "Input pos: " << LTP << endl;
	cout << "Input vel: " << ILPV << endl;
	cout << "Output vel: " << SK.TipFromInvVel(ILPV, ILP) << endl;
	cout << "Output vel: " << SK.TipFromInvVel(ILPV, LTP) << endl;
	cout << "Output Jacobian:" << endl << SK.TipFromInvJacob(ILP) << endl;
	cout << "Output Jacobian:" << endl << SK.TipFromInvJacob(LTP) << endl;
	cout << endl;

	cout << "Conversion: Tip --> Inverse" << endl;
	cout << "Input pos: " << ILP << endl;
	cout << "Input pos: " << LTP << endl;
	cout << "Input vel: " << LTPV << endl;
	cout << "Output vel: " << SK.InvFromTipVel(LTPV, ILP) << endl;
	cout << "Output vel: " << SK.InvFromTipVel(LTPV, LTP) << endl;
	cout << "Output Jacobian:" << endl << SK.InvFromTipJacob(ILP) << endl;
	cout << "Output Jacobian:" << endl << SK.InvFromTipJacob(LTP) << endl;
	cout << endl;

	cout << "-------------------------------------------------------------------" << endl;
}

// Temp function G
void tempG()
{
	FeedPlotManager PM(PM_COUNT, RESOURCE_PATH); PM.disable();
	FeedOdometryBasePtr odom = std::make_shared<trivial_odom::FeedTrivialOdom>(&PM);
	odom->reset();
	odom->setPose2D(1.0, 2.0, 3.0);
	OdometryInput odomInput;
	odom->update(odomInput);
}

// Temp function H
void tempH()
{
	// Note: That config variables are floats, so the calculations will never be more exact when compared to Matlab (all double)
	// than errors on the order of 5e-8. The problem is when a config variable is for example '0.65', then when converted to a
	// double its value is 0.6499999762...

	std::ios::fmtflags f(cout.flags());
	cout << fixed << setprecision(8);

	cout << "Testing keypoint trajectory..." << endl;

	FeedPlotManager PM(PM_COUNT, RESOURCE_PATH); PM.disable();
	FeedTrajectoryBasePtr T = std::make_shared<keypoint_traj::FeedKeypointTraj<SerialKinematics>>(&PM);
	FeedModelBasePtr M = std::make_shared<trivial_model::FeedTrivialModel>(&PM);

	TrajInfo trajInfo;
	T->getInfo(trajInfo);

	GaitPhaseInfo phaseInfo;
	ModelInput modelInput;
	TrajCommand trajCmd(trajInfo.fusedPitchN);
	ACFlag trajCmdUse = ACFlag::NONE;
	PoseCommand poseCmd;

	cout << "clear; clc; in = struct('wavemu', -3.1:0.1:3.1); out = CalcKeypointTraj(in); outt = CalcArmAngle(in); close all;" << endl;
	cout << "clear mu pos eff supp;" << endl;
	for(int i = 1; i <= 63; i++)
	{
		double gaitPhase = 0.1*(i - 32);
		phaseInfo.setGaitPhase(gaitPhase);

		M->callUpdate(phaseInfo, modelInput); // Note: The trivial model does not use modelInput at all so we just give it the default constructed value

		StepSizeCommand newCmd;
		M->getStepSize().writeTo(newCmd);
		trajCmd.gcv = newCmd.targetGcv();
		trajCmd.gcvLFAcc.setZero();
		M->writeActions(trajCmd, trajCmdUse);

		T->generate(trajCmd);
		T->evaluate(gaitPhase, poseCmd);

		cout << "mu(:, " << i << ") = " << gaitPhase << "; ";
		cout << "pos(:, " << i << ") = [";
		for(size_t j = 0; j < poseCmd.pos.size(); j++)
		{
			if(j != 0) cout << "; ";
			cout << poseCmd.pos[j];
		}
		cout << "]; ";
		cout << "eff(:, " << i << ") = [";
		for(size_t j = 0; j < poseCmd.effort.size(); j++)
		{
			if(j != 0) cout << "; ";
			cout << poseCmd.effort[j];
		}
		cout << "]; ";
		cout << "supp(:, " << i << ") = [" << poseCmd.suppCoeff[hk::LEFT] << "; " << poseCmd.suppCoeff[hk::RIGHT] << "]; ";
	}
	cout << "disp(max(abs(out.wave.mu' - mu))); ";
	cout << "disp(max(max(abs([StructToMat(out.wave.JPP{1})'; StructToMat(out.wave.JPP{2})'] - pos(1:12,:))))); ";
	cout << "disp(max(max(abs(out.wave.suppCoeff' - supp)))); ";
	cout << "disp(max(abs(outt.wave.mu' - mu))); ";
	cout << "disp(max(max(abs([StructToMat(outt.wave.AJPP{1})'; StructToMat(outt.wave.AJPP{2})'] - pos(13:18,:))))); ";
	cout << "disp(max(max(abs(pos(19:20,:) - zeros(2,size(pos,2)))))); ";
	cout << endl;

	cout.flags(f);
}

// Temp function I
void tempI()
{
	PoseCommand PC, PCa, PCb;

	PCa.pos = {1.0, 3.0, 4.0};
	PCa.effort = {-2.0, 1.0, 0.5, 7.0, 6.0};
	PCa.suppCoeff[LEFT] = 0.9;
	PCa.suppCoeff[RIGHT] = 0.4;

	PCb.pos = {5.0, -2.0, 6.0, 1.5};
	PCb.effort = {};
	PCb.suppCoeff[LEFT] = 0.0;
	PCb.suppCoeff[RIGHT] = 0.7;

	PC.setMeanOf(PCa, PCb);
	cout << PC << endl;
	PC.reset();
	cout << PC << endl;
	PC.setMeanOf(PCb, PCa);
	cout << PC << endl;

	cout << endl;
}

// Temp function J
void tempJ()
{
	cout << "Timing feed gait..." << endl;

	FeedPlotManager PM(PM_COUNT, RESOURCE_PATH, false);
	FeedTrajectoryBasePtr T = std::make_shared<keypoint_traj::FeedKeypointTraj<SerialKinematics>>(&PM);
	FeedModelBasePtr M = std::make_shared<tilt_phase_model::FeedTiltPhaseModel<SerialKinematics>>(&PM);

	ros::WallTime start;
	double duration;
	int N;

	TrajInfo trajInfo;
	T->getInfo(trajInfo);

	GaitPhaseInfo phaseInfo;
	ModelInput modelInput;
	TrajCommand trajCmd(trajInfo.fusedPitchN);
	PoseCommand poseCmd;

	N = 10000000;
	start = ros::WallTime::now();
	for(int i = 0; i < N; i++)
	{
		double gaitPhase = rc_utils::picut(0.1*i);
		phaseInfo.setGaitPhase(gaitPhase);
		M->callUpdate(phaseInfo, modelInput);
	}
	duration = 1e6 * (ros::WallTime::now() - start).toSec() / N;
	cout << "Model update: " << duration << "us" << endl;

	N = 300000;
	start = ros::WallTime::now();
	for(int i = 0; i < N; i++)
	{
		T->generate(trajCmd);
	}
	duration = 1e6 * (ros::WallTime::now() - start).toSec() / N;
	cout << "Traj generation: " << duration << "us" << endl;

	N = 10000000;
	start = ros::WallTime::now();
	for(int i = 0; i < N; i++)
	{
		double gaitPhase = rc_utils::picut(0.1*i);
		T->evaluate(gaitPhase, poseCmd);
	}
	duration = 1e6 * (ros::WallTime::now() - start).toSec() / N;
	cout << "Traj evaluation: " << duration << "us" << endl;
}

// Main function
int main(int argc, char **argv)
{
	// ROS init
	ros::init(argc, argv, "test_feed_gait");

	// Print header
	cout << "Test feedback gait..." << endl;
	cout << endl;

	// Do something
// 	tempA();
// 	tempB();
// 	tempC();
// 	tempD();
// 	tempE();
// 	tempF();
// 	tempG();
// 	tempH();
// 	tempI();
	tempJ();

	// Return success
	return 0;
}
// EOF