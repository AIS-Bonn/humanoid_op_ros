// Humanoid kinematics - Serial kinematics
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <humanoid_kinematics/serial/serial_kinematics.h>
#include <cfloat>
#include <cmath>

// Namespaces
using namespace humanoid_kinematics::serial;
using namespace rot_conv;
using namespace rc_utils;

//
// Local helper functions
//

// Quaternions from sequential rotations
void QuatFromYX(double angY, double angX, Quat& qYX);
void QuatFromYXE(double angY, double angX, double angE, Quat& qYXE); // E is a second Y rotation
void QuatFromYXE(double angY, double angX, double angE, Quat& qYXE, Quat& qYX); // E is a second Y rotation
void QuatFromZXY(double angZ, double angX, double angY, Quat& qZXY);
void QuatFromZXYK(double angZ, double angX, double angY, double angK, Quat& qZXYK, Quat& qZXY); // K is a second Y rotation

// Inverse leg kinematics helper functions
bool InvKinCalcAbsPose(SerialKinematics::AbsLegPose& ALP1, SerialKinematics::AbsLegPose& ALP2, double phiz, const Rotmat& RF, const Vec3& PA, double hx, double hy, double Ldbl);

// Joint pose cost struct
struct JointPoseCost
{
	// Constructors
	JointPoseCost() = default;
	explicit JointPoseCost(const SerialKinematics::JointLegPose& JLP) { setFromPose(JLP); }

	// Set functions
	void setFromPose(const SerialKinematics::JointLegPose& JLP);

	// Static cost calculation function
	static JointPoseCost Of(const SerialKinematics::JointLegPose& JLP) { JointPoseCost cost; cost.setFromPose(JLP); return cost; }

	// Comparison operator
	bool operator<=(const JointPoseCost& other) const;

	// Data members
	double maxCost;
	double sumCost;
};

// Humanoid kinematics namespace
namespace humanoid_kinematics
{
	// Serial kinematics namespace
	namespace serial
	{
		// ################################
		// #### SerialKinematics class ####
		// ################################

		//
		// Leg pose conversions
		//

		// Conversion: Joint --> Abstract
		void SerialKinematics::AbsFromJoint(const JointLegPose& JLP, AbsLegPose& ALP) const
		{
			// Transcribe the elements of the greatest common base class
			ALP.LegPose::operator=(JLP);

			// Convert between the poses as required
			double alpha = 0.5*picut(JLP.kneePitch); // Note: If we don't picut here, then an unwrapped input would produce an incorrect output, not just an unwrapped one...
			ALP.angleX = JLP.hipRoll;
			ALP.angleY = picut(JLP.hipPitch + alpha);
			ALP.angleZ = JLP.hipYaw;
			ALP.footAngleX = picut(JLP.ankleRoll + ALP.angleX);
			ALP.footAngleY = picut(JLP.anklePitch + ALP.angleY + alpha);
			ALP.retraction = coerce(1.0 - cos(alpha), 0.0, 1.0);
		}

		// Conversion: Joint --> Inverse
		void SerialKinematics::InvFromJoint(const JointLegPose& JLP, InvLegPose& ILP) const
		{
			// Transcribe the elements of the greatest common base class
			ILP.LegPose::operator=(JLP);

			// Calculate the foot rotation
			Quat thighRot, shankRot, ankleRot;
			QuatFromZXYK(JLP.hipYaw, JLP.hipRoll, JLP.hipPitch, JLP.kneePitch, shankRot, thighRot);
			QuatFromYX(JLP.anklePitch, JLP.ankleRoll, ankleRot);
			ILP.footRot = shankRot * ankleRot;

			// Robot dimensions
			double hx = sconfig.hipOffsetX();
			double hy = JLP.limbSign * sconfig.hipOffsetY();

			// Precalculate trigonometric values
			double Cz = cos(JLP.hipYaw) - 1.0;
			double sz = sin(JLP.hipYaw);

			// Calculate the ankle position
			ILP.anklePos.x() = hx*Cz - hy*sz;
			ILP.anklePos.y() = hx*sz + hy*Cz;
			ILP.anklePos.z() = sconfig.LLdbl; // Hip PR pos
			ILP.anklePos += QuatRotVecPureZ(thighRot, -sconfig.LL); // Knee pos
			ILP.anklePos += QuatRotVecPureZ(shankRot, -sconfig.LL); // Ankle pos
		}

		// Conversion: Abstract --> Joint
		void SerialKinematics::JointFromAbs(const AbsLegPose& ALP, JointLegPose& JLP) const
		{
			// Transcribe the elements of the greatest common base class
			JLP.LegPose::operator=(ALP);

			// Convert between the poses as required
			double alpha = acos(coerce(1.0 - ALP.retraction, 0.0, 1.0)); // In [0,pi/2]
			JLP.hipYaw = ALP.angleZ;
			JLP.hipRoll = ALP.angleX;
			JLP.hipPitch = picut(ALP.angleY - alpha);
			JLP.kneePitch = 2.0*alpha; // In [0,pi]
			JLP.anklePitch = picut(ALP.footAngleY - ALP.angleY - alpha);
			JLP.ankleRoll = picut(ALP.footAngleX - ALP.angleX);
		}

		// Conversion: Abstract --> Inverse
		void SerialKinematics::InvFromAbs(const AbsLegPose& ALP, InvLegPose& ILP) const
		{
			// Transcribe the elements of the greatest common base class
			ILP.LegPose::operator=(ALP);

			// Calculate the foot rotation
			Quat legAxisRot, footAngleRot;
			QuatFromZXY(ALP.angleZ, ALP.angleX, ALP.angleY, legAxisRot);
			QuatFromYX(ALP.footAngleY - ALP.angleY, ALP.footAngleX - ALP.angleX, footAngleRot);
			ILP.footRot = legAxisRot * footAngleRot;

			// Robot dimensions
			double hx = sconfig.hipOffsetX();
			double hy = ALP.limbSign * sconfig.hipOffsetY();

			// Precalculate trigonometric values
			double Cz = cos(ALP.angleZ) - 1.0;
			double sz = sin(ALP.angleZ);

			// Calculate the ankle position
			ILP.anklePos.x() = hx*Cz - hy*sz;
			ILP.anklePos.y() = hx*sz + hy*Cz;
			ILP.anklePos.z() = sconfig.LLdbl; // Hip PR pos
			ILP.anklePos += QuatRotVecPureZ(legAxisRot, sconfig.LLdbl*(ALP.retraction - 1.0)); // Ankle pos
		}

		// Conversion: Inverse --> Joint and Abstract
		bool SerialKinematics::JointAbsFromInv(const InvLegPose& ILP, JointLegPose& JLP, AbsLegPose& ALP) const
		{
			// Transcribe the elements of the greatest common base class
			JLP.LegPose::operator=(ILP);
			ALP.LegPose::operator=(ILP);

			// Constants
			static const double Tol = 1024.0*DBL_EPSILON;

			// Ensure that the foot rotation is a unit quaternion
			Quat footRot = NormalisedQuat(ILP.footRot);

			// Calculate the foot orientation rotation matrix
			Rotmat RF = RotmatFromQuat(footRot);
			double xFx = RF.coeff(0,0);
			double xFy = RF.coeff(1,0);
			double xFz = RF.coeff(2,0);

			// Robot dimensions
			double hx = sconfig.hipOffsetX();
			double hy = ILP.limbSign * sconfig.hipOffsetY();

			// Calculate the position of the ankle point relative to the hip point
			Vec3 PA(ILP.anklePos.x() + hx, ILP.anklePos.y() + hy, ILP.anklePos.z() - sconfig.LLdbl);

			// Calculate the singularity discriminating factor A
			double B = xFy*PA.z() - xFz*PA.y();
			double C = xFx*PA.z() - xFz*PA.x();
			double A = sqrt(B*B + C*C);

			// Handle the singularity case
			if(A < Tol)
			{
				// Calculate the abstract poses of the two possible solutions
				AbsLegPose ALP1(ILP.limbIndex), ALP2(ILP.limbIndex);
				bool exact = InvKinCalcAbsPose(ALP1, ALP2, 0.0, RF, PA, hx, hy, sconfig.LLdbl); // Every phiz is possible due to the singularity, so arbitrarily choose 0.0. Local sensitivity is extremely high anyway, so there's no point trying to optimise more than this...

				// Calculate the joint poses of the two possible solutions
				JointLegPose JLP1 = JointFromAbs(ALP1);
				JointLegPose JLP2 = JointFromAbs(ALP1);

				// Calculate the costs of the two possible solutions
				JointPoseCost cost1(JLP1);
				JointPoseCost cost2(JLP2);

				// Find the solution with the lowest cost
				bool better1 = (cost1 <= cost2);
				JLP = (better1 ? JLP1 : JLP2);
				ALP = (better1 ? ALP1 : ALP2);
				return exact;
			}

			// Calculate the two possible solutions for angleZ
			double xi = atan2(B, C);
			double D = asin(coerceAbs(xFz*hy / A, 1.0));
			double phizA = picut(xi + D);
			double phizB = picut(xi + M_PI - D);

			// Calculate the abstract poses of the four possible solutions
			AbsLegPose ALPA1(ILP.limbIndex), ALPA2(ILP.limbIndex), ALPB1(ILP.limbIndex), ALPB2(ILP.limbIndex);
			bool exactA = InvKinCalcAbsPose(ALPA1, ALPA2, phizA, RF, PA, hx, hy, sconfig.LLdbl); // Note: We rather choose a reasonable inexact pose, than a crazy exact pose,
			bool exactB = InvKinCalcAbsPose(ALPB1, ALPB2, phizB, RF, PA, hx, hy, sconfig.LLdbl); // so exact being false should NOT be used to disqualify any of the solutions!

			// Calculate the joint poses of the four possible solutions
			JointLegPose JLPA1 = JointFromAbs(ALPA1);
			JointLegPose JLPA2 = JointFromAbs(ALPA2);
			JointLegPose JLPB1 = JointFromAbs(ALPB1);
			JointLegPose JLPB2 = JointFromAbs(ALPB2);

			// Calculate the costs of the four possible solutions
			JointPoseCost costA1(JLPA1);
			JointPoseCost costA2(JLPA2);
			JointPoseCost costB1(JLPB1);
			JointPoseCost costB2(JLPB2);

			// Find the solution with the lowest cost
			bool A1betterA2 = (costA1 <= costA2);
			bool B1betterB2 = (costB1 <= costB2);
			const JointPoseCost& costA = (A1betterA2 ? costA1 : costA2);
			const JointPoseCost& costB = (B1betterB2 ? costB1 : costB2);
			if(costA <= costB)
			{
				JLP = (A1betterA2 ? JLPA1 : JLPA2);
				ALP = (A1betterA2 ? ALPA1 : ALPA2);
				return exactA;
			}
			else
			{
				JLP = (B1betterB2 ? JLPB1 : JLPB2);
				ALP = (B1betterB2 ? ALPB1 : ALPB2);
				return exactB;
			}
		}

		// Conversion: Inverse --> Tip
		void SerialKinematics::TipFromInv(const InvLegPose& ILP, LegTipPose& LTP) const
		{
			// Transcribe the elements of the greatest common base class
			LTP.LegPose::operator=(ILP);

			// Convert between the poses as required
			LTP.pos = ILP.anklePos + QuatRotVec(ILP.footRot, sconfig.footOffset[ILP.limbIndex]);
			LTP.rot = ILP.footRot;
		}

		// Conversion: Tip --> Inverse
		void SerialKinematics::InvFromTip(const LegTipPose& LTP, InvLegPose& ILP) const
		{
			// Transcribe the elements of the greatest common base class
			ILP.LegPose::operator=(LTP);

			// Convert between the poses as required
			ILP.anklePos = LTP.pos - QuatRotVec(LTP.rot, sconfig.footOffset[ILP.limbIndex]);
			ILP.footRot = LTP.rot;
		}

		//
		// Extra leg pose conversions
		//

		// Conversion: Inverse --> Hip yaw
		void SerialKinematics::HipYawFromInv(const InvLegPose& ILP, double& hipYaw, double& hipYawAlt) const
		{
			// Constants
			static const double Tol = 1024.0*DBL_EPSILON;

			// Ensure that the foot rotation is a unit quaternion
			Quat footRot = NormalisedQuat(ILP.footRot);

			// Calculate the foot x-axis unit vector
			double xFx = 1.0 - 2.0*(footRot.y()*footRot.y() + footRot.z()*footRot.z());
			double xFy = 2.0*(footRot.x()*footRot.y() + footRot.z()*footRot.w());
			double xFz = 2.0*(footRot.x()*footRot.z() - footRot.y()*footRot.w());

			// Robot dimensions
			double hx = sconfig.hipOffsetX();
			double hy = ILP.limbSign * sconfig.hipOffsetY();

			// Calculate the position of the ankle point relative to the hip point
			Vec3 PA(ILP.anklePos.x() + hx, ILP.anklePos.y() + hy, ILP.anklePos.z() - sconfig.LLdbl);

			// Calculate the singularity discriminating factor A
			double B = xFy*PA.z() - xFz*PA.y();
			double C = xFx*PA.z() - xFz*PA.x();
			double A = sqrt(B*B + C*C);

			// Calculate the required hip yaw
			if(A < Tol)
			{
				hipYaw = 0.0;     // Every hip yaw is possible, so arbitrarily choose zero => Local sensitivity is extremely high anyway, so there's no point trying to optimise more than this...
				hipYawAlt = M_PI; // Arbitrary second choice for hip yaw...
			}
			else
			{
				double xi = atan2(B, C);
				double D = asin(coerceAbs(xFz*hy / A, 1.0));
				double hipYawA = picut(xi + D);
				double hipYawB = picut(xi + M_PI - D);
				if(fabs(hipYawA) <= fabs(hipYawB))
				{
					hipYaw = hipYawA;
					hipYawAlt = hipYawB;
				}
				else
				{
					hipYaw = hipYawB;
					hipYawAlt = hipYawA;
				}
			}
		}

		//
		// Arm pose conversions
		//

		// Conversion: Joint --> Abstract
		void SerialKinematics::AbsFromJoint(const JointArmPose& JAP, AbsArmPose& AAP) const
		{
			// Transcribe the elements of the greatest common base class
			AAP.ArmPose::operator=(JAP);

			// Convert between the poses as required
			double alpha = 0.5*picut(JAP.elbowPitch); // Note: If we don't picut here, then an unwrapped input would produce an incorrect output, not just an unwrapped one...
			AAP.angleX = JAP.shoulderRoll;
			AAP.angleY = picut(JAP.shoulderPitch + alpha);
			AAP.retraction = coerce(1.0 - cos(alpha), 0.0, 1.0);
		}

		// Conversion: Joint --> Inverse
		void SerialKinematics::InvFromJoint(const JointArmPose& JAP, InvArmPose& IAP, rot_conv::Vec3& elbowPos) const
		{
			// Transcribe the elements of the greatest common base class
			IAP.ArmPose::operator=(JAP);

			// Calculate the upper and lower arm rotations
			Quat upperArmRot, lowerArmRot;
			QuatFromYXE(JAP.shoulderPitch, JAP.shoulderRoll, JAP.elbowPitch, lowerArmRot, upperArmRot);

			// Calculate the hand position
			IAP.handPos = sconfig.shoulderPos; // Shoulder pos
			IAP.handPos += QuatRotVecPureZ(upperArmRot, -sconfig.LA); // Elbow pos
			elbowPos = IAP.handPos;
			IAP.handPos += QuatRotVecPureZ(lowerArmRot, -sconfig.LA); // Hand pos
		}

		// Conversion: Joint --> Tip
		void SerialKinematics::TipFromJoint(const JointArmPose& JAP, ArmTipPose& ATP, rot_conv::Vec3& elbowPos) const
		{
			// Transcribe the elements of the greatest common base class
			ATP.ArmPose::operator=(JAP);

			// Calculate the upper and lower arm rotations
			Quat upperArmRot, lowerArmRot;
			QuatFromYXE(JAP.shoulderPitch, JAP.shoulderRoll, JAP.elbowPitch, lowerArmRot, upperArmRot);

			// Calculate the tip pose
			ATP.pos = sconfig.shoulderPos; // Shoulder pos
			ATP.pos += QuatRotVecPureZ(upperArmRot, -sconfig.LA); // Elbow pos
			elbowPos = ATP.pos;
			ATP.pos += QuatRotVecPureZ(lowerArmRot, -sconfig.LA); // Hand pos
			ATP.rot = lowerArmRot;
		}

		// Conversion: Abstract --> Joint
		void SerialKinematics::JointFromAbs(const AbsArmPose& AAP, JointArmPose& JAP) const
		{
			// Transcribe the elements of the greatest common base class
			JAP.ArmPose::operator=(AAP);

			// Convert between the poses as required
			double alpha = acos(coerce(1.0 - AAP.retraction, 0.0, 1.0)); // In [0,pi/2]
			JAP.shoulderPitch = picutMax(AAP.angleY + alpha, JointArmPose::ShoulderPitchMax);
			JAP.shoulderRoll = AAP.angleX;
			JAP.elbowPitch = -2.0*alpha; // In [-pi,0]
		}

		// Conversion: Abstract --> Inverse
		void SerialKinematics::InvFromAbs(const AbsArmPose& AAP, InvArmPose& IAP) const
		{
			// Transcribe the elements of the greatest common base class
			IAP.ArmPose::operator=(AAP);

			// Calculate the arm axis rotation
			Quat armAxisRot;
			double alpha = acos(coerce(1.0 - AAP.retraction, 0.0, 1.0)); // In [0,pi/2]
			QuatFromYXE(AAP.angleY + alpha, AAP.angleX, -alpha, armAxisRot);

			// Calculate the hand position
			IAP.handPos = sconfig.shoulderPos; // Shoulder pos
			IAP.handPos += QuatRotVecPureZ(armAxisRot, sconfig.LAdbl*(AAP.retraction - 1.0)); // Hand pos
		}

		// Conversion: Inverse --> Joint
		void SerialKinematics::JointFromInv(const InvArmPose& IAP, JointArmPose& JAP) const
		{
			// Transcribe the elements of the greatest common base class
			JAP.ArmPose::operator=(IAP);

			// Constants
			static const double Tol = 64.0*DBL_EPSILON;

			// Calculate the coordinates of the hand origin in terms of the shoulder frame {S}
			Vec3 handInShoulderFrame = IAP.handPos;
			handInShoulderFrame.z() -= sconfig.LAdbl;

			// Calculate the arm axis length as the distance between the shoulder and hand origins
			double armAxisLength = VecNorm(handInShoulderFrame);
			if(armAxisLength > sconfig.LAdbl)
			{
				handInShoulderFrame *= sconfig.LAdbl / armAxisLength;
				armAxisLength = sconfig.LAdbl;
			}

			// Handle the special case of zero arm axis length
			if(armAxisLength < Tol)
			{
				JAP.shoulderPitch = M_PI_2;
				JAP.shoulderRoll = 0.0;
				JAP.elbowPitch = -M_PI;
				return;
			}

			// Calculate the arm alpha angle based on the arm axis length
			double calpha = coerce(armAxisLength / sconfig.LAdbl, 0.0, 1.0);
			double alpha = acos(calpha);
			double salpha = sin(alpha);

			// Calculate the required elbow pitch
			JAP.elbowPitch = -2.0*alpha;

			// Calculate the required shoulder roll
			double yhat = handInShoulderFrame.y() / armAxisLength;
			double sphi = coerceAbs(yhat / calpha, 1.0);
			JAP.shoulderRoll = asin(sphi);

			// Calculate the required shoulder pitch
			double cphi = cos(JAP.shoulderRoll);
			if(fabs(salpha) < Tol && fabs(cphi) < Tol)
				JAP.shoulderPitch = 0.0;
			else if(fabs(handInShoulderFrame.x()) < Tol && fabs(handInShoulderFrame.z()) < Tol)
				JAP.shoulderPitch = 0.0;
			else
				JAP.shoulderPitch = picutMax(atan2(handInShoulderFrame.x(), handInShoulderFrame.z()) - atan2(salpha, -cphi*calpha), JointArmPose::ShoulderPitchMax);
		}

		// Conversion: Tip --> Inverse
		void SerialKinematics::InvFromTip(const ArmTipPose& ATP, InvArmPose& IAP) const
		{
			// Transcribe the elements of the greatest common base class
			IAP.ArmPose::operator=(ATP);

			// Calculate the hand position
			IAP.handPos = ATP.pos;
		}

		//
		// Extra arm pose conversions
		//

		// Conversion: Joint --> CoM
		void SerialKinematics::CoMFromJoint(const JointArmPose& JAP, rot_conv::Vec3& CoM) const
		{
			// Calculate the hand and elbow positions
			Vec3 elbowPos;
			InvArmPose IAP(JAP.limbIndex);
			InvFromJoint(JAP, IAP, elbowPos);

			// Calculate the CoM position
			CoM = 0.5*elbowPos + 0.25*(sconfig.shoulderPos + IAP.handPos);
		}

		// Conversion: CoMRay --> Joint
		void SerialKinematics::JointFromCoMRay(const rot_conv::Vec3& CoMRay, const JointArmPose& JAPRef, double shoulderRollMax, double shoulderRollBuf, JointArmPose& JAP) const
		{
			// Constants
			static const double Tol = 64.0*DBL_EPSILON;

			// Calculate the nominal alpha angle
			double alphaNom = coerce(fabs(-0.5*picut(JAPRef.elbowPitch)), 0.0, M_PI_2); // The nominal angle between the upper arm and the arm axis, in [0,pi/2]

			// Transcribe the CoM ray vector into a working copy
			Vec3 ray = CoMRay;

			// Normalise the CoM ray vector
			NormaliseVec(ray, Tol, Vec3(0.0, 0.0, -1.0));

			// Establish the soft bound on the sin of the shoulder roll
			double phimax = coerce(shoulderRollMax, Tol, M_PI_2);
			double phibuf = coerce(shoulderRollBuf, 0.0, phimax);
			double abssphimax = sin(phimax);
			double abssphibuf = coerce(abssphimax - sin(phimax - phibuf), sin(Tol), abssphimax);

			// Calculate the maximum allowed value of alpha, motivated by kinematics and feasibility
			double cdalphanom = cos(2.0*alphaNom);
			double pznom = -(3.0 + cdalphanom) / sqrt(10.0 + 6.0*cdalphanom);
			double abssphinom = fabs(-ray.y() / pznom);
			double alpha = alphaNom;
			if(abssphinom > abssphimax - abssphibuf) // Need to adjust alpha...
			{
				double abssphi = coerceSoftMax(abssphinom, abssphimax, abssphibuf); // Numerically abssphi > 0
				double K = fabs(ray.y() / abssphi); // We wish to solve for fabs(pz) >= K
				double discr = 9.0*K*K - 8.0;
				double calphaMax = (discr >= 0.0 ? 3.0*(K*K - 1.0) + K*sqrt(discr) : -1.0/3.0);
				alpha = 0.5*acos(coerceAbs(calphaMax, 1.0)); // The coercion in this line deals with K > 1
			}

			// Precalculate dependent alpha terms
			double dalpha = 2.0*alpha;
			double cdalpha = cos(dalpha);
			double sdalpha = sin(dalpha);
			double pnorm = sqrt(10.0 + 6.0*cdalpha); // Always strictly positive (non-zero) and well-defined
			double px = sdalpha / pnorm;
			double pz = -(3.0 + cdalpha) / pnorm; // Always strictly negative (non-zero)

			// Calculate the required elbow pitch
			JAP.elbowPitch = -dalpha;

			// Calculate the required shoulder roll
			double sphi = coerceAbs(-ray.y() / pz, 1.0); // Note that pz is non-zero, and that the coercion is analytically not required because calphaMax should guarantee that abs(ray.y() / pz) <= 1
			JAP.shoulderRoll = asin(sphi); // We take the solution to shoulder roll in the range [-pi/2,pi/2], instead of pi - shoulderRoll, to ensure the elbow is pointing in the correct logical direction

			// Calculate the required shoulder pitch
			double cphi = cos(JAP.shoulderRoll);
			if(fabs(px) < Tol && fabs(cphi) < Tol) // Note that pz ~= -1 if px ~= 0
				JAP.shoulderPitch = 0.0;
			else if(fabs(ray.x()) < Tol && fabs(ray.z()) < Tol) // Condition that ray = [0 +-1 0]
				JAP.shoulderPitch = 0.0;
			else
				JAP.shoulderPitch = picutMax(atan2(ray.x(), ray.z()) - atan2(px, cphi*pz), JointArmPose::ShoulderPitchMax);

			// Note: The final CoM is (0.25*pnorm) * ray
		}

		//
		// Head pose conversions
		//

		// Conversion: Angles --> Head rotation
		void SerialKinematics::HeadRotFromAngles(double angleZ, double angleY, rot_conv::Quat& headRot) const
		{
			// Precalculate trigonometric values
			double hcz = cos(0.5*angleZ);
			double hsz = sin(0.5*angleZ);
			double hcy = cos(0.5*angleY);
			double hsy = sin(0.5*angleY);

			// Calculate the required head rotation
			headRot.w() = hcy*hcz;
			headRot.x() = -hsy*hsz;
			headRot.y() = hcz*hsy;
			headRot.z() = hcy*hsz;
		}

		// Conversion: Angles --> Direction vector
		void SerialKinematics::DirnVecFromAngles(double angleZ, double angleY, rot_conv::Vec3& dirnVec) const
		{
			// Precalculate trigonometric values
			double cy = cos(angleY);

			// Calculate the required direction vector
			dirnVec.x() = cy*cos(angleZ);
			dirnVec.y() = cy*sin(angleZ);
			dirnVec.z() = -sin(angleY);
		}

		// Conversion: Direction vector --> Angles
		void SerialKinematics::AnglesFromDirnVec(const rot_conv::Vec3& dirnVec, double& angleZ, double& angleY) const
		{
			// Calculate the required angles
			angleZ = atan2(dirnVec.y(), dirnVec.x());
			angleY = atan2(-dirnVec.z(), sqrt(dirnVec.x()*dirnVec.x() + dirnVec.y()*dirnVec.y())); // In [-pi/2,pi/2]
		}

		// Conversion: Joint --> Abstract
		void SerialKinematics::AbsFromJoint(const JointHeadPose& JHP, AbsHeadPose& AHP) const
		{
			// Transcribe the elements of the greatest common base class
			AHP.HeadPose::operator=(JHP);

			// Convert between the poses as required
			AHP.angleY = JHP.neckPitch;
			AHP.angleZ = JHP.neckYaw;
		}

		// Conversion: Joint --> Inverse
		void SerialKinematics::InvFromJoint(const JointHeadPose& JHP, InvHeadPose& IHP) const
		{
			// Transcribe the elements of the greatest common base class
			IHP.HeadPose::operator=(JHP);

			// Calculate the required head pose
			DirnVecFromAngles(JHP.neckYaw, JHP.neckPitch, IHP.dirnVec);
			HeadRotFromAngles(JHP.neckYaw, JHP.neckPitch, IHP.headRot);
		}

		// Conversion: Joint --> Tip
		void SerialKinematics::TipFromJoint(const JointHeadPose& JHP, HeadTipPose& HTP) const
		{
			// Transcribe the elements of the greatest common base class
			HTP.HeadPose::operator=(JHP);

			// Calculate the required head pose
			HTP.pos.setZero();
			HeadRotFromAngles(JHP.neckYaw, JHP.neckPitch, HTP.rot);
		}

		// Conversion: Abstract --> Joint
		void SerialKinematics::JointFromAbs(const AbsHeadPose& AHP, JointHeadPose& JHP) const
		{
			// Transcribe the elements of the greatest common base class
			JHP.HeadPose::operator=(AHP);

			// Convert between the poses as required
			JHP.neckYaw = AHP.angleZ;
			JHP.neckPitch = AHP.angleY;
		}

		// Conversion: Abstract --> Inverse
		void SerialKinematics::InvFromAbs(const AbsHeadPose& AHP, InvHeadPose& IHP) const
		{
			// Transcribe the elements of the greatest common base class
			IHP.HeadPose::operator=(AHP);

			// Calculate the required head pose
			DirnVecFromAngles(AHP.angleZ, AHP.angleY, IHP.dirnVec);
			HeadRotFromAngles(AHP.angleZ, AHP.angleY, IHP.headRot);
		}

		// Conversion: Abstract --> Tip
		void SerialKinematics::TipFromAbs(const AbsHeadPose& AHP, HeadTipPose& HTP) const
		{
			// Transcribe the elements of the greatest common base class
			HTP.HeadPose::operator=(AHP);

			// Calculate the required head pose
			HTP.pos.setZero();
			HeadRotFromAngles(AHP.angleZ, AHP.angleY, HTP.rot);
		}

		// Conversion: Inverse --> Joint
		void SerialKinematics::JointFromInv(const InvHeadPose& IHP, JointHeadPose& JHP) const
		{
			// Transcribe the elements of the greatest common base class
			JHP.HeadPose::operator=(IHP);

			// Calculate the required head pose
			AnglesFromDirnVec(IHP.dirnVec, JHP.neckYaw, JHP.neckPitch);
		}

		// Conversion: Inverse --> Abstract
		void SerialKinematics::AbsFromInv(const InvHeadPose& IHP, AbsHeadPose& AHP) const
		{
			// Transcribe the elements of the greatest common base class
			AHP.HeadPose::operator=(IHP);

			// Calculate the required head pose
			AnglesFromDirnVec(IHP.dirnVec, AHP.angleZ, AHP.angleY);
		}

		// Conversion: Inverse --> Joint and Abstract
		void SerialKinematics::JointAbsFromInv(const InvHeadPose& IHP, JointHeadPose& JHP, AbsHeadPose& AHP) const
		{
			// Transcribe the elements of the greatest common base classes
			JHP.HeadPose::operator=(IHP);
			AHP.HeadPose::operator=(IHP);

			// Calculate the required head poses
			AnglesFromDirnVec(IHP.dirnVec, JHP.neckYaw, JHP.neckPitch);
			AnglesFromDirnVec(IHP.dirnVec, AHP.angleZ, AHP.angleY);
		}

		// Conversion: Inverse --> Tip
		void SerialKinematics::TipFromInv(const InvHeadPose& IHP, HeadTipPose& HTP) const
		{
			// Transcribe the elements of the greatest common base class
			HTP.HeadPose::operator=(IHP);

			// Set the tip position and rotation
			HTP.pos.setZero();
			InvHeadPose::RotFromVec(IHP.dirnVec, HTP.rot);
		}

		// Conversion: Tip --> Joint
		void SerialKinematics::JointFromTip(const HeadTipPose& HTP, JointHeadPose& JHP) const
		{
			// Transcribe the elements of the greatest common base class
			JHP.HeadPose::operator=(HTP);

			// Calculate the required head pose
			Vec3 dirnVec = InvHeadPose::VecFromRot(HTP.rot);
			AnglesFromDirnVec(dirnVec, JHP.neckYaw, JHP.neckPitch);
		}

		// Conversion: Tip --> Abstract
		void SerialKinematics::AbsFromTip(const HeadTipPose& HTP, AbsHeadPose& AHP) const
		{
			// Transcribe the elements of the greatest common base class
			AHP.HeadPose::operator=(HTP);

			// Calculate the required head pose
			Vec3 dirnVec = InvHeadPose::VecFromRot(HTP.rot);
			AnglesFromDirnVec(dirnVec, AHP.angleZ, AHP.angleY);
		}

		// Conversion: Tip --> Inverse
		void SerialKinematics::InvFromTip(const HeadTipPose& HTP, InvHeadPose& IHP) const
		{
			// Transcribe the elements of the greatest common base class
			IHP.HeadPose::operator=(HTP);

			// Calculate the required head pose
			InvHeadPose::VecFromRot(HTP.rot, IHP.dirnVec);
			InvHeadPose::RotFromVec(IHP.dirnVec, IHP.headRot);
		}

		//
		// Robot pose conversions
		//

		// Conversion: Joint --> Abstract
		void SerialKinematics::AbsFromJoint(const JointPose& JP, AbsPose& AP) const
		{
			// Perform the required conversion on all limbs
			AbsFromJoint(JP.legL, AP.legL);
			AbsFromJoint(JP.legR, AP.legR);
			AbsFromJoint(JP.armL, AP.armL);
			AbsFromJoint(JP.armR, AP.armR);
			AbsFromJoint(JP.head, AP.head);
		}

		// Conversion: Joint --> Inverse
		void SerialKinematics::InvFromJoint(const JointPose& JP, InvPose& IP) const
		{
			// Perform the required conversion on all limbs
			InvFromJoint(JP.legL, IP.legL);
			InvFromJoint(JP.legR, IP.legR);
			InvFromJoint(JP.armL, IP.armL);
			InvFromJoint(JP.armR, IP.armR);
			InvFromJoint(JP.head, IP.head);
		}

		// Conversion: Joint --> Tip
		void SerialKinematics::TipFromJoint(const JointPose& JP, TipPose& TP) const
		{
			// Perform the required conversion on all limbs
			TipFromJoint(JP.legL, TP.legL);
			TipFromJoint(JP.legR, TP.legR);
			TipFromJoint(JP.armL, TP.armL);
			TipFromJoint(JP.armR, TP.armR);
			TipFromJoint(JP.head, TP.head);
		}

		// Conversion: Abstract --> Joint
		void SerialKinematics::JointFromAbs(const AbsPose& AP, JointPose& JP) const
		{
			// Perform the required conversion on all limbs
			JointFromAbs(AP.legL, JP.legL);
			JointFromAbs(AP.legR, JP.legR);
			JointFromAbs(AP.armL, JP.armL);
			JointFromAbs(AP.armR, JP.armR);
			JointFromAbs(AP.head, JP.head);
		}

		// Conversion: Abstract --> Inverse
		void SerialKinematics::InvFromAbs(const AbsPose& AP, InvPose& IP) const
		{
			// Perform the required conversion on all limbs
			InvFromAbs(AP.legL, IP.legL);
			InvFromAbs(AP.legR, IP.legR);
			InvFromAbs(AP.armL, IP.armL);
			InvFromAbs(AP.armR, IP.armR);
			InvFromAbs(AP.head, IP.head);
		}

		// Conversion: Abstract --> Tip
		void SerialKinematics::TipFromAbs(const AbsPose& AP, TipPose& TP) const
		{
			// Perform the required conversion on all limbs
			TipFromAbs(AP.legL, TP.legL);
			TipFromAbs(AP.legR, TP.legR);
			TipFromAbs(AP.armL, TP.armL);
			TipFromAbs(AP.armR, TP.armR);
			TipFromAbs(AP.head, TP.head);
		}

		// Conversion: Inverse --> Joint
		void SerialKinematics::JointFromInv(const InvPose& IP, JointPose& JP) const
		{
			// Perform the required conversion on all limbs
			JointFromInv(IP.legL, JP.legL);
			JointFromInv(IP.legR, JP.legR);
			JointFromInv(IP.armL, JP.armL);
			JointFromInv(IP.armR, JP.armR);
			JointFromInv(IP.head, JP.head);
		}

		// Conversion: Inverse --> Abstract
		void SerialKinematics::AbsFromInv(const InvPose& IP, AbsPose& AP) const
		{
			// Perform the required conversion on all limbs
			AbsFromInv(IP.legL, AP.legL);
			AbsFromInv(IP.legR, AP.legR);
			AbsFromInv(IP.armL, AP.armL);
			AbsFromInv(IP.armR, AP.armR);
			AbsFromInv(IP.head, AP.head);
		}

		// Conversion: Inverse --> Joint and Abstract
		void SerialKinematics::JointAbsFromInv(const InvPose& IP, JointPose& JP, AbsPose& AP) const
		{
			// Perform the required conversion on all limbs
			JointAbsFromInv(IP.legL, JP.legL, AP.legL);
			JointAbsFromInv(IP.legR, JP.legR, AP.legR);
			JointAbsFromInv(IP.armL, JP.armL, AP.armL);
			JointAbsFromInv(IP.armR, JP.armR, AP.armR);
			JointAbsFromInv(IP.head, JP.head, AP.head);
		}

		// Conversion: Inverse --> Tip
		void SerialKinematics::TipFromInv(const InvPose& IP, TipPose& TP) const
		{
			// Perform the required conversion on all limbs
			TipFromInv(IP.legL, TP.legL);
			TipFromInv(IP.legR, TP.legR);
			TipFromInv(IP.armL, TP.armL);
			TipFromInv(IP.armR, TP.armR);
			TipFromInv(IP.head, TP.head);
		}

		// Conversion: Tip --> Joint
		void SerialKinematics::JointFromTip(const TipPose& TP, JointPose& JP) const
		{
			// Perform the required conversion on all limbs
			JointFromTip(TP.legL, JP.legL);
			JointFromTip(TP.legR, JP.legR);
			JointFromTip(TP.armL, JP.armL);
			JointFromTip(TP.armR, JP.armR);
			JointFromTip(TP.head, JP.head);
		}

		// Conversion: Tip --> Abstract
		void SerialKinematics::AbsFromTip(const TipPose& TP, AbsPose& AP) const
		{
			// Perform the required conversion on all limbs
			AbsFromTip(TP.legL, AP.legL);
			AbsFromTip(TP.legR, AP.legR);
			AbsFromTip(TP.armL, AP.armL);
			AbsFromTip(TP.armR, AP.armR);
			AbsFromTip(TP.head, AP.head);
		}

		// Conversion: Tip --> Inverse
		void SerialKinematics::InvFromTip(const TipPose& TP, InvPose& IP) const
		{
			// Perform the required conversion on all limbs
			InvFromTip(TP.legL, IP.legL);
			InvFromTip(TP.legR, IP.legR);
			InvFromTip(TP.armL, IP.armL);
			InvFromTip(TP.armR, IP.armR);
			InvFromTip(TP.head, IP.head);
		}

		//
		// Leg pose velocity conversions
		//

		// Conversion: Joint --> Abstract
		void SerialKinematics::AbsFromJointJacobHelper(double A, LegJacobian& LJ) const
		{
			// Calculate the required Jacobian
			LJ.setZero();
			LJ.coeffRef(0,1) = LJ.coeffRef(1,2) = LJ.coeffRef(2,0) = LJ.coeffRef(3,1) = LJ.coeffRef(3,5) = LJ.coeffRef(4,2) = LJ.coeffRef(4,3) = LJ.coeffRef(4,4) = 1.0;
			LJ.coeffRef(1,3) = 0.5;
			LJ.coeffRef(5,3) = A;
		}

		// Conversion: Joint --> Abstract
		void SerialKinematics::AbsFromJointVelHelper(const JointLegPoseVel& JLPV, double A, AbsLegPoseVel& ALPV) const
		{
			// Calculate the required abstract velocity
			ALPV.angleXVel = JLPV.hipRollVel;
			ALPV.angleYVel = JLPV.hipPitchVel + 0.5*JLPV.kneePitchVel;
			ALPV.angleZVel = JLPV.hipYawVel;
			ALPV.footAngleXVel = JLPV.hipRollVel + JLPV.ankleRollVel;
			ALPV.footAngleYVel = JLPV.hipPitchVel + JLPV.kneePitchVel + JLPV.anklePitchVel;
			ALPV.retractionVel = A * JLPV.kneePitchVel;
		}

		// Conversion: Joint --> Inverse
		void SerialKinematics::InvFromJointJacobHelper(const JointLegPose& JLP, LegJacobian& LJ, rot_conv::Vec3& PA, rot_conv::Rotmat& RF) const
		{
			// Calculate the kinematic coordinate frame orientations
			Rotmat RY = RotmatFromAxis(Z_AXIS, JLP.hipYaw);
			Rotmat RK = RY * RotmatFromAxis(X_AXIS, JLP.hipRoll) * RotmatFromAxis(Y_AXIS, JLP.hipPitch);
			Rotmat RS = RK * RotmatFromAxis(Y_AXIS, JLP.kneePitch);
			RF = RS * RotmatFromAxis(Y_AXIS, JLP.anklePitch) * RotmatFromAxis(X_AXIS, JLP.ankleRoll);

			// Calculate the required joint axis vectors
			Vec3 axishy = VecUnitZ(); // zH
			Vec3 axishr = RY.col(0);  // xY
			Vec3 axisp = RK.col(1);   // yK
			Vec3 axisar = RF.col(0);  // xF

			// Calculate the relative frame offsets
			Vec3 PKA = (-sconfig.LL) * RS.col(2);
			Vec3 PYA = PKA + (-sconfig.LL) * RK.col(2);
			Vec3 PHA = PYA + RY*sconfig.hipOffset[JLP.limbIndex];

			// Calculate the ankle position
			PA = PHA + sconfig.hipPos[JLP.limbIndex];

			// Calculate the required Jacobian
			Vec3 zero = Vec3::Zero();
			LJ << axishy.cross(PHA), axishr.cross(PYA), axisp.cross(PYA), axisp.cross(PKA), zero, zero,
			      axishy, axishr, axisp, axisp, axisp, axisar;
		}

		// Conversion: Joint --> Tip
		void SerialKinematics::TipFromJointJacobHelper(const JointLegPose& JLP, LegJacobian& LJ, rot_conv::Vec3& PF, rot_conv::Rotmat& RF) const
		{
			// Calculate the kinematic coordinate frame orientations
			Rotmat RY = RotmatFromAxis(Z_AXIS, JLP.hipYaw);
			Rotmat RK = RY * RotmatFromAxis(X_AXIS, JLP.hipRoll) * RotmatFromAxis(Y_AXIS, JLP.hipPitch);
			Rotmat RS = RK * RotmatFromAxis(Y_AXIS, JLP.kneePitch);
			RF = RS * RotmatFromAxis(Y_AXIS, JLP.anklePitch) * RotmatFromAxis(X_AXIS, JLP.ankleRoll);

			// Calculate the required joint axis vectors
			Vec3 axishy = VecUnitZ(); // zH
			Vec3 axishr = RY.col(0);  // xY
			Vec3 axisp = RK.col(1);   // yK
			Vec3 axisar = RF.col(0);  // xF

			// Calculate the relative frame offsets
			Vec3 PAF = RF * sconfig.footOffset[JLP.limbIndex];
			Vec3 PKF = PAF + (-sconfig.LL) * RS.col(2);
			Vec3 PYF = PKF + (-sconfig.LL) * RK.col(2);
			Vec3 PHF = PYF + RY*sconfig.hipOffset[JLP.limbIndex];

			// Calculate the foot position
			PF = PHF + sconfig.hipPos[JLP.limbIndex];

			// Calculate the required Jacobian
			LJ << axishy.cross(PHF), axishr.cross(PYF), axisp.cross(PYF), axisp.cross(PKF), axisp.cross(PAF), axisar.cross(PAF),
			      axishy, axishr, axisp, axisp, axisp, axisar;
		}

		// Conversion: Abstract --> Joint
		void SerialKinematics::JointFromAbsJacobHelper(double A, LegJacobian& LJ) const
		{
			// Calculate the required Jacobian
			LJ.setZero();
			LJ.coeffRef(0,2) = LJ.coeffRef(1,0) = LJ.coeffRef(2,1) = LJ.coeffRef(4,4) = LJ.coeffRef(5,3) = 1.0;
			LJ.coeffRef(4,1) = LJ.coeffRef(5,0) = -1.0;
			LJ.coeffRef(2,5) = LJ.coeffRef(4,5) = -A;
			LJ.coeffRef(3,5) = 2.0*A;
		}

		// Conversion: Abstract --> Joint
		void SerialKinematics::JointFromAbsVelHelper(const AbsLegPoseVel& ALPV, double A, JointLegPoseVel& JLPV) const
		{
			// Calculate the required joint velocity
			double AVel = A * ALPV.retractionVel;
			JLPV.hipYawVel = ALPV.angleZVel;
			JLPV.hipRollVel = ALPV.angleXVel;
			JLPV.hipPitchVel = ALPV.angleYVel - AVel;
			JLPV.kneePitchVel = 2.0*AVel;
			JLPV.anklePitchVel = ALPV.footAngleYVel - ALPV.angleYVel - AVel;
			JLPV.ankleRollVel = ALPV.footAngleXVel - ALPV.angleXVel;
		}

		// Conversion: Abstract --> Inverse
		void SerialKinematics::InvFromAbsJacobHelper(const AbsLegPose& ALP, LegJacobian& LJ, rot_conv::Vec3& PA, rot_conv::Rotmat& RF) const
		{
			// Calculate the kinematic coordinate frame orientations
			Rotmat RY = RotmatFromAxis(Z_AXIS, ALP.angleZ);
			Rotmat RL = RY * RotmatFromAxis(X_AXIS, ALP.angleX) * RotmatFromAxis(Y_AXIS, ALP.angleY);
			RF = RL * RotmatFromAxis(Y_AXIS, ALP.footAngleY - ALP.angleY) * RotmatFromAxis(X_AXIS, ALP.footAngleX - ALP.angleX);

			// Calculate the required abstract axis vectors
			Vec3 axisz = VecUnitZ();  // zH
			Vec3 axisx = RY.col(0);   // xY
			Vec3 axisy = RL.col(1);   // yL
			Vec3 axisfx = RF.col(0);  // xF
			Vec3 axisret = RL.col(2); // zL

			// Calculate the relative frame offsets
			Vec3 PYA = (sconfig.LLdbl*(ALP.retraction - 1.0)) * axisret;
			Vec3 PHA = PYA + RY*sconfig.hipOffset[ALP.limbIndex];

			// Calculate the ankle position
			PA = PHA + sconfig.hipPos[ALP.limbIndex];

			// Calculate the required Jacobian
			Vec3 zero = Vec3::Zero();
			LJ << axisx.cross(PYA), axisy.cross(PYA), axisz.cross(PHA), zero, zero, sconfig.LLdbl*axisret,
			      axisx - axisfx, zero, axisz, axisfx, axisy, zero;
		}

		// Conversion: Abstract --> Tip
		void SerialKinematics::TipFromAbsJacobHelper(const AbsLegPose& ALP, LegJacobian& LJ, rot_conv::Vec3& PF, rot_conv::Rotmat& RF) const
		{
			// Calculate the kinematic coordinate frame orientations
			Rotmat RY = RotmatFromAxis(Z_AXIS, ALP.angleZ);
			Rotmat RL = RY * RotmatFromAxis(X_AXIS, ALP.angleX) * RotmatFromAxis(Y_AXIS, ALP.angleY);
			RF = RL * RotmatFromAxis(Y_AXIS, ALP.footAngleY - ALP.angleY) * RotmatFromAxis(X_AXIS, ALP.footAngleX - ALP.angleX);

			// Calculate the required abstract axis vectors
			Vec3 axisz = VecUnitZ();  // zH
			Vec3 axisx = RY.col(0);   // xY
			Vec3 axisy = RL.col(1);   // yL
			Vec3 axisfx = RF.col(0);  // xF
			Vec3 axisret = RL.col(2); // zL

			// Calculate the relative frame offsets
			Vec3 PYA = (sconfig.LLdbl*(ALP.retraction - 1.0)) * axisret;
			Vec3 PAF = RF * sconfig.footOffset[ALP.limbIndex];
			Vec3 PYF = PYA + PAF;
			Vec3 PHF = PYF + RY*sconfig.hipOffset[ALP.limbIndex];

			// Calculate the foot position
			PF = PHF + sconfig.hipPos[ALP.limbIndex];

			// Calculate the required Jacobian
			Vec3 zero = Vec3::Zero();
			LJ << axisx.cross(PYF) - axisfx.cross(PAF), axisy.cross(PYA), axisz.cross(PHF), axisfx.cross(PAF), axisy.cross(PAF), sconfig.LLdbl*axisret,
			      axisx - axisfx, zero, axisz, axisfx, axisy, zero;
		}

		// Conversion: Inverse --> Tip
		void SerialKinematics::TipFromInvJacob(const rot_conv::Quat& footRot, LimbIndex limbIndex, LegJacobian& LJ) const
		{
			// Calculate the relative frame offset
			Vec3 PAF = QuatRotVec(footRot, sconfig.footOffset[ensureLimbIndex(limbIndex)]);

			// Calculate the required Jacobian
			LJ.setIdentity();
			LJ.topRightCorner<3,3>() << 0.0, PAF.z(), -PAF.y(),
			                            -PAF.z(), 0.0, PAF.x(),
			                            PAF.y(), -PAF.x(), 0.0;
		}

		// Conversion: Inverse --> Tip
		void SerialKinematics::TipFromInvVel(const InvLegPoseVel& ILPV, const rot_conv::Quat& footRot, LimbIndex limbIndex, LegTipPoseVel& LTPV) const
		{
			// Calculate the relative frame offset
			Vec3 PAF = QuatRotVec(footRot, sconfig.footOffset[ensureLimbIndex(limbIndex)]);

			// Calculate the required tip velocity
			LTPV.vel = ILPV.ankleVel + ILPV.footAngVel.cross(PAF);
			LTPV.angVel = ILPV.footAngVel;
		}

		// Conversion: Tip --> Inverse
		void SerialKinematics::InvFromTipJacob(const rot_conv::Quat& rot, LimbIndex limbIndex, LegJacobian& LJ) const
		{
			// Calculate the relative frame offset
			Vec3 PAF = QuatRotVec(rot, sconfig.footOffset[ensureLimbIndex(limbIndex)]);

			// Calculate the required Jacobian
			LJ.setIdentity();
			LJ.topRightCorner<3,3>() << 0.0, -PAF.z(), PAF.y(),
			                            PAF.z(), 0.0, -PAF.x(),
			                            -PAF.y(), PAF.x(), 0.0;
		}

		// Conversion: Tip --> Inverse
		void SerialKinematics::InvFromTipVel(const LegTipPoseVel& LTPV, const rot_conv::Quat& rot, LimbIndex limbIndex, InvLegPoseVel& ILPV) const
		{
			// Calculate the relative frame offset
			Vec3 PAF = QuatRotVec(rot, sconfig.footOffset[ensureLimbIndex(limbIndex)]);

			// Calculate the required tip velocity
			ILPV.ankleVel = LTPV.vel - LTPV.angVel.cross(PAF);
			ILPV.footAngVel = LTPV.angVel;
		}
	}
}

// ###############################################
// #### Quaternions from sequential rotations ####
// ###############################################

// Quaternion from sequential Y-X rotations
void QuatFromYX(double angY, double angX, Quat& qYX)
{
	// Precompute angles
	double hangY = 0.5*angY;
	double hangX = 0.5*angX;

	// Precompute trigonometric values
	double cy = cos(hangY);
	double sy = sin(hangY);
	double cx = cos(hangX);
	double sx = sin(hangX);

	// Compute the output quaternion
	qYX.w() = cx*cy;
	qYX.x() = sx*cy;
	qYX.y() = cx*sy;
	qYX.z() = -sx*sy;
}

// Quaternion from sequential Y-X-E rotations (E is a second Y rotation)
void QuatFromYXE(double angY, double angX, double angE, Quat& qYXE)
{
	// Precompute angles
	double hangY = 0.5*angY;
	double hangX = 0.5*angX;
	double hangE = 0.5*angE;

	// Precompute trigonometric values
	double cy = cos(hangY);
	double sy = sin(hangY);
	double cx = cos(hangX);
	double sx = sin(hangX);
	double ce = cos(hangE);
	double se = sin(hangE);

	// Precompute intermediate values
	double cxcy = cx*cy;
	double sxcy = sx*cy;
	double cxsy = cx*sy;
	double sxsy = sx*sy;

	// Compute the output quaternion
	qYXE.w() = ce*cxcy - se*cxsy;
	qYXE.x() = ce*sxcy + se*sxsy;
	qYXE.y() = ce*cxsy + se*cxcy;
	qYXE.z() = se*sxcy - ce*sxsy;
}

// Quaternion from sequential Y-X-E rotations (E is a second Y rotation)
void QuatFromYXE(double angY, double angX, double angE, Quat& qYXE, Quat& qYX)
{
	// Precompute angles
	double hangY = 0.5*angY;
	double hangX = 0.5*angX;
	double hangE = 0.5*angE;

	// Precompute trigonometric values
	double cy = cos(hangY);
	double sy = sin(hangY);
	double cx = cos(hangX);
	double sx = sin(hangX);
	double ce = cos(hangE);
	double se = sin(hangE);

	// Compute the output quaternion
	qYX.w() = cx*cy;
	qYX.x() = sx*cy;
	qYX.y() = cx*sy;
	qYX.z() = -sx*sy;
	qYXE.w() = ce*qYX.w() - se*qYX.y();
	qYXE.x() = ce*qYX.x() - se*qYX.z();
	qYXE.y() = ce*qYX.y() + se*qYX.w();
	qYXE.z() = ce*qYX.z() + se*qYX.x();
}

// Quaternion from sequential Z-X-Y rotations
void QuatFromZXY(double angZ, double angX, double angY, Quat& qZXY)
{
	// Precompute angles
	double hangZ = 0.5*angZ;
	double hangX = 0.5*angX;
	double hangY = 0.5*angY;

	// Precompute trigonometric values
	double cz = cos(hangZ);
	double sz = sin(hangZ);
	double cx = cos(hangX);
	double sx = sin(hangX);
	double cy = cos(hangY);
	double sy = sin(hangY);

	// Precompute intermediate values
	double czcx = cz*cx;
	double czsx = cz*sx;
	double szcx = sz*cx;
	double szsx = sz*sx;

	// Compute the output quaternions
	qZXY.w() = czcx*cy - szsx*sy;
	qZXY.x() = czsx*cy - szcx*sy;
	qZXY.y() = czcx*sy + cy*szsx;
	qZXY.z() = szcx*cy + czsx*sy;
}

// Quaternion from sequential Z-X-Y-K rotations (K is a second Y rotation)
void QuatFromZXYK(double angZ, double angX, double angY, double angK, Quat& qZXYK, Quat& qZXY)
{
	// Precompute angles
	double hangZ = 0.5*angZ;
	double hangX = 0.5*angX;
	double hangY = 0.5*angY;
	double hangK = 0.5*angK;
	double hangYK = hangY + hangK;

	// Precompute trigonometric values
	double cz = cos(hangZ);
	double sz = sin(hangZ);
	double cx = cos(hangX);
	double sx = sin(hangX);
	double cy = cos(hangY);
	double sy = sin(hangY);
	double cyk = cos(hangYK);
	double syk = sin(hangYK);

	// Precompute intermediate values
	double czcx = cz*cx;
	double czsx = cz*sx;
	double szcx = sz*cx;
	double szsx = sz*sx;

	// Compute the output quaternions
	qZXYK.w() = czcx*cyk - szsx*syk;
	qZXYK.x() = czsx*cyk - szcx*syk;
	qZXYK.y() = czcx*syk + cyk*szsx;
	qZXYK.z() = szcx*cyk + czsx*syk;
	qZXY.w() = czcx*cy - szsx*sy;
	qZXY.x() = czsx*cy - szcx*sy;
	qZXY.y() = czcx*sy + cy*szsx;
	qZXY.z() = szcx*cy + czsx*sy;
}

// #################################################
// #### Inverse leg kinematics helper functions ####
// #################################################

// Calculate the two inverse leg kinematics abstract pose solutions given a solution angleZ
bool InvKinCalcAbsPose(SerialKinematics::AbsLegPose& ALP1, SerialKinematics::AbsLegPose& ALP2, double phiz, const Rotmat& RF, const Vec3& PA, double hx, double hy, double Ldbl)
{
	// Initialise the exact variable
	bool exact = true;

	// Aliases for the foot orientation rotation matrix entries
	double xFx = RF.coeff(0,0);
	double xFy = RF.coeff(1,0);
	double xFz = RF.coeff(2,0);
	double yFx = RF.coeff(0,1);
	double yFy = RF.coeff(1,1);
	double yFz = RF.coeff(2,1);
	double zFx = RF.coeff(0,2);
	double zFy = RF.coeff(1,2);
	double zFz = RF.coeff(2,2);

	// Set phiz
	ALP1.angleZ = phiz;
	double cz = cos(ALP1.angleZ);
	double sz = sin(ALP1.angleZ);

	// Precalculate terms of phiz
	double fphiz = hx - PA.x()*cz - PA.y()*sz;
	double gphiz = PA.y()*cz - PA.x()*sz - hy;

	// Calculate phix
	ALP1.angleX = atan2(gphiz, -PA.z());
	double cx = cos(ALP1.angleX);
	double sx = sin(ALP1.angleX);

	// Calculate phiy
	if(fabs(cx) >= fabs(sx))
	{
		ALP1.angleY = atan2(cx*fphiz, -PA.z());
		if(cx < 0.0)
			ALP1.angleY = picut(ALP1.angleY + M_PI);
	}
	else
	{
		ALP1.angleY = atan2(sx*fphiz, gphiz);
		if(sx < 0.0)
			ALP1.angleY = picut(ALP1.angleY + M_PI);
	}

	// Calculate phify
	ALP1.footAngleY = atan2(xFy*cz*sx - xFz*cx - xFx*sz*sx, xFx*cz + xFy*sz);

	// Calculate phifx
	double czcx = cz*cx;
	double szcx = sz*cx;
	ALP1.footAngleX = picut(ALP1.angleX + atan2(zFx*szcx - zFy*czcx - zFz*sx, yFz*sx + yFy*czcx - yFx*szcx));

	// Calculate ret
	double lambdasq = coerceMin(PA.x()*PA.x() + PA.y()*PA.y() + PA.z()*PA.z() + hx*hx + hy*hy - 2.0*(PA.x()*(hx*cz - hy*sz) + PA.y()*(hx*sz + hy*cz)), 0.0); // This should be trimming eps only...
	ALP1.retraction = 1.0 - sqrt(lambdasq) / Ldbl;
	if(ALP1.retraction < 0.0)
	{
		ALP1.retraction = 0.0;
		exact = false;
	}

	// Construct the second solution
	ALP2.angleX = picut(M_PI + ALP1.angleX);
	ALP2.angleY = picut(M_PI - ALP1.angleY);
	ALP2.angleZ = ALP1.angleZ;
	ALP2.footAngleX = ALP1.footAngleX;
	ALP2.footAngleY = picut(-ALP1.footAngleY); // Yes, the picut is necessary...
	ALP2.retraction = ALP1.retraction;

	// Return whether the solutions are exact
	return exact;
}

// ################################
// #### Joint pose cost struct ####
// ################################

// Calculate the costs of a joint leg pose
void JointPoseCost::setFromPose(const SerialKinematics::JointLegPose& JLP)
{
	// Calculate the relevant components of the cost
	double A = fabs(JLP.hipYaw);
	double B = fabs(JLP.hipRoll);
	double C = fabs(JLP.hipPitch) / 1.5;

	// Calculate the required costs
	maxCost = std::max({A, B, C});
	sumCost = A + B + C;
}

// Comparison operator
bool JointPoseCost::operator<=(const JointPoseCost& other) const
{
	// Constants
	static const double CostTol = 64.0*DBL_EPSILON;

	// Return the required result of the comparison operator
	if(fabs(maxCost - other.maxCost) <= CostTol)
		return sumCost <= other.sumCost;
	else
		return maxCost <= other.maxCost;
}
// EOF