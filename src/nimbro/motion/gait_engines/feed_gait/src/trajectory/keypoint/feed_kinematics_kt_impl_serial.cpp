// Feedback gait serial kinematics for keypoint trajectory generation implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/trajectory/keypoint/feed_kinematics_kt_impl_serial.h>
#include <feed_gait/trajectory/keypoint/feed_keypoint_traj.h>

// Namespaces
using namespace feed_gait;
using namespace feed_gait::keypoint_traj;

//
// FeedKinematicsKT<serial::SerialKinematics> class
//

// Leg step size generation function: Abstract method
template<> void FeedKinematicsKT<serial::SerialKinematics>::generateAbstractStepSizes(const CommonVars& CV, const Vec3& gcv, const AbsPose& baseAP, LimbPhases& limbPhase, LegTipPoints& LTP, MotionCentre& MC, FootYaws& footYaw, NominalFootTilts& nomFootTilt, double& stepHeightDist) const
{
	// Generate the required limb phases
	double D = ktconfig.D; // Note: This is in the range [ktconfig.MinPhaseSep,pi/2]
	double maxPhaseOffset = 0.5*(M_PI - D) - ktconfig.MinPhaseSep;
	double hDE = rc_utils::coerce<double>(ktconfig.phaseOffsetDE(), ktconfig.MinPhaseSep, maxPhaseOffset);
	double hAG = rc_utils::coerce<double>(ktconfig.phaseOffsetAG(), ktconfig.MinPhaseSep, maxPhaseOffset);
	limbPhase[KEY_A] = -M_PI;
	limbPhase[KEY_B] = D - M_PI;
	limbPhase[KEY_N] = 0.5*(D - M_PI);
	limbPhase[KEY_C] = 0.0;
	limbPhase[KEY_D] = D;
	limbPhase[KEY_E] = D + hDE;
	limbPhase[KEY_F] = 0.5*(D + M_PI);
	limbPhase[KEY_G] = M_PI - hAG;

	// Gait command vector variables
	double absGcvX = fabs(gcv.x());
	double absGcvY = fabs(gcv.y());
	double absGcvZ = fabs(gcv.z());

	// Retrieve the required pushout magnitudes for the given gcv
	double legLatPushoutMag = rc_utils::coerce<double>(absGcvX*ktkonfig.asLegLatPushoutMagGradX() + absGcvY*ktkonfig.asLegLatPushoutMagGradY() + absGcvZ*ktkonfig.asLegLatPushoutMagGradZ(), 0.0, ktkonfig.asLegLatPushoutMagMax());
	double legRotVPushoutMag = rc_utils::coerce<double>(absGcvZ*ktkonfig.asLegRotVPushoutMagGradZ(), 0.0, ktkonfig.asLegRotVPushoutMagMax());

	// Retrieve the required hip swing magnitude for the given gcv
	double legLatHipSwingMag = ktkonfig.asLegLatHipSwingMag() + absGcvX*ktkonfig.asLegLatHipSwingMagGradX() + absGcvY*ktkonfig.asLegLatHipSwingMagGradY();

	// Retrieve the required swing magnitudes for the given gcv
	double legSagSwingMag = gcv.x()*ktkonfig.asLegSagSwingMagGradX();
	double legLatSwingMag = gcv.y()*ktkonfig.asLegLatSwingMagGradY();
	double legRotSwingMag = gcv.z()*ktkonfig.asLegRotSwingMagGradZ();

	// Retrieve the required step height for the given gcv
	double legScaleInv = RK.legScaleInv();
	stepHeightDist = legScaleInv * rc_utils::coerce<double>(ktkonfig.asLegStepHeight() + absGcvX*ktkonfig.asLegStepHeightGradX() + absGcvY*ktkonfig.asLegStepHeightGradY(), 0.0, ktkonfig.asLegStepHeightMax());
	if(ktkonfig.asTuningNoLegStepHeight())
		stepHeightDist = 0.0;

	// Retrieve copies of the abstract leg halt poses
	AbsLegPose ALPM[NUM_LR] = {baseAP.leg(hk::INDEX0), baseAP.leg(hk::INDEX1)};

	// Calculate left and right halt and motion centre poses for the legs
	LegTipPoint LTPM[NUM_LR];
	for(LimbIndex l : CA.limbIndices)
	{
		// Retrieve the abstract leg halt pose
		AbsLegPose& ALP = ALPM[l];

		// Calculate the nominal foot tilt
		InvLegPose ILP = K.InvFromAbs(ALP);
		nomFootTilt[l].set(ILP.invRot());

		// Incorporate pushout into the abstract leg pose
		if(!ktkonfig.asTuningNoLegPushout())
		{
			LimbSign ls = hk::limbSignOf(l);
			ALP.angleX += ls * legLatPushoutMag;
			ALP.angleZ += ls * legRotVPushoutMag;
		}

		// Calculate the resulting leg tip point in global coordinates
		LegTipPose LTPose = K.TipFromAbs(ALP);
		LTPM[l] = LTPose.pos + CV.localToGlobal[l];
	}

	// Calculate the motion centre point
	double lambda = rc_utils::interpolateCoerced(LTPM[LEFT].y(), LTPM[RIGHT].y(), 0.0, 1.0, 0.0);
	MC.point = LTPM[LEFT] + lambda*(LTPM[RIGHT] - LTPM[LEFT]);
	MC.point.y() = 0.0;

	// Calculate the unit vector corresponding to the motion centre line
	MC.lineVec = rot_conv::VecSlerp(CV.BzN, -MC.point, ktconfig.legMCLHipCentreRatio()); // Returns a normalised unit vector

	// Calculate the dimensionless swing values
	double legSwing[NUM_KEYS];
	for(int n : CA.keypoints)
	{
		if(n >= KEY_E && n <= KEY_G)
			legSwing[n] = 0.0;
		else
			legSwing[n] = (D - M_PI - 2.0*limbPhase[n]) / (D + M_PI); // 1 => Swing at touchdown (A), -1 => Swing at lift-off (D)
	}

	// Calculate the dimensionless hip swing values
	double dblSuppHipSwing = sin(M_PI*D / (M_PI + D));
	double legHipSwing[NUM_KEYS];
	legHipSwing[KEY_A] = dblSuppHipSwing;
	legHipSwing[KEY_B] = -dblSuppHipSwing;
	legHipSwing[KEY_N] = -1.0;
	legHipSwing[KEY_C] = -dblSuppHipSwing;
	legHipSwing[KEY_D] = dblSuppHipSwing;
	legHipSwing[KEY_E] = 1.0;
	legHipSwing[KEY_F] = 1.0;
	legHipSwing[KEY_G] = 1.0;

	// Calculate the required raw support keypoints
	for(LimbIndex l : CA.limbIndices)
	{
		LimbSign ls = hk::limbSignOf(l);
		for(int n : CA.keypoints)
		{
			// Retrieve a copy of the abstract leg motion centre pose
			AbsLegPose ALP = ALPM[l];

			// Incorporate hip swing into the abstract pose
			if(!ktkonfig.asTuningNoLegHipSwing())
				ALP.angleX += ls * legHipSwing[n] * legLatHipSwingMag;

			// Incorporate swing into the abstract pose
			if(!ktkonfig.asTuningNoLegSwing())
			{
				ALP.angleX += legSwing[n] * legLatSwingMag;
				ALP.angleY -= legSwing[n] * legSagSwingMag;
				ALP.angleZ += legSwing[n] * legRotSwingMag;
			}

			// Calculate the resulting leg tip point in global coordinates
			LegTipPose LTPose = K.TipFromAbs(ALP);
			LTP[l][n] = LTPose.pos + CV.localToGlobal[l];
			footYaw[l][n] = rot_conv::FYawOfQuat(LTPose.rot);
		}
	}
}

// Arm base motion generation function: Swing method
template<> void FeedKinematicsKT<serial::SerialKinematics>::genArmBaseMotionSwing(const CommonVars& CV, const Vec3& gcv, double phase, const LimbPhases& limbPhase, AbsArmPose& AAP) const
{
	// Calculate the required limb phases
	double Dmu = rc_utils::picutMod(phase - limbPhase[KEY_D]);
	double DmuA = rc_utils::coerce(rc_utils::picutMod(limbPhase[KEY_A] - limbPhase[KEY_D]), ktconfig.MinPhaseSep, M_2PI - ktconfig.MinPhaseSep);

	// Calculate the phase-dependent dimensionless swing angle (ranging from -1 to 1)
	double swingAngle;
	if(Dmu <= DmuA)
		swingAngle = -cos(M_PI * Dmu / DmuA);                 // Sinusoid forwards swing from dimensionless angle -1 to +1 (from D to A)
	else
		swingAngle = 1.0 - 2.0*(Dmu - DmuA) / (M_2PI - DmuA); // Linear backwards swing from dimensionless angle +1 to -1 (from A to D)

	// Retrieve the required swing magnitude for the given gcv
	double armSagSwingMag = rc_utils::coerceAbs<double>(ktkonfig.saArmSagSwingMag() + gcv.x()*ktkonfig.saArmSagSwingMagGradX(), ktkonfig.saArmSagSwingMagMax());

	// Apply the sagittal arm swing to the abstract arm pose (that initially contains the halt pose)
	AAP.angleY += -swingAngle * armSagSwingMag;
}

// Calculate final adjustment vector helper function
void calcFinalAdjustLambdaRange(const serial::SerialKinematics::InvLegPose& ILP, double phiz, const Vec3& adjustHat, double lmaxsq, double Ldbl, double hx, double hyl, double& lambdaMin, double& lambdaMax)
{
	// Precalculate values
	double Cz = cos(phiz) - 1.0;
	double sz = sin(phiz);

	// Calculate the hip PR point in local coordinates
	Vec3 h(hx*Cz - hyl*sz, hx*sz + hyl*Cz, Ldbl);

	// Calculate the allowed lambda range that respects the maximum leg length
	Vec3 hpdiff = h - ILP.anklePos;
	double hpdotm = adjustHat.dot(hpdiff);
	Vec3 perp = hpdiff - hpdotm*adjustHat;
	double dsq = perp.x()*perp.x() + perp.y()*perp.y() + perp.z()*perp.z();
	double lambdaH = sqrt(rc_utils::coerceMin(lmaxsq - dsq, 0.0));
	lambdaMin = hpdotm - lambdaH;
	lambdaMax = hpdotm + lambdaH;
}

// Calculate a final adjustment vector to make keypoints respect hip height and other kinematics-specific constraints (hip height is measured from the motion centre point to the hip centre point, projected onto the heightHat unit vector)
template<> Vec3 FeedKinematicsKT<serial::SerialKinematics>::calcFinalAdjustVec(const InvLegPoses& ILP, const MotionCentre& MC, const Vec3& heightHat, double hipHeightMax) const
{
	// Constants
	static const int NumElem = hk::NUM_LR * NUM_KEYS;
	static const int MaxIts = 10;
	static const double Tol = 1e-9;

	// Robot dimensions
	double hx = K.sconfig.hipOffsetX();
	double hy = K.sconfig.hipOffsetY();

	// Calculate the maximum allowed leg length
	double lmax = K.sconfig.LLdbl * (1.0 - konfig.limLegRetSoftMin);
	double lmaxsq = lmax * lmax;

	// Working copy of the inverse leg poses
	InvLegPoses ILPW = ILP;

	// Lambda arrays
	double lambdaMin[hk::NUM_LR][NUM_KEYS] = {{0.0}};
	double lambdaMax[hk::NUM_LR][NUM_KEYS] = {{0.0}};
	double lambdaMinOld[hk::NUM_LR][NUM_KEYS];
	double lambdaMaxOld[hk::NUM_LR][NUM_KEYS];

	// Initialise variables
	double phiz[hk::NUM_LR][NUM_KEYS] = {{0.0}};
	double lambdaOld, lambda = 0.0;
	double bestDelta = INFINITY;
	double bestLambda = 0.0;
	bool converged = false;

	// Numerically solve for a lambda that just satisfies the minimum leg retraction requirement
	for(int k = 0; k < MaxIts; k++)
	{
		// Save the old keypoint lambda ranges
		std::copy(&lambdaMin[0][0], &lambdaMin[0][0] + NumElem, &lambdaMinOld[0][0]);
		std::copy(&lambdaMax[0][0], &lambdaMax[0][0] + NumElem, &lambdaMaxOld[0][0]);
		lambdaOld = lambda;

		// Calculate the keypoint lambda ranges based on the assumed values of hip yaw
		for(LimbIndex l : CA.limbIndices)
		{
			int ls = hk::limbSignOf(l);
			double hyl = ls * hy;
			for(int n : CA.keypoints)
				calcFinalAdjustLambdaRange(ILP[l][n], phiz[l][n], MC.lineVec, lmaxsq, K.sconfig.LLdbl, hx, hyl, lambdaMin[l][n], lambdaMax[l][n]);
		}

		// Calculate the greatest minimum bound on lambda, and the lowest maximum bound
		double lambdaIntMin = -INFINITY;
		double lambdaIntMax = INFINITY;
		for(LimbIndex l : CA.limbIndices)
		{
			for(int n : CA.keypoints)
			{
				if(lambdaMin[l][n] > lambdaIntMin)
					lambdaIntMin = lambdaMin[l][n];
				if(lambdaMax[l][n] < lambdaIntMax)
					lambdaIntMax = lambdaMax[l][n];
			}
		}

		// Calculate the minimum possible value of lambda that satisfies all ranges (if possible)
		if(lambdaIntMin > lambdaIntMax)
			lambda = 0.5*(lambdaIntMin + lambdaIntMax);
		else
			lambda = lambdaIntMin;

		// Check whether the process has converged
		if(k >= 1)
		{
			double maxDeltaLambdaRange = -INFINITY;
			for(LimbIndex l : CA.limbIndices)
			{
				for(int n : CA.keypoints)
				{
					double localMaxDeltaLambdaRange = std::max(fabs(lambdaMin[l][n] - lambdaMinOld[l][n]), fabs(lambdaMax[l][n] - lambdaMaxOld[l][n]));
					if(localMaxDeltaLambdaRange > maxDeltaLambdaRange)
						maxDeltaLambdaRange = localMaxDeltaLambdaRange;
				}
			}
			if(maxDeltaLambdaRange <= bestDelta)
			{
				bestDelta = maxDeltaLambdaRange;
				bestLambda = lambdaOld;
			}
			if(maxDeltaLambdaRange < Tol)
			{
				converged = true;
				break;
			}
		}

		// Update the assumed values of hip yaw
		if(k < MaxIts - 1)
		{
			for(LimbIndex l : CA.limbIndices)
			{
				for(int n : CA.keypoints)
				{
					ILPW[l][n].anklePos = ILP[l][n].anklePos + lambda * MC.lineVec;
					phiz[l][n] = K.HipYawFromInv(ILPW[l][n]);
				}
			}
		}
	}

	// Choose the best seen lambda if the process did not converge
	if(!converged)
		lambda = bestLambda;

	// Calculate the minimum allowed lambda to use based on hip height restrictions
	double lambdaMinHip = (heightHat.dot(-MC.point) - K.sconfig.LTS * rc_utils::coerceMax<double>(ktconfig.legHipHeightNom(), hipHeightMax)) / heightHat.dot(MC.lineVec);

	// Select the final lambda to use
	lambda = rc_utils::coerceMin(lambda, lambdaMinHip);

	// Return the associated adjustment vector
	return lambda * MC.lineVec;
}
// EOF