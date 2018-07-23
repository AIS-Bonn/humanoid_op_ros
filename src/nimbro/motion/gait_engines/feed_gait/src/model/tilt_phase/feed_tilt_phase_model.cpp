// Feedback gait tilt phase model
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/model/tilt_phase/feed_tilt_phase_model.h>

// Namespaces
using namespace feed_gait;
using namespace feed_gait::tilt_phase_model;
using namespace rot_conv;

//
// TPMHelper class
//

// Calculate the restoring deviation tilt relative to N of the robot
Quat TPMHelper::deviationTiltPhaseN(double phaseX, double phaseY, double expectedPhaseX, double expectedPhaseY, double fusedPitchN)
{
	// Constants
	const int MaxIts = 40; // Note: In extreme tests the maximum number of iterations that was required was 12...
	const double PsiTol = 1e-8;

	// Calculate the required quaternions
	Quat qNB = QuatFromAxis(Y_AXIS, fusedPitchN);
	Quat qNG = qNB * QuatFromPhase(-phaseX, -phaseY); // qNG = qNB * qBG
	Quat qPost = QuatFromPhase(expectedPhaseX, expectedPhaseY) * QuatInv(qNB); // qPost = qGE[PR] * qBN (qGE[PR] is the pitch/roll component of qGE)

	// Initialise variables
	double psiE = 0.0;
	double bestPsiE = psiE;
	double psiNCAbs = INFINITY;
	double bestPsiNCAbs = INFINITY;

	// Numerically iterate to find the required deviation tilt
	for(int k = 0; k < MaxIts; k++)
	{
		// Calculate the yaw of the supposed tilt rotation (function of psiE) relative to N
		double psiNC = FYawOfQuat(qNG * QuatFromAxis(Z_AXIS, psiE) * qPost); // qNC = qNG * qGE[Y] * qGE[PR] * qBN (qGE[Y] is the yaw component of qGE)

		// Keep a record of the best yaw for the E frame seen so far
		psiNCAbs = fabs(psiNC);
		if(psiNCAbs <= bestPsiNCAbs)
		{
			bestPsiNCAbs = psiNCAbs;
			bestPsiE = psiE;
		}

		// Estimate a better yaw for the E frame, which is then (hopefully) closer to being a pure tilt rotation of B relative to N
		psiE -= psiNC;

		// See if we have converged to the required solution
		if(psiNCAbs <= PsiTol)
			break;
	}

	// Choose the best seen yaw for the E frame if the process did not converge
	if(psiNCAbs > PsiTol)
		psiE = bestPsiE;

	// Calculate and return the output tilt deviation
	return qNG * QuatFromAxis(Z_AXIS, psiE) * qPost;
}

// Calculate the tilt rotation from N to the required swing ground plane S
TiltPhase2D TPMHelper::swingGroundPlaneN(double phaseX, double phaseY, double expectedPhaseX, double expectedPhaseY, double fusedPitchN)
{
	// Calculate the required quaternions
	Quat qME = QuatFromAxis(Y_AXIS, fusedPitchN);
	Quat qEG = QuatFromPhase(-expectedPhaseX, -expectedPhaseY);
	Quat qGB = QuatFromPhase(phaseX, phaseY);
	Quat qBN = QuatInv(qME);

	// Calculate the rotation from the expected N frame to the current N frame (assuming B, E have the same fused yaw)
	Quat qMN = qME * qEG * qGB * qBN;

	// Calculate and return the required tilt rotation from N to S
	double px, py;
	PhaseFromQuat(qMN, px, py);
	return TiltPhase2D(-px, -py);
}

// Calculate the fused pitch and roll of the robot torso B relative to the swing ground plane S
void TPMHelper::swingGroundPlaneAction(const rot_conv::TiltPhase2D& PNS, double fusedPitchN, double& fusedPitchS, double& fusedRollS)
{
	// Calculate the required actions
	Quat qSN = QuatFromPhase(-PNS.px, -PNS.py);
	Quat qNB = QuatFromAxis(Y_AXIS, fusedPitchN);
	Quat qSB = qSN * qNB;
	FusedFromQuat(qSB, fusedPitchS, fusedRollS);
}
// EOF