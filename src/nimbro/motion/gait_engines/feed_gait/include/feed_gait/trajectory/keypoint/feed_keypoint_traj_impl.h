// Feedback gait keypoint trajectory generation implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_KEYPOINT_TRAJ_IMPL_H
#define FEED_KEYPOINT_TRAJ_IMPL_H

// Includes
#include <feed_gait/trajectory/keypoint/feed_keypoint_traj.h>
#include <Eigen/Core>
#include <Eigen/QR>

// Macros
#define FEED_KEY_TRAJ_PRINTE()       do { using rot_conv::operator<<; std::cout << std::endl; } while(0)
#define FEED_KEY_TRAJ_PRINT(var)     do { using rot_conv::operator<<; std::cout << #var " = " << var << std::endl; } while(0)
#define FEED_KEY_TRAJ_PRINTL(var)    do { using rot_conv::operator<<; for(LimbIndex l : CA.limbIndices) std::cout << #var "[" << l << "] = " << var[l] << std::endl; std::cout << std::endl; } while(0)
#define FEED_KEY_TRAJ_PRINTLN(var)   do { using rot_conv::operator<<; for(LimbIndex l : CA.limbIndices) for(int n : CA.keypoints) std::cout << #var "[" << l << "][" << n << "] = " << var[l][n] << std::endl; std::cout << std::endl; } while(0)
#define FEED_KEY_TRAJ_PRINTLNK(var)  do { using rot_conv::operator<<; for(LimbIndex l : CA.limbIndices) for(int n : CA.keypoints) for(int k : CA.absFields) std::cout << #var "[" << l << "][" << n << "][" << k << "] = " << var[l][n][k] << std::endl; std::cout << std::endl; } while(0)

// Feedback gait namespace
namespace feed_gait
{
	// Keypoint trajectory generation namespace
	namespace keypoint_traj
	{
		//
		// FeedKeypointTraj class
		//

		// Note:
		// - All local coordinate systems and the global coordinate system have x forwards, y left and z upwards relative to the robot trunk (just the origins differ)
		// - The global coordinate system has its origin at the hip centre point
		// - The local coordinate system for each leg has its origin at the corresponding leg cartesian origin (see RobotKinematics origin functions)

		// Initialisation function
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::init()
		{
			// Initialise whether the halt pose is currently being evaluated
			m_evaluatingHaltPose = false;

			// Initialise the trajectory command
			m_cmd.reset(ktconfig.fusedPitchN());

			// Initialise the base pose variables
			updateBasePose();

			// Initialise the halt pose variables
			m_haltPoseAP = m_basePoseAP;
			m_haltPoseJE = m_basePoseJE;
			K.JointFromAbs(m_haltPoseAP).toVector(m_haltPose.pos);
			m_haltPoseJE.toVector(m_haltPose.effort);
			m_haltPose.setSuppCoeff(m_basePoseSC);

			// Initialise the common variables
			updateCommonVars();

			// Initialise the generated trajectory
			for(int n : CA.keypoints)
			{
				m_limbPhase[n] = 0.0;
				for(LimbIndex l : CA.limbIndices)
				{
					for(int k : CA.absFields)
						m_absCoeff[l][n][k].reset();
				}
			}
			m_suppCoeffVars.reset();

			// Initialise the debugging variables
			m_genCount = 0;
			m_evalCount = 0;

			// Evaluate the true halt pose using calls to generate and evaluate
			evaluateHaltPose();
		}

		// Information function
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::getInfo(TrajInfo& trajInfo) const
		{
			// Populate the required information
			trajInfo.dblSuppPhaseLen = ktconfig.D;
			trajInfo.fusedPitchN = ktconfig.fusedPitchN();
			trajInfo.hipHeightMin = ktconfig.legHipHeightMin();
			trajInfo.hipHeightNom = ktconfig.legHipHeightNom();
		}

		// Evaluate gait halt pose function (Note: Internally calls generate and evaluate and thus changes the state of the class!)
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::evaluateHaltPose()
		{
			// Indicate that evaluation of the halt pose if starting
			m_evaluatingHaltPose = true;

			// Suspend plotting
			bool plotterEnabled = m_PM->getEnabled();
			m_PM->disable();

			// Generate the trajectory for the default command (i.e. zero gcv, no corrective actions)
			TrajCommand trajCmd(ktconfig.fusedPitchN());
			generate(trajCmd); // Note: Updates m_limbPhase

			// Calculate the phase values at the middle of each double support phase
			double phaseAB = rc_utils::picutMod(m_limbPhase[KEY_B] - m_limbPhase[KEY_A]);
			double phaseCD = rc_utils::picutMod(m_limbPhase[KEY_D] - m_limbPhase[KEY_C]);
			double phaseABMid = rc_utils::picut(m_limbPhase[KEY_A] + 0.5*phaseAB);
			double phaseCDMid = rc_utils::picut(m_limbPhase[KEY_C] + 0.5*phaseCD);

			// Evaluate the keypoint trajectory at the middle of each double support phase
			PoseCommand poseCmdABMid, poseCmdCDMid;
			evaluate(phaseABMid, poseCmdABMid);
			evaluate(phaseCDMid, poseCmdCDMid);

			// Calculate the halt pose as the average of the two resulting pose commands
			m_haltPose.setMeanOf(poseCmdABMid, poseCmdCDMid);
			JointPose haltPoseJP;
			haltPoseJP.fromVector(m_haltPose.pos);
			K.AbsFromJoint(haltPoseJP, m_haltPoseAP);
			m_haltPoseJE.fromVector(m_haltPose.effort);

			// Resume plotting if required
			if(plotterEnabled)
				m_PM->enable();

			// Indicate that evaluation of the halt pose is over
			m_evaluatingHaltPose = false;
		}

		// Gait halt pose function
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::getHaltPose(PoseCommand& haltPose) const
		{
			// Return the required halt pose
			haltPose = m_haltPose;
		}

		// Trajectory generation function
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::generate(const TrajCommand& trajCmd)
		{
			//
			// General
			//

			// Evaluate the halt pose if required
			if((ktconfig.tuningForceEvalHalt() || ktconfig.tuningNoLegs() || ktconfig.tuningNoLegTrajectory()) && !m_evaluatingHaltPose)
				evaluateHaltPose();

			// Update the generate count
			if(!m_evaluatingHaltPose)
				m_genCount++;

			// Update the trajectory command
			updateTrajCommand(trajCmd);

			// Update the common variables based on the new trajectory command
			updateCommonVars();

			// Update the base pose
			updateBasePose();

			//
			// Leg trajectory
			//

			// Declare variables
			LegTipPoints LTP; // In global coordinates
			MotionCentre MC;  // In global coordinates
			FootYaws footYaw;
			NominalFootTilts nomFootTilt;
			double stepHeightDist;

			// Generate the required step sizes
			generateStepSizes(m_basePoseAP, m_limbPhase, LTP, MC, footYaw, nomFootTilt, stepHeightDist);

			// Declare variables
			LegTipPoints relLTP;
			FootTilts footTilt;

			// Project, rotate, reconcile and adjust the keypoints
			projectKeypoints(LTP, MC, relLTP);
			rotateKeypoints(relLTP);
			reconcileKeypoints(relLTP, footYaw);
			adjustKeypoints(relLTP, MC, nomFootTilt, stepHeightDist, LTP, footYaw, footTilt);

			// Declare variables
			LegTipPointVels LTPVel;
			FootAngVels footAngVel;

			// Calculate the keypoint velocities
			calcSuppLinVel(m_limbPhase, LTP, LTPVel);
			calcSwingLinVel(m_limbPhase, LTP, LTPVel);
			calcAngVel(m_limbPhase, footYaw, footTilt, footAngVel);

			// Declare variables
			InvLegPoses ILP = {{hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0},
			                   {hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1}};

			// Consolidate, lean and shift the keypoints
			consolidateKeypoints(LTP, footYaw, footTilt, ILP);
			leanKeypoints(ILP, MC, LTPVel, footAngVel);
			shiftKeypoints(ILP, MC);

			// Declare variables
			AbsLegPoses ALP = {{hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0, hk::INDEX0},
			                   {hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1, hk::INDEX1}};
			bool exactKin;

			// Finalise the keypoints and calculate the final abstract leg space trajectory
			finaliseKeypoints(ILP, ALP, exactKin, MC);
			calcLegTrajectory(m_limbPhase, ALP, LTPVel, footAngVel, m_absCoeff);

			// Calculate the support coefficient variables
			calcSuppCoeffVars(m_limbPhase, m_suppCoeffVars);

			//
			// Debug
			//

			// Printing for debugging purposes
			if(m_genCount % 512 == 1 && ktconfig.debugPrintGenLeg() && !m_evaluatingHaltPose)
			{
				std::ios::fmtflags f(std::cout.flags());
				std::cout << std::fixed << std::setprecision(ktconfig.debugPrintPrecision());
				std::cout << "DEBUG PRINT: Generate Leg" << std::endl;
				FEED_KEY_TRAJ_PRINTE();
				FEED_KEY_TRAJ_PRINT(m_cmd.gcv);
				FEED_KEY_TRAJ_PRINT(m_cmd.fusedPitchS);
				FEED_KEY_TRAJ_PRINT(m_cmd.fusedRollS);
				FEED_KEY_TRAJ_PRINT(m_cmd.footTiltCts);
				FEED_KEY_TRAJ_PRINT(m_cmd.footTiltSupp);
				FEED_KEY_TRAJ_PRINT(m_cmd.swingOut);
				FEED_KEY_TRAJ_PRINT(m_cmd.leanTilt);
				FEED_KEY_TRAJ_PRINT(m_cmd.hipShift);
				FEED_KEY_TRAJ_PRINT(m_cmd.hipHeightMax);
				FEED_KEY_TRAJ_PRINT(m_cmd.armTilt);
				FEED_KEY_TRAJ_PRINTE();
				FEED_KEY_TRAJ_PRINTLN(LTP);
				FEED_KEY_TRAJ_PRINT(MC.point);
				FEED_KEY_TRAJ_PRINT(MC.lineVec);
				FEED_KEY_TRAJ_PRINTE();
				FEED_KEY_TRAJ_PRINTLN(footYaw);
				FEED_KEY_TRAJ_PRINT(nomFootTilt[LEFT].tilt);
				FEED_KEY_TRAJ_PRINT(nomFootTilt[RIGHT].tilt);
				FEED_KEY_TRAJ_PRINT(nomFootTilt[LEFT].absTilt);
				FEED_KEY_TRAJ_PRINT(nomFootTilt[RIGHT].absTilt);
				FEED_KEY_TRAJ_PRINTE();
				FEED_KEY_TRAJ_PRINT(stepHeightDist);
				FEED_KEY_TRAJ_PRINTE();
				FEED_KEY_TRAJ_PRINTLN(relLTP);
				FEED_KEY_TRAJ_PRINTLN(footTilt);
				FEED_KEY_TRAJ_PRINTLN(LTPVel);
				FEED_KEY_TRAJ_PRINTLN(footAngVel);
				FEED_KEY_TRAJ_PRINTLN(ILP);
				FEED_KEY_TRAJ_PRINTLN(ALP);
				FEED_KEY_TRAJ_PRINT(exactKin);
				FEED_KEY_TRAJ_PRINTE();
				FEED_KEY_TRAJ_PRINTLNK(m_absCoeff);
				std::cout.flags(f);
			}
		}

		// Update trajectory command
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::updateTrajCommand(const TrajCommand& trajCmd)
		{
			// Copy out the trajectory command
			m_cmd = trajCmd;

			// Apply the gait command vector prescaler and bias
			m_cmd.gcv.x() *= ktconfig.gcvPrescalerLinVelX();
			m_cmd.gcv.y() *= ktconfig.gcvPrescalerLinVelY();
			m_cmd.gcv.z() *= ktconfig.gcvPrescalerAngVelZ();
			m_cmd.gcv += ktconfig.gcvBias;

			// Limit the maximum magnitude of each gait command vector component
			m_cmd.gcv.x() = rc_utils::coerceAbs<double>(m_cmd.gcv.x(), ktconfig.gcvInternalMaxLinVelX());
			m_cmd.gcv.y() = rc_utils::coerceAbs<double>(m_cmd.gcv.y(), ktconfig.gcvInternalMaxLinVelY());
			m_cmd.gcv.z() = rc_utils::coerceAbs<double>(m_cmd.gcv.z(), ktconfig.gcvInternalMaxAngVelZ());

			// Apply the gait command acceleration prescaler
			m_cmd.gcvLFAcc.x() *= ktconfig.gcvPrescalerLinVelX();
			m_cmd.gcvLFAcc.y() *= ktconfig.gcvPrescalerLinVelY();
			m_cmd.gcvLFAcc.z() *= ktconfig.gcvPrescalerAngVelZ();

			// Disable components of the trajectory command as required by the configuration parameters
			if(!ktconfig.enableGcv())
				m_cmd.gcv.setZero();
			if(!ktconfig.enableGcvAcc())
				m_cmd.gcvLFAcc.setZero();
			if(!ktconfig.enableFusedPitchS())
				m_cmd.fusedPitchS = ktconfig.fusedPitchN();
			if(!ktconfig.enableFusedRollS())
				m_cmd.fusedRollS = 0.0;
			if(!ktconfig.enableFootTilt())
			{
				m_cmd.footTiltCts.setIdentity();
				m_cmd.footTiltSupp.setIdentity();
			}
			if(!ktconfig.enableSwingOut())
				m_cmd.swingOut.setIdentity();
			if(!ktconfig.enableLean())
				m_cmd.leanTilt.setIdentity();
			if(!ktconfig.enableHipXYZ())
			{
				m_cmd.hipShift.setZero();
				m_cmd.hipHeightMax = INFINITY;
			}
			if(!ktconfig.enableArmTilt())
				m_cmd.armTilt.setIdentity();

			// Apply feedforward leaning
			rot_conv::AbsTiltPhase2D leanPhase = rot_conv::AbsPhaseFromAbsYawTilt(m_cmd.leanTilt);
			if(ktconfig.enableLeanFF())
			{
				double feedFwdLeanY = ktconfig.leanFFGainGcvX()*m_cmd.gcv.x() + ktconfig.leanFFGainGcvAbsZ()*fabs(m_cmd.gcv.z()) + ktconfig.leanFFGainGcvAccX()*m_cmd.gcvLFAcc.x();
				leanPhase.py += rc_utils::coerceSoftAbs<double>(feedFwdLeanY, ktconfig.leanFFPhaseMaxAbsY(), ktconfig.leanFFPhaseBuf());
				rot_conv::AbsYawTiltFromAbsPhase(leanPhase, m_cmd.leanTilt);
			}

			// Plotting
			if(m_PM->getEnabled())
			{
				m_PM->plotVec3d (m_cmd.gcv, PM_KT_CMD_GCVINTERNALX);
				m_PM->plotVec3d (m_cmd.gcvLFAcc, PM_KT_CMD_GCVACCINTERNALX);
				m_PM->plotScalar(leanPhase.px, PM_KT_CMD_LEANPHASEX);
				m_PM->plotScalar(leanPhase.py, PM_KT_CMD_LEANPHASEY);
			}
		}

		// Update common variables
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::updateCommonVars()
		{
			// Update the position vectors
			for(LimbIndex l : CA.limbIndices)
			{
				CV.hipPointLocal[l] = RK.hipPoint(l);
				CV.hipPointGlobal[l] = RK.rconfig.Hvec[l];
				CV.hipCentrePointLocal[l] = CV.hipPointLocal[l] - RK.rconfig.Hvec[l];
				CV.localToGlobal[l] = CV.hipPointGlobal[l] - CV.hipPointLocal[l];
			}

			// Update the nominal ground plane rotations
			CV.qNB = rot_conv::QuatFromFused(ktconfig.fusedPitchN(), 0.0);
			rot_conv::QuatInv(CV.qNB, CV.qBN);
			rot_conv::RotmatFromQuat(CV.qBN, CV.RBN);
			rot_conv::AxisZFromRotmat(CV.RBN, CV.BzN);

			// Update the swing ground plane rotations
			Vec3 BzS = rot_conv::ZVecFromFused(m_cmd.fusedPitchS, m_cmd.fusedRollS);
			Vec3 NzS = rot_conv::QuatRotVec(CV.qNB, BzS);
			Quat qSN = rot_conv::QuatFromZVec(NzS);
			rot_conv::QuatInv(qSN, CV.qNS);
			CV.qNS = rot_conv::QuatSlerp(CV.qNS, ktconfig.legPlaneRatioS());
			CV.qBS = CV.qBN * CV.qNS;
			rot_conv::RotmatFromQuat(CV.qBS, CV.RBS);
			rot_conv::AxisXFromRotmat(CV.RBS, CV.BxS);
			rot_conv::AxisYFromRotmat(CV.RBS, CV.ByS);
			rot_conv::AxisZFromRotmat(CV.RBS, CV.BzS);
			CV.qBNS = CV.qBS * CV.qNB;
			rot_conv::RotmatFromQuat(CV.qBNS, CV.RBNS);

			// Update the intermediate ground plane rotations
			CV.qNI = rot_conv::QuatSlerp(CV.qNS, ktconfig.legPlaneRatioI());
			CV.qBI = CV.qBN * CV.qNI;
			rot_conv::RotmatFromQuat(CV.qBI, CV.RBI);
			rot_conv::AxisXFromRotmat(CV.RBI, CV.BxI);
			rot_conv::AxisYFromRotmat(CV.RBI, CV.ByI);
			rot_conv::AxisZFromRotmat(CV.RBI, CV.BzI);
			CV.qBNI = CV.qBI * CV.qNB;
			rot_conv::RotmatFromQuat(CV.qBNI, CV.RBNI);

			// Update the support ground plane rotations
			CV.qNJ = rot_conv::QuatSlerp(CV.qNS, ktconfig.legPlaneRatioJ());
			CV.qBJ = CV.qBN * CV.qNJ;
			rot_conv::AxisZFromQuat(CV.qBJ, CV.BzJ);
			CV.qBNJ = CV.qBJ * CV.qNB;
			rot_conv::RotmatFromQuat(CV.qBNJ, CV.RBNJ);

			// Update the leaned ground plane rotations
			CV.qNL = rot_conv::QuatFromTilt(m_cmd.leanTilt.absTiltAxisAngle, -m_cmd.leanTilt.tiltAngle);
			CV.qBL = CV.qBN * CV.qNL;
			rot_conv::RotmatFromQuat(CV.qBL, CV.RBL);
			rot_conv::AxisZFromRotmat(CV.RBL, CV.BzL);
			rot_conv::QuatInv(CV.qBL, CV.qLB);
			CV.qBNL = CV.qBL * CV.qNB;
			rot_conv::RotmatFromQuat(CV.qBNL, CV.RBNL);
		}

		// Update base pose
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::updateBasePose()
		{
			// Update the base pose with the raw kinematic halt pose (not the same as the true halt pose)
			KI.getHaltPose(m_basePoseAP);
			KI.getHaltPoseEffort(m_basePoseJE);
			KI.getHaltPoseSuppCoeff(m_basePoseSC);
		}

		// Generate step sizes
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::generateStepSizes(const AbsPose& baseAP, LimbPhases& limbPhase, LegTipPoints& LTP, MotionCentre& MC, FootYaws& footYaw, NominalFootTilts& nomFootTilt, double& stepHeightDist) const
		{
			// Use the desired step size generator to generate the required data
			int type = ktconfig.ssgType();
			if(type == SSGT_ABSTRACT)
				KI.generateAbstractStepSizes(CV, m_cmd.gcv, baseAP, limbPhase, LTP, MC, footYaw, nomFootTilt, stepHeightDist);
			else
			{
				ROS_ERROR_THROTTLE(3.0, "Unknown step size generator type (%d) => Using abstract step size generator...", type);
				KI.generateAbstractStepSizes(CV, m_cmd.gcv, baseAP, limbPhase, LTP, MC, footYaw, nomFootTilt, stepHeightDist);
			}
		}

		// Project keypoints
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::projectKeypoints(const LegTipPoints& LTP, const MotionCentre& MC, LegTipPoints& relLTP) const
		{
			// Calculate the required projected keypoints
			for(LimbIndex l : CA.limbIndices)
			{
				for(int n : CA.keypoints)
				{
					relLTP[l][n] = LTP[l][n] - MC.point;
					relLTP[l][n].z() = 0.0;
				}
			}
		}

		// Rotate keypoints
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::rotateKeypoints(LegTipPoints& relLTP) const
		{
			// Calculate the required rotated keypoints
			for(LimbIndex l : CA.limbIndices)
			{
				for(int n : CA.keypoints)
					rot_conv::RotmatRotVecInPlace(CV.RBN, relLTP[l][n]);
			}
		}

		// Reconcile keypoints
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::reconcileKeypoints(LegTipPoints& relLTP, FootYaws& footYaw) const
		{
			// Calculate the required reconciled keypoints
			for(LimbIndex l : CA.limbIndices)
			{
				// Get the limb index of the other leg
				int o = hk::otherLimbIndex(l);

				// Adjust lengths of AC and BD to be equal
				Vec3 AC = relLTP[o][KEY_C] - relLTP[l][KEY_A];
				Vec3 BD = relLTP[o][KEY_D] - relLTP[l][KEY_B];
				double ACnorm = AC.norm();
				double BDnorm = BD.norm();
				double ACdecBDinc = 0.25*(ACnorm - BDnorm);
				if(ACnorm > 0.0 && BDnorm > 0.0)
				{
					Vec3 ACvec = (ACdecBDinc / ACnorm) * AC;
					Vec3 BDvec = (ACdecBDinc / BDnorm) * BD;
					relLTP[l][KEY_A] += ACvec;
					relLTP[l][KEY_B] -= BDvec;
					relLTP[o][KEY_C] -= ACvec;
					relLTP[o][KEY_D] += BDvec;
				}

				// Adjust the relative foot yaws of AC and BD to be equal
				double yawAC = footYaw[o][KEY_C] - footYaw[l][KEY_A];
				double yawBD = footYaw[o][KEY_D] - footYaw[l][KEY_B];
				double yawACdecBDinc = 0.5*(yawAC - yawBD);
				footYaw[o][KEY_C] -= yawACdecBDinc;
				footYaw[l][KEY_B] -= yawACdecBDinc;
			}
		}

		// Adjust keypoints
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::adjustKeypoints(LegTipPoints& relLTP, const MotionCentre& MC, const NominalFootTilts& nomFootTilt, const double& stepHeightDist, LegTipPoints& LTP, FootYaws& footYaw, FootTilts& footTilt) const
		{
			// Tilt rotate the AC, BD and NN segments into the required ground planes
			for(LimbIndex l : CA.limbIndices)
			{
				rot_conv::RotmatRotVecInPlace(CV.RBNS, relLTP[l][KEY_A]);
				rot_conv::RotmatRotVecInPlace(CV.RBNS, relLTP[l][KEY_C]);
				rot_conv::RotmatRotVecInPlace(CV.RBNI, relLTP[l][KEY_B]);
				rot_conv::RotmatRotVecInPlace(CV.RBNI, relLTP[l][KEY_D]);
				rot_conv::RotmatRotVecInPlace(CV.RBNJ, relLTP[l][KEY_N]);
				rot_conv::RotmatRotVecInPlace(CV.RBNJ, relLTP[l][KEY_E]);
				rot_conv::RotmatRotVecInPlace(CV.RBNJ, relLTP[l][KEY_F]);
				rot_conv::RotmatRotVecInPlace(CV.RBNJ, relLTP[l][KEY_G]);
			}

			// Calculate how much to adjust AC relative to BD along the MCL
			double ACAdjustAB = -0.5*CV.BzS.dot(relLTP[LEFT][KEY_A] - relLTP[LEFT][KEY_B] + relLTP[RIGHT][KEY_A] - relLTP[RIGHT][KEY_B]); // How much to S-normal adjust AC to make AB S-neutral (i.e. so that AB makes no S-normal adjustment in gait)
			double ACAdjustCD = -0.5*CV.BzS.dot(relLTP[LEFT][KEY_C] - relLTP[LEFT][KEY_D] + relLTP[RIGHT][KEY_C] - relLTP[RIGHT][KEY_D]); // How much to S-normal adjust AC to make CD S-neutral (i.e. so that CD makes no S-normal adjustment in gait)
			double ACAdjust = ACAdjustAB + ktconfig.legACAdjustABRatio() * (ACAdjustCD - ACAdjustAB); // How much S-normal adjustment of AC is desired (a balance between adjusting the front keypoints AB, and adjusting the back keypoints CD)
			double lambdaAC = ACAdjust / CV.BzS.dot(MC.lineVec); // How much adjustment of AC is required along the MCL to produce the desired amount of S-normal adjustment

			// Calculate how much to further adjust ABCD relative to NN along the MCL
			double ABCDAdjust = -0.25*CV.BzJ.dot(relLTP[LEFT][KEY_B] + relLTP[LEFT][KEY_C] + relLTP[RIGHT][KEY_B] + relLTP[RIGHT][KEY_C] - 2.0*(relLTP[LEFT][KEY_N] + relLTP[RIGHT][KEY_N])); // How much to J-normal adjust ABCD to make BC J-neutral with NN (not accounting for the effect of lambdaAC yet)
			double lambdaABCD = ABCDAdjust / CV.BzJ.dot(MC.lineVec) - 0.5*lambdaAC; // How much adjustment of ABCD is required along the MCL to produce the desired amount of J-normal adjustment (now also accounting for the effect of lambdaAC)

			// Calculate the required final adjustments of AC and BD
			Vec3 ACAdjustMCL = (lambdaABCD + lambdaAC) * MC.lineVec;
			Vec3 BDAdjustMCL = lambdaABCD * MC.lineVec;

			// Apply the required adjustments
			for(LimbIndex l : CA.limbIndices)
			{
				relLTP[l][KEY_A] += ACAdjustMCL;
				relLTP[l][KEY_C] += ACAdjustMCL;
				relLTP[l][KEY_B] += BDAdjustMCL;
				relLTP[l][KEY_D] += BDAdjustMCL;
			}

			// Calculate the finalised adjusted support keypoints
			rot_conv::AbsTiltRot footTiltSupp(m_cmd.footTiltSupp.absTiltAxisAngle, m_cmd.footTiltSupp.tiltAngle); // Torso-fixed specification of foot tilt relative to N
			for(LimbIndex l : CA.limbIndices)
			{
				for(int n : CA.keypoints)
				{
					// Calculate the absolute leg tip point
					LTP[l][n] = MC.point + relLTP[l][n];

					// Retrieve the relevant foot tilts for this keypoint
					double gammaOffset = ktconfig.legFootTiltIsRelRatio() * (footYaw[l][n] - nomFootTilt[l].tilt.fusedYaw); // The required gamma offset to adjust the interpretation of the required foot tilts between torso-fixed and foot-fixed depending on the value of legFootTiltIsRelRatio (0 => Torso-fixed, 1 => Foot-fixed)
					rot_conv::AbsTiltRot footTiltNom(nomFootTilt[l].absTilt.absTiltAxisAngle + gammaOffset, nomFootTilt[l].absTilt.tiltAngle); // Torso-fixed specification of foot tilt relative to N
					rot_conv::AbsTiltRot footTiltCts(m_cmd.footTiltCts.absTiltAxisAngle + gammaOffset, m_cmd.footTiltCts.tiltAngle); // Torso-fixed specification of foot tilt relative to N

					// Calculate the required adjusted foot tilt
					if(n == KEY_B || n == KEY_N || n == KEY_C)
						footTilt[l][n] = rot_conv::AbsTiltRotSum(footTiltNom, footTiltCts, footTiltSupp);
					else
						footTilt[l][n] = rot_conv::AbsTiltRotSum(footTiltNom, footTiltCts);
				}
			}

			// Calculate suitable F keypoints
			for(LimbIndex l : CA.limbIndices)
			{
				// Limb indices and signs
				LimbSign ls = hk::limbSignOf(l);
				LimbIndex o = hk::otherLimbIndex(l);

				// Calculate the nominal F keypoint that incorporates step height
				Vec3 pointF = LTP[l][KEY_F] + ktconfig.legFOverNRatio() * (LTP[l][KEY_N] - LTP[l][KEY_F]);
				LTP[l][KEY_F] = pointF + CV.BzS * rc_utils::coerceMin(stepHeightDist + CV.BzS.dot(LTP[o][KEY_N] - pointF), 0.0); // The F point height is calculated relative to the S plane through the support foot N point

				// Calculate the nominal F keypoint orientation that respects the S plane
				rot_conv::TiltAngles TSF = rot_conv::TiltFromAbsYawTilt(footTilt[l][KEY_F], footYaw[l][KEY_F]); // Reinterpret the foot orientation relative to N at F as being relative to S instead
				Quat qSF = rot_conv::QuatFromTilt(TSF);
				Quat qNF = CV.qNS * qSF;

				// Calculate the swing out rotation
				rot_conv::AbsTiltPhase2D swingOutTiltPhase = rot_conv::AbsPhaseFromAbsYawTilt(m_cmd.swingOut);
				swingOutTiltPhase.px = ls * rc_utils::coerceSoftMin<double>(swingOutTiltPhase.px / ls, -ktconfig.legSwingOutMaxIwd(), ktconfig.legSwingOutBuf());
				rot_conv::AbsTiltAngles swingOutAbsTilt = rot_conv::AbsTiltFromAbsPhase(swingOutTiltPhase);
				rot_conv::TiltAngles swingOutTilt = rot_conv::TiltFromAbsTilt(swingOutAbsTilt);
				Quat qNO = rot_conv::QuatFromTilt(swingOutTilt);
				Quat qBNO = CV.qBN * qNO * CV.qNB;

				// Apply swing out to the F keypoint position
				LTP[l][KEY_F] = CV.hipPointGlobal[l] + rot_conv::QuatRotVec(qBNO, LTP[l][KEY_F] - CV.hipPointGlobal[l]);

				// Apply swing out to the F keypoint rotation
				rot_conv::TiltAngles TNFswung = rot_conv::TiltFromQuat(qNO * qNF);
				rot_conv::AbsYawTiltFromTilt(TNFswung, footTilt[l][KEY_F], footYaw[l][KEY_F]);
			}
		}

		// Calculate support keypoint linear velocities
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::calcSuppLinVel(const LimbPhases& limbPhase, const LegTipPoints& LTP, LegTipPointVels& LTPVel) const
		{
			// Constants
			static const int NumSupp = KEY_D - KEY_A + 1; // Index n: Order is A, B, N, C, D
			static const int NumSuppD = NumSupp - 1;      // Index m: Order is AB, BN, NC, CD
			static const int NumCh = 6;                   // Index c: Order is Lx, Ly, Lz, Rx, Ry, Rz
			static const int NumV = NumSupp * NumCh;      // Index i: Order is vLx(ABNCD), vLy(ABNCD), vLz(ABNCD), vRx(ABNCD), vRy(ABNCD), vRz(ABNCD)
			static const int NumSuppDD = NumSupp - 2;     // Index k

			// Typedefs
			typedef Eigen::Matrix<double, NumV, NumV> MatrixA;
			typedef Eigen::Matrix<double, NumSuppDD, NumSupp> MatrixAsub;
			typedef Eigen::Matrix<double, NumV, 1> Vectorb;
			typedef Eigen::Matrix<double, NumV, 1> Vectorv;

			// Retrieve the input position data
			double linSuppX[NumSupp][NumCh];
			for(int n = 0; n < NumSupp; n++)
			{
				linSuppX[n][0] = LTP[LEFT][KEY_A + n].x();
				linSuppX[n][1] = LTP[LEFT][KEY_A + n].y();
				linSuppX[n][2] = LTP[LEFT][KEY_A + n].z();
				linSuppX[n][3] = LTP[RIGHT][KEY_A + n].x();
				linSuppX[n][4] = LTP[RIGHT][KEY_A + n].y();
				linSuppX[n][5] = LTP[RIGHT][KEY_A + n].z();
			}

			// Retrieve further input data
			double linSuppH[NumSuppD];
			double linSuppVbar[NumSuppD][NumCh];
			for(int m = 0; m < NumSuppD; m++)
			{
				linSuppH[m] = rc_utils::picutMod(limbPhase[m+1] - limbPhase[m]);
				for(int c = 0; c < NumCh; c++)
					linSuppVbar[m][c] = (linSuppX[m+1][c] - linSuppX[m][c]) / linSuppH[m];
			}

			// Initialise the matrix equation that governs the support keypoint velocities (Av = b)
			MatrixA A = MatrixA::Zero();
			Vectorb b = Vectorb::Zero();

			// Apply the conditions of continuous velocity and acceleration at every support keypoint
			MatrixAsub Asub = MatrixAsub::Zero();
			for(int k = 0; k < NumSuppDD; k++)
			{
				Asub.coeffRef(k, k) = linSuppH[k+1];
				Asub.coeffRef(k, k+1) = 2.0*(linSuppH[k] + linSuppH[k+1]);
				Asub.coeffRef(k, k+2) = linSuppH[k];
			}
			for(int c = 0; c < NumCh; c++)
			{
				A.block<NumSuppDD, NumSupp>(NumSuppDD*c, NumSupp*c) = Asub;
				for(int k = 0; k < NumSuppDD; k++)
					b.coeffRef(NumSuppDD*c + k) = 3.0*(linSuppH[k+1] * linSuppVbar[k][c] + linSuppH[k] * linSuppVbar[k+1][c]);
			}
			int curRow = NumSuppDD * NumCh;

			// Make the xS and yS accelerations zero at A
			for(int s = 0; s < 2; s++)
			{
				for(int t = 0; t < 2; t++, curRow++)
				{
					for(int j = 0; j < 3; j++)
					{
						int tj = 3*t + j;
						int index = NumSupp*tj;
						double factor = CV.RBS.coeff(j, s);
						A.coeffRef(curRow, index) = 2.0*factor;
						A.coeffRef(curRow, index + 1) = factor;
						b.coeffRef(curRow) += 3.0*factor*linSuppVbar[0][tj];
					}
				}
			}

			// Make the xI and yI accelerations zero at D
			for(int s = 0; s < 2; s++)
			{
				for(int t = 0; t < 2; t++, curRow++)
				{
					for(int j = 0; j < 3; j++)
					{
						int tj = 3*t + j;
						int index = NumSupp*tj + NumSuppD;
						double factor = CV.RBI.coeff(j, s);
						A.coeffRef(curRow, index) = 2.0*factor;
						A.coeffRef(curRow, index - 1) = factor;
						b.coeffRef(curRow) += 3.0*factor*linSuppVbar[NumSuppDD][tj];
					}
				}
			}

			// Make the mean zS velocity zero at AC
			for(int j = 0; j < 3; j++)
			{
				int indexL = NumSupp*j;
				int indexR = NumSupp*(j + 3);
				double factor = CV.RBS.coeff(j, 2);
				A.coeffRef(curRow, indexL) = factor;
				A.coeffRef(curRow, indexR + NumSuppDD) = factor;
				A.coeffRef(curRow + 1, indexL + NumSuppDD) = factor;
				A.coeffRef(curRow + 1, indexR) = factor;
			}
			curRow += 2;

			// Make the mean zI velocity zero at BD
			for(int j = 0; j < 3; j++)
			{
				int indexL = NumSupp*j + 1;
				int indexR = NumSupp*(j + 3) + 1;
				double factor = CV.RBI.coeff(j, 2);
				A.coeffRef(curRow, indexL) = factor;
				A.coeffRef(curRow, indexR + NumSuppDD) = factor;
				A.coeffRef(curRow + 1, indexL + NumSuppDD) = factor;
				A.coeffRef(curRow + 1, indexR) = factor;
			}
			curRow += 2;

			// Solve the matrix equation that governs the support keypoint velocities (Av = b)
			Vectorv v = A.colPivHouseholderQr().solve(b);

			// Transcribe the calculated support keypoint velocities
			for(int n = 0; n < NumSupp; n++)
			{
				LTPVel[LEFT][KEY_A + n] << v.coeff(n), v.coeff(n + NumSupp), v.coeff(n + 2*NumSupp);
				LTPVel[RIGHT][KEY_A + n] << v.coeff(n + 3*NumSupp), v.coeff(n + 4*NumSupp), v.coeff(n + 5*NumSupp);
			}
		}

		// Calculate swing keypoint linear velocities
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::calcSwingLinVel(const LimbPhases& limbPhase, LegTipPoints& LTP, LegTipPointVels& LTPVel) const
		{
			// Constants
			static const int NumSwing = 5;              // Index n: Order is D, E, F, G, A
			static const int NumSwingD = NumSwing - 1;  // Index m: Order is DE, EF, FG, GA
			static const int NumX = 3*NumSwing;         // Index i: Order is vx(EFG) xEx xGx vy(EFG) xEy xGy vz(EFG) xEz xGz
			static const int NumSwingDD = NumSwing - 2; // Index k

			// Typedefs
			typedef Eigen::Matrix<double, NumX, NumX> MatrixA;
			typedef Eigen::Matrix<double, NumSwingDD, NumSwing> MatrixAsub;
			typedef Eigen::Matrix<double, NumX, 1> Vectorb;
			typedef Eigen::Matrix<double, NumX, 1> Vectorx;

			// Retrieve the limb phase input data
			double linSwingH[NumSwingD];
			for(int m = 0; m < NumSwingD; m++)
			{
				int index = KEY_D + m;
				int nindex = (index + 1) % NUM_KEYS;
				linSwingH[m] = rc_utils::picutMod(limbPhase[nindex] - limbPhase[index]);
			}

			// Precalculate terms
			double hr01 = 3.0*linSwingH[0] / linSwingH[1];
			double hr10 = 3.0*linSwingH[1] / linSwingH[0];
			double hr12 = 3.0*linSwingH[1] / linSwingH[2];
			double hr21 = 3.0*linSwingH[2] / linSwingH[1];
			double hr23 = 3.0*linSwingH[2] / linSwingH[3];
			double hr32 = 3.0*linSwingH[3] / linSwingH[2];

			// Initialise the matrix equations that govern the swing keypoint positions and velocities (Ax = b, once for each side)
			MatrixA A = MatrixA::Zero();
			Vectorb b;

			// Matrix: Apply the conditions of continuous velocity and acceleration at every swing keypoint
			MatrixAsub Asub;
			Asub << 2.0*(linSwingH[0] + linSwingH[1]), linSwingH[0], 0.0, hr01 - hr10, 0.0,
			        linSwingH[2], 2.0*(linSwingH[1] + linSwingH[2]), linSwingH[1], hr21, -hr12,
			        0.0, linSwingH[3], 2.0*(linSwingH[2] + linSwingH[3]), 0.0, hr23 - hr32;
			for(int j = 0; j < 3; j++)
				A.block<NumSwingDD, NumSwing>(NumSwingDD*j, NumSwing*j) = Asub;
			int curRow = 3*NumSwingDD;

			// Matrix: Specify the S-height of E and G
			for(int s = 0; s < 2; s++, curRow++)
			{
				for(int j = 0; j < 3; j++)
					A.coeffRef(curRow, NumSwing*j + s + 3) = CV.RBS.coeff(j, 2);
			}

			// Matrix: Make the xS and yS accelerations zero at A
			for(int s = 0; s < 2; s++, curRow++)
			{
				for(int j = 0; j < 3; j++)
				{
					int index = NumSwing*j;
					double factor = CV.RBS.coeff(j, s);
					A.coeffRef(curRow, index + 2) = linSwingH[3] * factor;
					A.coeffRef(curRow, index + 4) = 3.0*factor;
				}
			}

			// Matrix: Make the xI and yI accelerations zero at D
			for(int s = 0; s < 2; s++, curRow++)
			{
				for(int j = 0; j < 3; j++)
				{
					int index = NumSwing*j;
					double factor = CV.RBI.coeff(j, s);
					A.coeffRef(curRow, index) = linSwingH[0] * factor;
					A.coeffRef(curRow, index + 3) = -3.0*factor;
				}
			}

			// Perform a QR decomposition of A
			Eigen::ColPivHouseholderQR<MatrixA> AQR(A);

			// Calculate the required swing keypoint linear velocities
			for(LimbIndex l : CA.limbIndices)
			{
				// Retrieve the known positions and velocities
				const Vec3& x0 = LTP[l][KEY_D];
				const Vec3& x2 = LTP[l][KEY_F];
				const Vec3& x4 = LTP[l][KEY_A];
				const Vec3& v0 = LTPVel[l][KEY_D];
				const Vec3& v4 = LTPVel[l][KEY_A];

				// Vector: Apply the conditions of continuous velocity and acceleration at every swing keypoint
				Vec3 btmp1 = hr01*x2 - hr10*x0 - linSwingH[1]*v0;
				Vec3 btmp2 = (hr21 - hr12)*x2;
				Vec3 btmp3 = hr23*x4 - hr32*x2 - linSwingH[2]*v4;
				for(int j = 0; j < 3; j++)
				{
					int index = 3*j;
					b.coeffRef(index) = btmp1.coeff(j);
					b.coeffRef(index + 1) = btmp2.coeff(j);
					b.coeffRef(index + 2) = btmp3.coeff(j);
				}
				int curRow = 9;

				// Vector: Specify the S-height of E and G
				b.coeffRef(curRow++) = CV.BzS.dot(x0 + ktconfig.legHeightRatioE() * (x2 - x0));
				b.coeffRef(curRow++) = CV.BzS.dot(x4 + ktconfig.legHeightRatioG() * (x2 - x4));

				// Vector: Make the xS and yS accelerations zero at A
				Vec3 tmpRhs3 = 3.0*x4 - (2.0*linSwingH[3])*v4;
				b.coeffRef(curRow++) = CV.BxS.dot(tmpRhs3);
				b.coeffRef(curRow++) = CV.ByS.dot(tmpRhs3);

				// Vector: Make the xI and yI accelerations zero at D
				Vec3 tmpRhs0 = -3.0*x0 - (2.0*linSwingH[0])*v0;
				b.coeffRef(curRow++) = CV.BxI.dot(tmpRhs0);
				b.coeffRef(curRow++) = CV.ByI.dot(tmpRhs0);

				// Solve the matrix equation that governs the swing keypoint positions and velocities for this side (Ax = b)
				Vectorx x = AQR.solve(b);

				// Transcribe the calculated swing keypoint positions and velocities
				LTPVel[l][KEY_E] << x.coeff(0), x.coeff(5), x.coeff(10);
				LTPVel[l][KEY_F] << x.coeff(1), x.coeff(6), x.coeff(11);
				LTPVel[l][KEY_G] << x.coeff(2), x.coeff(7), x.coeff(12);
				LTP[l][KEY_E] << x.coeff(3), x.coeff(8), x.coeff(13);
				LTP[l][KEY_G] << x.coeff(4), x.coeff(9), x.coeff(14);
			}
		}

		// Calculate angular velocities
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::calcAngVel(const LimbPhases& limbPhase, FootYaws& footYaw, FootTilts& footTilt, FootAngVels& footAngVel) const
		{
			// Constants
			static const int NumRot = 6; // Index n: Order is A, B, N, C, D, F

			// Indexing to only consider the cyclic loop ABNCDF
			int indexMap[NumRot] = {KEY_A, KEY_B, KEY_N, KEY_C, KEY_D, KEY_F};

			// Retrieve the limb phase input data
			double rotH[NumRot];
			for(int n = 0; n < NumRot; n++)
			{
				int nn = (n + 1) % NumRot;
				int k = indexMap[n];
				int kk = indexMap[nn];
				rotH[n] = rc_utils::picutMod(limbPhase[kk] - limbPhase[k]);
			}

			// Retrieve the limb phase input data for the E and G keypoints
			double hDE = rc_utils::picutMod(limbPhase[KEY_E] - limbPhase[KEY_D]);
			double hFG = rc_utils::picutMod(limbPhase[KEY_G] - limbPhase[KEY_F]);

			// Calculate the 1x1 matrix A used to get the velocity at F, using continuous second derivative cubic spline interpolation (Av = b)
			double A = 2.0*(rotH[4] + rotH[5]);

			// Calculate the required keypoint angular velocities
			for(LimbIndex l : CA.limbIndices)
			{
				// Construct the absolute tilt phase source data for the cubic spline interpolation
				rot_conv::AbsTiltPhase3D rotX[NumRot];
				for(int n = 0; n < NumRot; n++)
				{
					int k = indexMap[n];
					rot_conv::AbsPhaseFromAbsYawTilt(footTilt[l][k], footYaw[l][k], rotX[n]);
				}

				// Calculate the linear interpolant slopes
				rot_conv::AbsTiltPhaseVel3D rotVbar[NumRot];
				for(int n = 0; n < NumRot; n++)
				{
					int nn = (n + 1) % NumRot;
					rot_conv::TiltPhaseDiff(rotX[n], rotX[nn], rotH[n], rotVbar[n]);
				}

				// Calculate keypoint angular velocities using a shape-preserving piecewise cubic Hermite interpolating polynomial
				rot_conv::AbsTiltPhaseVel3D rotV[NumRot];
				for(int n = 0; n < NumRot; n++)
				{
					int nn = (n + 1) % NumRot;
					double hA = rotH[n];
					double hB = rotH[nn];
					double ws = 3.0*(hA + hB);
					double w1 = (2.0*hA + hB) / ws;
					double w2 = (hA + 2.0*hB) / ws;
					for(int j = 0; j < 3; j++)
					{
						double vbarA = rotVbar[n][j];
						double vbarB = rotVbar[nn][j];
						double vbarAB = vbarA * vbarB;
						if(vbarAB > 0.0) // If vbarA and vbarB have the same sign and neither is zero...
							rotV[nn][j] = vbarAB / (w1*vbarA + w2*vbarB);
						else
							rotV[nn][j] = 0.0;
					}
				}

				// Recalculate the velocity at F using continuous second derivative cubic spline interpolation (analytic shortcut for solving the matrix equation Av = b)
				for(int j = 0; j < 3; j++)
				{
					double b = rotH[4]*(3.0*rotVbar[5][j] - rotV[0][j]) + rotH[5]*(3.0*rotVbar[4][j] - rotV[4][j]);
					rotV[5][j] = b / A;
				}

				// Calculate the keypoint angular velocities
				for(int n = 0; n < NumRot; n++)
				{
					int k = indexMap[n];
					footAngVel[l][k] = rot_conv::QuatRotVec(CV.qBN, rot_conv::AngVelFromAbsPhaseVel(rotV[n], footTilt[l][k])); // Angular velocity in B frame
				}

				// Calculate the intermediate orientations and angular velocities at E and G (evaluate the calculated cubic splines from D to F at E, and F to A at G)
				for(int s = 0; s < 2; s++)
				{
					SplineCoeff sc;
					rot_conv::AbsTiltPhase3D x;
					rot_conv::AbsTiltPhaseVel3D v;
					int k = (s == 0 ? KEY_E : KEY_G);
					double t = (s == 0 ? hDE : hFG);
					for(int j = 0; j < 3; j++)
					{
						int n = 4 + s;
						int nn = (n + 1) % NumRot;
						sc.setMV(rotH[n], rotX[n][j], rotVbar[n][j], rotV[n][j], rotV[nn][j]);
						x[j] = sc.evalX(t);
						v[j] = sc.evalV(t);
					}
					rot_conv::AbsYawTiltFromAbsPhase(x, footTilt[l][k], footYaw[l][k]);
					footAngVel[l][k] = rot_conv::QuatRotVec(CV.qBN, rot_conv::AngVelFromAbsPhaseVel(v, footTilt[l][k])); // Angular velocity in B frame
				}
			}
		}

		// Consolidate keypoints
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::consolidateKeypoints(const LegTipPoints& LTP, const FootYaws& footYaw, const FootTilts& footTilt, InvLegPoses& ILP) const
		{
			// Consolidate the keypoints as required
			rot_conv::TiltAngles tNF;
			for(LimbIndex l : CA.limbIndices)
			{
				LegTipPose LTPose(l);
				for(int n : CA.keypoints)
				{
					rot_conv::TiltFromAbsYawTilt(footTilt[l][n], footYaw[l][n], tNF);
					Quat qNF = rot_conv::QuatFromTilt(tNF);
					LTPose.rot = CV.qBN * qNF;
					LTPose.pos = LTP[l][n] - CV.localToGlobal[l];
					K.InvFromTip(LTPose, ILP[l][n]);
				}
			}
		}

		// Lean keypoints
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::leanKeypoints(InvLegPoses& ILP, MotionCentre& MC, LegTipPointVels& LTPVel, FootAngVels& footAngVel) const
		{
			// Lean the keypoints as required
			for(LimbIndex l : CA.limbIndices)
			{
				const Vec3& hipCentrePointLocal = CV.hipCentrePointLocal[l];
				for(int n : CA.keypoints)
				{
					ILP[l][n].rotateAround(hipCentrePointLocal, CV.qBNL);
					rot_conv::RotmatRotVecInPlace(CV.RBNL, LTPVel[l][n]);
					rot_conv::RotmatRotVecInPlace(CV.RBNL, footAngVel[l][n]);
				}
			}

			// Apply the required lean to the motion centre and motion centre line
			rot_conv::RotmatRotVecInPlace(CV.RBNL, MC.point); // Note: MC.point is in global coordinates, and should be rotated about the hip centre point, which is the origin of the global coordinate system!
			rot_conv::RotmatRotVecInPlace(CV.RBNL, MC.lineVec);
		}

		// Shift keypoints
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::shiftKeypoints(InvLegPoses& ILP, MotionCentre& MC) const
		{
			// Calculate the required shift
			double legScaleInv = RK.legScaleInv();
			Vec3 leanFootShift(-m_cmd.hipShift.x() * legScaleInv, -m_cmd.hipShift.y() * legScaleInv, 0.0);
			rot_conv::RotmatRotVecInPlace(CV.RBL, leanFootShift);

			// Calculate the required shifted keypoints
			for(LimbIndex l : CA.limbIndices)
			{
				for(int n : CA.keypoints)
					ILP[l][n].shiftBy(leanFootShift);
			}

			// Apply the required shift to the motion centre
			MC.point += leanFootShift;
		}

		// Finalise keypoints
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::finaliseKeypoints(InvLegPoses& ILP, AbsLegPoses& ALP, bool& exactKin, MotionCentre& MC) const
		{
			// Calculate the required adjustment vector
			Vec3 adjustVec = KI.calcFinalAdjustVec(ILP, MC, CV.BzL, m_cmd.hipHeightMax);

			// Initialise that the kinematics calculations were exact
			exactKin = true;

			// Calculate the required final keypoints
			for(LimbIndex l : CA.limbIndices)
			{
				for(int n : CA.keypoints)
				{
					// Adjust the keypoint as required
					ILP[l][n].shiftBy(adjustVec);

					// Safely perform the inverse kinematics to get the equivalent abstract pose
					exactKin &= KI.AbsFromInvSafe(ILP[l][n], ALP[l][n]);
				}
			}

			// Adjust the motion centre as required
			MC.point += adjustVec;
		}

		// Calculate trajectory
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::calcLegTrajectory(const LimbPhases& limbPhase, const AbsLegPoses& ALP, const LegTipPointVels& LTPVel, const FootAngVels& footAngVel, AbsLegSplineCoeffs& absCoeff) const
		{
			// Calculate the final keypoint abstract space position and velocity data
			AbsLegSplineData absPos, absVel;
			for(LimbIndex l : CA.limbIndices)
			{
				AbsLegPoseVel ALPVel(l);
				LegTipPoseVel LTPoseVel(l);
				for(int n : CA.keypoints)
				{
					LTPoseVel.vel = LTPVel[l][n];
					LTPoseVel.angVel = footAngVel[l][n];
					K.AbsFromTipVel(LTPoseVel, ALP[l][n], ALPVel);
					ALP[l][n].toArray(&absPos[l][n][0]);
					ALPVel.toArray(&absVel[l][n][0]);
				}
			}

			// Calculate the coefficients of the cubic splines that uniquely connect the keypoints
			for(LimbIndex l : CA.limbIndices)
			{
				for(int n : CA.keypoints)
				{
					int nn = (n + 1) % NUM_KEYS;
					double h = rc_utils::picutMod(limbPhase[nn] - limbPhase[n]);
					for(int k : CA.absFields)
						absCoeff[l][n][k].set(h, absPos[l][n][k], absPos[l][nn][k], absVel[l][n][k], absVel[l][nn][k]);
				}
			}
		}

		// Calculate support coefficient variables
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::calcSuppCoeffVars(const LimbPhases& limbPhase, SuppCoeffVars& suppCoeffVars) const
		{
			// Constants
			static const double MinPhaseLen = 1e-10;

			// Calculate the double support phase length from the limb phases
			double phaseAB = rc_utils::picutMod(limbPhase[KEY_B] - limbPhase[KEY_A]);
			double phaseCD = rc_utils::picutMod(limbPhase[KEY_D] - limbPhase[KEY_C]);
			double D = 0.5*(phaseAB + phaseCD);

			// Calculate the support transition slope and margin
			double supportTransitionPhaseLen = rc_utils::coerceMin(D + ktconfig.legSuppCoeffPhaseExtra(), MinPhaseLen);
			suppCoeffVars.transitionSlope = 1.0 / supportTransitionPhaseLen;
			suppCoeffVars.transitionMargin = 0.5 * supportTransitionPhaseLen;

			// Calculate the support transition centre points
			double phaseM = limbPhase[KEY_A] + 0.5*phaseAB;        // Phase of midpoint of AB according to A and B
			double phaseN = limbPhase[KEY_C] + 0.5*phaseCD - M_PI; // Phase of midpoint of AB according to C and D
			double phaseMNAvg = phaseM + 0.5*rc_utils::picut(phaseN - phaseM);
			suppCoeffVars.centrePtRise = rc_utils::picut(phaseMNAvg);
			suppCoeffVars.centrePtFall = rc_utils::picut(phaseMNAvg + M_PI);
		}

		// Evaluate function
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::evaluate(double gaitPhase, PoseCommand& poseCmd) const
		{
			//
			// General
			//

			// Update the evaluate count
			if(!m_evaluatingHaltPose)
				m_evalCount++;

			// Wrap the input gait phase and calculate the opposite gait phase
			rc_utils::picutVar(gaitPhase);
			double gaitPhaseOpp = rc_utils::picut(gaitPhase + M_PI);

			// Calculate the limb phase of each leg
			LegPhases legPhase;
			legPhase[LEFT] = gaitPhase;
			legPhase[RIGHT] = gaitPhaseOpp;

			// Calculate the limb phase of each arm
			ArmPhases armPhase;
			armPhase[LEFT] = gaitPhaseOpp;
			armPhase[RIGHT] = gaitPhase;

			// Set the default tuning pose of the robot
			AbsPose     AP(m_haltPoseAP.legL, m_haltPoseAP.legR, m_basePoseAP.armL, m_basePoseAP.armR, m_basePoseAP.head); // Note: The halt pose is used for legs as the base leg pose is not necessarily balanced,
			JointEffort JE(m_haltPoseJE.legL, m_haltPoseJE.legR, m_basePoseJE.armL, m_basePoseJE.armR, m_basePoseJE.head); //       and evalLegTrajectory(), if called, simply replaces these values without using them.
			PoseCommand::SuppCoeff suppCoeff = {m_haltPose.suppCoeff[hk::INDEX0], m_haltPose.suppCoeff[hk::INDEX1]};

			//
			// Leg trajectory
			//

			// Evaluate the keypoint trajectory for the legs
			if(!ktconfig.tuningNoLegs() || m_evaluatingHaltPose)
			{
				if(!ktconfig.tuningNoLegTrajectory() || m_evaluatingHaltPose)
					evalLegTrajectory(legPhase, m_limbPhase, m_absCoeff, AP.legL, AP.legR);
				if(!ktconfig.tuningNoLegEfforts() || m_evaluatingHaltPose)
					evalLegEfforts(legPhase, m_basePoseJE, JE.legL, JE.legR);
				if(!ktconfig.tuningNoLegSuppCoeff() || m_evaluatingHaltPose)
					evalLegSuppCoeff(legPhase, m_basePoseSC, m_suppCoeffVars, suppCoeff);
			}

			//
			// Arm trajectory
			//

			// Evaluate the keypoint trajectory for the arms
			if(!ktconfig.tuningNoArms() || m_evaluatingHaltPose)
			{
				if(!ktconfig.tuningNoArmTrajectory() || m_evaluatingHaltPose)
					evalArmTrajectory(armPhase, m_basePoseAP, m_limbPhase, AP.armL, AP.armR);
				if(!ktconfig.tuningNoArmEfforts() || m_evaluatingHaltPose)
					evalArmEfforts(armPhase, m_basePoseJE, JE.armL, JE.armR);
			}

			//
			// Finalisation
			//

			// Convert the abstract pose to the equivalent joint pose
			JointPose JP = K.JointFromAbs(AP);

			// Transcribe the evaluated outputs to the output pose command
			JP.toVector(poseCmd.pos);
			JE.toVector(poseCmd.effort);
			poseCmd.suppCoeff[LEFT] = suppCoeff[LEFT];
			poseCmd.suppCoeff[RIGHT] = suppCoeff[RIGHT];
		}

		// Evaluate leg trajectory
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::evalLegTrajectory(const LegPhases& legPhase, const LimbPhases& limbPhase, const AbsLegSplineCoeffs& absCoeff, AbsLegPose& legL, AbsLegPose& legR) const
		{
			// Calculate the abstract pose of each leg
			for(LimbIndex l : CA.limbIndices)
			{
				// Retrieve the current leg
				AbsLegPose& ALP = (l == LEFT ? legL : legR);

				// Retrieve the current leg phase
				double phase = legPhase[l];

				// Select the correct cubic spline interval to use for the current leg phase
				int usen = 0;
				double uset = 0.0;
				for(int n : CA.keypoints)
				{
					int nn = (n + 1) % NUM_KEYS;
					double h = rc_utils::picutMod(limbPhase[nn] - limbPhase[n]);
					double t = rc_utils::picutMod(phase - limbPhase[n]);
					if(t <= h)
					{
						usen = n;
						uset = t;
						break;
					}
				}

				// Construct the required abstract pose
				double ALPSerial[NUM_ABS_LEG];
				for(int k : CA.absFields)
					ALPSerial[k] = absCoeff[l][usen][k].evalX(uset);
				ALP.fromArray(ALPSerial);

				// Incorporate abstract leg pose biases
				KI.biasAbsPose(ALP);

				// Soft-coerce the evaluated abstract leg pose
				KI.AbsCoerceSoft(ALP);
			}
		}

		// Evaluate leg efforts
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::evalLegEfforts(const LegPhases& legPhase, const JointEffort& baseJE, JointLegEffort& legL, JointLegEffort& legR) const
		{
			// Use the base pose efforts for the entire trajectory (Thank you for your efforts!)
			legL = baseJE.legL;
			legR = baseJE.legR;
		}

		// Evaluate leg support coefficients
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::evalLegSuppCoeff(const LegPhases& legPhase, const PoseCommand::SuppCoeff& baseSC, const SuppCoeffVars& suppCoeffVars, PoseCommand::SuppCoeff& suppCoeff) const
		{
			// Calculate the support coefficient of each leg
			for(LimbIndex l : CA.limbIndices)
			{
				double relToRise = rc_utils::picut(legPhase[l] - suppCoeffVars.centrePtRise);
				double relToFall = rc_utils::picut(legPhase[l] - suppCoeffVars.centrePtFall);
				if(fabs(relToFall) <= suppCoeffVars.transitionMargin)
					suppCoeff[l] = 0.5 - relToFall * suppCoeffVars.transitionSlope;
				else if(fabs(relToRise) <= suppCoeffVars.transitionMargin)
					suppCoeff[l] = 0.5 + relToRise * suppCoeffVars.transitionSlope;
				else if(relToFall >= 0.0)
					suppCoeff[l] = 0.0;
				else
					suppCoeff[l] = 1.0;
				suppCoeff[l] = ktconfig.legSuppCoeffRange() * (suppCoeff[l] - 0.5) + 0.5;
			}
		}

		// Evaluate arm trajectory
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::evalArmTrajectory(const ArmPhases& armPhase, const AbsPose& baseAP, const LimbPhases& limbPhase, AbsArmPose& armL, AbsArmPose& armR) const
		{
			// Evaluate the trajectory for each arm
			for(LimbIndex l : CA.limbIndices)
			{
				// Retrieve a reference to the abstract arm pose of the current arm
				AbsArmPose& AAP = (l == LEFT ? armL : armR);

				// Start with the arm base pose
				AAP = (l == LEFT ? baseAP.armL : baseAP.armR);

				// Declare variables
				JointArmPose JAP(l);
				Vec3 CoM;

				// Evaluate the arm base motion
				genArmBaseMotion(armPhase[l], limbPhase, AAP, JAP, CoM);

				// Tilt and limit the arms
				tiltArms(JAP, CoM);
				limitArms(JAP);

				// Update the abstract arm pose
				K.AbsFromJoint(JAP, AAP);

				// Printing for debugging purposes
				if(m_evalCount % 512 == 1 && ktconfig.debugPrintEvalArm() && !m_evaluatingHaltPose)
				{
					std::ios::fmtflags f(std::cout.flags());
					std::cout << std::fixed << std::setprecision(ktconfig.debugPrintPrecision());
					std::cout << "DEBUG PRINT: Evaluate Arm (" << l << ")" << std::endl;
					FEED_KEY_TRAJ_PRINTE();
					FEED_KEY_TRAJ_PRINT(m_cmd.gcv);
					FEED_KEY_TRAJ_PRINT(m_cmd.armTilt);
					FEED_KEY_TRAJ_PRINTE();
					FEED_KEY_TRAJ_PRINT(l);
					FEED_KEY_TRAJ_PRINT(armPhase[l]);
					FEED_KEY_TRAJ_PRINT(JAP);
					FEED_KEY_TRAJ_PRINT(AAP);
					FEED_KEY_TRAJ_PRINT(CoM);
					FEED_KEY_TRAJ_PRINTE();
					std::cout.flags(f);
				}
			}
		}

		// Evaluate arm efforts
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::evalArmEfforts(const ArmPhases& armPhase, const JointEffort& baseJE, JointArmEffort& armL, JointArmEffort& armR) const
		{
			// Use the base pose efforts for the entire trajectory (Thank you for your efforts!)
			armL = baseJE.armL;
			armR = baseJE.armR;
		}

		// Generate arm base motion (AAP is assumed to contain the abstract halt pose)
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::genArmBaseMotion(double phase, const LimbPhases& limbPhase, AbsArmPose& AAP, JointArmPose& JAP, Vec3& CoM) const
		{
			// Use the desired arm base motion generator to generate the required data (should update AAP)
			int type = ktconfig.abmType();
			if(type == ABMT_SWING)
				KI.genArmBaseMotionSwing(CV, m_cmd.gcv, phase, limbPhase, AAP);
			else
			{
				ROS_ERROR_THROTTLE(3.0, "Unknown arm base motion type (%d) => Using swing method...", type);
				KI.genArmBaseMotionSwing(CV, m_cmd.gcv, phase, limbPhase, AAP);
			}

			// Calculate the output joint pose
			K.JointFromAbs(AAP, JAP);

			// Compute the centre of mass of the arm
			K.CoMFromJoint(JAP, CoM);
		}

		// Tilt arms
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::tiltArms(JointArmPose& JAP, Vec3& CoM) const
		{
			// Constants
			static const double Tol = 64.0*DBL_EPSILON;

			// Retrieve the limb index and limb sign
			LimbIndex l = JAP.limbIndex;
			LimbSign ls = JAP.limbSign;

			// Soft-coerce the allowed arm tilt
			rot_conv::AbsTiltRot armTilt = m_cmd.armTilt;
			armTilt.tiltAngle = rc_utils::coerceSoftAbs<double>(armTilt.tiltAngle, ktconfig.armArmTiltAngleMaxAbs(), ktconfig.armArmTiltAngleBuf());

			// Calculate the nominal CoM ray (vector relative to the shoulder in N coordinates) after the application of arm tilt
			rot_conv::TiltAngles TLC = rot_conv::TiltFromAbsYawTilt(armTilt);
			Quat qLC = rot_conv::QuatFromTilt(TLC);
			Vec3 CoMRayN = rot_conv::QuatRotVec(CV.qNL * qLC * CV.qLB, CoM - K.shoulderPoint(l)); // Convert CoM relative to shoulder from B to L coordinates, then apply the rotation from L to C and convert from L to N coordinates

			// Normalise the nominal CoM ray vector
			rot_conv::NormaliseVec(CoMRayN, Tol, Vec3(0.0, 0.0, -1.0));

			// Calculate the CoM ray yaw and tilt
			double CoMRayYawN = atan2(CoMRayN.y(), CoMRayN.x()); // In (-pi,pi] where 0 is tilt in the direction of xN
			double CoMRayTiltN = acos(rc_utils::coerceAbs(-CoMRayN.z(), 1.0)); // In [0, pi]

			// Config aliases
			double a = ktconfig.armCoMRayTiltMaxP(); // Max absolute tilt along the xN direction (must have a > 0)
			double b = ktconfig.armCoMRayTiltMaxR(); // Max absolute tilt along the yN direction (must have b > 0)
			double m = ktconfig.armCoMRayTiltBuf();  // Soft coercion buffer value for the absolute tilt (must have m <= a,b)

			// Calculate the maximum desired CoM ray tilt given the yaw
			double cosCoMYaw = cos(CoMRayYawN);
			double sinCoMYaw = sin(CoMRayYawN);
			double CoMRayTiltNMax = a*b / sqrt(a*a*sinCoMYaw*sinCoMYaw + b*b*cosCoMYaw*cosCoMYaw); // In [a,b]

			// Soft-limit the absolute tilt of the CoM ray relative to straight down in N coordinates in an elliptical fashion
			if(CoMRayTiltN > CoMRayTiltNMax - m)
			{
				CoMRayTiltN = rc_utils::coerceSoftMax(CoMRayTiltN, CoMRayTiltNMax, m);
				double cosCoMTilt = cos(CoMRayTiltN);
				double sinCoMTilt = sin(CoMRayTiltN);
				CoMRayN << cosCoMYaw*sinCoMTilt, sinCoMYaw*sinCoMTilt, -cosCoMTilt;
			}

			// Adjust the CoM ray in N coordinates to respect an inwards y-bound
			CoMRayN.y() = ls * rc_utils::coerceSoftMin<double>(CoMRayN.y() / ls, -ktconfig.armCoMRayYMaxIwd(), ktconfig.armCoMRayYBuf());
			CoMRayN.y() = rc_utils::coerceAbs(CoMRayN.y(), 1.0);
			double CoMRayNysq = CoMRayN.y() * CoMRayN.y();
			double xysqsum = CoMRayN.x() * CoMRayN.x() + CoMRayNysq;
			if(xysqsum > 1.0)
			{
				CoMRayN.x() = rc_utils::sign(CoMRayN.x()) * sqrt(1.0 - CoMRayNysq);
				CoMRayN.z() = 0.0;
			}
			else
				CoMRayN.z() = -sqrt(1.0 - xysqsum); // Always pick the solution in the bottom N-hemisphere

			// Convert the CoM ray to body-fixed coordinates
			Vec3 CoMRayB = rot_conv::QuatRotVec(CV.qBN, CoMRayN);

			// Perform inverse kinematics to place the arm CoM on the CoM ray
			JAP = KI.JointFromCoMRay(CoMRayB, JAP);

			// Compute the centre of mass of the arm
			K.CoMFromJoint(JAP, CoM);
		}

		// Limit arms
		template<class Kinematics> void FeedKeypointTraj<Kinematics>::limitArms(JointArmPose& JAP) const
		{
			// Enforce the required joint space limits
			KI.JointCoerceSoft(JAP);
		}
	}
}

#endif
// EOF