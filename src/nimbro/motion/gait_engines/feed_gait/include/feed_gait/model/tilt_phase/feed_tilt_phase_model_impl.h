// Feedback gait tilt phase model implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_TILT_PHASE_MODEL_IMPL_H
#define FEED_TILT_PHASE_MODEL_IMPL_H

// Includes
#include <feed_gait/model/tilt_phase/feed_tilt_phase_model.h>
#include <rc_utils/smooth_deadband_ell.h>
#include <rc_utils/smooth_deadband.h>

// Feedback gait namespace
namespace feed_gait
{
	// Tilt phase model namespace
	namespace tilt_phase_model
	{
		// ##################################
		// #### FeedTiltPhaseModel class ####
		// ##################################

		//
		// Reset and configure functions
		//

		// Reset members function
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::resetMembers(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput)
		{
			// Reset and configure the general feedback
			resetGeneral();
			configGeneral();

			// Reset and configure the PID feedback
			resetPID();
			configPID();

			// Reset and configure the swing ground plane feedback
			resetPlaneS();
			configPlaneS();

			// Reset and configure the swing out feedback
			resetSwingOut();
			configSwingOut();

			// Reset and configure the hip height maximum feedback
			resetHipMax();
			configHipMax(modelInput);

			// Reset and configure the timing feedback
			resetTiming();
			configTiming();

			// Reset and configure the step size feedback
			resetStepSize();
			configStepSize();
		}

		// Reset the general feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::resetGeneral()
		{
			// Reset the general feedback
			phasePMeanFilter.resetAll();
			syncPhaseFilter.resetAll();
		}

		// Configure the general feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::configGeneral()
		{
			// Resize the proportional mean filter
			std::size_t phasePMeanFilterN = rc_utils::coerceMin(tpmconfig.phasePMeanFilterN(), 1);
			if(phasePMeanFilter.len() != phasePMeanFilterN) phasePMeanFilter.resize(phasePMeanFilterN);

			// Resize the synchronised tilt phase filter
			std::size_t syncPhaseFilterN = rc_utils::coerceMin(tpmconfig.syncPhaseFilterN(), 1);
			if(syncPhaseFilter.len() != syncPhaseFilterN) syncPhaseFilter.resize(syncPhaseFilterN);
		}

		// Reset the PID feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::resetPID()
		{
			// Reset the PID feedback
			phaseDDerivFilter.resetAll();
			phaseIEllBndInteg.resetAll();
			phaseIMeanFilter.resetAll();
		}

		// Configure the PID feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::configPID()
		{
			// Resize the derivative WLBF filter
			std::size_t phaseDDerivFilterN = rc_utils::coerceMin(tpmconfig.phaseDDerivFilterN(), 1);
			if(phaseDDerivFilter.len() != phaseDDerivFilterN) phaseDDerivFilter.resize(phaseDDerivFilterN);

			// Update the parameters of the ellipsoidally bounded integrator
			Vec2 phaseIIntegralMaxAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.phaseIXIntegralMax(), tpmconfig.phaseIYIntegralMax());
			phaseIEllBndInteg.setEllBound(phaseIIntegralMaxAxes, tpmconfig.phaseIIntegralBuf());

			// Resize the integral mean filter
			std::size_t phaseIMeanFilterN = rc_utils::coerceMin(tpmconfig.phaseIMeanFilterN(), 1);
			if(phaseIMeanFilter.len() != phaseIMeanFilterN) phaseIMeanFilter.resize(phaseIMeanFilterN);
		}

		// Reset the swing ground plane feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::resetPlaneS()
		{
			// Reset the swing ground plane feedback
			planeNSMeanFilter.resetAll();
		}

		// Configure the swing ground plane feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::configPlaneS()
		{
			// Resize the swing ground plane mean filter
			std::size_t planeNSMeanFilterN = rc_utils::coerceMin(tpmconfig.planeNSMeanFilterN(), 1);
			if(planeNSMeanFilter.len() != planeNSMeanFilterN) planeNSMeanFilter.resize(planeNSMeanFilterN);
		}

		// Reset the swing out feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::resetSwingOut()
		{
			// Reset the swing out feedback
			swingXLHoldFilter.resetAll();
			swingXRHoldFilter.resetAll();
		}

		// Configure the swing out feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::configSwingOut()
		{
			// Resize the swing out hold filters
			std::size_t swingHoldFilterN = rc_utils::coerceMin(tpmconfig.swingHoldFilterN(), 1);
			if(swingXLHoldFilter.numPoints() != swingHoldFilterN) swingXLHoldFilter.resize(swingHoldFilterN);
			if(swingXRHoldFilter.numPoints() != swingHoldFilterN) swingXRHoldFilter.resize(swingHoldFilterN);
		}

		// Reset the hip height maximum feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::resetHipMax()
		{
			// Reset the hip height maximum feedback
			instLowPassFilter.resetAll();
			hipMaxSlopeLimiter.reset();
		}

		// Configure the hip height maximum feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::configHipMax(const ModelInput& modelInput)
		{
			// Update the settling time of the instability low pass filter
			instLowPassFilter.setTs(tpmconfig.instLowPassTs() / modelInput.nominaldT);
			instLowPassFilter.setMaxDelta(tpmconfig.instLowPassMaxSlope() * modelInput.nominaldT);

			// Update the maximum delta of the hip height maximum slope limiter
			if(!hipMaxSlopeLimiter.valueSet()) hipMaxSlopeLimiter.setValue(modelInput.trajInfo.hipHeightNom);
			hipMaxSlopeLimiter.setMaxDelta(tpmconfig.hipHeightMaxSlope() * modelInput.nominaldT);
		}

		// Reset the timing feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::resetTiming()
		{
		}

		// Configure the timing feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::configTiming()
		{
		}

		// Reset the step size feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::resetStepSize()
		{
			// Reset the step size feedback
			stepTriPendModelSag.resetParam();
			stepGcvXBHoldFilter.resetAll();
			stepGcvXFHoldFilter.resetAll();
		}

		// Configure the step size feedback
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::configStepSize()
		{
			// Set the parameters of the sagittal tripendulum model
			double Cbfsq = tpmconfig.stepSTPMCsqConstBF() / K.legScaleTip();
			double Cmsq = tpmconfig.stepSTPMCsqConstM() / K.legScaleTip();
			double thm = tpmconfig.expPhaseYSinOffset();
			double thb = rc_utils::coerceMax<double>(tpmconfig.stepSTPMCrossingPhaseB(), thm);
			double thf = rc_utils::coerceMin<double>(tpmconfig.stepSTPMCrossingPhaseF(), thm);
			stepTriPendModelSag.setParam(thb, thm, thf, Cbfsq, Cmsq, Cbfsq);

			// Resize the step size hold filters
			std::size_t stepGcvXHoldFilterN = rc_utils::coerceMin(tpmconfig.stepGcvXHoldFilterN(), 1);
			if(stepGcvXBHoldFilter.numPoints() != stepGcvXHoldFilterN) stepGcvXBHoldFilter.resize(stepGcvXHoldFilterN);
			if(stepGcvXFHoldFilter.numPoints() != stepGcvXHoldFilterN) stepGcvXFHoldFilter.resize(stepGcvXHoldFilterN);
		}

		//
		// Update function
		//

		// Update function
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::update(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput)
		{
			// Reset the model output
			m_out.reset();

			// Do nothing if the model is not enabled
			if(!tpmconfig.enable())
			{
				resetMembers(phaseInfo, modelInput);
				return;
			}

			// Reconfigure the feedback as required
			configGeneral();
			configPID();
			configPlaneS();
			configSwingOut();
			configHipMax(modelInput);
			configTiming();
			configStepSize();

			// Create a state information struct
			StateInfo stateInfo;

			// Calculate the current tilt phase
			rot_conv::TiltPhase2D phase = rot_conv::PhaseFromFused(modelInput.robotOrient.fusedPitch, modelInput.robotOrient.fusedRoll);
			stateInfo.phase << phase.px, phase.py;

			// Calculate the expected tilt phase
			stateInfo.expectedPhase.x() = tpmconfig.expPhaseXSinOffset() + tpmconfig.expPhaseXSinMag() * sin(phaseInfo.gaitPhase - tpmconfig.expPhaseXSinPhase());
			stateInfo.expectedPhase.y() = tpmconfig.expPhaseYSinOffset() + tpmconfig.expPhaseYSinMag() * sin(phaseInfo.gaitPhase - tpmconfig.expPhaseYSinPhase());

			// Calculate the deviation tilt of the robot in the tilt phase representation
			Quat qNC = TPMHelper::deviationTiltPhaseN(stateInfo.phase.x(), stateInfo.phase.y(), stateInfo.expectedPhase.x(), stateInfo.expectedPhase.y(), modelInput.trajInfo.fusedPitchN);
			rot_conv::TiltPhase3D PNC = rot_conv::PhaseFromQuat(qNC); // Note: We expect the z component to be approximately zero here, as qNC should always be a pure tilt rotation relative to N!
			stateInfo.phaseDev << -PNC.px, -PNC.py; // Note: The phase components are negated so that the output quantity is a 'deviation from expected' (i.e. positive values indicate that the robot is too positively tilted about the corresponding axis)

			// Mean filter the phase deviation of the robot
			stateInfo.phaseDevFiltLast = phasePMeanFilter.mean();
			stateInfo.phaseDevFilt = phasePMeanFilter.update(stateInfo.phaseDev);

			// Calculate the time-synchronised filtered tilt phase value and derivative
			syncPhaseFilter.addPW(modelInput.timestamp, stateInfo.phase.x(), stateInfo.phase.y(), 1.0).update();
			stateInfo.syncPhase = syncPhaseFilter.centreValue();
			stateInfo.syncPhaseD = syncPhaseFilter.deriv();

			// Update the actions command
			updatePID(phaseInfo, modelInput, stateInfo);
			updatePlaneS(phaseInfo, modelInput, stateInfo);
			updateSwingOut(phaseInfo, modelInput, stateInfo);
			updateHipMax(phaseInfo, modelInput, stateInfo);

			// Update the timing command
			updateTiming(phaseInfo, modelInput, stateInfo);

			// Update the step size command
			updateStepSize(phaseInfo, modelInput, stateInfo);

			// Disable certain model outputs if the configuration parameters say so
			if(!tpmconfig.useTiming())   m_out.useTiming = false;
			if(!tpmconfig.useStepSize()) m_out.useStepSize = false;
			if(!tpmconfig.useActions())  m_out.useActions = ACFlag::NONE;

			// Plotting
			if(m_PM->getEnabled())
			{
				m_PM->plotVec2d(stateInfo.phase, PM_TPM_SI_PHASEX);
				m_PM->plotVec2d(stateInfo.expectedPhase, PM_TPM_SI_EXPPHASEX);
				m_PM->plotVec2d(stateInfo.syncPhase, PM_TPM_SI_SYNCPHASEX);
				m_PM->plotVec2d(stateInfo.syncPhaseD, PM_TPM_SI_SYNCPHASEDX);
			}
		}

		//
		// Action functions
		//

		// Update PID feedback function
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::updatePID(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo)
		{
			/// Tilt phase deviation proportional feedback

			// Calculate the tilt phase deviation proportional feedback
			Vec2 phasePDeadAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.phasePXDeadRadius(), tpmconfig.phasePYDeadRadius());
			Vec2 phasePFeed = rc_utils::SmoothEllDeadband2D::eval(stateInfo.phaseDevFilt, phasePDeadAxes);

			/// Tilt phase deviation derivative feedback

			// Calculate the tilt phase deviation derivative feedback
			phaseDDerivFilter.addPW(modelInput.timestamp, stateInfo.phaseDev, 1.0).update();
			Vec2 phaseDDeriv = phaseDDerivFilter.deriv();
			Vec2 phaseDDeadAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.phaseDXDeadRadius(), tpmconfig.phaseDYDeadRadius());
			Vec2 phaseDFeed = rc_utils::SmoothEllDeadband2D::eval(phaseDDeriv, phaseDDeadAxes);

			/// Tilt phase deviation integral feedback

			// Update the tilt phase deviation integral
			Vec2 phaseIDev = stateInfo.phaseDev;
			Vec2 phaseIDevMaxAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.phaseIXDevMax(), tpmconfig.phaseIYDevMax());
			rc_utils::coerceEllC(phaseIDevMaxAxes, phaseIDev);
			phaseIDev *= tpmconfig.phaseIGainIntegrand();
			phaseIEllBndInteg.integrate(phaseIDev, modelInput.truedT);

			// Work out whether the integrated value will contribute anything to the model output
			bool usingIntegralX = (tpmconfig.useActions() && tpmconfig.enablePhaseIX() && (
			                      (tpmconfig.enableFootTiltCtsX() && tpmconfig.phaseIGainFootTiltCts() != 0.0) ||
			                      (tpmconfig.enableHipShiftY() && tpmconfig.phaseIGainHipShift() != 0.0)));
			bool usingIntegralY = (tpmconfig.useActions() && tpmconfig.enablePhaseIY() && (
			                      (tpmconfig.enableFootTiltCtsY() && tpmconfig.phaseIGainFootTiltCts() != 0.0) ||
			                      (tpmconfig.enableHipShiftX() && tpmconfig.phaseIGainHipShift() != 0.0)));

			// Disable axes of integration that are not in use
			if(!usingIntegralX) phaseIEllBndInteg.setIntegralXZero();
			if(!usingIntegralY) phaseIEllBndInteg.setIntegralYZero();

			// Calculate the tilt phase deviation integral feedback
			Vec2 phaseIIntegral = phaseIEllBndInteg.integral();
			Vec2 phaseIFeed = phaseIMeanFilter.update(phaseIIntegral);

			/// Combined PID feedback

			// Save the directions of the PD feedback components
			Vec2 phasePFeedRay = phasePFeed;
			Vec2 phaseDFeedRay = phaseDFeed;

			// Disable feedback components as required
			if(!tpmconfig.enablePhasePX()) phasePFeed.x() = 0.0;
			if(!tpmconfig.enablePhasePY()) phasePFeed.y() = 0.0;
			if(!tpmconfig.enablePhaseDX()) phaseDFeed.x() = 0.0;
			if(!tpmconfig.enablePhaseDY()) phaseDFeed.y() = 0.0;
			if(!tpmconfig.enablePhaseIX()) phaseIFeed.x() = 0.0;
			if(!tpmconfig.enablePhaseIY()) phaseIFeed.y() = 0.0;

			// Calculate the required proportional/derivative feedback components for each corrective action
			Vec2 phasePGainArmTiltAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.phasePXGainAll()*tpmconfig.phasePDGainArmTiltX(), tpmconfig.phasePYGainAll()*tpmconfig.phasePDGainArmTiltY());
			Vec2 phaseDGainArmTiltAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.phaseDXGainAll()*tpmconfig.phasePDGainArmTiltX(), tpmconfig.phaseDYGainAll()*tpmconfig.phasePDGainArmTiltY());
			Vec2 phasePGainFootTiltSuppAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.phasePXGainAll()*tpmconfig.phasePDGainFootTiltSX(), tpmconfig.phasePYGainAll()*tpmconfig.phasePDGainFootTiltSY());
			Vec2 phaseDGainFootTiltSuppAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.phaseDXGainAll()*tpmconfig.phasePDGainFootTiltSX(), tpmconfig.phaseDYGainAll()*tpmconfig.phasePDGainFootTiltSY());
			Vec2 feedArmTiltP = phasePFeed * rc_utils::ellipsoidRadius(phasePGainArmTiltAxes, phasePFeedRay);
			Vec2 feedArmTiltD = phaseDFeed * rc_utils::ellipsoidRadius(phaseDGainArmTiltAxes, phaseDFeedRay);
			Vec2 feedFootTiltSuppP = phasePFeed * rc_utils::ellipsoidRadius(phasePGainFootTiltSuppAxes, phasePFeedRay);
			Vec2 feedFootTiltSuppD = phaseDFeed * rc_utils::ellipsoidRadius(phaseDGainFootTiltSuppAxes, phaseDFeedRay);

			// Calculate the required integral feedback components for each corrective action
			Vec2 feedFootTiltCtsI = phaseIFeed * tpmconfig.phaseIGainFootTiltCts();
			Vec2 feedHipShiftI = phaseIFeed * tpmconfig.phaseIGainHipShift();

			// Calculate the combined corrective action feedback components
			Vec2 feedArmTilt = feedArmTiltP + feedArmTiltD;
			Vec2 feedFootTiltCts = feedFootTiltCtsI;
			Vec2 feedFootTiltSupp = feedFootTiltSuppP + feedFootTiltSuppD;
			Vec2 feedHipShift = feedHipShiftI;
			Vec2 feedLeanTilt = Vec2::Zero();

			// Correct the hip shift feedback sign convention
			double feedHipShiftTmp = feedHipShift.x();
			feedHipShift.x() = -feedHipShift.y();
			feedHipShift.y() = feedHipShiftTmp;

			/// Output PID feedback corrective actions

			// Incorporate the required feedback corrective action biases
			feedArmTilt.x() += tpmconfig.biasArmTiltX();
			feedArmTilt.y() += tpmconfig.biasArmTiltY();
			feedFootTiltCts.x() += tpmconfig.biasFootTiltCtsX();
			feedFootTiltCts.y() += tpmconfig.biasFootTiltCtsY();
			feedFootTiltSupp.x() += tpmconfig.biasFootTiltSuppX();
			feedFootTiltSupp.y() += tpmconfig.biasFootTiltSuppY();
			feedHipShift.x() += tpmconfig.biasHipShiftX();
			feedHipShift.y() += tpmconfig.biasHipShiftY();
			feedLeanTilt.x() += tpmconfig.biasLeanTiltX();
			feedLeanTilt.y() += tpmconfig.biasLeanTiltY();

			// Respect which feedback corrective action components are enabled
			if(!tpmconfig.enableArmTiltX()) feedArmTilt.x() = 0.0;
			if(!tpmconfig.enableArmTiltY()) feedArmTilt.y() = 0.0;
			if(!tpmconfig.enableFootTiltCtsX()) feedFootTiltCts.x() = 0.0;
			if(!tpmconfig.enableFootTiltCtsY()) feedFootTiltCts.y() = 0.0;
			if(!tpmconfig.enableFootTiltSuppX()) feedFootTiltSupp.x() = 0.0;
			if(!tpmconfig.enableFootTiltSuppY()) feedFootTiltSupp.y() = 0.0;
			if(!tpmconfig.enableHipShiftX()) feedHipShift.x() = 0.0;
			if(!tpmconfig.enableHipShiftY()) feedHipShift.y() = 0.0;
			if(!tpmconfig.enableLeanTiltX()) feedLeanTilt.x() = 0.0;
			if(!tpmconfig.enableLeanTiltY()) feedLeanTilt.y() = 0.0;

			// Set the output arm tilt
			if(tpmconfig.enableArmTiltX() || tpmconfig.enableArmTiltY())
			{
				rot_conv::AbsYawTiltFromAbsPhase(rot_conv::AbsTiltPhase2D(feedArmTilt.x(), feedArmTilt.y()), m_out.actionsCmd.armTilt);
				m_out.actionsCmd.armTilt.tiltAngle = rc_utils::coerceSoftAbs<double>(m_out.actionsCmd.armTilt.tiltAngle, tpmconfig.limArmTiltAMax(), tpmconfig.limArmTiltABuf());
				m_out.useActions.setFlags(ACFlag::ARMTILT);
			}

			// Set the output continuous foot tilt
			if(tpmconfig.enableFootTiltCtsX() || tpmconfig.enableFootTiltCtsY())
			{
				rot_conv::AbsYawTiltFromAbsPhase(rot_conv::AbsTiltPhase2D(feedFootTiltCts.x(), feedFootTiltCts.y()), m_out.actionsCmd.footTiltCts);
				m_out.actionsCmd.footTiltCts.tiltAngle = rc_utils::coerceSoftAbs<double>(m_out.actionsCmd.footTiltCts.tiltAngle, tpmconfig.limFootTiltCtsAMax(), tpmconfig.limFootTiltCtsABuf());
				m_out.useActions.setFlags(ACFlag::FOOTTILTCTS);
			}

			// Set the output support foot tilt
			if(tpmconfig.enableFootTiltSuppX() || tpmconfig.enableFootTiltSuppY())
			{
				rot_conv::AbsYawTiltFromAbsPhase(rot_conv::AbsTiltPhase2D(feedFootTiltSupp.x(), feedFootTiltSupp.y()), m_out.actionsCmd.footTiltSupp);
				m_out.actionsCmd.footTiltSupp.tiltAngle = rc_utils::coerceSoftAbs<double>(m_out.actionsCmd.footTiltSupp.tiltAngle, tpmconfig.limFootTiltSuppAMax(), tpmconfig.limFootTiltSuppABuf());
				m_out.useActions.setFlags(ACFlag::FOOTTILTSUPP);
			}

			// Set the output hip shift
			if(tpmconfig.enableHipShiftX() || tpmconfig.enableHipShiftY())
			{
				Vec2 limHipShiftMaxAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.limHipShiftXMax(), tpmconfig.limHipShiftYMax());
				rc_utils::coerceSoftEllC<double>(limHipShiftMaxAxes, feedHipShift, tpmconfig.limHipShiftBuf(), m_out.actionsCmd.hipShift);
				m_out.useActions.setFlags(ACFlag::HIPSHIFT);
			}

			// Set the output lean tilt
			if(tpmconfig.enableLeanTiltX() || tpmconfig.enableLeanTiltY())
			{
				rot_conv::AbsYawTiltFromAbsPhase(rot_conv::AbsTiltPhase2D(feedLeanTilt.x(), feedLeanTilt.y()), m_out.actionsCmd.leanTilt);
				m_out.actionsCmd.leanTilt.tiltAngle = rc_utils::coerceSoftAbs<double>(m_out.actionsCmd.leanTilt.tiltAngle, tpmconfig.limLeanTiltAMax(), tpmconfig.limLeanTiltABuf());
				m_out.useActions.setFlags(ACFlag::LEANTILT);
			}

			/// PID feedback plotting

			// Plotting
			if(m_PM->getEnabled())
			{
				m_PM->plotVec2d(stateInfo.phaseDev, PM_TPM_PID_DEVIATIONX);
				m_PM->plotVec2d(stateInfo.phaseDevFilt, PM_TPM_PID_MEANFILTEREDX);
				m_PM->plotVec2d(phaseDDeriv, PM_TPM_PID_WLBFDERIVX);
				m_PM->plotVec2d(phaseIDev, PM_TPM_PID_INTEGRANDX);
				m_PM->plotVec2d(phaseIIntegral, PM_TPM_PID_INTEGRALX);
				m_PM->plotVec2d(phasePFeed, PM_TPM_PID_FEEDPHASEPX);
				m_PM->plotVec2d(phaseDFeed, PM_TPM_PID_FEEDPHASEDX);
				m_PM->plotVec2d(phaseIFeed, PM_TPM_PID_FEEDPHASEIX);
			}
		}

		// Update swing ground plane feedback function
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::updatePlaneS(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo)
		{
			// Calculate the instantaneous tilt rotation from N to the swing ground plane S
			rot_conv::TiltPhase2D planeNSPhase = TPMHelper::swingGroundPlaneN(stateInfo.phase.x(), stateInfo.phase.y(), stateInfo.expectedPhase.x(), stateInfo.expectedPhase.y(), modelInput.trajInfo.fusedPitchN);

			// Disable S relative to N components as required
			if(!tpmconfig.enablePlaneNSX()) planeNSPhase.px = 0.0;
			if(!tpmconfig.enablePlaneNSY()) planeNSPhase.py = 0.0;

			// Mean filter, deadband and scale the orientation of S relative to N
			Vec2 planeNSPhaseFilt = planeNSMeanFilter.update(planeNSPhase.px, planeNSPhase.py);
			Vec2 planeNSDeadAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.planeNSDeadRadiusX(), tpmconfig.planeNSDeadRadiusY());
			Vec2 planeNSFeedRaw = tpmconfig.planeNSScale() * rc_utils::SmoothEllDeadband2D::eval(planeNSPhaseFilt, planeNSDeadAxes);

			// Apply limiting to the orientation of the swing ground plane
			rot_conv::TiltPhase2D planeNSFeed(planeNSFeedRaw.x(), planeNSFeedRaw.y());
			Vec2 limSwingPlaneNSAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.limSwingPlaneNSXMax(), tpmconfig.limSwingPlaneNSYMax());
			rc_utils::coerceSoftEllC<double>(planeNSFeed.px, planeNSFeed.py, limSwingPlaneNSAxes.x(), limSwingPlaneNSAxes.y(), tpmconfig.limSwingPlaneNSBuf());

			// Calculate and set the output fused pitch and roll of the S plane
			if(tpmconfig.enablePlaneNSX() || tpmconfig.enablePlaneNSY())
			{
				TPMHelper::swingGroundPlaneAction(planeNSFeed, modelInput.trajInfo.fusedPitchN, m_out.actionsCmd.fusedPitchS, m_out.actionsCmd.fusedRollS);
				m_out.useActions.setFlags(ACFlag::FUSEDPITCHS | ACFlag::FUSEDROLLS);
			}

			// Plotting
			if(m_PM->getEnabled())
			{
				m_PM->plotScalar(planeNSPhase.px, PM_TPM_PS_NSDEVIATION_X);
				m_PM->plotScalar(planeNSPhase.py, PM_TPM_PS_NSDEVIATION_Y);
				m_PM->plotScalar(planeNSFeed.px, PM_TPM_PS_NSFEED_X);
				m_PM->plotScalar(planeNSFeed.py, PM_TPM_PS_NSFEED_Y);
			}
		}

		// Update swing out feedback function
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::updateSwingOut(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo)
		{
			// Calculate the states of the crossing models
			double crossingAngleL = tpmconfig.crossingPhaseXL() - stateInfo.syncPhase.x();
			double crossingAngleR = stateInfo.syncPhase.x() - tpmconfig.crossingPhaseXR();
			double crossingAngleVelL = -stateInfo.syncPhaseD.x();
			double crossingAngleVelR =  stateInfo.syncPhaseD.x();

			// Calculate the specific crossing energy
			double crossingCsqConst = tpmconfig.crossingCsqConst() / K.legScaleTip();
			double limpEnergyL = (crossingAngleVelL*crossingAngleVelL / crossingCsqConst) + 2.0*(cos(crossingAngleL) - 1.0);
			double limpEnergyR = (crossingAngleVelR*crossingAngleVelR / crossingCsqConst) + 2.0*(cos(crossingAngleR) - 1.0);
			double crossingEnergyL = rc_utils::sign(crossingAngleVelL) * (crossingAngleVelL*crossingAngleVelL / crossingCsqConst) - 2.0*(cos(crossingAngleL) - 1.0) * rc_utils::sign(crossingAngleL);
			double crossingEnergyR = rc_utils::sign(crossingAngleVelR) * (crossingAngleVelR*crossingAngleVelR / crossingCsqConst) - 2.0*(cos(crossingAngleR) - 1.0) * rc_utils::sign(crossingAngleR);

			// Calculate the left and right lateral swing out angles
			double swingOutXL = 0.0, swingOutXR = 0.0;
			if(crossingEnergyL >= tpmconfig.crossingEnergyMin())
				swingOutXL = -tpmconfig.crossingEnergyToSwingX() * rc_utils::SmoothDeadband::eval(crossingEnergyL, tpmconfig.crossingEnergyDeadRad(), tpmconfig.crossingEnergyMin());
			if(crossingEnergyR >= tpmconfig.crossingEnergyMin())
				swingOutXR = tpmconfig.crossingEnergyToSwingX() * rc_utils::SmoothDeadband::eval(crossingEnergyR, tpmconfig.crossingEnergyDeadRad(), tpmconfig.crossingEnergyMin());

			// Hold the swing out values for some time to give them a chance to take effect
			double swingOutXLH = swingXLHoldFilter.update(swingOutXL);
			double swingOutXRH = swingXRHoldFilter.update(swingOutXR);

			// Initialise the desired swing out
			Vec2 swingOut = Vec2::Zero();

			// Fade between the left and right swing out angles depending on the gait phase
			if(tpmconfig.enableSwingOutX())
			{
				double swingOutXSupp = (phaseInfo.suppLegIndex == hk::LEFT ? swingOutXLH : swingOutXRH);
				double swingOutXSwing = (phaseInfo.suppLegIndex == hk::LEFT ? swingOutXRH : swingOutXLH);
				swingOut.x() = rc_utils::interpolateCoerced(0.0, modelInput.trajInfo.dblSuppPhaseLen, swingOutXSwing, swingOutXSupp, phaseInfo.elapGaitPhase);
			}

			// Calculate the required swing out in the pitch direction
			if(tpmconfig.enableSwingOutY())
			{
				double phaseDevXAbs = fabs(stateInfo.phaseDevFilt.x());
				double phaseDevY = stateInfo.phaseDevFilt.y();
				double swingTiltAxis = atan2(phaseDevY, phaseDevXAbs);
				swingTiltAxis = rc_utils::interpolateCoerced<double>(tpmconfig.swingYPhaseDevXLow(), tpmconfig.swingYPhaseDevXHigh(), 0.0, swingTiltAxis, phaseDevXAbs);
				swingTiltAxis = rc_utils::coerceSoftAbs<double>(swingTiltAxis, tpmconfig.swingYGammaMaxAbs(), tpmconfig.swingYGammaBuf());
				swingOut.y() = fabs(swingOut.x()) * tan(swingTiltAxis);
			}

			// Set the swing out tilt
			if(tpmconfig.enableSwingOutX() || tpmconfig.enableSwingOutY())
			{
				rot_conv::AbsTiltPhase2D swingOutFeed(swingOut.x(), swingOut.y());
				Vec2 limSwingOutMaxAxes = rc_utils::collapseFlatEllipsoid<double>(tpmconfig.limSwingOutXMax(), tpmconfig.limSwingOutYMax());
				rc_utils::coerceSoftEllC<double>(swingOutFeed.px, swingOutFeed.py, limSwingOutMaxAxes.x(), limSwingOutMaxAxes.y(), tpmconfig.limSwingOutBuf());
				rot_conv::AbsYawTiltFromAbsPhase(swingOutFeed, m_out.actionsCmd.swingOut);
				m_out.useActions.setFlags(ACFlag::SWINGOUT);
			}

			// Plotting
			if(m_PM->getEnabled())
			{
				m_PM->plotScalar(limpEnergyL, PM_TPM_SO_LIMPENERGY_L);
				m_PM->plotScalar(limpEnergyR, PM_TPM_SO_LIMPENERGY_R);
				m_PM->plotScalar(crossingEnergyL, PM_TPM_SO_CROSSINGENERGY_L);
				m_PM->plotScalar(crossingEnergyR, PM_TPM_SO_CROSSINGENERGY_R);
				m_PM->plotScalar(swingOutXL, PM_TPM_SO_SWINGOUTX_L);
				m_PM->plotScalar(swingOutXR, PM_TPM_SO_SWINGOUTX_R);
				m_PM->plotVec2d (swingOut, PM_TPM_SO_SWINGOUTX);
			}
		}

		// Update hip height maximum feedback function
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::updateHipMax(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo)
		{
			// Constants
			static const double InstabilityTol = 1e-8;
			static const double HipHeightMaxTol = 1e-8;

			// Data aliases
			const double hipHeightMin = modelInput.trajInfo.hipHeightMin;
			const double hipHeightNom = modelInput.trajInfo.hipHeightNom;

			// Calculate the change in filtered phase deviation since the last cycle
			Vec2 phaseDevDelta = stateInfo.phaseDevFilt - stateInfo.phaseDevFiltLast;

			// Disable axis contributions to instability as required
			if(!tpmconfig.instBasedOnX()) phaseDevDelta.x() = 0.0;
			if(!tpmconfig.instBasedOnY()) phaseDevDelta.y() = 0.0;

			// Calculate and low pass filter the rate of change of the phase deviation in terms of absolute arc traversal speed to get the instability
			double phaseDevSpeed = phaseDevDelta.norm() / modelInput.truedT;
			double instability = instLowPassFilter.put(phaseDevSpeed, tpmconfig.instLowPassMaxSlope() * modelInput.truedT);

			// Map instability values to a target hip height maximum
			double instForHipHeightNom = tpmconfig.instForHipHeightNom();
			double instForHipHeightMin = rc_utils::coerceMin<double>(tpmconfig.instForHipHeightMin(), instForHipHeightNom + InstabilityTol);
			double hipHeightMaxTarget = rc_utils::interpolateCoerced(instForHipHeightNom, instForHipHeightMin, hipHeightNom, hipHeightMin, instability);
			if(!tpmconfig.enableHipHeightMax()) hipHeightMaxTarget = hipHeightNom;

			// Slope limit the required maximum normalised hip height
			double hipHeightMaxLast = hipMaxSlopeLimiter.value();
			double hipHeightMax = hipMaxSlopeLimiter.put(hipHeightMaxTarget);

			// Set the hip height maximum
			if(tpmconfig.enableHipHeightMax() || fabs(hipHeightMax - hipHeightNom) > HipHeightMaxTol || fabs(hipHeightMaxLast - hipHeightNom) > HipHeightMaxTol)
			{
				m_out.actionsCmd.hipHeightMax = hipHeightMax;
				m_out.useActions.setFlags(ACFlag::HIPHEIGHTMAX);
			}

			// Plotting
			if(m_PM->getEnabled())
			{
				m_PM->plotScalar(phaseDevSpeed, PM_TPM_HHM_PHASEDEVSPEED);
				m_PM->plotScalar(instability, PM_TPM_HHM_INSTABILITY);
				m_PM->plotScalar(hipHeightMaxTarget, PM_TPM_HHM_HIPHEIGHTMAX_TGT);
				m_PM->plotScalar(hipHeightMax, PM_TPM_HHM_HIPHEIGHTMAX);
			}
		}

		//
		// Timing functions
		//

		// Update timing feedback function
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::updateTiming(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo)
		{
			// Calculate a modification to the gait phase frequency based on tilt phase feedback terms
			double timingFeedWeight = rc_utils::coerceAbs(-tpmconfig.timingWeightFactor() * sin(phaseInfo.gaitPhase - 0.5*modelInput.trajInfo.dblSuppPhaseLen), 1.0);
			double timingFeed = rc_utils::SmoothDeadband::eval(stateInfo.phaseDevFilt.x() * timingFeedWeight, tpmconfig.timingFeedDeadRadius());
			double timingFreqDelta = (timingFeed >= 0.0 ? tpmconfig.timingGainSpeedUp()*timingFeed : tpmconfig.timingGainSlowDown()*timingFeed);

			// Decide on the required commanded gait frequency
			double gaitFrequency = modelInput.nomGaitFrequency + timingFreqDelta;

			// Set the output timing command
			if(tpmconfig.enableTiming())
			{
				m_out.timingCmd.gaitFrequency = gaitFrequency;
				m_out.useTiming = true;
			}

			// Plotting
			if(m_PM->getEnabled())
			{
				m_PM->plotScalar(timingFeedWeight, PM_TPM_TIM_FEEDWEIGHT);
				m_PM->plotScalar(timingFreqDelta, PM_TPM_TIM_FREQDELTA);
			}
		}

		//
		// Step size functions
		//

		// Update step size feedback function
		template<class Kinematics> void FeedTiltPhaseModel<Kinematics>::updateStepSize(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo)
		{
			// Data aliases
			double th = stateInfo.syncPhase.y();
			double thdot = stateInfo.syncPhaseD.y();

			// Calculate the sagittal crossing energies of the current state, normalised by the pendulum constants
			tripendulum::CrossingEnergy CE = stepTriPendModelSag.crossingEnergy(th, thdot);
			CE.B /= stepTriPendModelSag.Cbsq;
			CE.F /= stepTriPendModelSag.Cfsq;

			// Calculate the backwards crossing sagittal gait command vector adjustment
			double gcvDeltaXB = 0.0;
			if(CE.B >= tpmconfig.stepSCrossingEMinB())
				gcvDeltaXB = -tpmconfig.stepSCrossingEToGcvX() * rc_utils::SmoothDeadband::eval(CE.B, tpmconfig.stepSCrossingEDeadRadB(), tpmconfig.stepSCrossingEMinB());
			gcvDeltaXB = rc_utils::interpolateCoerced(stepTriPendModelSag.ths, stepTriPendModelSag.thm, 0.0, gcvDeltaXB, th);

			// Calculate the forwards crossing sagittal gait command vector adjustment
			double gcvDeltaXF = 0.0;
			if(CE.F >= tpmconfig.stepSCrossingEMinF())
				gcvDeltaXF = tpmconfig.stepSCrossingEToGcvX() * rc_utils::SmoothDeadband::eval(CE.F, tpmconfig.stepSCrossingEDeadRadF(), tpmconfig.stepSCrossingEMinF());
			gcvDeltaXF = rc_utils::interpolateCoerced(stepTriPendModelSag.tht, stepTriPendModelSag.thm, 0.0, gcvDeltaXF, th);

			// Hold the sagittal gait command vector adjustments for some time to give them a chance to take effect
			double gcvDeltaXBH = stepGcvXBHoldFilter.update(gcvDeltaXB);
			double gcvDeltaXFH = stepGcvXFHoldFilter.update(gcvDeltaXF);

			// Combine the forwards and backwards components into a total sagittal gait command vector adjustment
			double gcvDeltaX = rc_utils::coerceSoftAbs<double>(gcvDeltaXBH + gcvDeltaXFH, tpmconfig.stepGcvDeltaXMaxAbs(), tpmconfig.stepGcvDeltaXBuf());

			// Set the step size x
			if(tpmconfig.enableStepSizeX())
			{
				m_out.stepSizeCmd.X.set(modelInput.inputGcv.x(), 0.0, gcvDeltaX);
				m_out.useStepSize = true;
			}

			// Set the step size y
			if(tpmconfig.enableStepSizeY())
			{
				m_out.stepSizeCmd.Y.set(modelInput.inputGcv.y(), 0.0, 0.0);
				m_out.useStepSize = true;
			}

			// Set the step size z
			if(tpmconfig.enableStepSizeZ())
			{
				m_out.stepSizeCmd.Z.set(modelInput.inputGcv.z(), 0.0, 0.0);
				m_out.useStepSize = true;
			}

			// Plotting
			if(m_PM->getEnabled())
			{
				m_PM->plotScalar(stepTriPendModelSag.tht, PM_TPM_SS_THETABM);
				m_PM->plotScalar(stepTriPendModelSag.ths, PM_TPM_SS_THETAMF);
				m_PM->plotScalar(CE.B, PM_TPM_SS_CROSSENERGY_B);
				m_PM->plotScalar(CE.F, PM_TPM_SS_CROSSENERGY_F);
				m_PM->plotScalar(gcvDeltaXB, PM_TPM_SS_GCVDELTAX_B);
				m_PM->plotScalar(gcvDeltaXF, PM_TPM_SS_GCVDELTAX_F);
				m_PM->plotScalar(gcvDeltaX, PM_TPM_SS_GCVDELTAX);
			}
		}
	}
}

#endif
// EOF