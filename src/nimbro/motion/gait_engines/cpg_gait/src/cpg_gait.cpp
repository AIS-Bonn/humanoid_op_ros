// Central pattern generated gait
// File: cpg_gait.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <cpg_gait/cpg_gait.h>
#include <robotcontrol/model/robotmodel.h>
#include <pluginlib/class_list_macros.h>
#include <nimbro_utils/math_funcs.h>

// Namespaces
using namespace cpg_gait;
using namespace gait;
using namespace nimbro_utils;

//
// CPGGait class
//

// Default constructor
CPGGait::CPGGait() : GaitEngine()
 , config()
 , CONFIG_PARAM_PATH("/cpg_gait/")
 , m_plotData(CONFIG_PARAM_PATH + "plotData", false)
 , m_PM(PM_COUNT, "/cpg_gait")
{
	// Reset the gait engine
	CPGGait::reset();

	// Configure the plot manager
	configurePlotManager();

	// Set configuration parameter callbacks
	m_plotData.setCallback(boost::bind(&CPGGait::callbackPlotData, this));

	// Initial call to configuration parameter callbacks
	callbackPlotData();
}

// Reset function
void CPGGait::reset()
{
	// Reset the pose variables
	m_jointPose.reset();
	m_jointHaltPose.reset();
	m_inversePose.reset();
	m_abstractPose.reset();
	m_abstractHaltPose.reset();

	// Reset the gait command vector variables
	m_gcv.setZero();
	m_gcvInput.setZero();

	// Reset the gait flags
	m_walk = false;
	m_walking = false;
	m_leftLegFirst = true;
	m_walkStartBlending = true;

	// Reset the step motion variables
	m_gaitPhase = 0.0;
	m_virtualSlope = 0.0;
}

// Update halt pose function
void CPGGait::updateHaltPose()
{
	// Do not use raw joint commands
	haltUseRawJointCmds = false;

	// Set the halt pose for the legs
	m_abstractHaltPose.leftLeg .setPoseMirrored(config.haltLegExtension(), config.haltLegAngleX(), config.haltLegAngleY(), config.haltLegAngleZ());
	m_abstractHaltPose.rightLeg.setPoseMirrored(config.haltLegExtension(), config.haltLegAngleX(), config.haltLegAngleY(), config.haltLegAngleZ());
	m_abstractHaltPose.leftLeg .setFootPoseMirrored(config.haltFootAngleX(), config.haltFootAngleY());
	m_abstractHaltPose.rightLeg.setFootPoseMirrored(config.haltFootAngleX(), config.haltFootAngleY());

	// Set the halt pose for the arms
	m_abstractHaltPose.leftArm .setPoseMirrored(config.haltArmExtension(), config.haltArmAngleX(), config.haltArmAngleY());
	m_abstractHaltPose.rightArm.setPoseMirrored(config.haltArmExtension(), config.haltArmAngleX(), config.haltArmAngleY());

	// Set the halt joint efforts for the arms
	m_abstractHaltPose.leftArm .cad.setEffort(config.haltEffortArm());
	m_abstractHaltPose.rightArm.cad.setEffort(config.haltEffortArm());

	// Set the halt joint efforts for the legs
	m_abstractHaltPose.leftLeg .cld.setEffort(config.haltEffortHipYaw(), config.haltEffortHipRoll(), config.haltEffortHipPitch(), config.haltEffortKneePitch(), config.haltEffortAnklePitch(), config.haltEffortAnkleRoll());
	m_abstractHaltPose.rightLeg.cld.setEffort(config.haltEffortHipYaw(), config.haltEffortHipRoll(), config.haltEffortHipPitch(), config.haltEffortKneePitch(), config.haltEffortAnklePitch(), config.haltEffortAnkleRoll());

	// Set the halt pose support coefficients
	m_abstractHaltPose.leftLeg .cld.setSupportCoeff(0.5);
	m_abstractHaltPose.rightLeg.cld.setSupportCoeff(0.5);

	// Convert the halt pose into the joint representation
	m_jointHaltPose.setFromAbstractPose(m_abstractHaltPose);

	// Transcribe the joint halt pose to the required halt pose arrays
	m_jointHaltPose.writeJointPosArray(haltJointCmd);
	m_jointHaltPose.writeJointEffortArray(haltJointEffort);
}

// Step function
void CPGGait::step()
{
	// Clear the plot manager for a new step
	m_PM.clear();

	// Process the gait engine inputs
	processInputs();

	// Update the halt pose (updates m_abstractHaltPose, m_jointHaltPose and the inherited halt pose variables)
	updateHaltPose();

	// Generate the arm and leg motions if the gait is currently supposed to be walking
	if(m_walking)
	{
		// Initialise the abstract pose to the halt pose
		m_abstractPose = m_abstractHaltPose;

		// Leg motions
		if(config.tuningNoLegs())
		{
			// Convert legs: Abstract --> Joint
			m_jointPose.setLegsFromAbstractPose(m_abstractPose.leftLeg, m_abstractPose.rightLeg);
		}
		else
		{
			// Generate the abstract leg motion
			abstractLegMotion(m_abstractPose.leftLeg);
			abstractLegMotion(m_abstractPose.rightLeg);

			// Convert legs: Abstract --> Inverse
			m_inversePose.setLegsFromAbstractPose(m_abstractPose.leftLeg, m_abstractPose.rightLeg);

			// Generate the inverse leg motion
			inverseLegMotion(m_inversePose.leftLeg);
			inverseLegMotion(m_inversePose.rightLeg);

			// Convert legs: Inverse --> Joint
			m_jointPose.setLegsFromInversePose(m_inversePose.leftLeg, m_inversePose.rightLeg);
		}

		// Arm motions
		if(config.tuningNoArms())
		{
			// Convert arms: Abstract --> Joint
			m_jointPose.setArmsFromAbstractPose(m_abstractPose.leftArm, m_abstractPose.rightArm);
		}
		else
		{
			// Calculate the abstract arm motion
			abstractArmMotion(m_abstractPose.leftArm);
			abstractArmMotion(m_abstractPose.rightArm);

			// Convert arms: Abstract --> Joint
			m_jointPose.setArmsFromAbstractPose(m_abstractPose.leftArm, m_abstractPose.rightArm);
		}

		// TODO: m_jointPose.blendTowards(m_jointHaltPose);
		// TODO: Perform blending on the joint level, dependent on gait phase! (cos^2 factor, m_walkStartBlending)
	}
	else
	{
		// Set the working joint pose to the halt pose
		m_jointPose = m_jointHaltPose;
	}

	// Update the gait engine outputs
	updateOutputs();

	// Publish the plot data
	m_PM.publish();
}

// Reset walking function
void CPGGait::resetWalking(bool walking, const Eigen::Vector3d& gcvBias)
{
	// Reset variables
	m_walking = walking;
	m_gcv = gcvBias;
	m_gaitPhase = 0.0;
	m_virtualSlope = 0.0;
	m_leftLegFirst = config.leftLegFirst();
	m_walkStartBlending = true;
}

// Process inputs function
void CPGGait::processInputs()
{
	// Retrieve the robot's fused angle
	const double fusedY = model->robotFPitchPR();
       
	// Transcribe whether it is desired of us to walk right now
	m_walk = in.gaitCmd.walk;

	// Transcribe and bias the desired gait velocity
	Eigen::Vector3d gcvBias(config.gcvBiasLinVelX(), config.gcvBiasLinVelY(), config.gcvBiasAngVelZ());
	if(m_walk) m_gcvInput = gcvBias + Eigen::Vector3d(in.gaitCmd.linVelX, in.gaitCmd.linVelY, in.gaitCmd.angVelZ);
	else       m_gcvInput = gcvBias;

	// Perform update tasks if walking is active
	if(m_walking)
	{
		// Calculate the unbiased gcv
		Eigen::Vector3d gcvUnbiased = m_gcv - gcvBias;

		// Update the internal gcv based on the gcv input using a slope-limiting approach
		double dT = in.truedT;
		float D = config.gcvDecToAccRatio();
		if(gcvUnbiased.x() >= 0.0) m_gcv.x() += coerce(m_gcvInput.x() - m_gcv.x(), -dT*config.gcvAccForwards()*D  , dT*config.gcvAccForwards()    );
		else                       m_gcv.x() += coerce(m_gcvInput.x() - m_gcv.x(), -dT*config.gcvAccBackwards()   , dT*config.gcvAccBackwards()*D );
		if(gcvUnbiased.y() >= 0.0) m_gcv.y() += coerce(m_gcvInput.y() - m_gcv.y(), -dT*config.gcvAccSidewards()*D , dT*config.gcvAccSidewards()   );
		else                       m_gcv.y() += coerce(m_gcvInput.y() - m_gcv.y(), -dT*config.gcvAccSidewards()   , dT*config.gcvAccSidewards()*D );
		if(gcvUnbiased.z() >= 0.0) m_gcv.z() += coerce(m_gcvInput.z() - m_gcv.z(), -dT*config.gcvAccRotational()*D, dT*config.gcvAccRotational()  );
		else                       m_gcv.z() += coerce(m_gcvInput.z() - m_gcv.z(), -dT*config.gcvAccRotational()  , dT*config.gcvAccRotational()*D);

		// Save the old gait phase
		double oldGaitPhase = m_gaitPhase;

		// Update the gait phase
		double stepTime = config.nominalStepTime();
		double gaitPhaseInc = M_PI * in.truedT / stepTime;
		m_gaitPhase = picut(m_gaitPhase + gaitPhaseInc); // The gait phase must be in the range (-pi,pi], and thus must be wrapped!

		// Virtual slope
		m_virtualSlope = 0.0;
		if(config.cmdVirtualSlope())
		{
			m_virtualSlope = config.mgVirtualSlopeOffset();
			double dev = fusedY - config.mgVirtualSlopeMidAngle();
			double absdev = fabs(dev);
			if(absdev > config.mgVirtualSlopeMinAngle())
				m_virtualSlope += sign(dev) * (dev*m_gcv.x() > 0 ? config.mgVirtualSlopeGainAsc() : config.mgVirtualSlopeGainDsc()) * (absdev - config.mgVirtualSlopeMinAngle());
		}

		// Check whether we should stop walking
		double nominalGaitPhaseInc = M_PI * in.nominaldT / config.nominalStepTime();
		double LB = nominalGaitPhaseInc * config.stoppingPhaseTolLB(); // Lower bound tolerance config variable is in the units of nominal phase increments, calculated using nominalStepTime (also a config variable)
		double UB = nominalGaitPhaseInc * config.stoppingPhaseTolUB(); // Upper bound tolerance config variable is in the units of nominal phase increments, calculated using nominalStepTime (also a config variable)
		if(!m_walk && gcvUnbiased.norm() <= config.stoppingGcvMag() &&
		  ((m_gaitPhase >= 0.0 && oldGaitPhase <= 0.0 && (m_gaitPhase <= UB || oldGaitPhase >= -LB)) ||
		   (m_gaitPhase <= 0.0 && oldGaitPhase >= 0.0 && (m_gaitPhase <= UB - M_PI || oldGaitPhase >= M_PI - LB))))
		{
			resetWalking(false, gcvBias);
		}
	}

	// Check whether we should start walking
	if(m_walk && !m_walking)
		resetWalking(true, gcvBias);

	// Plot data
	if(m_PM.getEnabled())
	{
		m_PM.plotVec3d(m_gcv, PM_GCV);
		m_PM.plotScalar(m_gaitPhase, PM_GAIT_PHASE);
	}
}

// Step the gait arm motion
void CPGGait::abstractArmMotion(gait::AbstractArmPose& arm) // 'arm' is assumed to be passed to this function containing the desired arm halt pose
{
	//
	// Limb phase and swing angle calculations
	//

	// The limb phase and swing angles for the arms are calculated exactly like for the legs, only the limb phase is inverted.
	// That is, the support and swing phases of the left arm match those of the right leg, and vice versa.

	// Calculate the limb phase (either exactly the same as the gait phase, or in exact antiphase to it)
	double limbPhase = (arm.cad.isLeft != m_leftLegFirst ? m_gaitPhase : picut(m_gaitPhase + M_PI)); // Negated comparison => Left leg first implies right arm is first!

	// Retrieve the swing phase offsets
	double swingStartPhase = config.swingStartPhase();          // Limb phase at which the swing phase should start
	double swingStopPhase = config.swingStopPhase();            // Limb phase at which the swing phase should stop
	double sinusoidPhaseLen = swingStopPhase - swingStartPhase; // Length of the swing phase
	double linearPhaseLen = M_2PI - sinusoidPhaseLen;           // Length of the support phase

	// Calculate the current dimensionless swing angle for the limb
	double swingAngle;
	if(limbPhase >= swingStartPhase && limbPhase <= swingStopPhase)
		swingAngle = -cos(M_PI * (limbPhase - swingStartPhase) / sinusoidPhaseLen);       // Sinusoid forwards swing from dimensionless angle -1 to +1 (section from phase swingStartPhase to phase swingStopPhase)
	else if(limbPhase > swingStopPhase)
		swingAngle = 1.0 - (2.0 / linearPhaseLen) * (limbPhase - swingStopPhase);         // Linear backwards swing from dimensionless angle +1 to a mid-way point C (section from phase swingStopPhase to phase pi)
	else
		swingAngle = 1.0 - (2.0 / linearPhaseLen) * (limbPhase - swingStopPhase + M_2PI); // Linear backwards swing from dimensionless angle C to -1 (section from phase -pi to phase swingStartPhase)
	if(config.tuningNoArmSwing())
		swingAngle = 0.0;

	//
	// Sagittal arm swing (angleY)
	//

	// Apply the sagittal arm swing to the abstract arm pose
	double armSagSwingMag = config.armSagSwingMag() + m_gcv.x()*config.armSagSwingMagGradX();
	double armSagSwing = -swingAngle * armSagSwingMag;
	arm.angleY += armSagSwing;

	//
	// Plotting
	//

	// Plot the arm motion components
	if(m_PM.getEnabled())
	{
		m_PM.plotScalar(swingAngle, PM_ARM_SWING_ANGLE_R + arm.cad.isLeft);
		m_PM.plotScalar(armSagSwing, PM_ARM_SAG_SWING_R + arm.cad.isLeft);
	}
}

// Step the gait leg motion
void CPGGait::abstractLegMotion(gait::AbstractLegPose& leg) // 'leg' is assumed to be passed to this function containing the desired leg halt pose
{
	//
	// Initialise variables
	//

	// Calculate the absolute values of the current gait command
	double absGcvX = fabs(m_gcv.x());
	double absGcvY = fabs(m_gcv.y());
	double absGcvZ = fabs(m_gcv.z());

	//
	// Limb phase and swing angle calculations
	//

	// The legs swing about all three axes to achieve the required walking velocity. They do this with a special timing and waveform,
	// summarised by the dimensionless swing angle waveform template. This template consists of a sinusoidal 'forwards' motion, followed
	// by a linear 'backwards' motion. The forwards motion is referred to as the swing phase of the corresponding limb, while the backwards
	// motion is referred to as the support phase. This alludes to the expected behaviour that the foot of a particular leg should be in
	// contact with the ground during that leg's support phase, and in the air during its swing phase. The ratio of swing time to support
	// time in a human gait is typically 40 to 60, and so the swing phase is made shorter. The exact timing of the swing phase is defined
	// by the swingStartPhase and swingStopPhase configuration variables. It is expected that the foot achieves toe-off at approximately
	// the swing start phase, and heel-strike at approximately the swing stop phase. The asymmetry of the two phases leads to a brief
	// double support phase, where both legs are expected to be in contact with the ground simultaneously, performing their backwards swing.

	// Calculate the limb phase (either exactly the same as the gait phase, or in exact antiphase to it)
	// m_gaitPhase starts at 0 at the beginning of walking, and has the swing phase of the first leg in [0,pi] (left or right depending on m_leftLegFirst)
	// absPhase is in exact phase or antiphase to m_gaitPhase, and always has the swing phase of the left leg in [0,pi]
	// limbPhase is in exact phase or antiphase to m_gaitPhase, and always has the swing phase of 'this' leg in [0,pi]
	double oppGaitPhase = picut(m_gaitPhase + M_PI);
	double absPhase = (m_leftLegFirst ? m_gaitPhase : oppGaitPhase);
	double limbPhase = (leg.cld.isLeft == m_leftLegFirst ? m_gaitPhase : oppGaitPhase);

	// Retrieve the swing phase offsets
	double swingStartPhase = config.swingStartPhase();          // Limb phase at which the swing phase should start
	double swingStopPhase = config.swingStopPhase();            // Limb phase at which the swing phase should stop
	double sinusoidPhaseLen = swingStopPhase - swingStartPhase; // Length of the swing phase
	double linearPhaseLen = M_2PI - sinusoidPhaseLen;           // Length of the support phase

	// Calculate the current dimensionless swing angle for the limb
	double swingAngle;
	if(limbPhase >= swingStartPhase && limbPhase <= swingStopPhase)
		swingAngle = -cos(M_PI * (limbPhase - swingStartPhase) / sinusoidPhaseLen);       // Sinusoid forwards swing from dimensionless angle -1 to +1 (section from phase swingStartPhase to phase swingStopPhase)
	else if(limbPhase > swingStopPhase)
		swingAngle = 1.0 - (2.0 / linearPhaseLen) * (limbPhase - swingStopPhase);         // Linear backwards swing from dimensionless angle +1 to a mid-way point C (section from phase swingStopPhase to phase pi)
	else
		swingAngle = 1.0 - (2.0 / linearPhaseLen) * (limbPhase - swingStopPhase + M_2PI); // Linear backwards swing from dimensionless angle C to -1 (section from phase -pi to phase swingStartPhase)
	if(config.tuningNoLegSwing())
		swingAngle = 0.0;

	//
	// Leg lifting (extension)
	//

	// The leg is alternately lifted off the ground (step) and pushed down into it (push), synchronised by the exact intervals of pi in the
	// gait phase (as opposed to being synchronised with the swing/support phases), with no breaks in-between. The leg push phase assists
	// with achieving foot clearance of the swing foot.

	// Apply leg stepping and pushing to the abstract leg pose
	double legExtensionOffset = 0.0;
	if(!config.tuningNoLegLifting())
	{
		if(limbPhase >= 0.0) // Leg stepping phase => Gait phase in the range [0,pi]
		{
			double stepHeight = config.legStepHeight() + absGcvX*config.legStepHeightGradX() + absGcvY*config.legStepHeightGradY();
			legExtensionOffset = stepHeight * sin(limbPhase);
		}
		else // Leg pushing phase => Gait phase in the range [-pi,0)
		{
			double pushHeight = config.legPushHeight() + absGcvX*config.legPushHeightGradX();
			legExtensionOffset = pushHeight * sin(limbPhase);
		}
		leg.extension += legExtensionOffset;
	}

	//
	// Sagittal leg swing (angleY)
	//

	// Apply the sagittal leg swing to the abstract leg pose (responsible for fulfilling the gcv x-velocity)
	// We use a different sagittal leg swing gradient depending on whether we're walking forwards or backwards.
	double legSagSwingMag = m_gcv.x() * (m_gcv.x() >= 0.0 ? config.legSagSwingMagGradXFwd() : config.legSagSwingMagGradXBwd());
	double legSagSwing = -swingAngle * legSagSwingMag;
	leg.angleY += legSagSwing;

	//
	// Lateral leg pushout, lateral leg swing and lateral hip swing (angleX)
	//

	// Apply the lateral leg pushout to the abstract leg pose
	// This rolls the legs outwards (from the hip) in proportion to each of the absolute gcv values. This acts to separate the feet
	// more the faster the robot is walking, and the more the robot is trying to walk with combined velocity components (e.g. forwards
	// velocity coupled with a rotational velocity). This term seeks to prevent foot to foot self-collisions, and should more be
	// seen as a bias to the halt pose, as opposed to a dynamic motion component of the gait. This is because for constant walking
	// velocity command, this term is constant.
	double legLatPushoutMag = absGcvX*config.legLatPushoutMagGradX() + absGcvY*config.legLatPushoutMagGradY() + absGcvZ*config.legLatPushoutMagGradZ();
	if(!config.tuningNoLegPushout()) leg.angleX += leg.cld.limbSign * legLatPushoutMag;

	// Apply the lateral leg swing to the abstract leg pose
	// This is responsible for fulfilling the gcv y-velocity.
	double legLatSwingMag = m_gcv.y() * config.legLatSwingMagGradY();
	double legLatSwing = swingAngle * legLatSwingMag;
	leg.angleX += legLatSwing;

	// The lateral hip swing motion component sways the hips left and right relative to the feet. This is achieved via direct addition
	// of a hip swing term to the hip roll (i.e. angleX). Hip swing to the left occurs when the left foot is in its support phase
	// (i.e. absolute phase in (-pi,0]), and should correspond to negative roll. Hip swing to the right occurs when the right foot is
	// in its support phase (i.e. absolute phase in (0,pi]), and should correspond to positive roll. The hip swing is applied equally
	// to both hip roll joints, and is calculated as the sum of two overlapping sinusoid 'halves', one for the hip swing to the left,
	// and one for the hip swing to the right. The overlap between the sinusoids occurs during the double support phase, during which
	// time the sinusoids sum up, and the hip travels from one body side to the other. To be even more explicit, the left hip swing
	// sinusoid completes exactly half a sinusoid in the time that the left leg is in its support phase, and is zero everywhere else.
	// The right hip swing sinusoid behaves similarly, and they are added up to get the full dimensionless hip swing waveform.

	// Perform phase calculations for the left and right halves (sinusoids) of the hip swing motion
	double swingStopPhaseR = swingStopPhase - M_PI;
	double hipSwingPhaseL  = (absPhase < swingStopPhase  ? absPhase + M_2PI : absPhase);
	double hipSwingPhaseR  = (absPhase < swingStopPhaseR ? absPhase + M_2PI : absPhase);

	// Calculate the dimensionless hip swing angle by summing up the two zero-coerced sinusoid sub-waveforms
	double hipSwingAngleL = -coerceMin(sin(M_PI*(hipSwingPhaseL - swingStopPhase )/linearPhaseLen), 0.0);
	double hipSwingAngleR =  coerceMin(sin(M_PI*(hipSwingPhaseR - swingStopPhaseR)/linearPhaseLen), 0.0);
	double hipSwingAngle  =  hipSwingAngleL + hipSwingAngleR;

	// Apply the lateral hip swing to the abstract leg pose
	double legLatHipSwingMag = config.legLatHipSwingMag() + absGcvX*config.legLatHipSwingMagGradX() + absGcvY*config.legLatHipSwingMagGradY();
	double legLatHipSwing = config.legLatHipSwingBias() + hipSwingAngle * legLatHipSwingMag;
	if(!config.tuningNoLegHipSwing()) leg.angleX += legLatHipSwing;

	//
	// Rotational leg V pushout and rotational leg swing (angleZ)
	//

	// Apply the rotational leg V pushout to the abstract leg pose
	// Offset the yaw of the feet to be in a tendentially more toe-out configuration, the faster we are expecting the robot to turn.
	// This is referred to as rotational V pushout, and has the effect of attempting to avoid toe to toe self-collisions. For a constant
	// walking velocity command, this term is constant, and hence should be considered to be more of a bias to the hold pose, rather
	// than a dynamic motion component of the gait.
	double legRotVPushoutMag = absGcvZ * config.legRotVPushoutMagGradZ();
	if(!config.tuningNoLegPushout()) leg.angleZ += leg.cld.limbSign * legRotVPushoutMag;

	// Apply the rotational leg swing to the abstract leg pose
	// This is responsible for fulfilling the gcv z-velocity.
	double legRotSwingMag = m_gcv.z() * config.legRotSwingMagGradZ();
	double legRotSwing = swingAngle * legRotSwingMag;
	leg.angleZ += legRotSwing;

	//
	// Leaning (angleX, angleY)
	//

	// Apply the leaning to the abstract leg pose
	// When walking at higher speeds, it is advantageous to make use of leaning to assist with balance. Velocity-based leaning
	// is chosen over acceleration-based leaning in the interest of producing continuous abstract pose outputs.
	// TODO: Lean based on a smoothed out difference between gcv and gcvInput, so it's a little like acceleration?
	if(!config.tuningNoLegLeaning())
	{
		// Lean forwards and backwards based on the sagittal walking velocity
		double legSagLean = m_gcv.x() * (m_gcv.x() >= 0.0 ? config.legSagLeanGradXFwd() : config.legSagLeanGradXBwd()) + absGcvZ * config.legSagLeanGradZ();
		leg.angleY += legSagLean;

		// Lean laterally into curves (based on the rotational walking velocity)
		double legLatLean = absGcvX * m_gcv.z() * config.legLatLeanGradXZ();
		leg.angleX += -legLatLean;
	}

	//
	// Ankle rotations (footAngleX, footAngleY)
	//

	// TODO: Rotate the ankles how?

	//
	// Support coefficients
	//

	// The support coefficient gives the proportion of the robot's weight that is expected to be on this leg at the current time.
	// It is a dimensionless parameter in the range from 0 to 1, and ideally the sum of the left and right leg support coefficients
	// should be 1 at all times. We anticipate that the foot strike and lifting occurs in synchronisation to swingStartPhase and
	// swingStopPhase, and so we set the support coefficient to 0 between these two limb phases (during the swing phase of this leg),
	// and to 1 during the swing phase of the opposite leg. Inbetween we linearly blend the support coefficients between 0 and 1.

	// Calculate the phase length over which the support coefficient transition should take place
	double supportTransitionPhaseLen = M_PI - sinusoidPhaseLen;
	double supportTransitionSlope = 1e10;
	if(supportTransitionPhaseLen > 1e-10)
		supportTransitionSlope = 1.0 / supportTransitionPhaseLen;

	// Calculate the required 
	double shiftedLimbPhase = picut(limbPhase - swingStartPhase);
	if(shiftedLimbPhase >= 0.0)
		leg.cld.supportCoeff = coerce(supportTransitionSlope*(shiftedLimbPhase - sinusoidPhaseLen), 0.0, 1.0);
	else
		leg.cld.supportCoeff = coerce(supportTransitionSlope*(-shiftedLimbPhase), 0.0, 1.0);

	//
	// Plotting
	//

	// Plot the leg motion components
	if(m_PM.getEnabled())
	{
		m_PM.plotScalar(legExtensionOffset, PM_LEG_EXTENSION_R + leg.cld.isLeft);
		m_PM.plotScalar(swingAngle, PM_LEG_SWING_ANGLE_R + leg.cld.isLeft);
		m_PM.plotScalar(legSagSwing, PM_LEG_SAG_SWING_R + leg.cld.isLeft);
		m_PM.plotScalar(legLatSwing, PM_LEG_LAT_SWING_R + leg.cld.isLeft);
		m_PM.plotScalar(legRotSwing, PM_LEG_ROT_SWING_R + leg.cld.isLeft);
		m_PM.plotScalar(legLatHipSwing, PM_LEG_LAT_HIP_SWING_R + leg.cld.isLeft);
	}
}

// Generate the inverse leg motion
void CPGGait::inverseLegMotion(InverseLegPose& leg) // 'leg' is assumed to contain the desired abstract motion
{
	//
	// Limb phase calculation
	//

	// Calculate the limb phase (either exactly the same as the gait phase, or in exact antiphase to it)
	// m_gaitPhase starts at 0 at the beginning of walking, and has the swing phase of the first leg in [0,pi] (left or right depending on m_leftLegFirst)
	// limbPhase is in exact phase or antiphase to m_gaitPhase, and always has the swing phase of 'this' leg in [0,pi]
	double oppGaitPhase = picut(m_gaitPhase + M_PI);
	double limbPhase = (leg.cld.isLeft == m_leftLegFirst ? m_gaitPhase : oppGaitPhase);

	//
	// Virtual slope leg lifting
	//

	// Adjust the lift height of the foot depending on the virtual slope (a slope derived from the pitch fused angle and a configured offset).
	// This has the qualitative effect that the robot lifts its feet more when it is falling forwards.
	double virtualComponent = 0.0;
	if(config.cmdVirtualSlope())
	{
		double virtualSlopeFactor = m_virtualSlope * m_gcv.x();
		if(virtualSlopeFactor >= 0.0)
			virtualComponent = virtualSlopeFactor * fabs(limbPhase / M_PI);
		else
			virtualComponent = virtualSlopeFactor * (fabs(limbPhase / M_PI) - 1.0);
		leg.footPos.z() += virtualComponent;
	}

	//
	// Plotting
	//

	// Plot the inverse leg motion components
	if(m_PM.getEnabled())
	{
		m_PM.plotScalar(m_virtualSlope, PM_LEG_VIRTUAL_SLOPE_R + leg.cld.isLeft);
		m_PM.plotScalar(virtualComponent, PM_LEG_VIRTUAL_COMP_R + leg.cld.isLeft);
	}
}

// Update outputs function
void CPGGait::updateOutputs()
{
	// Transcribe the joint commands
	m_jointPose.writeJointPosArray(out.jointCmd);
	m_jointPose.writeJointEffortArray(out.jointEffort);
	out.useRawJointCmds = haltUseRawJointCmds;

	// Transcribe the walking flag
	out.walking = m_walking;

	// Transcribe the leg support coefficients
	out.supportCoeffLeftLeg  = m_jointPose.leftLeg.cld.supportCoeff;
	out.supportCoeffRightLeg = m_jointPose.rightLeg.cld.supportCoeff;

	// Set the robot odometry to zero
	out.odomPosition[0] = 0.0;
	out.odomPosition[1] = 0.0;
	out.odomPosition[2] = 0.0;
	out.odomOrientation[0] = 1.0;
	out.odomOrientation[1] = 0.0;
	out.odomOrientation[2] = 0.0;
	out.odomOrientation[3] = 0.0;

	// TODO: Odometry information: out.odomPosition, out.odomOrientation
}

// Configure the plot manager
void CPGGait::configurePlotManager()
{
	// Configure gait command vector variables
	m_PM.setName(PM_GCV_X, "gcv/linVelX");
	m_PM.setName(PM_GCV_Y, "gcv/linVelY");
	m_PM.setName(PM_GCV_Z, "gcv/angVelZ");

	// Configure step motion variables
	m_PM.setName(PM_GAIT_PHASE,          "gaitPhase");
	m_PM.setName(PM_ARM_SWING_ANGLE_L,   "leftArm/unitSwingAngle");
	m_PM.setName(PM_ARM_SWING_ANGLE_R,   "rightArm/unitSwingAngle");
	m_PM.setName(PM_ARM_SAG_SWING_L,     "leftArm/sagittalSwing");
	m_PM.setName(PM_ARM_SAG_SWING_R,     "rightArm/sagittalSwing");
	m_PM.setName(PM_LEG_EXTENSION_L,     "leftLeg/extensionOffset");
	m_PM.setName(PM_LEG_EXTENSION_R,     "rightLeg/extensionOffset");
	m_PM.setName(PM_LEG_SWING_ANGLE_L,   "leftLeg/swingAngle");
	m_PM.setName(PM_LEG_SWING_ANGLE_R,   "rightLeg/swingAngle");
	m_PM.setName(PM_LEG_SAG_SWING_L,     "leftLeg/sagittalSwing");
	m_PM.setName(PM_LEG_SAG_SWING_R,     "rightLeg/sagittalSwing");
	m_PM.setName(PM_LEG_LAT_SWING_L,     "leftLeg/lateralSwing");
	m_PM.setName(PM_LEG_LAT_SWING_R,     "rightLeg/lateralSwing");
	m_PM.setName(PM_LEG_ROT_SWING_L,     "leftLeg/rotationalSwing");
	m_PM.setName(PM_LEG_ROT_SWING_R,     "rightLeg/rotationalSwing");
	m_PM.setName(PM_LEG_LAT_HIP_SWING_L, "leftLeg/lateralHipSwing");
	m_PM.setName(PM_LEG_LAT_HIP_SWING_R, "rightLeg/lateralHipSwing");
	m_PM.setName(PM_LEG_VIRTUAL_COMP_L,  "leftLeg/virtualComponent");
	m_PM.setName(PM_LEG_VIRTUAL_COMP_R,  "rightLeg/virtualComponent");
	m_PM.setName(PM_LEG_VIRTUAL_SLOPE_L, "leftLeg/virtualSlope");
	m_PM.setName(PM_LEG_VIRTUAL_SLOPE_R, "rightLeg/virtualSlope");
}

// Callback for when the plotData parameter is updated
void CPGGait::callbackPlotData()
{
	// Enable or disable plotting as required
	if(m_plotData()) m_PM.enable();
	else             m_PM.disable();
}

PLUGINLIB_EXPORT_CLASS(cpg_gait::CPGGait, gait::GaitEngine)
// EOF