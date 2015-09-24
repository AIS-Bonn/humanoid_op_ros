#include "DynamicGait.h"
#include "Globals.h"
// #include "StepController/limp.h"
#include <math.h>

namespace indep_cpg_gait
{
	DynamicGait::DynamicGait()
	{
		gaitPhase = 0;
		firstStepTimer = 0;
		gaitFrequency = 0;
		expectedEnergy = 0;
		currentXOffsetOffset = 0;
	}

	void DynamicGait::init()
	{
		gaitPhase = 0;
		firstStepTimer = config.firstStepDuration;
		gaitFrequency = config.gaitFrequency;
		currentXOffsetOffset = 0;
	}

	void DynamicGait::reset()
	{
		gcv = Vec3f();
		gaitPhase = 0;
		gaitFrequency = config.gaitFrequency;
		firstStepTimer = config.firstStepDuration;
		currentXOffsetOffset = 0;
	}

	// Step controller.
	// Gait target handling.
	// Applies limits on the gcv target.
	// Slopes the gcv towards the target.
	// Calculates the gait frequency.
	// Handles first step.
	void DynamicGait::update()
	{
		// RESET
		// After the gait phase flip (or support exchange), the gcv, the frequency and the expected energy are reset to a nominal value.
		// The gait phase flip is better to use than the support exchange.
		if (gaitPhase * lastGaitPhase <= 0)
		{
			// Take in a new gcv.
			gcvUserTarget = command.GCV;

			// LIMITS
			// Constrain the gait control target according to the configured gait limits using normalization.
			// (We limit explicitely after the reset so that the frequency reset will use decent values).
			float norm;
			float p = config.GCVNormP;
			norm = pow( pow(fabs(gcvUserTarget.x), p) + pow(fabs(gcvUserTarget.y), p) + pow(fabs(gcvUserTarget.z), p), 1/p);
			if (norm > 1)
				gcvUserTarget /= norm;

			// Halt forces a zero gait target.
			if (!command.walk)
				gcvUserTarget = Vec3f();

			// If we are outside of our stable region, modify the target GCV to get back inside.
			double err_pos = fusedAngle_sag - config.stableSagAngleFront;
			if(err_pos > 0)
			{
				gcvUserTarget.x += err_pos * config.stableSagAngleSlope;
			}

			gcvTarget = gcvUserTarget;

			// Reset the gait frequency.
			gaitFrequency = config.gaitFrequency;

			// Feed forward adjustment of the gait frequency depending on the gait target.
			// Walking fast forward usually increases the frequency.
			gaitFrequency += qMax(0.0, gcvTarget.x) * config.gaitFrequencySlopeX;

			// Walking sideways modulates the frequency such that it's slower for the leading leg.
			if (gaitPhase * gcv.y < 0)
				gaitFrequency -= qAbs(gcvTarget.y) * config.gaitFrequencySlopeY;

			// Reset the expected energy to the default energy level.
			expectedEnergy = -config.energy;
		}


		// LIMITS
		// Constrain the gait control target according to the configured gait limits using normalization.
		float norm;
		float p = config.GCVNormP;
		norm = pow( pow(fabs(gcvTarget.x), p) + pow(fabs(gcvTarget.y), p) + pow(fabs(gcvTarget.z), p), 1/p);
		if (norm > 1)
			gcvTarget /= norm;

		state.gcvTarget = gcvTarget;

		lastGCV = gcv;

		// SLOPES
		// Move the current gait control vector towards the gait control target with the configured slopes.
		if (gcv.x >= 0 )
		{
			if (gcvTarget.x - gcv.x > config.accelerationForward * config.systemIterationTime)
				gcv.x += config.accelerationForward * config.systemIterationTime;
			else if (gcvTarget.x - gcv.x < -config.accelerationForward * config.decelerationFactor * config.systemIterationTime)
				gcv.x -= config.accelerationForward * config.decelerationFactor * config.systemIterationTime;
			else
				gcv.x = gcvTarget.x;
		}
		else
		{
			if (gcvTarget.x - gcv.x > qMin(config.accelerationForward, config.accelerationBackward * config.decelerationFactor) * config.systemIterationTime)
				gcv.x += qMin(config.accelerationForward, config.accelerationBackward * config.decelerationFactor) * config.systemIterationTime;
			else if (gcvTarget.x - gcv.x < -config.accelerationBackward * config.systemIterationTime)
				gcv.x -= config.accelerationBackward * config.systemIterationTime;
			else
				gcv.x = gcvTarget.x;
		}

		if (gcv.y >= 0)
		{
			if (gcvTarget.y - gcv.y > config.accelerationSideward * config.systemIterationTime)
				gcv.y += config.accelerationSideward * config.systemIterationTime;
			else if (gcvTarget.y - gcv.y < -config.accelerationSideward * config.decelerationFactor * config.systemIterationTime)
				gcv.y -= config.accelerationSideward * config.decelerationFactor * config.systemIterationTime;
			else
				gcv.y = gcvTarget.y;
		}
		else
		{
			if (gcvTarget.y - gcv.y > config.accelerationSideward * config.decelerationFactor * config.systemIterationTime)
				gcv.y += config.accelerationSideward * config.decelerationFactor * config.systemIterationTime;
			else if (gcvTarget.y - gcv.y < -config.accelerationSideward * config.systemIterationTime)
				gcv.y -= config.accelerationSideward * config.systemIterationTime;
			else
				gcv.y = gcvTarget.y;
		}

		if (gcv.z >= 0)
		{
			if (gcvTarget.z - gcv.z > config.accelerationRotational * config.systemIterationTime)
				gcv.z += config.accelerationRotational * config.systemIterationTime;
			else if (gcvTarget.z - gcv.z < -config.accelerationRotational * config.decelerationFactor * config.systemIterationTime)
				gcv.z -= config.accelerationRotational * config.decelerationFactor * config.systemIterationTime;
			else
				gcv.z = gcvTarget.z;
		}
		else
		{
			if (gcvTarget.z - gcv.z > config.accelerationRotational * config.decelerationFactor * config.systemIterationTime)
				gcv.z += config.accelerationRotational * config.decelerationFactor * config.systemIterationTime;
			else if (gcvTarget.z - gcv.z < -config.accelerationRotational * config.systemIterationTime)
				gcv.z -= config.accelerationRotational * config.systemIterationTime;
			else
				gcv.z = gcvTarget.z;
		}

		state.gcv = gcv;

		// TODO: Verify this is correct
		effectiveGCV = gcv;

		gcvAcc = (gcv - lastGCV) / config.systemIterationTime;


		// Update the gait phase.
		lastGaitPhase = gaitPhase;
		gaitPhase = picut(gaitPhase + gaitFrequency * PI * config.systemIterationTime);

		// FIRST STEP
		// Hold the gait phase at 0 in the first half of the first step motion.
		if (firstStepTimer > 0.5*config.firstStepDuration)
			gaitPhase = 0;
		firstStepTimer = qMax(0.0, firstStepTimer - config.systemIterationTime);

		state.gaitPhase = gaitPhase;
	}

	HeadPose DynamicGait::headFunction()
	{
		HeadPose hp;
		hp.neck.compliance = config.complianceTrunk;
		return hp;
	}

	InverseArmPose DynamicGait::inverseArmFunction(double armSign)
	{
		InverseArmPose iap;
		iap.handPosition.x = config.armOffsetX;
		iap.handPosition.y = armSign*config.armOffsetY;
		iap.handPosition.z = config.armOffsetZ;
		iap.compliance = config.complianceArm;
		return iap;
	}

	AbstractArmPose DynamicGait::abstractArmFunction(double armSign)
	{
		AbstractArmPose ap;

		float armPhase = gaitPhase;
		if (armSign == 1) // arm phase is opposite to leg phase!
			armPhase = picut(armPhase + PI);


		// ARM SWINGING
		// The swinging component swings the arm forward with a fast sinusoid motion and moves it back with a linear motion in the support phase.
		// The arm swing is not so perfectly embedded into the gait phase, because the swing phase is actually shorter than the support phase.
		// Two parameters define the embedding of the swing phase into the gait. The swingStartTiming is a small positive constant that defines
		// the swing start offset relative to gait phase 0. The swingStopTiming is a small negative constant that defines the swing end offset
		// relative to gait phase pi. The arms swing opposite to the legs, i.e. the right arm swings forward when the left leg does and vice versa.
		float swingStartTiming = config.swingStartTiming;
		float swingStopTiming = config.swingStopTiming;
		float swingAngle = 0;
		if (armPhase >= swingStartTiming && armPhase <= swingStopTiming)
		{
			// swing phase (sinusoid)
			swingAngle = cos( (armPhase - swingStartTiming) * PI/(swingStopTiming - swingStartTiming) );
		}
		else
		{
			// support phase (linear)
			if (armPhase >= swingStopTiming)
				swingAngle = -1 + ( armPhase - swingStopTiming) * 2/(2*PI - swingStopTiming + swingStartTiming);
			else
				swingAngle = -1 + ( armPhase + 2*PI - swingStopTiming) * 2/(2*PI - swingStopTiming + swingStartTiming);
		}
		ap.armAngle.y += swingAngle * (config.armSwing + gcv.x * config.armSwingSlope);

		return ap;
	}

	ArmPose DynamicGait::armFunction(double armSign)
	{
		ArmPose ap;
		ap.shoulder.x = -armSign * config.armOffsetY;

		if (armSign > 0 && config.balanceOffset > 0)
			ap.shoulder.x -= config.balanceOffset;

		if (armSign < 0 && config.balanceOffset < 0)
			ap.shoulder.x -= config.balanceOffset;

		return ap;
	}

	InverseLegPose DynamicGait::inverseLegFunction(double legSign)
	{
		InverseLegPose ilp;

		// BASE STANCE
		// These define the base gait stance. The gait trajectories are generated "around" this pose.
		// The base stance is equal to the halt position.

		const double ALPHA = 0.1;
		double targetXOffsetOffset = -config.footOffsetXAccSlope * gcvAcc.x;
		currentXOffsetOffset = (1.0 - ALPHA) * currentXOffsetOffset + ALPHA * targetXOffsetOffset;

		ilp.footPosition.x = config.footOffsetX + currentXOffsetOffset;
		ilp.footPosition.y = legSign*config.footOffsetY + config.footShiftY;
		ilp.footPosition.z = config.footOffsetZ;
		ilp.footAngle.x = config.footAngleX;
		ilp.footAngle.y = config.footAngleY;
		ilp.compliance = config.complianceLeg;


		// PHASE INDICATORS
		// The gait phase runs from -pi to pi. 0 and pi are the "centers" of the gait, where the pose is symmetrical and both
		// feet are on the ground. The phase of the left leg is shifted by pi. This way both legs can follow the same code.
		double legPhase = gaitPhase;
		if (legSign == -1)
			legPhase = picut(legPhase + PI);


		// LEG LIFTING
		// The essence of the gait is a rhythmic leg lifting left and right according to the gait frequency.
		// The leg lifting brings the robot's body to a stable swing and achieves foot clearance, so that the legs
		// can be swung in all directions to implement the omnidirectional walking.
		if (legPhase <= 0) // support (push) phase
		{
			double amplitude = config.pushHeight + qMax(0.0, gcv.x) * config.pushHeightSlope;
			ilp.footPosition.z += sin(legPhase*2.0 + PI2) * amplitude / 2.0 - amplitude / 2.0;
		}
		if (legPhase > 0) // swing (step) phase
		{
			double amplitude = config.stepHeight + qMax(0.0, qMax(gcv.x, fabs(gcv.y))) * config.stepHeightSlope;
			ilp.footPosition.z += sin(legPhase*2.0 - PI2) * amplitude/2.0 + amplitude/2.0;
		}

		// FIRST STEP HIP SWING
		// The first step is a special case, because the robot needs to enter the gait smoothly.
		// The gait always starts with the right leg. The first step starts with a preparing motion, i.e. a hip swing to the left,
		// to shift the com of the robot. During the first half of the preparing motion the gait phase stays 0. Then in the
		// second half the prepared pose is smoothly faded with the lateral hip swing.
		float firstStepHipSwing = -config.firstStepHipSwing * 0.5*(1 - cos( PI * qMin(1.0, 2.0*(config.firstStepDuration - firstStepTimer) / config.firstStepDuration )));
		float firstStepFade =  qMin(1.0, 2.0*(firstStepTimer / config.firstStepDuration));

		// LATERAL HIP SWING
		// The lateral hip swing sways the pelvis left and right during walking.
		// The swing to the left and the swing to the right are designed as separate symmetrical motions that are summed up and the result
		// yields the real hip swing. The left swing starts when the left foot touches the ground (swingStopTiming) and ends when the left
		// foot is lifted off the ground (swingStartTiming). The same goes for the right swing along with the right foot. In the double
		// support phase the two swings are overlapping and their addition cancel each other out (sort of).
		double swingStartTiming = config.swingStartTiming;
		double swingStopTiming = config.swingStopTiming;
		double hipSwingStartTimingLeft = config.swingStopTiming;
		double hipSwingStopTimingLeft = config.swingStartTiming;
		double hipSwingStartTimingRight = config.swingStopTiming - PI;
		double hipSwingStopTimingRight = config.swingStartTiming - PI;
		double hipSwingPhaseLeft = gaitPhase < hipSwingStartTimingLeft ? gaitPhase + 2*PI - hipSwingStartTimingLeft : gaitPhase - hipSwingStartTimingLeft;
		double hipSwingPhaseRight = gaitPhase < hipSwingStartTimingRight ? gaitPhase + 2*PI - hipSwingStartTimingRight : gaitPhase - hipSwingStartTimingRight;
		double hipSwingDuration = 2*PI - swingStopTiming + swingStartTiming;

		double hipSwingLeft = 0.f;
		if (gaitPhase <= hipSwingStopTimingLeft || gaitPhase >= hipSwingStartTimingLeft)
			hipSwingLeft = sin( hipSwingPhaseLeft * PI/hipSwingDuration );

		double hipSwingRight = 0.f;
		if (gaitPhase <= hipSwingStopTimingRight || gaitPhase >= hipSwingStartTimingRight)
			hipSwingRight = -sin( hipSwingPhaseRight * PI/hipSwingDuration );

		double lateralHipSwing = (hipSwingLeft + hipSwingRight) * (config.lateralHipSwing + qMax(0.0, gcv.x) * config.lateralHipSwingSlope);


		// Fade the first step and the hip swing.
		ilp.footPosition.y += firstStepFade * firstStepHipSwing + (1.0 - firstStepFade) * lateralHipSwing;

		// Leaning
// 		double leanOffsetX = qMin((config.tiltSpeedFwd / 1.5) * (effectiveGCV.x / 4.0), config.tiltSpeedFwd);
		double leanOffsetX = qMax(-config.tiltSpeedBwd * effectiveGCV.x, 0.0) + config.tiltSpeedFwd * fabs(effectiveGCV.y);

		ilp.footPosition.x += leanOffsetX;


		return ilp;
	}

	AbstractLegPose DynamicGait::abstractLegFunction(double legSign)
	{
		AbstractLegPose lp;


		// PHASE INDICATORS
		// The gait phase runs from -pi to pi. 0 and pi are the "centers" of the gait, where the pose is symmetrical and both
		// feet are on the ground. The phase of the left leg is shifted by pi. This way both legs can follow the same code.
		double legPhase = gaitPhase;
		if (legSign == -1)
			legPhase = picut(legPhase + PI);

		// LEG SWING
		// The swinging component swings the leg forward with a fast sinusoid motion and moves it back with a linear motion in the support phase.
		// The leg swing is not so perfectly embedded into the gait phase, like the leg lifting, because the swing phase is actually shorter than
		// the support phase. That means that there is a short period of time when both feet are on the ground, also known as the double support
		// phase. The ratio of support time to swing time in a human gait is typically 60 to 40. The swingStartTiming and swingStopTiming parameters
		// describe the start and the end of the leg swing relative to gait phase 0.
		double swingStartTiming = config.swingStartTiming;
		double swingStopTiming = config.swingStopTiming;
		double swingAngle = 0;
		if (legPhase >= swingStartTiming && legPhase < swingStopTiming)
		{
			// swing phase (sinusoid)
			swingAngle = cos( (legPhase - swingStartTiming) * PI/(swingStopTiming - swingStartTiming) );
		}
		else
		{
			// support phase (linear)
			if (legPhase >= swingStopTiming)
				swingAngle = -1 + ( legPhase - swingStopTiming) * 2/(2*PI - swingStopTiming + swingStartTiming);
			else
				swingAngle = -1 + ( legPhase + 2*PI - swingStopTiming) * 2/(2*PI - swingStopTiming + swingStartTiming);
		}

		// In lateral direction the legAngle gets pushed out a bit in addition to the basic swing in lateral direction,
		// because robots can't cross their legs. The combination of the lateral and rotational gait component require
		// even more leg spreading.
		lp.legAngle.x += swingAngle * -gcv.y * config.stepLengthYSlope;
		lp.legAngle.x += -legSign * qMax(0.0, gcv.x) * config.stepLengthXPushout;
		lp.legAngle.x += -legSign * fabs(gcv.y) * config.stepLengthYPushout;
		lp.legAngle.x += -legSign * fabs(gcv.z) * config.stepLengthZPushout;

		// The swing in sagittal direction is very straight forward, except that different parameters are used for forward and backward walking.
		lp.legAngle.y += swingAngle * gcv.x * (gcv.x >= 0 ? config.stepLengthXSlopeFwd : config.stepLengthXSlopeBwd);

		// For the Z swing there is a V pushout. The faster the robot turns, the more the feet are pushed out in a V shape.
		lp.legAngle.z += swingAngle * -gcv.z * config.stepLengthZSlope;
		lp.legAngle.z += -legSign * fabs(gcv.z) * config.stepLengthZVPushout;

		// Rotational hip swing.
		// While moving forward, the robot is able to slightly rotate its hips, but this is inhibited by lateral walking.
		double rotationalHipSwing = fabs(gaitPhase) - PI2; //linear triangle function.
		lp.legAngle.z -= legSign * rotationalHipSwing * gcv.x * (1.0-fabs(gcv.y)) * config.rotationalHipSwingSlope;




		// LEANING
		// When accelerating and walking at high speeds, the robot's body needs to be leaned to maintain balance.
		Vec2f lean;

		// Lean forward and backward with speed.
// 		lean.x += gcv.x * (gcv.x > 0 ? config.tiltSpeedFwd : config.tiltSpeedBwd);

		// Lateral lean into curves.
		lean.y += gcv.z * qAbs(gcv.x) * config.tiltRotLean;

// 		lp.legAngle.y += lean.x;
		lp.legAngle.x -= lean.y;

		return lp;
	}

	double DynamicGait::supportCoefficient(DynamicGait::Leg leg) const
	{
		double legPhase = gaitPhase;

		if(leg == LEFT_LEG)
			legPhase += PI;

		double slopeDuration = 1.0 / config.supportSlope;

		legPhase = picut(legPhase - slopeDuration / 2.0);

		if(legPhase < 0)
			return qBound(0.0, -config.supportSlope * legPhase, 1.0);

		return qBound(0.0, config.supportSlope * picut(legPhase - PI) + 1.0, 1.0);
	}

}