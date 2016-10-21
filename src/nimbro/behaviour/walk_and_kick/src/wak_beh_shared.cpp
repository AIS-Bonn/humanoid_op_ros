// Walk and kick: Class for shared walk and kick behaviour state variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_beh_shared.h>
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/wak_beh_manager.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// WAKBehShared class
//

// Constructor
WAKBehShared::WAKBehShared(WAKConfig& config, const SensorVars& SV, const WAKRosInterface& RI, WAKBehManager& BM)
 : PM(RI.getPM())
 , MM(RI.getMM())
 , config(config)
 , SV(SV)
 , RI(RI)
 , BM(BM)
{
	// Reset the shared variables
	resetShared();
}

// Behaviour state registration function
void WAKBehShared::registerState(WAKBehState* state, int ID, const std::string& name) const
{
	// Pass the state registration request on up to the behaviour manager
	BM.registerState(state, ID, name);
}

// Reset function
void WAKBehShared::resetShared()
{
	// Reset the game variable inputs
	GV.reset();

	// Reset the shared behaviour state variables
	reqBallDirMidY = 0.0;
	reqBallDirLeftKb.setZero();
	reqBallDirRightKb.setZero();
	reqBallDirLeftDb.setZero();
	reqBallDirRightDb.setZero();
	haveBallTarget = false;
	ballTargetDist = 0.0f;
	ballTargetAngle = 0.0f;
	ballToTargetDir.setZero();
	ballToTargetDist = 0.0f;
	ballToTargetAngle = 0.0f;
	ballToTargetAngleOffsetKick = 0.0f;
}

// Update function
void WAKBehShared::updateShared(const GameVars& GV)
{
	// Update the game variable inputs
	this->GV = GV;

	// Update the required ball offset variables
	reqBallDirMidY = 0.5f*(config.reqBallOffsetYLeft() + config.reqBallOffsetYRight());
	reqBallDirLeftKb << config.reqBallOffsetX(), config.reqBallOffsetYLeft();
	reqBallDirRightKb << config.reqBallOffsetX(), config.reqBallOffsetYRight();
	reqBallDirLeftDb << config.reqBallOffsetXDribble(), reqBallDirMidY + config.reqBallOffsetYDribbleMag();
	reqBallDirRightDb << config.reqBallOffsetXDribble(), reqBallDirMidY - config.reqBallOffsetYDribbleMag();

	// Update the ball target variables
	haveBallTarget = (GV.ballTargetConf > config.confLimitBallTarget());
	ballTargetDist = GV.ballTargetDir.norm();
	ballTargetAngle = eigenAngleOf(GV.ballTargetDir);
	ballToTargetDir = GV.ballTargetDir - SV.ballDir;
	ballToTargetDist = ballToTargetDir.norm();
	ballToTargetAngle = eigenAngleOf<float>(ballToTargetDir);

	// Update the ball to target angle offset due to drift during kicking (additive offset to ballToTargetAngle to compensate for kick drift)
	if(BM.GoBehindBall->isActive())
		ballToTargetAngleOffsetKick = (BM.GoBehindBall->useRightFoot() ? -config.kickAngleDriftRightKick() : -config.kickAngleDriftLeftKick());
	else if(BM.KickBall->isActive())
		ballToTargetAngleOffsetKick = (BM.KickBall->bestKickRight() ? -config.kickAngleDriftRightKick() : -config.kickAngleDriftLeftKick());
	else
		ballToTargetAngleOffsetKick = -0.5f*(config.kickAngleDriftRightKick() + config.kickAngleDriftLeftKick());

	// Plotting
	if(config.plotData())
	{
		RI.PM->plotScalar(GV.forceBehStateByID, PM_GV_FORCEBEHSTATE);
		RI.PM->plotScalar(GV.suggestFoot * PMSCALE_FOOTSEL, PM_GV_SUGGESTFOOT);
		RI.PM->plotScalar(GV.dribbleIfPossible * PMSCALE_DRIBBLE, PM_GV_DRIBBLEIFPOSSIBLE);
		RI.PM->plotScalar(GV.kickIfPossible * PMSCALE_KICK, PM_GV_KICKIFPOSSIBLE);
		RI.PM->plotScalar(GV.diveIfPossible, PM_GV_DIVEIFPOSSIBLE);
		RI.PM->plotScalar(haveBallTarget * PMSCALE_HAVETARGET, PM_GV_HAVEBALLTARGET);
		RI.PM->plotScalar(GV.ballTargetDir.x(), PM_GV_BALLTARGETX);
		RI.PM->plotScalar(GV.ballTargetDir.y(), PM_GV_BALLTARGETY);
		RI.PM->plotScalar(GV.ballTargetConf, PM_GV_BALLTARGETCONF);
		RI.PM->plotScalar(ballTargetDist, PM_GV_BALLTARGETDIST);
		RI.PM->plotScalar(ballTargetAngle, PM_GV_BALLTARGETANGLE);
		RI.PM->plotScalar(GV.ballTargetWedge, PM_GV_BALLTARGETWEDGE);
		RI.PM->plotScalar(GV.ballTargetType, PM_GV_BALLTARGETTYPE);
		RI.PM->plotScalar(ballToTargetDist, PM_GV_BALLTOTARGETDIST);
		RI.PM->plotScalar(ballToTargetAngle, PM_GV_BALLTOTARGETANGLE);
		RI.PM->plotScalar(ballToTargetAngleOffsetKick, PM_GV_BTTANGLEOFFSETKICK);
		RI.PM->plotScalar(GV.targetPose.x(), PM_GV_TARGETPOSEX);
		RI.PM->plotScalar(GV.targetPose.y(), PM_GV_TARGETPOSEY);
		RI.PM->plotScalar(GV.targetPose.z(), PM_GV_TARGETPOSEZ);
		RI.PM->plotScalar(GV.targetPoseTol, PM_GV_TARGETPOSETOL);
		RI.PM->plotScalar(GV.targetPoseValid * PMSCALE_HAVETARGET, PM_GV_TARGETPOSEVALID);
	}

	// Visualisation markers
	if(RI.MM->willPublish())
	{
		if(SV.haveBall && haveBallTarget)
		{
			RI.MM->updateMarkerXY(RI.MM->BallTarget, true, GV.ballTargetDir.x(), GV.ballTargetDir.y(), GV.ballTargetConf);
			RI.MM->BallTargetType.setPosition(GV.ballTargetDir.x(), GV.ballTargetDir.y());
			RI.MM->BallTargetType.setText(GV.ballTargetTypeName());
			RI.MM->BallTargetType.show();
			RI.MM->BallTargetWidthBox.setPosition(GV.ballTargetDir.x(), GV.ballTargetDir.y());
			RI.MM->BallTargetWidthBox.setOrientation(cos(0.5*ballToTargetAngle), 0.0, 0.0, sin(0.5*ballToTargetAngle));
			RI.MM->BallTargetWidthBox.setScaleY(coerce<double>(2.0*ballToTargetDist*tan(0.5*coerce<double>(GV.ballTargetWedge, 0.0, M_PI - 1e-6)), 1e-6, 50.0));
			RI.MM->BallTargetWidthBox.show();
			RI.MM->BallToTargetLine.setPoint(0, SV.ballDir.x(), SV.ballDir.y());
			RI.MM->BallToTargetLine.setPoint(1, GV.ballTargetDir.x(), GV.ballTargetDir.y());
			RI.MM->BallToTargetLine.setPoint(2, SV.ballDir.x(), SV.ballDir.y());
			RI.MM->BallToTargetLine.setPoint(3, SV.ballDir.x() + ballToTargetDist, SV.ballDir.y());
			RI.MM->BallToTargetLine.show();
			if(GV.suggestFoot == GameVars::FS_EITHER_FOOT)
				RI.MM->SuggestedFoot.hide();
			else
			{
				if(GV.suggestFoot == GameVars::FS_LEFT_FOOT)
					RI.MM->SuggestedFoot.marker.pose.position.y = 0.3;
				else if(GV.suggestFoot == GameVars::FS_RIGHT_FOOT)
					RI.MM->SuggestedFoot.marker.pose.position.y = -0.3;
				else
					RI.MM->SuggestedFoot.marker.pose.position.y = 0.0;
				RI.MM->SuggestedFoot.show();
			}
			if(ballAction() == BA_KICK)
			{
				const Vec2f& left = (config.forceKickRightFoot ? reqBallDirRightKb : reqBallDirLeftKb);
				const Vec2f& right = (config.forceKickLeftFoot ? reqBallDirLeftKb : reqBallDirRightKb);
				RI.MM->ReqBallOffset.setPoint(0, left.x(), left.y());
				RI.MM->ReqBallOffset.setPoint(1, right.x(), right.y());
			}
			else
			{
				RI.MM->ReqBallOffset.setPoint(0, reqBallDirLeftDb.x(), reqBallDirLeftDb.y());
				RI.MM->ReqBallOffset.setPoint(1, reqBallDirRightDb.x(), reqBallDirRightDb.y());
			}
			RI.MM->ReqBallOffset.show();
		}
		else
		{
			RI.MM->BallTarget.hide();
			RI.MM->BallTarget.updateAdd();
			RI.MM->BallTargetType.hide();
			RI.MM->BallTargetWidthBox.hide();
			RI.MM->BallToTargetLine.hide();
			RI.MM->SuggestedFoot.hide();
			RI.MM->ReqBallOffset.hide();
		}
		RI.MM->BallTargetType.updateAdd();
		RI.MM->BallTargetWidthBox.updateAdd();
		RI.MM->BallToTargetLine.updateAdd();
		RI.MM->SuggestedFoot.updateAdd();
		RI.MM->ReqBallOffset.updateAdd();
		if(GV.targetPoseValid)
		{
			RI.MM->TargetPoseArrow.setPosition(GV.targetPose.x(), GV.targetPose.y(), 0.55);
			RI.MM->TargetPoseArrow.setPoint(0, 0.0, 0.0, 0.0);
			RI.MM->TargetPoseArrow.setPoint(1, 0.3*cos(GV.targetPose.z()), 0.3*sin(GV.targetPose.z()), 0.0);
			RI.MM->TargetPoseArrow.show();
		}
		else
			RI.MM->TargetPoseArrow.hide();
		RI.MM->TargetPoseArrow.updateAdd();
	}
}

// Get functions
BAType WAKBehShared::ballAction() const { return BM.m_ballAction; }

// Set functions
void WAKBehShared::setWalkingTarget(const Vec2f& target, float tol) const { BM.setWalkingTarget(target, tol); }
void WAKBehShared::setWalkingTargetTol(float tol) const { BM.setWalkingTargetTol(tol); }

// Cycle numbers and times
cycle_t WAKBehShared::wakCycle() const { return BM.m_wakCycle; }
cycle_t WAKBehShared::stateCycle() const { return BM.m_stateCycle; }
float WAKBehShared::wakTime() const { return TINC * BM.m_wakCycle; }
float WAKBehShared::stateTime() const { return TINC * BM.m_stateCycle; }

// Set AV.GCV to try to walk to a particular global pose (returns a metric of proximity to the target: dist error + config.wtgpAngleErrorCost() * angle error)
float WAKBehShared::walkToGlobalPose(ActuatorVars& AV, float targetX, float targetY, float targetZ, bool useZ) const
{
	// Initialise variables
	float distCost = INVALID_DIST;

	// We can only walk to a global pose if we are localised
	if(SV.haveRobotPose)
	{
		// Coerce the desired global location to the field boundary
		targetX = coerceAbs(targetX, field.fieldLengthH());
		targetY = coerceAbs(targetY, field.fieldWidthH());

		// Calculate the XY vector to the target in body-fixed coordinates
		Vec2f relativeTarget(targetX - SV.robotPose.x(), targetY - SV.robotPose.y()); // relativeTarget is in global coordinates here
		eigenRotateCCW(relativeTarget, -SV.robotPose.z()); // relativeTarget is now in body-fixed coordinates

		// Calculate the target direction (unit vector), distance and angle
		Vec2f targetDir = eigenNormalized(relativeTarget);
		float targetDist = relativeTarget.norm();
		float targetAngle = eigenAngleOf(relativeTarget);

		// Calculate the angle deviation to the global Z target
		float targetZDeviation = picut(targetZ - SV.robotPose.z());

		// Calculate the gait command for a far target
		Vec3f gcvFar;
		gcvFar.x() = coerceMin<float>(config.wtgpSpeedFarX()*(1.0f - fabs(targetAngle)/config.wtgpAngleLimitFar()), 0.0f);
		gcvFar.y() = 0.0f;
		gcvFar.z() = config.wtgpSpeedFarZ() * coerceAbs(targetAngle / config.wtgpAngleLimitFar(), 1.0f);

		// Calculate the gait command for a near target
		Vec3f gcvNear;
		float nearSpeed = config.wtgpSpeedNearXY() * coerce(targetDist / config.wtgpDistNear(), 0.0f, 1.0f);
		gcvNear.x() = nearSpeed * targetDir.x();
		gcvNear.y() = nearSpeed * targetDir.y();
		if(useZ)
			gcvNear.z() = config.wtgpSpeedNearZ() * coerceAbs(targetZDeviation / config.wtgpAngleLimitNear(), 1.0f);
		else
			gcvNear.z() = 0.0f;

		// Choose a gait command to work with
		if(targetDist <= config.wtgpDistNear())
			AV.GCV = gcvNear;
		else if(targetDist <= config.wtgpDistFar())
			AV.GCV = gcvNear + ((targetDist - config.wtgpDistNear()) / (config.wtgpDistFar() - config.wtgpDistNear())) * (gcvFar - gcvNear);
		else
			AV.GCV = gcvFar;

		// Calculate the current distance cost to the target
		distCost = (useZ ? targetDist + config.wtgpAngleErrorCost() * fabs(targetZDeviation) : targetDist);

		// Apply obstacle avoidance and set the walking target
		obstacleAvoidance(AV.GCV, relativeTarget);
		setWalkingTarget(relativeTarget);

		// Print info about the positioning state
		if(config.debugMsgWTGP() && wakCycle() % 20 == 1)
		{
			printf("WTGP: %s %s GBLTGT(%.2f, %.2f, %.2f) %s(%.2f, %.2f, %.2f) LCLTGT(%.2f, %.2f) ROTERR(%.2f) GCV(%.2f, %.2f, %.2f)\n",
				   (useZ ? "USE Z" : "NO Z"), (targetDist <= config.wtgpDistNear() ? "NEAR" : (targetDist <= config.wtgpDistFar() ? "MIX" : "FAR")),
				   targetX, targetY, targetZ, (SV.haveRobotPose ? "POSE" : "pose"), SV.robotPose.x(), SV.robotPose.y(), SV.robotPose.z(),
				   relativeTarget.x(), relativeTarget.y(), targetZDeviation, AV.GCV.x(), AV.GCV.y(), AV.GCV.z());
		}

		// Plotting
		if(config.plotData())
		{
			PM.plotScalar(relativeTarget.x(), PM_WTGP_TARGETX);
			PM.plotScalar(relativeTarget.y(), PM_WTGP_TARGETY);
			PM.plotScalar(targetZDeviation, PM_WTGP_TARGETZERR);
			PM.plotScalar(targetDist, PM_WTGP_TARGETDIST);
			PM.plotScalar(targetAngle, PM_WTGP_TARGETANGLE);
		}
	}
	else
	{
		// Don't walk anywhere if we are not localised
		AV.GCV.setZero();
		distCost = INVALID_DIST;
	}

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(targetX, PM_WTGP_GBLTARGETX);
		PM.plotScalar(targetY, PM_WTGP_GBLTARGETY);
		PM.plotScalar(targetZ, PM_WTGP_GBLTARGETZ);
		PM.plotScalar(useZ * PMSCALE_WTGPBOOL, PM_WTGP_USEZ);
		PM.plotScalar((distCost == INVALID_DIST ? -1.0 : distCost), PM_WTGP_DISTCOST);
	}

	// Return the current distance cost to the global walking target
	return distCost;
}

// Avoid obstacles in a generic way, based only on the current desired GCV and a desired walking target
bool WAKBehShared::obstacleAvoidance(Vec3f& GCV, const Vec2f& walkingTarget) const
{
	// Don't do anything if obstacle avoidance is disabled
	if(!config.sEnableObstacles() || !config.oaEnableObstacleAvoidance())
		return false;

	// Retrieve the 2D GCV
	Vec2f GCV2D = GCV.head<2>();
	float speedXY = GCV2D.norm();

	// Find the most limiting obstacle (if any)
	int numObst = 0;
	float avgAngAdjustDes = 0.0f, avgGcvZAdjust = 0.0f, weightSum = 0.0f;
	for(ObstacleList::const_iterator it = SV.obstacles.begin(); it != SV.obstacles.end(); ++it)
	{
		// Get the radius of the robot from the obstacle
		float radiusRobot = it->vec.norm(); // The radius of the robot from the obstacle
		if(radiusRobot <= 0.0f || radiusRobot > config.oaObstacleRadiusHigh()) continue;

		// Construct our coordinate system
		Vec2f rxhat = it->vec / radiusRobot; // Unit vector pointing from the robot to the obstacle
		Vec2f ryhat = eigenRotatedCCW90(rxhat);

		// Calculate the ball is in-between factor (0.0 => Use normal obstacle avoidance, 1.0 => Allow the robot to get closer to the obstacle as the ball is in the way anyway)
		float ballFact = 0.0f;
		if(SV.haveBall)
		{
			Vec2f ballToObstacleUnitVec = eigenNormalized<float, 2>(it->vec - SV.ballDir);
			float angleAtObst = acos(coerceAbs(ballToObstacleUnitVec.dot(rxhat), 1.0f));
			ballFact = interpolateCoerced(0.0f, config.oaAngleAtObstacleHigh(), 1.0f, 0.0f, angleAtObst);
		}

		// Calculate the maximum allowed radial GCV
		float obstacleRadiusHigh = (ballFact * config.oaObstacleRadiusMid() + (1.0f - ballFact) * config.oaObstacleRadiusHigh());
		float maxRadialGcv = interpolateCoerced(config.oaObstacleRadiusLow(), obstacleRadiusHigh, config.oaMaxRadialGcvLow(), config.oaMaxRadialGcvHigh(), radiusRobot);

		// Calculate the current desired radial GCV relative to this obstacle
		float radialGcv = GCV2D.dot(rxhat);
		float tangentGcv = GCV2D.dot(ryhat);
		int adjustSign = sign(tangentGcv);

		// Work out how much the radial GCV is over the allowed value
		float excessRadialGcv = radialGcv - maxRadialGcv;
		if(excessRadialGcv <= 0.0f) continue;

		// We have just found ourself a limiting obstacle
		numObst++;

		// Calculate what angle of GCV obeys the allowed radial GCV
		float angleOrigMag = 0.0f, angleDesMag = 0.0f;
		if(speedXY > 0.0f)
		{
			angleOrigMag = acos(coerceAbs(radialGcv / speedXY, 1.0f));
			angleDesMag = acos(coerceAbs(maxRadialGcv / speedXY, 1.0f));
		}
		float angleAdjustMag = angleDesMag - angleOrigMag;
		float angleAdjustDes = adjustSign * angleAdjustMag;

		// Calculate the required adjustment to the turning GCV (to turn away from the obstacle)
		float gcvZAdjustMag = speedXY * interpolateCoerced(0.0f, config.oaAdjustAngleXYHigh(), 0.0f, config.oaAdjustGcvZHigh(), angleAdjustMag);
		float gcvZAdjust = adjustSign * gcvZAdjustMag;

		// Incrementally update the averages
		float weight = excessRadialGcv;
		weightSum += weight;
		float incrementFactor = (weight / weightSum);
		avgAngAdjustDes += (angleAdjustDes - avgAngAdjustDes) * incrementFactor;
		avgGcvZAdjust += (gcvZAdjust - avgGcvZAdjust) * incrementFactor;
	}

	// If no obstacle is impeding us then we are home free without modifying the GCV
	if(numObst <= 0)
		return false;

	// Calculate the desired replacement GCV
	Vec2f GCVDes2D = eigenRotatedCCW(GCV2D, avgAngAdjustDes);
	float GCVDesZ = GCV.z() + avgGcvZAdjust;

	// Update the output GCV
	GCV << GCVDes2D.x(), GCVDes2D.y(), GCVDesZ;

	// Visualisation markers
	if(RI.MM->willPublish())
	{
		RI.MM->WalkingTarget.setColor(0.0, 0.0, 1.0);
		RI.MM->WalkingTargetTol.setColor(0.0, 0.0, 1.0);
	}

	// Return that the GCV was modified
	return true;
}

// Set AV.GazeYaw and AV.GazePitch to try to gaze at the ball (returns false if there is no ball)
bool WAKBehShared::gazeAtBall(ActuatorVars& AV, const ActuatorVars& lastAV) const
{
	// If we don't have a ball then we can't gaze at it
	if(!SV.haveBall)
	{
		AV.gazeYaw = lastAV.gazeYaw;
		AV.gazePitch = lastAV.gazePitch;
		return false;
	}

	// Decide on a gaze 90% settling time based on the distance to the ball (the closer the ball is, the higher the settling time should be as the level of noise in the ball angle is higher)
	float Ts = interpolate(config.gazeBallDistNear(), config.gazeBallDistFar(), config.gazeBallTsNear(), config.gazeBallTsFar(), SV.ballDist);
	Ts = coerce(Ts, std::max(config.gazeBallTsMin(), config.gazeBallTsFar()), config.gazeBallTsMax());

	// Calculate the target gaze yaw
	float targetGazeYaw = SV.ballAngle;
	if(SV.ballDist >= config.gazeBallDistFar())
		targetGazeYaw = SV.ballAngle;
	else if(SV.ballDir.x() >= config.gazeBallBoxXMin() && SV.ballDir.x() <= config.gazeBallBoxXMax() && SV.ballDir.y() >= config.gazeBallBoxYMin() && SV.ballDir.y() <= config.gazeBallBoxYMax())
		targetGazeYaw = 0.0f;
	else
	{
		float u = interpolateCoerced(config.gazeBallYawDeadband(), config.gazeBallYawFreeLook(), 1.0f, 0.0f, SV.ballAngle); // u = 1 means look straight ahead, u = 0 means look directly at the ball
		u = interpolateCoerced(config.gazeBallDistNear(), config.gazeBallDistFar(), u, 0.0f, SV.ballDist);
		targetGazeYaw = interpolateCoerced(SV.ballAngle, 0.0f, u);
	}

	// Calculate the target gaze pitch
	float targetGazePitch = interpolateCoerced(config.gazeBallDistFar(), config.gazeBallDistHorizon(), config.gazeBallPitchFar(), config.gazeBallPitchHorizon(), SV.ballDist);
	targetGazePitch = coerce(targetGazePitch, config.gazePitchMin(), config.gazePitchMax());

	// Update the gaze to look towards the ball
	float beta = 1.0f - pow(0.10f, TINC / Ts); // The 0.10 comes from Ts being a 90% settling time
	AV.gazeYaw = coerceAbs(lastAV.gazeYaw + beta*(targetGazeYaw - lastAV.gazeYaw), config.gazeYawAbsMax());
	AV.gazePitch = coerce(lastAV.gazePitch + beta*(targetGazePitch - lastAV.gazePitch), config.gazePitchMin(), config.gazePitchMax());

	// Plotting
	if(config.plotData())
		PM.plotScalar(Ts, PM_GAB_TS90);

	// Return that we successfully commanded to gaze at the ball
	return true;
}

// Calculate an XY gcv based on maximum X and Y walking velocities and an elliptical directional model
Vec2f WAKBehShared::calcGcvXY(float maxGcvX, float maxGcvY, float angle) const
{
	// Calculate the required walking velocity based on an elliptical model
	float ycangle = maxGcvY*cos(angle);
	float xsangle = maxGcvX*sin(angle);
	float ronxy = 1.0f / sqrt(ycangle*ycangle + xsangle*xsangle);
	return Vec2f(maxGcvX*ronxy*ycangle, maxGcvY*ronxy*xsangle);
}
// EOF