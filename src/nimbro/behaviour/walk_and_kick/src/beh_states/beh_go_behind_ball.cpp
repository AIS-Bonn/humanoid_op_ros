// Walk and kick behaviour state: Go behind ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/beh_go_behind_ball.h>
#include <walk_and_kick/wak_utils.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// BehGoBehindBall class
//

// Constants
const float BehGoBehindBall::MinReqBallDirX = 1e-4f; // Must be strictly greater than 0!
const float BehGoBehindBall::MinRadius = 1e-4f;  // Must be strictly greater than 0!

// Constructor
BehGoBehindBall::BehGoBehindBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID), GazeBehLookForBall(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());

	// Config parameter callbacks
	config.gbbFootChangeWormTime.setCallback(boost::bind(&TheWorm::updateWormTime, &m_changeToRightFoot, &config.gbbFootChangeWormTime), true);
}

// Handle activation function
void BehGoBehindBall::handleActivation(bool nowActive)
{
	// Handle activation of the base class
	GazeBehLookForBall::handleActivation(nowActive);

	// Reset variables
	m_useRightFoot = true;
	m_changeToRightFoot.reset();
	m_unforceFoot.reset();
	m_stuck.reset();
	m_ballActionTip = BA_UNKNOWN;
}

// Execute function
void BehGoBehindBall::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Look for the ball
	GazeBehLookForBall::execute(AV, lastAV, justActivated);

	//
	// Walking control
	//

	// #################### Case if Ball not Present ####################

	// If we have no ball then stop where you are
	if(!SV.haveBall)
	{
		// Set our walking GCV
		AV.GCV.setZero();
		AV.halt = false;
		AV.doKick = false;
		AV.rightKick = true;
		AV.doDive = DD_NONE;

		// Plotting
		if(config.plotData())
		{
			PM.plotScalar(AV.GCV.x(), PM_GBB_GCVX);
			PM.plotScalar(AV.GCV.y(), PM_GBB_GCVY);
			PM.plotScalar(AV.GCV.z(), PM_GBB_GCVZ);
			PM.plotScalar(AV.GCV.norm(), PM_GBB_GCVN);
		}

		// Return as we have nothing more to do
		return;
	}

	// #################### Ball and Ball Target Vectors ####################

	// Check what the required ball action is
	bool ballActionIsKick = (WBS.ballAction() == BA_KICK);
	bool ballActionIsDribble = (WBS.ballAction() == BA_DRIBBLE);

	// Save the positions of the ball and target in local variables
	Vec2f ball = SV.ballDir;             // Vector from robot to ball in body-fixed coordinates
	Vec2f ballTarget = GV.ballTargetDir; // Vector from robot to ball target in body-fixed coordinates
	if(!WBS.haveBallTarget)              // If we don't have a ball target then take one that's in the direction we're facing in front of the ball
		ballTarget << ball.x() + config.minBallToTargetDist(), ball.y();

	// Calculate the ball distance
	float ballDist = ball.norm();

	// Calculate the ball unit vectors bx and by
	Vec2f bxhat(1.0f, 0.0f);
	if(ballDist > 0.0f)
		bxhat = ball / ballDist;
	Vec2f byhat = eigenRotatedCCW90(bxhat);
	float bxAngle = eigenAngleOf(bxhat);

	// Calculate the ball to ball target vector and distance
	Vec2f ballToBallTargetVec = ballTarget - ball;
	if(ballActionIsKick)
		eigenRotateCCW(ballToBallTargetVec, WBS.ballToTargetAngleOffsetKick); // Correct for kick drifting
	float ballToBallTargetDist = ballToBallTargetVec.norm();

	// Calculate the ball to ball target unit vectors ex and ey
	Vec2f exhat = bxhat;
	if(ballToBallTargetDist > 0.0f)
		exhat = ballToBallTargetVec / ballToBallTargetDist;
	Vec2f eyhat = eigenRotatedCCW90(exhat);
	float exAngle = eigenAngleOf(exhat);

	// #################### Foot Selection ####################

	// Retrieve the required ball dir vectors for kicking by default
	Vec2f usedRBDLeft = WBS.reqBallDirLeftKb;
	Vec2f usedRBDRight = WBS.reqBallDirRightKb;

	// Recompute the required ball dir vectors if our aim is to dribble
	if(ballActionIsDribble)
	{
		float angleAtBall = acos(coerceAbs(bxhat.dot(exhat), 1.0f)); // The angle at the ball between the robot to ball and ball to ball target vectors, in the range [0,pi]
		float facingFactor = interpolateCoerced<float>(config.gbbReqBallOffFadingAngle(), M_PI_2, 1.0f, 0.0f, fabs(exAngle));
		float useDribbleOffset = interpolateCoerced<float>(config.gbbReqBallOffFadingAngle(), M_PI_2, facingFactor, 0.0f, angleAtBall);
		usedRBDLeft = useDribbleOffset * WBS.reqBallDirLeftDb + (1.0f - useDribbleOffset) * WBS.reqBallDirLeftKb;
		usedRBDRight = useDribbleOffset * WBS.reqBallDirRightDb + (1.0f - useDribbleOffset) * WBS.reqBallDirRightKb;
	}

	// Calculate the distance between the two possible behind ball poses
	float reqBallDirDistLR = (usedRBDLeft - usedRBDRight).norm();

	// Calculate the distance to the two possible behind ball poses from the robot
	float distToReqLeft = (ball - usedRBDLeft.x()*exhat - usedRBDLeft.y()*eyhat).norm();
	float distToReqRight = (ball - usedRBDRight.x()*exhat - usedRBDRight.y()*eyhat).norm();
	float footDueToLessDist = coerceAbs((distToReqLeft - distToReqRight) / coerceMin(reqBallDirDistLR, config.gbbFootSelMinLRDist()), 1.0f); // 1 = Definitely use right foot, -1 = Definitely use left foot

	// See which foot the ball pose suggests (try to use the outer foot near the sides of the field)
	float footDueToBallPose = 0.0f; // Note: This is out here, and not only calculated if the foot is being reconsidered (the only time it is used), for plotting purposes
	if(SV.haveBallPose)
		footDueToBallPose = -SV.goalSign * sign(SV.ballPose.y()) * coerce<float>(fabs(SV.ballPose.y()) - (config.gbbFootSelBallPoseYRatio() * field.fieldWidthH()), 0.0f, 1.0f); // 1 = Definitely use right foot, -1 = Definitely use left foot

	// Decide whether we should reconsider the foot we're currently trying to line up
	if(GV.suggestRightFoot()) // The right foot is being strongly suggested to us from above
		m_changeToRightFoot.vote(true, 2);
	else if(GV.suggestLeftFoot()) // The left foot is being strongly suggested to us from above
		m_changeToRightFoot.vote(false, 2);
	else if(footDueToLessDist > config.gbbFootChangeMinConf()) // The right foot would be better right now
		m_changeToRightFoot.vote(true);
	else if(footDueToLessDist < -config.gbbFootChangeMinConf()) // The left foot would be better right now
		m_changeToRightFoot.vote(false);
	bool reconsiderFoot = (m_changeToRightFoot.unanimous() && m_changeToRightFoot.decision() != m_useRightFoot);

	// Decide which foot to line up with the ball and the ball target
	if(justActivated || reconsiderFoot)
	{
		// Decide on a foot to line up the ball with
		if(GV.suggestRightFoot())
			m_useRightFoot = true;
		else if(GV.suggestLeftFoot())
			m_useRightFoot = false;
		else
			m_useRightFoot = ((config.gbbFootSelWeightBallPose()*footDueToBallPose + config.gbbFootSelWeightLessDist()*footDueToLessDist) >= 0.0f);
	}

	// Force the use of a particular foot if required
	if(ballActionIsKick)
		m_unforceFoot.reset();
	else
		m_unforceFoot.increment();
	if(!m_unforceFoot.reached(TO_COUNT(config.gbbUnforceFootTime())))
	{
		if(config.forceKickRightFoot)
			m_useRightFoot = true;
		else if(config.forceKickLeftFoot)
			m_useRightFoot = false;
	}

	// Desired body-fixed offset from the robot to the ball for good positioning behind the ball
	Vec2f reqBallDir = (m_useRightFoot ? usedRBDRight : usedRBDLeft);

	// Sanity check the required ball offset (after this ReqBallDir.x must be greater than zero)
	if(reqBallDir.x() < MinReqBallDirX)
		reqBallDir.x() = MinReqBallDirX;

	// #################### GCV Calculation ####################

	// Required gait control vector (GCV)
	Vec3f GCV(0.0f, 0.0f, 0.0f); // x forwards, y left, z CCW

	// Compute the required GCV to walk with
	Vec2f walkingTarget;
	if(ballDist <= 0.0f)
	{
		// Whatever happens, just back away from the ball
		GCV << -config.gbbSpeedNearXY(), 0.0f, 0.0f;

		// Set the walking target to a small distance behind us
		walkingTarget << -0.5f, 0.0f;

		// Visualisation markers
		if(MM.willPublish())
		{
			MM.GBBPath.setNumPoints(8);
			MM.GBBPath.setPoint(0, 0.5, 1.0, 0.0);
			MM.GBBPath.setPoint(1, 0.5, -1.0, 0.0);
			MM.GBBPath.setPoint(2, 2.0, -1.0, 0.0);
			MM.GBBPath.setPoint(3, 2.0, 1.0, 0.0);
			MM.GBBPath.setPoint(4, 0.5, 1.0, 0.0);
			MM.GBBPath.setPoint(5, 2.0, -1.0, 0.0);
			MM.GBBPath.setPoint(6, 2.0, 1.0, 0.0);
			MM.GBBPath.setPoint(7, 0.5, -1.0, 0.0);
			MM.GBBPath.show();
		}
	}
	else // Normal case...
	{
		// Calculate the ball to behind ball vector and distance
		Vec2f ballToBehindBallVec = -reqBallDir.x()*exhat - reqBallDir.y()*eyhat;
		float ballToBehindBallDist = ballToBehindBallVec.norm(); // Note: Cannot be zero as ReqBallDir.x > 0

		// Calculate the ball to behind ball unit vectors tx and ty
		Vec2f txhat = -bxhat;
		if(ballToBehindBallDist > 0.0f)
			txhat = ballToBehindBallVec / ballToBehindBallDist;
		Vec2f tyhat = eigenRotatedCCW90(txhat);

		// Calculate the behind ball position
		Vec2f behindBall = ball + ballToBehindBallVec;
		walkingTarget = behindBall;

		// Calculate the current beta value (the CCW angle from the 'ball to behind ball vector' to the 'ball to robot vector')
		float beta = atan2(-bxhat.dot(tyhat), -bxhat.dot(txhat));
		float absBeta = fabs(beta);

		// Calculate the minimum, behind ball and robot radii from the ball
		float Rmin = minDesiredBallDist(bxAngle, reqBallDir.x());
		float Rbb = ballToBehindBallDist;
		float Rrobot = ballDist;

		// Calculate the desired and used halo radii
		float Rdes = coerceMin(interpolateCoerced(config.gbbAbsBetaLow(), config.gbbAbsBetaHigh(), std::min(Rmin, Rbb), Rmin, absBeta), MinRadius);
		float Rhalo = coerceMin(std::min(Rdes, Rrobot), MinRadius);

		// Calculate a path from the current robot position to the behind ball position that respects a halo of radius Rhalo around the ball
		Vec2f pathNormal(0.0f, 0.0f);
		float pathLen = WAKUtils::calculateHaloPath(pathNormal, m_path, Vec2f::Zero(), behindBall, ball, Rhalo, true);

		// Adjust the total path length for paths inside the desired halo
		if(Rhalo < Rdes)
			pathLen += Rdes - Rhalo;

		// Calculate a desired walking speed from the path length
		float walkSpeed = config.gbbSpeedNearXY() * coerce(pathLen / config.gbbFullPathLen(), 0.0f, 1.0f);

		// Enforce a minimum (near) walking speed
		if(walkSpeed != 0.0f)
			walkSpeed = coerceMin(walkSpeed, config.gbbSpeedNearXYMin());

		// Calculate the desired walking direction
		Vec2f walkDir = -bxhat;
		float pathNormalNorm = pathNormal.norm();
		if(pathNormalNorm > 0.0f)
			walkDir = pathNormal / pathNormalNorm;

		// Calculate the desired walking velocity of the robot
		Vec2f walkVel = walkSpeed * walkDir;

		// Calculate the desired direction to face for the robot
		float psiBall = bxAngle;
		float psiTarget = exAngle;
		float psiAwayFromBall = interpolateCoerced(config.gbbAbsBetaLow(), config.gbbAbsBetaHigh(), picut(psiTarget - psiBall), 0.0f, absBeta);
		float psiDes = picut(psiBall + coerceSoftAbs(psiAwayFromBall, config.gbbPsiAwayFromBallMax(), config.gbbPsiAwayFromBallBuf()));

		// Calculate the desired rotation velocity of the robot
		float rotVel = config.gbbSpeedNearZ() * coerceAbs(psiDes / config.gbbAngleLimitNear(), 1.0f);

		// Enforce a minimum (near) rotation speed when the robot is very close to the behind ball position
		float proximityR = fabs((ballDist - ballToBehindBallDist) / config.gbbProximityRadiusTol());
		float proximityB = fabs(beta * (ballToBehindBallDist / config.gbbProximityBetaDistTol()));
		float proximityValue = 1.0f - std::max(proximityR, proximityB); // Proximity value in the range [0,1], where 0 = Robot is on the border of being proximate, 1 = Robot is exactly at the behind ball position
		if(proximityR < 1.0f && proximityB < 1.0f)
		{
			float minRotSpeed = interpolateCoerced(0.0f, config.gbbProximityValueMax(), 0.0f, config.gbbSpeedNearZMin(), proximityValue);
			if(fabs(rotVel) < minRotSpeed)
				rotVel = sign0(rotVel) * minRotSpeed;
		}

		// Construct the near GCV
		Vec3f GCVNear(walkVel.x(), walkVel.y(), rotVel);

		// Calculate the far GCV
		float farWalkAngleRatio = eigenAngleOf(walkDir) / config.gbbAngleLimitFar();
		float farWalkVelX = config.gbbSpeedFarX() * coerceMin<float>(1.0f - fabs(farWalkAngleRatio), 0.0f);
		float farWalkVelZ = config.gbbSpeedFarZ() * coerceAbs(farWalkAngleRatio, 1.0f);
		Vec3f GCVFar(farWalkVelX, 0.0f, farWalkVelZ);

		// Interpolate between the near and far GCVs depending on the distance to the ball to get the final GCV
		float nearFactor = interpolateCoerced(config.gbbRadiusNear(), config.gbbRadiusFar(), 1.0f, 0.0f, ballDist);
		GCV = nearFactor*GCVNear + (1.0f - nearFactor)*GCVFar;

		// Normalise the final GCV to a desired maximum walking speed
		float gcvNorm = GCV.norm();
		float speedLimit = fabs(config.gbbSpeedLimitXYZ());
		if(gcvNorm > speedLimit)
			GCV *= speedLimit / gcvNorm;

		// Plotting
		if(config.plotData())
		{
			PM.plotScalar(beta, PM_GBB_BETA);
			PM.plotScalar(Rmin, PM_GBB_RADIUS_MIN);
			PM.plotScalar(Rdes, PM_GBB_RADIUS_DES);
			PM.plotScalar(Rhalo, PM_GBB_RADIUS_HALO);
			PM.plotScalar(pathLen, PM_GBB_PATH_LEN);
			PM.plotScalar(walkSpeed, PM_GBB_WALK_SPEED);
			PM.plotScalar(psiBall, PM_GBB_PSI_BALL);
			PM.plotScalar(psiTarget, PM_GBB_PSI_TARGET);
			PM.plotScalar(psiDes, PM_GBB_PSI_DES);
			PM.plotScalar(proximityValue, PM_GBB_PROXIMITY_VALUE);
			PM.plotScalar(nearFactor, PM_GBB_NEAR_FACTOR);
		}

		// Visualisation markers
		if(MM.willPublish())
		{
			size_t numPathPoints = m_path.size();
			MM.GBBPath.setNumPoints(numPathPoints);
			for(size_t i = 0; i < numPathPoints; i++)
			{
				const Vec2f& pt = m_path[i];
				MM.GBBPath.setPoint(i, pt.x(), pt.y(), 0.0);
			}
			MM.GBBPath.show();
			if(!config.gbbVisSimple())
			{
				MM.GBBPsiDes.setPoint(0, 0.0, 0.0);
				MM.GBBPsiDes.setPoint(1, 3.0*cos(psiDes), 3.0*sin(psiDes));
				MM.GBBPsiDes.show();
				bool showRobotHalo = config.gbbVisRobotHalo();
				for(int i = 0; i <= WAKMarkerMan::NumCirclePoints - 1; i++)
				{
					float theta = i * (M_2PI / (WAKMarkerMan::NumCirclePoints - 1));
					float ctheta = cos(theta);
					float stheta = sin(theta);
					MM.GBBFarCircle.setPoint(i, ball.x() + config.gbbRadiusFar()*ctheta, ball.y() + config.gbbRadiusFar()*stheta, 0.0);
					MM.GBBNearCircle.setPoint(i, ball.x() + config.gbbRadiusNear()*ctheta, ball.y() + config.gbbRadiusNear()*stheta, 0.0);
					MM.GBBHaloCircle.setPoint(i, ball.x() + Rdes*ctheta, ball.y() + Rdes*stheta, 0.0);
					if(showRobotHalo)
					{
						float minBallDist = minDesiredBallDist(theta, reqBallDir.x());
						MM.GBBRobotHalo.setPoint(i, minBallDist*ctheta, minBallDist*stheta, 0.0);
					}
				}
				MM.GBBFarCircle.show();
				MM.GBBNearCircle.show();
				MM.GBBHaloCircle.show();
				if(showRobotHalo)
					MM.GBBRobotHalo.show();
				MM.GBBBetaAngle.setPoint(0, ball.x() - config.gbbRadiusFar()*bxhat.x(), ball.y() - config.gbbRadiusFar()*bxhat.y());
				MM.GBBBetaAngle.setPoint(1, ball.x(), ball.y());
				MM.GBBBetaAngle.setPoint(2, ball.x() + config.gbbRadiusFar()*txhat.x(), ball.y() + config.gbbRadiusFar()*txhat.y());
				MM.GBBBetaAngle.show();
			}
			MM.GBBBehindBall.setPoint(0, behindBall.x(), behindBall.y());
			MM.GBBBehindBall.setPoint(1, behindBall.x() + reqBallDir.y()*eyhat.x(), behindBall.y() + reqBallDir.y()*eyhat.y());
			MM.GBBBehindBall.setPoint(2, ball.x(), ball.y());
			MM.GBBBehindBall.show();
		}
	}

	// #################### Robot Stuck Detection ####################

	// Detect whether go behind ball is somehow stuck
	bool ballClose = (ballDist < config.gbbStuckMaxBallDist());
	bool nearZeroGCV = (GCV.norm() < config.gbbStuckMaxGcv());
	m_stuck.add(ballClose && nearZeroGCV);
	if(m_stuck.reached(TO_COUNT(config.gbbStuckTime())))
	{
		if(ballActionIsDribble)
			m_ballActionTip = BA_KICK;
		else
			m_ballActionTip = BA_DRIBBLE;
		m_stuck.reset();
	}

	// #################### Finalisation ####################

	// Apply obstacle avoidance and set the walking target
	WBS.obstacleAvoidance(GCV, walkingTarget);
	WBS.setWalkingTarget(walkingTarget);

	// Set our walking GCV
	AV.GCV = GCV;
	AV.halt = false;
	AV.doKick = false;
	AV.rightKick = true;
	AV.doDive = DD_NONE;

	// Print info about the go behind ball state
	if(config.debugMsgGBB() && WBS.stateCycle() % 20 == 1)
	{
		printf("GBB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), GCV(%.2f, %.2f, %.2f) FOR %s\n",
		       (SV.haveBall ? "BALL" : "ball"), ball.x(), ball.y(), (WBS.haveBallTarget ? "BALLTGT" : "balltgt"), GV.ballTargetTypeChar(),
		       ballTarget.x(), ballTarget.y(), AV.GCV.x(), AV.GCV.y(), AV.GCV.z(), (ballActionIsDribble ? "DRIBBLE" : "KICK"));
	}

	// #################### Plotting and Visualisation ####################

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(ballDist, PM_GBB_BALLDIST);
		PM.plotScalar(footDueToLessDist, PM_GBB_FOOTDUETOLESSDIST);
		PM.plotScalar(footDueToBallPose, PM_GBB_FOOTDUETOBALLPOSE);
		PM.plotScalar(reconsiderFoot * PMSCALE_FOOTOK, PM_GBB_RECONSIDERFOOT);
		PM.plotScalar(m_useRightFoot * PMSCALE_FOOTSEL, PM_GBB_USERIGHTFOOT);
		PM.plotScalar(reqBallDir.x(), PM_GBB_REQBALLX);
		PM.plotScalar(reqBallDir.y(), PM_GBB_REQBALLY);
		PM.plotScalar(GCV.x(), PM_GBB_GCVX);
		PM.plotScalar(GCV.y(), PM_GBB_GCVY);
		PM.plotScalar(GCV.z(), PM_GBB_GCVZ);
		PM.plotScalar(GCV.norm(), PM_GBB_GCVN);
		PM.plotScalar(m_ballActionTip, PM_GBB_BALLACTIONTIP);
	}

	// Visualisation markers
	if(MM.willPublish())
	{
		MM.SubStateText.setText((m_useRightFoot ? "Right for " : "Left for ") + ballActionTypeName(WBS.ballAction()));
		MM.SubStateText.updateAdd();
		if(!config.gbbVisSimple())
		{
			Vec2f ballViewU = 2.0f*cos(config.gbbPsiAwayFromBallMax())*bxhat;
			Vec2f ballViewV = 2.0f*sin(config.gbbPsiAwayFromBallMax())*byhat;
			MM.GBBBallView.setPoint(0, ballViewU.x() + ballViewV.x(), ballViewU.y() + ballViewV.y());
			MM.GBBBallView.setPoint(1, 0.0, 0.0);
			MM.GBBBallView.setPoint(2, ballViewU.x() - ballViewV.x(), ballViewU.y() - ballViewV.y());
			MM.GBBBallView.show();
		}
	}
}

// Calculate the minimum desired radial distance of the robot to the ball given the current conditions
float BehGoBehindBall::minDesiredBallDist(float ballAngle, float reqBallDirX) const
{
	// The minimum distance to the ball is calculated from a halo around the robot that consists of a rectangular
	// section capped by two semicircular arcs on the robot's left and right side. The x value of the front of the
	// rectangle is given by reqBallDirX, the x value of the back is given by -reqBallDirX, and consequently the
	// radii of both of the two semicircular arcs is reqBallDirX. The y value of the leftmost point of the halo
	// (occurs on the y-axis) is given by config.gbbMinBallDistLeft. The y value of the rightmost point of the halo
	// (occurs on the y-axis) is given by config.gbbMinBallDistRight. If reqBallDirX < MinReqBallDirX, which should
	// never happen, then the value of MinReqBallDirX > 0 is used instead.

	// Precalculate terms
	float cosBallAngle = cos(ballAngle);
	float sinBallAngle = sin(ballAngle);
	float dx = std::max(reqBallDirX, MinReqBallDirX);
	float fyMax = std::max(config.gbbMinBallDistLeft(), MinRadius) - dx;
	float fyMin = dx - std::max(config.gbbMinBallDistRight(), MinRadius);
	float fyAvg = 0.5f*(fyMax + fyMin);

	// Sanity check the min and max values
	if(fyMin > fyMax)
		fyMin = fyMax = fyAvg;

	// Calculate the required minimum desired radial distance to the ball
	if(cosBallAngle == 0.0f)
		return (sinBallAngle >= 0.0f ? dx + fyMax : dx - fyMin);
	else
	{
		float fy = dx * sinBallAngle / fabs(cosBallAngle);
		if(fyMin <= fy && fy <= fyMax)
			return fabs(dx / cosBallAngle);
		else if(fy >= fyAvg)
			return fabs(fyMax*sinBallAngle + sqrt(dx*dx - fyMax*fyMax*cosBallAngle*cosBallAngle));
		else
			return fabs(fyMin*sinBallAngle + sqrt(dx*dx - fyMin*fyMin*cosBallAngle*cosBallAngle));
	}
}
// EOF