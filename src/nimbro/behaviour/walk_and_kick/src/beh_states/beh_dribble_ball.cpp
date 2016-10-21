// Walk and kick behaviour state: Dribble ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/beh_dribble_ball.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// BehDribbleBall class
//

// Constructor
BehDribbleBall::BehDribbleBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID), GazeBehLookForBall(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());

	// Config parameter callbacks
	config.dbAppFootChangeWormTime.setCallback(boost::bind(&TheWorm::updateWormTime, &m_changeToRightFoot, &config.dbAppFootChangeWormTime), true);
}

// Handle activation function
void BehDribbleBall::handleActivation(bool nowActive)
{
	// Handle activation of the base class
	GazeBehLookForBall::handleActivation(nowActive);

	// Reset variables
	m_useRightFoot = true;
	m_changeToRightFoot.reset();
}

// Calculate the required minimum ball target wedge for dribbling
float BehDribbleBall::minBallTargetWedge() const
{
	// Return the required minimum ball target wedge
	float desiredMinWedge = interpolateCoerced(config.dbTargetDistForMinWedge(), config.dbTargetDistForMinWedgeHigh(), config.minBallTargetWedge(), config.dbTargetMinWedgeHigh(), WBS.ballToTargetDist);
	return coerce(desiredMinWedge, config.minBallTargetWedge(), config.maxBallTargetWedge());
}

// Decide whether we are in a position to dribble
bool BehDribbleBall::okToDribble() const
{
	// Don't dribble if we don't have a ball or ball target to dribble to
	if(!SV.haveBall || !WBS.haveBallTarget)
		return false;

	// Calculate whether we are facing the target
	float wedgeTolerance = coerceMin(GV.ballTargetWedge - config.dribbleAccuracyWedge(), minBallTargetWedge());
	bool facingTarget = (fabs(WBS.ballToTargetAngle) <= 0.5f*wedgeTolerance);

	// Calculate the allowed ball spread
	float spreadSlope = config.dbBallSpreadSlope();
	float spreadAcc = config.dbBallSpreadAcc();
	float ballErrorLeftX = SV.ballDir.x() - WBS.reqBallDirLeftDb.x();
	float ballErrorRightX = SV.ballDir.x() - WBS.reqBallDirRightDb.x();
	float leftSpread = coerceMin(ballErrorLeftX*(spreadSlope + 0.5f*ballErrorLeftX*spreadAcc), 0.0f);
	float rightSpread = coerceMin(ballErrorRightX*(spreadSlope + 0.5f*ballErrorRightX*spreadAcc), 0.0f);

	// Calculate whether the ball is suitably in front of us
	float ballMaxYMag = config.dbBallMaxYMag();
	float leftBound = WBS.reqBallDirMidY + ballMaxYMag;
	float rightBound = WBS.reqBallDirMidY - ballMaxYMag;
	float minBallX = 0.0f;
	float maxBallX = config.dbBallMaxX();
	float minBallY = rightBound - rightSpread;
	float maxBallY = leftBound + leftSpread;
	bool ballXOk = (SV.ballDir.x() > minBallX && SV.ballDir.x() < maxBallX);
	bool ballYOk = (SV.ballDir.y() > minBallY && SV.ballDir.y() < maxBallY);
	bool ballOk = (ballXOk && ballYOk);

	// Decide whether we are in the position to dribble
	bool inDribblePosition = (facingTarget && ballOk);

	// Print information about checking whether we can dribble
	if(config.debugMsgDB() && WBS.stateCycle() % 20 == 1 && !isActive())
	{
		printf("DB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f), %s, %s\n",
		       (SV.haveBall ? "BALL" : "ball"), SV.ballDir.x(), SV.ballDir.y(), (WBS.haveBallTarget ? "BALLTGT" : "balltgt"),
		       GV.ballTargetTypeChar(), GV.ballTargetDir.x(), GV.ballTargetDir.y(), (ballXOk ? "BALLXLIM" : "ballxlim"),
		       minBallX, maxBallX, (ballYOk ? "BALLYLIM" : "ballylim"), minBallY, maxBallY, (facingTarget ? "ANGLE" : "angle"),
		       WBS.ballToTargetAngle, (facingTarget ? "AIMED" : "not aimed"), (inDribblePosition ? "CAN DRIBBLE" : "no"));
	}

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(wedgeTolerance, PM_DB_WEDGETOL);
		PM.plotScalar(facingTarget * PMSCALE_FACINGOK, PM_DB_FACINGTARGET);
		PM.plotScalar(ballXOk * PMSCALE_FOOTOK, PM_DB_BALLXOK);
		PM.plotScalar(ballYOk * PMSCALE_FOOTOK, PM_DB_BALLYOK);
		PM.plotScalar(inDribblePosition * PMSCALE_OKTOKDB, PM_DB_OKTODRIBBLE);
	}

	// Visualisation markers
	if(MM.willPublish() && WBS.ballAction() == BA_DRIBBLE && !isActive() && !WGS.gameStateIsNew())
	{
		double angleLeft = WBS.ballToTargetAngle + 0.5*wedgeTolerance;
		double angleRight = WBS.ballToTargetAngle - 0.5*wedgeTolerance;
		double distLeft = coerce(WBS.ballToTargetDist / cos(coerceAbs(picut(angleLeft - WBS.ballToTargetAngle), M_PI_2 - 1e-6)), 1e-6, 50.0);
		double distRight = coerce(WBS.ballToTargetDist / cos(coerceAbs(picut(angleRight - WBS.ballToTargetAngle), M_PI_2 - 1e-6)), 1e-6, 50.0);
		MM.BallToTargetWedge.setPoint(0, SV.ballDir.x() + distLeft*cos(angleLeft), SV.ballDir.y() + distLeft*sin(angleLeft));
		MM.BallToTargetWedge.setPoint(1, SV.ballDir.x(), SV.ballDir.y());
		MM.BallToTargetWedge.setPoint(2, SV.ballDir.x() + distRight*cos(angleRight), SV.ballDir.y() + distRight*sin(angleRight));
		MM.BallToTargetWedge.show();
		const std::size_t N = 30, M = 2*(N + 1) + 3; // Note: If you change N here then you also need to change the value in the WAKMarkerMan constructor
		MM.DBBallRegion.setNumPoints(M);
		MM.DBBallRegion.setPoint(0, minBallX, leftBound);
		for(std::size_t i = 0; i <= N; i++)
		{
			float xL = i * (maxBallX - WBS.reqBallDirLeftDb.x()) / N;
			float xR = i * (maxBallX - WBS.reqBallDirRightDb.x()) / N;
			double leftY = leftBound + coerceMin(xL*(spreadSlope + 0.5f*xL*spreadAcc), 0.0f);
			double rightY = rightBound - coerceMin(xR*(spreadSlope + 0.5f*xR*spreadAcc), 0.0f);
			MM.DBBallRegion.setPoint(i + 1, WBS.reqBallDirLeftDb.x() + xL, leftY);
			MM.DBBallRegion.setPoint(M - i - 3, WBS.reqBallDirRightDb.x() + xR, rightY);
		}
		MM.DBBallRegion.setPoint(M - 2, minBallX, rightBound);
		MM.DBBallRegion.setPoint(M - 1, minBallX, leftBound);
		if(inDribblePosition) MM.DBBallRegion.setColor(0.0, 1.0, 0.3);
		else if(ballOk) MM.DBBallRegion.setColor(0.0, 1.0, 1.0);
		else MM.DBBallRegion.setColor(0.0, 0.3, 1.0);
		MM.DBBallRegion.show();
	}

	// Return whether we can dribble
	return inDribblePosition;
}

// Decide whether we are still in good dribbling form, assuming that okToDribble() returned true in the recent past
bool BehDribbleBall::stillOkToDribble() const
{
	// Not ok if we don't have a ball or ball target to dribble to
	if(!SV.haveBall || !WBS.haveBallTarget)
		return false;

	// Calculate whether we are approximately still facing the target
	float wedgeTolerance = coerceMin(GV.ballTargetWedge, minBallTargetWedge() + config.minBallTargetWedgeExtra());
	float reqBallDist = std::max(WBS.reqBallDirLeftDb.norm(), WBS.reqBallDirRightDb.norm());
	wedgeTolerance = interpolateCoerced(0.0f, config.dbBallDistForFarBall(), wedgeTolerance, std::max(config.dbTargetWedgeForFarBall(), wedgeTolerance), SV.ballDist - reqBallDist);
	bool stillFacingTarget = (fabs(WBS.ballToTargetAngle) <= 0.5f*wedgeTolerance);

	// Calculate the allowed ball spread
	float spreadSlope = config.dbBallSpreadSlope() + config.dbBallSpreadSlopeExtra();
	float spreadAcc = config.dbBallSpreadAcc() + config.dbBallSpreadAccExtra();
	float ballErrorLeftX = SV.ballDir.x() - WBS.reqBallDirLeftDb.x();
	float ballErrorRightX = SV.ballDir.x() - WBS.reqBallDirRightDb.x();
	float leftSpread = coerceMin(ballErrorLeftX*(spreadSlope + 0.5f*ballErrorLeftX*spreadAcc), 0.0f);
	float rightSpread = coerceMin(ballErrorRightX*(spreadSlope + 0.5f*ballErrorRightX*spreadAcc), 0.0f);

	// Calculate whether the ball is approximately still suitably in front of us
	float ballMaxYMag = config.dbBallMaxYMag() + config.dbBallMaxYMagExtra();
	float leftBound = WBS.reqBallDirMidY + ballMaxYMag;
	float rightBound = WBS.reqBallDirMidY - ballMaxYMag;
	float minBallX = 0.0f;
	float maxBallX = config.dbBallMaxX();
	float minBallY = rightBound - rightSpread;
	float maxBallY = leftBound + leftSpread;
	bool ballXStillOk = (SV.ballDir.x() > minBallX && SV.ballDir.x() < maxBallX);
	bool ballYStillOk = (SV.ballDir.y() > minBallY && SV.ballDir.y() < maxBallY);
	bool ballStillOk = (ballXStillOk && ballYStillOk);

	// Decide whether we are still in the position to dribble
	bool stillInDribblePosition = (stillFacingTarget && ballStillOk);

	// Print information about checking whether we can dribble
	if(config.debugMsgDB() && WBS.stateCycle() % 20 == 1 && isActive())
	{
		printf("STILL DB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f), %s, %s\n",
		       (SV.haveBall ? "BALL" : "ball"), SV.ballDir.x(), SV.ballDir.y(), (WBS.haveBallTarget ? "BALLTGT" : "balltgt"),
		       GV.ballTargetTypeChar(), GV.ballTargetDir.x(), GV.ballTargetDir.y(), (ballXStillOk ? "BALLXLIM" : "ballxlim"),
		       minBallX, maxBallX, (ballYStillOk ? "BALLYLIM" : "ballylim"), minBallY, maxBallY, (stillFacingTarget ? "ANGLE" : "angle"),
		       WBS.ballToTargetAngle, (stillFacingTarget ? "AIMED" : "not aimed"), (stillInDribblePosition ? "CAN DRIBBLE" : "no"));
	}

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(wedgeTolerance, PM_DB_WEDGETOLSTILL);
		PM.plotScalar(stillFacingTarget * PMSCALE_FACINGOK, PM_DB_FACINGTARGETSTILL);
		PM.plotScalar(ballXStillOk * PMSCALE_FOOTOK, PM_DB_BALLXOKSTILL);
		PM.plotScalar(ballYStillOk * PMSCALE_FOOTOK, PM_DB_BALLYOKSTILL);
		PM.plotScalar(stillInDribblePosition * PMSCALE_OKTOKDB, PM_DB_OKTODRIBBLESTILL);
	}

	// Visualisation markers
	if(MM.willPublish() && isActive() && !WGS.gameStateIsNew())
	{
		double angleLeft = WBS.ballToTargetAngle + 0.5*wedgeTolerance;
		double angleRight = WBS.ballToTargetAngle - 0.5*wedgeTolerance;
		double distLeft = coerce(WBS.ballToTargetDist / cos(coerceAbs(picut(angleLeft - WBS.ballToTargetAngle), M_PI_2 - 1e-6)), 1e-6, 50.0);
		double distRight = coerce(WBS.ballToTargetDist / cos(coerceAbs(picut(angleRight - WBS.ballToTargetAngle), M_PI_2 - 1e-6)), 1e-6, 50.0);
		MM.BallToTargetWedge.setPoint(0, SV.ballDir.x() + distLeft*cos(angleLeft), SV.ballDir.y() + distLeft*sin(angleLeft));
		MM.BallToTargetWedge.setPoint(1, SV.ballDir.x(), SV.ballDir.y());
		MM.BallToTargetWedge.setPoint(2, SV.ballDir.x() + distRight*cos(angleRight), SV.ballDir.y() + distRight*sin(angleRight));
		MM.BallToTargetWedge.show();
		const std::size_t N = 30, M = 2*(N + 1) + 3; // Note: If you change N here then you also need to change the value in the WAKMarkerMan constructor
		MM.DBBallRegion.setNumPoints(M);
		MM.DBBallRegion.setPoint(0, minBallX, leftBound);
		for(std::size_t i = 0; i <= N; i++)
		{
			float xL = i * (maxBallX - WBS.reqBallDirLeftDb.x()) / N;
			float xR = i * (maxBallX - WBS.reqBallDirRightDb.x()) / N;
			double leftY = leftBound + coerceMin(xL*(spreadSlope + 0.5f*xL*spreadAcc), 0.0f);
			double rightY = rightBound - coerceMin(xR*(spreadSlope + 0.5f*xR*spreadAcc), 0.0f);
			MM.DBBallRegion.setPoint(i + 1, WBS.reqBallDirLeftDb.x() + xL, leftY);
			MM.DBBallRegion.setPoint(M - i - 3, WBS.reqBallDirRightDb.x() + xR, rightY);
		}
		MM.DBBallRegion.setPoint(M - 2, minBallX, rightBound);
		MM.DBBallRegion.setPoint(M - 1, minBallX, leftBound);
		if(stillInDribblePosition) MM.DBBallRegion.setColor(0.0, 1.0, 0.3);
		else if(ballStillOk) MM.DBBallRegion.setColor(0.0, 1.0, 1.0);
		else MM.DBBallRegion.setColor(0.0, 0.3, 1.0);
		MM.DBBallRegion.show();
	}

	// Return whether we are still ok to dribble
	return stillInDribblePosition;
}

// Execute function
void BehDribbleBall::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Look for the ball
	GazeBehLookForBall::execute(AV, lastAV, justActivated);

	//
	// Walking control
	//

	// If we have no ball then stop where you are
	if(!SV.haveBall)
	{
		// Set our walking GCV
		AV.GCV.setZero();
		AV.halt = false;
		AV.doKick = false;
		AV.rightKick = true;
		AV.doDive = DD_NONE;

		// Visualisation markers
		if(MM.willPublish())
		{
			MM.SubStateText.setText("Dribble without Ball");
			MM.SubStateText.updateAdd();
		}

		// Return as we have nothing more to do
		return;
	}

	// Required gait control vector
	Vec3f GCV(0.0f, 0.0f, 0.0f); // x forwards, y left, z CCW

	// Save the relative offsets to the ball and target in local variables
	Vec2f ball = SV.ballDir;         // Vector from robot to ball in body-fixed coordinates
	Vec2f target = GV.ballTargetDir; // Vector from robot to target in body-fixed coordinates
	if(!WBS.haveBallTarget)          // If we don't have a ball target then take one that's in the direction we're facing in front of the ball
		target << ball.x() + config.minBallToTargetDist(), ball.y();

	// Calculate the vector from the ball to the target in body-fixed coordinates
	Vec2f E = target - ball;                  // The E coordinate system is centred at the ball
	Vec2f unitEx = eigenNormalized(E);        // x-axis of E: Unit vector from the ball towards the ball target
	Vec2f unitEy = eigenRotatedCCW90(unitEx); // y-axis of E: Unit vector 90 deg CCW from the x-axis of E

	// Transform the robot position and orientation into the E coordinate frame
	Vec2f robotE(-ball.dot(unitEx), -ball.dot(unitEy));
	float robotAngle = -eigenAngleOf(unitEx);

	// Decide which foot we would prefer for dribbling based on the current robot position
	bool preferRightFoot = (robotE.y() >= WBS.reqBallDirMidY);

	// Decide whether we should reconsider the foot we're currently using for dribbling
	if(GV.suggestRightFoot()) // The right foot is being strongly suggested to us from above
		m_changeToRightFoot.vote(true, 2);
	else if(GV.suggestLeftFoot()) // The left foot is being strongly suggested to us from above
		m_changeToRightFoot.vote(false, 2);
	else
		m_changeToRightFoot.vote(preferRightFoot);
	bool reconsiderFoot = (m_changeToRightFoot.unanimous() && m_changeToRightFoot.decision() != m_useRightFoot);

	// Decide which foot to use for dribbling
	if(justActivated || reconsiderFoot)
		m_useRightFoot = m_changeToRightFoot.decision();

	// Calculate the desired path angle for the dribble approach
	Vec2f robotToBehindBallE, robotToPathTargetE;
	float pathAngle = calcPathAngle(m_useRightFoot, robotE, robotToBehindBallE, robotToPathTargetE);
	float localPathAngle = picut(pathAngle - robotAngle);

	// Calculate the robot to path angle interpolation factor u (0.0 = Walk to robot angle, 1.0 = Walk to path angle)
	float u = calcPathInterpFactor(robotToBehindBallE, localPathAngle);

	// Calculate the desired XY walking velocity
	float walkAngle = calcWalkAngle(robotToBehindBallE, u, localPathAngle, pathAngle);
	Vec2f walkVecXY = WBS.calcGcvXY(config.dbAppGcvSpeedX(), config.dbAppGcvSpeedY(), walkAngle);

	// Calculate the final GCV by factoring in the turning Z velocity and XY reduction for angular misalignments
	float angleRatio = localPathAngle / config.dbAppAngleErrLimit();
	float ratioXY = coerce<float>(1.0f - fabs(angleRatio), 0.0f, 1.0f);
	float signedRatioZ = coerceAbs(angleRatio, 1.0f);
	GCV.x() = ratioXY * walkVecXY.x();
	GCV.y() = ratioXY * walkVecXY.y();
	GCV.z() = signedRatioZ * config.dbAppGcvSpeedZ();

	// Normalise the GCV to our desired maximum walking speed
	float gcvNorm = GCV.norm();
	float speedLimit = fabs(config.dbAppGcvSpeedLimit());
	if(gcvNorm > speedLimit)
		GCV *= speedLimit / gcvNorm;

	// Apply obstacle avoidance and set the walking target
	Vec2f walkingTarget = robotToPathTargetE.x() * unitEx + robotToPathTargetE.y() * unitEy;
	WBS.obstacleAvoidance(GCV, walkingTarget);
	WBS.setWalkingTarget(walkingTarget);

	// Set our walking GCV
	AV.GCV = GCV;
	AV.halt = false;
	AV.doKick = false;
	AV.rightKick = true;
	AV.doDive = DD_NONE;

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(preferRightFoot * PMSCALE_FOOTSEL, PM_DBAPP_PREFERRIGHTFOOT);
		PM.plotScalar(reconsiderFoot * PMSCALE_FOOTOK, PM_DBAPP_RECONSIDERFOOT);
		PM.plotScalar(m_useRightFoot * PMSCALE_FOOTSEL, PM_DBAPP_USERIGHTFOOT);
		PM.plotScalar(robotE.x(), PM_DBAPP_ROBOTEX);
		PM.plotScalar(robotE.y(), PM_DBAPP_ROBOTEY);
		PM.plotScalar(localPathAngle, PM_DBAPP_LOCALPATHANGLE);
		PM.plotScalar(u, PM_DBAPP_UFACTOR);
		PM.plotScalar(ratioXY, PM_DBAPP_RATIOXY);
	}

	// Visualisation markers
	if(MM.willPublish())
	{
		MM.SubStateText.setText(m_useRightFoot ? "Right Dribble" : "Left Dribble");
		MM.SubStateText.updateAdd();
	}
}

// Calculate the angle of an arc given the start to stop vector, and assuming that the arc is tangentially of zero slope at the stop point
float BehDribbleBall::calcArcTheta(const Vec2f& vector) const
{
	// Calculate and return theta, the signed CCW rotation that is performed by tracing the arc from start to stop
	float theta = 0.0f;
	if(vector.y() == 0.0f)
	{
		if(vector.x() >= 0.0f)
			theta = 0.0f;
		else
			theta = M_2PI;
	}
	else
	{
		float ysq = vector.y()*vector.y();
		float sqnorm = vector.x()*vector.x() + ysq;
		theta = acos(coerceAbs(1.0f - 2.0f*ysq/sqnorm, 1.0f));
		if(vector.x() > 0.0f && vector.y() > 0.0f)
			theta = -theta;
		else if(vector.x() <= 0.0f && vector.y() > 0.0f)
			theta = theta - M_2PI;
		else if(vector.x() <= 0.0f && vector.y() <= 0.0f)
			theta = M_2PI - theta;
	}
	return theta;
}

// Calculate the required path angle for the dribble approach given a robot position in the E coordinate system
float BehDribbleBall::calcPathAngle(bool useRightFoot, const Vec2f& robotE, Vec2f& robotToBehindBallE, Vec2f& robotToPathTargetE) const
{
	// Decide on the required ball offset
	Vec2f reqBallDir = (useRightFoot ? WBS.reqBallDirRightDb : WBS.reqBallDirLeftDb);

	// Calculate the vector from the robot to the behind ball dribble position
	robotToBehindBallE = -reqBallDir - robotE;

	// Calculate the robot to path target vector
	robotToPathTargetE = robotToBehindBallE;
	robotToPathTargetE.x() -= config.dbAppPathTargetLineOffsetX();
	float minRobotToPathTargetX = std::max<float>(config.dbAppMinPathTargetX(), fabs(robotToPathTargetE.y()) / config.dbAppMaxPathTargetSlope());
	if(robotToPathTargetE.x() < minRobotToPathTargetX)
		robotToPathTargetE.x() = minRobotToPathTargetX;
	robotToPathTargetE.y() *= config.dbAppPathOverdriveY();

	// Calculate the initial angle of the required circular arc path to the target
	return picut(-calcArcTheta(robotToPathTargetE));
}

// Calculate the robot to path angle interpolation factor u (0.0 = Walk to robot angle, 1.0 = Walk to path angle)
float BehDribbleBall::calcPathInterpFactor(const Vec2f& robotToBehindBallE, float localPathAngle) const
{
	// Calculate the ball-fixed funnel coordinates
	float funnelOffsetX = config.dbAppPathTargetLineOffsetX();
	float funnelX = robotToBehindBallE.x() - funnelOffsetX;
	float funnelY = robotToBehindBallE.y();
	float funnelYAbs = fabs(funnelY);

	// Calculate the robot to path angle interpolation factor u (0.0 = Walk to robot angle, 1.0 = Walk to path angle)
	float uCorner = (localPathAngle*funnelY > 0.0f ? 1.0f : 0.0f);
	float u = 0.0f;
	if(funnelX <= 0.0f)
		u = interpolateCoerced(config.dbAppFunnelNeckMag(), config.dbAppFunnelNeckMag() + config.dbAppFunnelEdgeExtra(), 0.0f, uCorner, funnelYAbs);
	else
	{
		float value = funnelX;
		float radialY = funnelYAbs - config.dbAppFunnelNeckMag() - config.dbAppFunnelCurveRadius();
		if(radialY < 0.0f)
			value = sqrt(funnelX*funnelX + radialY*radialY);
		u = interpolateCoerced(config.dbAppFunnelCurveRadius(), config.dbAppFunnelCurveRadius() - config.dbAppFunnelEdgeExtra(), 0.0f, uCorner, value);
	}
	u = coerce(u, 0.0f, 1.0f);

	// Return the required factor
	return u;
}

// Calculate the required local walk angle (i.e. relative to the robot fixed frame) based on the path angle interpolation factor and path angle
float BehDribbleBall::calcWalkAngle(const Vec2f& robotToBehindBallE, float interpFactor, float localPathAngle, float pathAngle) const
{
	// Apply the interpolation factor to calculate the base walk angle
	float walkAngle = interpFactor * localPathAngle;

	// Adjust the walk angle with a bit of extra sidestepping (caused by rotation of the walk angle) to get behind the ball more effectively
	float walkAngleAdjustDes = interpolateCoerced(-config.dbAppAdjustPathAngleHigh(), config.dbAppAdjustPathAngleHigh(), -config.dbAppAdjustWalkAngleMax(), config.dbAppAdjustWalkAngleMax(), pathAngle);
	float walkAngleAdjust = interpolateCoerced(0.0f, config.dbAppFunnelEdgeExtra(), walkAngleAdjustDes, 0.0f, robotToBehindBallE.x() - config.dbAppPathTargetLineOffsetX());
	walkAngle += walkAngleAdjust;

	// Return the calculated walk angle
	return walkAngle;
}
// EOF