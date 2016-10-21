// Walk and kick behaviour state: Kick ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/beh_kick_ball.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// BehKickBall class
//

// Constructor
BehKickBall::BehKickBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID), GazeBehLookForBall(config, SV, WBS, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void BehKickBall::handleActivation(bool nowActive)
{
	// Handle activation of the base class
	GazeBehLookForBall::handleActivation(nowActive);

	// Reset variables
	m_kickLock = false;
}

// Decide whether we are in a position to kick
bool BehKickBall::okToKick() const
{
	// Don't kick if we don't have a ball or ball target to kick to
	if(!SV.haveBall || !WBS.haveBallTarget)
		return false;

	// Calculate whether we are facing the target
	float ballToTargetAngleBiased = WBS.ballToTargetAngle + WBS.ballToTargetAngleOffsetKick;
	float wedgeTolerance = coerceMin(GV.ballTargetWedge - config.kickAccuracyWedge(), config.minBallTargetWedge());
	bool facingTarget = (fabs(ballToTargetAngleBiased) <= 0.5f*wedgeTolerance);

	// Calculate whether the ball is in front of our left or right foot
	float allowedErrorXFwd = config.kbBallErrorXFwd();
	float allowedErrorYIwd = config.kbBallErrorYIwd();
	float allowedErrorYOwd = config.kbBallErrorYOwd();
	Vec2f ballErrorLeft = SV.ballDir - WBS.reqBallDirLeftKb;
	Vec2f ballErrorRight = SV.ballDir - WBS.reqBallDirRightKb;
	bool leftFootInPosition = (SV.ballDir.x() > 0.0f && ballErrorLeft.x() < allowedErrorXFwd && ballErrorLeft.y() < allowedErrorYOwd && ballErrorLeft.y() > -allowedErrorYIwd); // The ball is within a rectangle of where it should be in front of the left foot
	bool rightFootInPosition = (SV.ballDir.x() > 0.0f && ballErrorRight.x() < allowedErrorXFwd && ballErrorRight.y() < allowedErrorYIwd && ballErrorRight.y() > -allowedErrorYOwd); // The ball is within a rectangle of where it should be in front of the right foot

	// Account for the case where the use of a particular foot is forced
	if(config.forceKickRightFoot)
		leftFootInPosition = false; // Ignore balls in position in front of the left foot
	else if(config.forceKickLeftFoot)
		rightFootInPosition = false; // Ignore balls in position in front of the right foot

	// Decide whether we are in the position to kick
	bool inKickPosition = (facingTarget && (leftFootInPosition || rightFootInPosition));

	// Print information about checking whether we can kick
	if(config.debugMsgKB() && WBS.stateCycle() % 20 == 1)
	{
		printf("KB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f), %s, %s\n",
		       (SV.haveBall ? "BALL" : "ball"), SV.ballDir.x(), SV.ballDir.y(), (WBS.haveBallTarget ? "BALLTGT" : "balltgt"),
		       GV.ballTargetTypeChar(), GV.ballTargetDir.x(), GV.ballTargetDir.y(), (leftFootInPosition ? "ERR_LEFT" : "err_left"),
		       ballErrorLeft.x(), ballErrorLeft.y(), (rightFootInPosition ? "ERR_RIGHT" : "err_right"), ballErrorRight.x(), ballErrorRight.y(),
		       (facingTarget ? "ANGLE" : "angle"), ballToTargetAngleBiased, (facingTarget ? "AIMED" : "not aimed"), (inKickPosition ? "CAN KICK" : "no"));
	}

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(wedgeTolerance, PM_KB_WEDGETOL);
		PM.plotScalar(ballErrorLeft.x(), PM_KB_BALLERRORLEFTX);
		PM.plotScalar(ballErrorLeft.y(), PM_KB_BALLERRORLEFTY);
		PM.plotScalar(ballErrorRight.x(), PM_KB_BALLERRORRIGHTX);
		PM.plotScalar(ballErrorRight.y(), PM_KB_BALLERRORRIGHTY);
		PM.plotScalar(facingTarget * PMSCALE_FACINGOK, PM_KB_FACINGTARGET);
		PM.plotScalar(leftFootInPosition * PMSCALE_FOOTOK, PM_KB_LEFTFOOTOK);
		PM.plotScalar(rightFootInPosition * PMSCALE_FOOTOK, PM_KB_RIGHTFOOTOK);
		PM.plotScalar(inKickPosition * PMSCALE_OKTOKDB, PM_KB_OKTOKICK);
	}

	// Visualisation markers
	if(MM.willPublish() && WBS.ballAction() == BA_KICK && !isActive() && !WGS.gameStateIsNew())
	{
		double angleLeft = ballToTargetAngleBiased + 0.5*wedgeTolerance;
		double angleRight = ballToTargetAngleBiased - 0.5*wedgeTolerance;
		double distLeft = coerce(WBS.ballToTargetDist / cos(coerceAbs(picut(angleLeft - WBS.ballToTargetAngle), M_PI_2 - 1e-6)), 1e-6, 50.0);
		double distRight = coerce(WBS.ballToTargetDist / cos(coerceAbs(picut(angleRight - WBS.ballToTargetAngle), M_PI_2 - 1e-6)), 1e-6, 50.0);
		MM.BallToTargetWedge.setPoint(0, SV.ballDir.x() + distLeft*cos(angleLeft), SV.ballDir.y() + distLeft*sin(angleLeft));
		MM.BallToTargetWedge.setPoint(1, SV.ballDir.x(), SV.ballDir.y());
		MM.BallToTargetWedge.setPoint(2, SV.ballDir.x() + distRight*cos(angleRight), SV.ballDir.y() + distRight*sin(angleRight));
		MM.BallToTargetWedge.show();
		if(!config.forceKickRightFoot)
		{
			double minX = 0.0f;
			double maxX = WBS.reqBallDirLeftKb.x() + allowedErrorXFwd;
			double minY = WBS.reqBallDirLeftKb.y() - allowedErrorYIwd;
			double maxY = WBS.reqBallDirLeftKb.y() + allowedErrorYOwd;
			MM.KBBallRegionL.setPoint(0, minX, minY);
			MM.KBBallRegionL.setPoint(1, minX, maxY);
			MM.KBBallRegionL.setPoint(2, maxX, maxY);
			MM.KBBallRegionL.setPoint(3, maxX, minY);
			MM.KBBallRegionL.setPoint(4, minX, minY);
			if(inKickPosition && leftFootInPosition) MM.KBBallRegionL.setColor(0.0, 1.0, 0.3);
			else if(leftFootInPosition) MM.KBBallRegionL.setColor(0.0, 1.0, 1.0);
			else MM.KBBallRegionL.setColor(0.0, 0.3, 1.0);
			MM.KBBallRegionL.show();
		}
		if(!config.forceKickLeftFoot || config.forceKickRightFoot)
		{
			double minX = 0.0f;
			double maxX = WBS.reqBallDirRightKb.x() + allowedErrorXFwd;
			double minY = WBS.reqBallDirRightKb.y() - allowedErrorYOwd;
			double maxY = WBS.reqBallDirRightKb.y() + allowedErrorYIwd;
			MM.KBBallRegionR.setPoint(0, minX, minY);
			MM.KBBallRegionR.setPoint(1, minX, maxY);
			MM.KBBallRegionR.setPoint(2, maxX, maxY);
			MM.KBBallRegionR.setPoint(3, maxX, minY);
			MM.KBBallRegionR.setPoint(4, minX, minY);
			if(inKickPosition && rightFootInPosition) MM.KBBallRegionR.setColor(0.0, 1.0, 0.3);
			else if(rightFootInPosition) MM.KBBallRegionR.setColor(0.0, 1.0, 1.0);
			else MM.KBBallRegionR.setColor(0.0, 0.3, 1.0);
			MM.KBBallRegionR.show();
		}
	}

	// Return whether we can kick
	return inKickPosition;
}

// Decide whether we are still in an acceptable position to kick, assuming that okToKick() returned true in the recent past
bool BehKickBall::stillOkToKick() const
{
	// Don't kick if we don't have a ball or ball target to kick to
	if(!SV.haveBall || !WBS.haveBallTarget)
		return false;

	// Calculate whether we are approximately still facing the target
	float ballToTargetAngleBiased = WBS.ballToTargetAngle + WBS.ballToTargetAngleOffsetKick;
	float wedgeTolerance = coerceMin(GV.ballTargetWedge, config.minBallTargetWedge() + config.minBallTargetWedgeExtra());
	bool stillFacingTarget = (fabs(ballToTargetAngleBiased) <= 0.5f*wedgeTolerance);

	// Calculate whether the ball is approximately still in front of our left or right foot
	float allowedErrorXFwd = config.kbBallErrorXFwd() + config.kbBallErrorXFwdExtra();
	float allowedErrorYIwd = config.kbBallErrorYIwd() + config.kbBallErrorYIwdExtra();
	float allowedErrorYOwd = config.kbBallErrorYOwd() + config.kbBallErrorYOwdExtra();
	Vec2f ballErrorLeft = SV.ballDir - WBS.reqBallDirLeftKb;
	Vec2f ballErrorRight = SV.ballDir - WBS.reqBallDirRightKb;
	bool leftFootStillInPosition = (SV.ballDir.x() > 0.0f && ballErrorLeft.x() < allowedErrorXFwd && ballErrorLeft.y() < allowedErrorYOwd && ballErrorLeft.y() > -allowedErrorYIwd); // The ball is within a rectangle of where it should be in front of the left foot
	bool rightFootStillInPosition = (SV.ballDir.x() > 0.0f && ballErrorRight.x() < allowedErrorXFwd && ballErrorRight.y() < allowedErrorYIwd && ballErrorRight.y() > -allowedErrorYOwd); // The ball is within a rectangle of where it should be in front of the right foot

	// Account for the case where the use of a particular foot is forced
	if(config.forceKickRightFoot)
		leftFootStillInPosition = false; // Ignore balls in position in front of the left foot
	else if(config.forceKickLeftFoot)
		rightFootStillInPosition = false; // Ignore balls in position in front of the right foot

	// Decide whether we are still in the position to kick
	bool stillInKickPosition = (stillFacingTarget && (leftFootStillInPosition || rightFootStillInPosition));

	// Print information about checking whether we can still kick
	if(config.debugMsgKB() && WBS.stateCycle() % 20 == 1)
	{
		printf("STILL KB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f), %s, %s\n",
		       (SV.haveBall ? "BALL" : "ball"), SV.ballDir.x(), SV.ballDir.y(), (WBS.haveBallTarget ? "BALLTGT" : "balltgt"),
		       GV.ballTargetTypeChar(), GV.ballTargetDir.x(), GV.ballTargetDir.y(), (leftFootStillInPosition ? "ERR_LEFT" : "err_left"),
		       ballErrorLeft.x(), ballErrorLeft.y(), (rightFootStillInPosition ? "ERR_RIGHT" : "err_right"), ballErrorRight.x(), ballErrorRight.y(),
		       (stillFacingTarget ? "ANGLE" : "angle"), ballToTargetAngleBiased, (stillFacingTarget ? "AIMED" : "not aimed"), (stillInKickPosition ? "CAN KICK" : "no"));
	}

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(stillFacingTarget * PMSCALE_FACINGOK, PM_KB_FACINGTARGETSTILL);
		PM.plotScalar(leftFootStillInPosition * PMSCALE_FOOTOK, PM_KB_LEFTFOOTOKSTILL);
		PM.plotScalar(rightFootStillInPosition * PMSCALE_FOOTOK, PM_KB_RIGHTFOOTOKSTILL);
		PM.plotScalar(stillInKickPosition * PMSCALE_OKTOKDB, PM_KB_OKTOKICKSTILL);
	}

	// Visualisation markers
	if(MM.willPublish() && isActive() && !WGS.gameStateIsNew())
	{
		double angleLeft = ballToTargetAngleBiased + 0.5*wedgeTolerance;
		double angleRight = ballToTargetAngleBiased - 0.5*wedgeTolerance;
		double distLeft = coerce(WBS.ballToTargetDist / cos(coerceAbs(picut(angleLeft - WBS.ballToTargetAngle), M_PI_2 - 1e-6)), 1e-6, 50.0);
		double distRight = coerce(WBS.ballToTargetDist / cos(coerceAbs(picut(angleRight - WBS.ballToTargetAngle), M_PI_2 - 1e-6)), 1e-6, 50.0);
		MM.BallToTargetWedge.setPoint(0, SV.ballDir.x() + distLeft*cos(angleLeft), SV.ballDir.y() + distLeft*sin(angleLeft));
		MM.BallToTargetWedge.setPoint(1, SV.ballDir.x(), SV.ballDir.y());
		MM.BallToTargetWedge.setPoint(2, SV.ballDir.x() + distRight*cos(angleRight), SV.ballDir.y() + distRight*sin(angleRight));
		MM.BallToTargetWedge.show();
		if(!config.forceKickRightFoot)
		{
			double minX = 0.0f;
			double maxX = WBS.reqBallDirLeftKb.x() + allowedErrorXFwd;
			double minY = WBS.reqBallDirLeftKb.y() - allowedErrorYIwd;
			double maxY = WBS.reqBallDirLeftKb.y() + allowedErrorYOwd;
			MM.KBBallRegionL.setPoint(0, minX, minY);
			MM.KBBallRegionL.setPoint(1, minX, maxY);
			MM.KBBallRegionL.setPoint(2, maxX, maxY);
			MM.KBBallRegionL.setPoint(3, maxX, minY);
			MM.KBBallRegionL.setPoint(4, minX, minY);
			if(stillInKickPosition && leftFootStillInPosition) MM.KBBallRegionL.setColor(0.0, 1.0, 0.3);
			else if(leftFootStillInPosition) MM.KBBallRegionL.setColor(0.0, 1.0, 1.0);
			else MM.KBBallRegionL.setColor(0.0, 0.3, 1.0);
			MM.KBBallRegionL.show();
		}
		if(!config.forceKickLeftFoot || config.forceKickRightFoot)
		{
			double minX = 0.0f;
			double maxX = WBS.reqBallDirRightKb.x() + allowedErrorXFwd;
			double minY = WBS.reqBallDirRightKb.y() - allowedErrorYOwd;
			double maxY = WBS.reqBallDirRightKb.y() + allowedErrorYIwd;
			MM.KBBallRegionR.setPoint(0, minX, minY);
			MM.KBBallRegionR.setPoint(1, minX, maxY);
			MM.KBBallRegionR.setPoint(2, maxX, maxY);
			MM.KBBallRegionR.setPoint(3, maxX, minY);
			MM.KBBallRegionR.setPoint(4, minX, minY);
			if(stillInKickPosition && rightFootStillInPosition) MM.KBBallRegionR.setColor(0.0, 1.0, 0.3);
			else if(rightFootStillInPosition) MM.KBBallRegionR.setColor(0.0, 1.0, 1.0);
			else MM.KBBallRegionR.setColor(0.0, 0.3, 1.0);
			MM.KBBallRegionR.show();
		}
	}

	// Return whether we are still ok to kick
	return stillInKickPosition;
}

// Decision of best kick foot
bool BehKickBall::bestKickRight() const
{
	// Force the use of a particular foot if required
	if(config.forceKickRightFoot)
		return true;
	else if(config.forceKickLeftFoot)
		return false;

	// Return whether the right foot is currently more appropriate to kick with
	return (SV.ballDir.y() < WBS.reqBallDirMidY); // Note: We have to make a choice, whether SV.haveBall or not!
}

// Execute function
void BehKickBall::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Look for the ball
	GazeBehLookForBall::execute(AV, lastAV, justActivated);

	//
	// Walking control
	//

	// Stop walking and don't do anything else
	AV.halt = true;
	AV.GCV.setZero();
	AV.doDive = DD_NONE;

	//
	// Kicking control
	//

	// Select the best kick foot for right now
	AV.rightKick = bestKickRight();

	// Evaluate whether we're still ok to kick
	bool stillOk = stillOkToKick();

	// Decide whether we are in a state yet where we can command the kick
	if(SV.isWalking())
	{
		AV.doKick = false;
		m_kickLock = true;
	}
	else if(SV.isStanding() && stillOk)
	{
		AV.doKick = true;
		m_kickLock = true;
	}
	else
	{
		AV.doKick = false;
		m_kickLock = false;
	}

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(AV.doKick * PMSCALE_KICK, PM_KB_DOKICK);
		PM.plotScalar(AV.rightKick * PMSCALE_FOOTSEL, PM_KB_BESTKICKRIGHT);
		PM.plotScalar(m_kickLock * PMSCALE_LOCK, PM_KB_KICKLOCK);
	}

	// Visualisation markers
	if(MM.willPublish())
	{
		MM.SubStateText.setText(AV.rightKick ? "Right Kick" : "Left Kick");
		MM.SubStateText.updateAdd();
		MM.KBKickVector.setPoint(0, SV.ballDir.x(), SV.ballDir.y());
		MM.KBKickVector.setPoint(1, SV.ballDir.x() + config.kickMaxDist(), SV.ballDir.y());
		MM.KBKickVector.show();
	}
}
// EOF