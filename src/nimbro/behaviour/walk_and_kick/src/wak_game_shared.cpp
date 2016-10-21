// Walk and kick: Class for shared walk and kick game state variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_game_shared.h>
#include <walk_and_kick/wak_game_state.h>
#include <walk_and_kick/wak_game_manager.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// WAKGameShared class
//

// Constructor
WAKGameShared::WAKGameShared(WAKConfig& config, const SensorVars& SV, const WAKRosInterface& RI, WAKGameManager& GM)
 : PM(RI.getPM())
 , MM(RI.getMM())
 , config(config)
 , SV(SV)
 , RI(RI)
 , GM(GM)
{
	// Reset the shared variables
	resetShared();
}

// Game state registration function
void WAKGameShared::registerState(WAKGameState* state, int ID, const std::string& name) const
{
	// Pass the state registration request on up to the game manager
	GM.registerState(state, ID, name);
}

// Reset function
void WAKGameShared::resetShared()
{
	// Reset the shared game state variables
	/* None yet */
}

// Update function
void WAKGameShared::updateShared()
{
	// Update the shared game state variables
	/* None yet */
}

// Get functions
bool WAKGameShared::gameStateIsNew() const { return GM.m_stateIsNew; }

// Cycle numbers and times
cycle_t WAKGameShared::wakCycle() const { return GM.m_wakCycle; }
cycle_t WAKGameShared::stateCycle() const { return GM.m_stateCycle; }
float WAKGameShared::wakTime() const { return TINC * GM.m_wakCycle; }
float WAKGameShared::stateTime() const { return TINC * GM.m_stateCycle; }

// Handle obstacles in a generic way by post-adjusting a set of commanded game vars
bool WAKGameShared::obstacleBallHandling(GameVars& GV) const
{
	// Plot that obstacle ball handling is not active for the case that this function returns early
	if(config.plotData())
		PM.plotScalar(false * PMSCALE_LEGAL, PM_OBH_ACTIVE);

	// Don't do anything if obstacle avoidance is disabled or we don't have a ball
	if(!config.sEnableObstacles() || !config.obhEnableObstBallHandling() || !SV.haveBall)
		return false;

	// Don't do anything if the closest obstacle is not valid
	if(!SV.obstClosest.valid())
		return false;

	// Calculate the ball to target unit vector
	Vec2f ballToTargetVec = GV.ballTargetDir - SV.ballDir;
	float ballToTargetDist = ballToTargetVec.norm();
	if(ballToTargetDist <= 0.0f)
		return false;
	Vec2f txhat = ballToTargetVec / ballToTargetDist;

	// Calculate the ball to obstacle unit vectors
	Vec2f ballToObstVec = SV.obstClosest.vec - SV.ballDir;
	float ballToObstDist = ballToObstVec.norm();
	if(ballToObstDist <= 0.0f || ballToObstDist >= std::min(config.kickMaxDist(), ballToTargetDist + config.obhObstBeyondTargetBuf()))
		return false;
	Vec2f oxhat = ballToObstVec / ballToObstDist;
	Vec2f oyhat = eigenRotatedCCW90(oxhat);

	// Calculate the angle at the ball from the obstacle to the ball target
	float angleAtBall = atan2(txhat.dot(oyhat), txhat.dot(oxhat));
	bool ballBehindObst = (fabs(angleAtBall) > M_PI_2);
	if(ballBehindObst && ballToObstDist > config.obhObstBeforeBallBuf())
		return false;

	// Wrap the angle at the ball to (-pi/2, pi/2] by factors of pi and adjust for the sign
	float angleAtBallStd = angleAtBall;
	if(angleAtBallStd > M_PI_2)
		angleAtBallStd -= M_PI;
	if(angleAtBallStd <= -M_PI_2)
		angleAtBallStd += M_PI;
	int angleAtBallStdSign = sign(angleAtBallStd);
	angleAtBallStd = fabs(angleAtBallStd);

	// Calculate the high and low angles at the ball, and the appropriate difference
	float angleAtBallHigh = coerce<float>(asin(coerceAbs(config.obhObstClearanceHigh() / ballToObstDist, 1.0f)), 0.0f, config.obhClearanceAngleHighMax());
	float angleAtBallLow = coerce<float>(asin(coerceAbs(config.obhObstClearanceLow() / ballToObstDist, 1.0f)), 0.0f, std::min(angleAtBallHigh, config.obhClearanceAngleLowMax()));

	// Calculate the angle to adjust the target angle by
	float angleAtBallDiff = angleAtBallHigh - angleAtBallLow;
	float targetAngleAdjust = 0.0f;
	if(angleAtBallStd < angleAtBallLow)
		targetAngleAdjust = angleAtBallStdSign * interpolateCoerced(0.0f, angleAtBallLow, 0.0f, angleAtBallDiff, angleAtBallStd);
	else if(angleAtBallStd < angleAtBallHigh)
		targetAngleAdjust = angleAtBallStdSign * interpolateCoerced(angleAtBallLow, angleAtBallHigh, angleAtBallDiff, 0.0f, angleAtBallStd);
	float targetAngleAdjustAbs = fabs(targetAngleAdjust);

	// Calculate the angle adjust limits for dribbling and foot selection
	float angleAdjustForDribble = coerceMin(0.5f * GV.ballTargetWedge * config.obhAngleAdjustWedgeRatio(), 0.0f);
	float angleAdjustForFootSel = config.obhAngleAdjustForFootSel();

	// See whether the obstacle is blocking
	bool obstMightBlock = (angleAtBallDiff > angleAdjustForDribble);
	bool obstIsBlocking = (obstMightBlock && (angleAtBallStd < angleAtBallLow || targetAngleAdjustAbs > angleAdjustForDribble));

	// Disable kick and suggest a foot to use if the obstacle is blocking
	bool kickIfPossible = !obstIsBlocking;
	GameVars::FootSelection suggestFoot = GameVars::FS_EITHER_FOOT;
	if(obstIsBlocking && ballToObstDist < config.obhFootSelObstBallDistMax())
	{
		if(targetAngleAdjust > angleAdjustForFootSel)
			suggestFoot = (ballBehindObst ? GameVars::FS_LEFT_FOOT : GameVars::FS_RIGHT_FOOT);
		else if(targetAngleAdjust < -angleAdjustForFootSel)
			suggestFoot = (ballBehindObst ? GameVars::FS_RIGHT_FOOT : GameVars::FS_LEFT_FOOT);
	}

	// Decide whether the obstacle ball handling has to make an adjustment to the game variables
	bool adjustNotNeeded = (targetAngleAdjustAbs <= 0.0f && kickIfPossible && suggestFoot == GameVars::FS_EITHER_FOOT);

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(!adjustNotNeeded * PMSCALE_LEGAL, PM_OBH_ACTIVE);
		PM.plotScalar(ballToObstDist, PM_OBH_BALLTOOBSTDIST);
		PM.plotScalar(angleAtBallLow, PM_OBH_ANGLEATBALLLOW);
		PM.plotScalar(angleAtBallHigh, PM_OBH_ANGLEATBALLHIGH);
		PM.plotScalar(angleAtBallStd, PM_OBH_ANGLEATBALLSTD);
		PM.plotScalar(targetAngleAdjust, PM_OBH_TARGETANGLEADJUST);
		PM.plotScalar(angleAdjustForDribble, PM_OBH_ANGLEADJUSTFORDRIBBLE);
		PM.plotScalar(angleAdjustForFootSel, PM_OBH_ANGLEADJUSTFORFOOTSEL);
		PM.plotScalar(kickIfPossible * PMSCALE_KICK, PM_OBH_KICKIFPOSSIBLE);
		PM.plotScalar(suggestFoot, PM_OBH_SUGGESTFOOT);
	}

	// If no adjustment is required then we are fine
	if(adjustNotNeeded)
		return false;

	// Calculate the new ball target as a rotation of the nominal ball target
	Vec2f ballToTargetVecDes = eigenRotatedCCW(ballToTargetVec, targetAngleAdjust);
	Vec2f ballTargetVec = SV.ballDir + ballToTargetVecDes;

	// Update the game variables
	GV.ballTargetDir = ballTargetVec;
	if(GV.kickIfPossible)
		GV.kickIfPossible = kickIfPossible;
	if(GV.suggestFoot == GameVars::FS_EITHER_FOOT)
		GV.suggestFoot = suggestFoot;

	// Return that the game variables were modified
	return true;
}

// Calculate the desired pose to be in for the current kickoff
Vec3f WAKGameShared::getKickoffPoseTarget() const
{
	// Return the required target kickoff pose
	if(SV.kickoffType == KT_MANUAL)
		return (SV.haveRobotPose ? SV.robotPose : applyGoalSign(Vec3f(0.0f, 0.0f, 0.0f)));
	else if(SV.isGoalie())
		return applyGoalSign(Vec3f(config.posPoseGoalieX() - field.fieldLengthH(), config.posPoseGoalieY(), config.posPoseGoalieT()));
	else if(SV.kickoffType == KT_ATTACKING)
		return applyGoalSign(Vec3f(config.posPoseAttackingX(), config.posPoseAttackingY(), config.posPoseAttackingT()));
	else // If KT_DEFENDING or KT_DROPBALL
		return applyGoalSign(Vec3f(config.posPoseDefendingX() - field.circleRadius(), config.posPoseDefendingY(), config.posPoseDefendingT()));
}

// Judge whether a given pose is legal for the set phase given the current game configuration
bool WAKGameShared::poseIsLegalForKickoff(const Vec3f& pose) const
{
	// Don't consider positioning legality if this is a manual kickoff type
	if(SV.kickoffType == KT_MANUAL)
		return true;

	// Account for the goal sign
	Vec2f pose2D = pose.head<2>();
	Vec2f pose2DGS = applyGoalSign(pose2D);

	// Calculate whether the pose is legal
	float B = config.posPoseLegalityBuffer();
	if(pose2DGS.x() > -B) return false;
	if((SV.kickoffType == KT_DEFENDING || SV.kickoffType == KT_DROPBALL) && pose2DGS.norm() < field.circleRadius() + B) return false;
	return true;
}
// EOF