// Walk and kick game state: Default ball handling
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/game_states/game_default_ball_handling.h>
#include <walk_and_kick/wak_utils.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// GameDefaultBallHandling class
//

// Constructor
GameDefaultBallHandling::GameDefaultBallHandling(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID) : WAKGameState(config, SV, WGS, ID)
{
	// Initialise the activation of the state
	handleActivation(isActive());
}

// Handle activation function
void GameDefaultBallHandling::handleActivation(bool nowActive)
{
}

// Execute function
void GameDefaultBallHandling::execute(GameVars& GV, const GameVars& lastGV, bool justActivated)
{
	// Separate handling if the robot is localised or not
	if(SV.haveRobotPose)
	{
		// Retrieve the global ball position (or the position of the robot if we have no ball)
		Vec2f ballPose = (SV.haveBallPose ? SV.ballPose : SV.robotPose2D);

		// Calculate the game variable outputs for a ball target based on the localised ball pose
		calcPoseBallTarget(GV, ballPose, SV.goalSign);
	}
	else
	{
		// Retrieve the local ball position (or the position of the robot if we have no ball)
		Vec2f ballDir = (SV.haveBall ? SV.ballDir : Vec2f::Zero());

		// Calculate the game variable outputs for a ball target based on the compass
		calcCompassBallTarget(GV, ballDir, SV.goalSign);
	}

	// Advise against kicking if a direct kick into goals is not allowed yet
	if(!SV.directGoalAllowed)
		GV.kickIfPossible = false;

	// Apply obstacle ball handling
	WGS.obstacleBallHandling(GV);
}

// Calculate a ball target specification based on a local ball position
void GameDefaultBallHandling::calcCompassBallTarget(GameVars& GV, const Vec2f& ballDir, int goalSign) const
{
	// Calculate the angle and vector relative to the robot of the direction of the positive goal (i.e. the direction of the positive global x axis)
	float northAngle = -SV.compassHeading;
	Vec2f northUnitVec(cos(northAngle), sin(northAngle));

	// Calculate a compass ball target relative to the robot
	Vec2f ballTargetDir = ballDir + (goalSign * config.minBallToTargetDist()) * northUnitVec;

	// Set the game output variables
	GV.forceBehStateByID = WAKBehManager::BS_UNKNOWN;
	GV.suggestFoot = GameVars::FS_EITHER_FOOT;
	GV.dribbleIfPossible = true;
	GV.kickIfPossible = true;
	GV.diveIfPossible = DD_NONE;
	GV.ballTargetConf = 1.0f;
	GV.ballTargetDir = ballTargetDir;
	GV.ballTargetWedge = config.minBallTargetWedge();
	GV.ballTargetType = GameVars::BTT_COMPASS;
	GV.targetPose.setZero();
	GV.targetPoseTol = -1.0f;
	GV.targetPoseValid = false;
}

// Calculate a ball target specification based on a ball pose (global position of the ball on the field) and direction of play
void GameDefaultBallHandling::calcPoseBallTarget(GameVars& GV, const Vec2f& ballPose, int goalSign) const
{
	// Constants
	const float eps = 1e-6f;
	const float bigEps = 1e-3f;

	// Retrieve the required field dimensions
	float Lh = field.fieldLengthH();
	float Gh = field.goalWidthH();

	// Be totally sure about the validity of the goal sign (+1 = Positive yellow goal, -1 = Negative blue goal)
	goalSign = (goalSign >= 0 ? +1 : -1);

	// Convert the ball pose and goal sign into an equivalent ball pose that is scoring into the positive goal (the variable ballPose should never be used beyond here!)
	Vec2f ballPoseGS = goalSign * ballPose;

	// See whether the ball is out of bounds in the end zone (centre past the goal line)
	bool ballInEndZone = (ballPoseGS.x() > Lh);

	// Work out the poses of the goal posts
	Vec2f postL(Lh, Gh);
	Vec2f postR(Lh, -Gh);

	// Decide on appropriate goal post/net radii
	float postRadius = config.dbhGoalPostRadius();
	float postNetRadius = postRadius + config.dbhGoalPostRadiusExtra();

	// Calculate the vectors and distances from the ball to the goal posts
	Vec2f ballToPostLVec = postL - ballPoseGS;
	Vec2f ballToPostRVec = postR - ballPoseGS;
	float ballToPostLDist = ballToPostLVec.norm();
	float ballToPostRDist = ballToPostRVec.norm();

	// Calculate the halo radii of the goal posts
	float postLRadius = std::min(postRadius, ballToPostLDist);
	float postRRadius = std::min(postRadius, ballToPostRDist);

	// Calculate the tangents from the ball to the goal post halos
	WAKUtils::TangentInfo LT, RT;
	WAKUtils::calcCircleTangent(LT, ballPoseGS, postL, postLRadius);
	WAKUtils::calcCircleTangent(RT, ballPoseGS, postR, postRRadius);

	// Relax the maximum angle adjustments if the ball is significantly far from the corresponding goal post
	float largeTangentLen = std::max(Gh - postRadius, 0.0f);
	if(LT.length > largeTangentLen)
		LT.maxAdjust = M_PI + eps;
	if(RT.length > largeTangentLen)
		RT.maxAdjust = M_PI + eps;

	// Adjust the wrapping of the relevant tangent angles to resolve complications just outside the goal nets
	if(ballInEndZone)
	{
		if(ballPoseGS.y() > Gh && LT.negAngle > 0.0f)
			LT.negAngle -= M_2PI;
		if(ballPoseGS.y() < -Gh && RT.posAngle < 0.0f)
			RT.posAngle += M_2PI;
	}

	// Decide whether we need to force the sign of the goal post tangent wedge
	int forceTangentWedgeSign = 0;
	if(ballInEndZone)
		forceTangentWedgeSign = (fabs(ballPoseGS.y()) > Gh ? -1 : 1);

	// Calculate the goal post tangent wedge
	float tangentWedge = WAKUtils::calcWedge(RT.posAngle, LT.negAngle, forceTangentWedgeSign);

	// Retrieve the configured corner adjustment angle
	float cornerAdjustAngle = config.dbhCornerAdjustmentAngle();

	// Calculate the required tangent angle corner adjustment values
	float postLNegAdjust = interpolateCoerced<float>(config.dbhCornerTangentAngleLow(), config.dbhCornerTangentAngleHigh(), cornerAdjustAngle, 0.0f, M_PI_2 + LT.negAngle);
	float postRPosAdjust = interpolateCoerced<float>(config.dbhCornerTangentAngleLow(), config.dbhCornerTangentAngleHigh(), cornerAdjustAngle, 0.0f, M_PI_2 - RT.posAngle);

	// Calculate suitable limits for the tangent angle corner adjustment values
	float tangentWedgeHEps = std::max(0.5f*tangentWedge - eps, 0.0f);
	float postLAdjustLimit = std::min(LT.maxAdjust, cornerAdjustAngle);
	float postRAdjustLimit = std::min(RT.maxAdjust, cornerAdjustAngle);
	if(tangentWedgeHEps < postLAdjustLimit)
		postLAdjustLimit = interpolateCoerced(config.dbhCornerTangentWedgeLow(), config.kickAccuracyWedge(), postLAdjustLimit, tangentWedgeHEps, tangentWedge);
	if(tangentWedgeHEps < postRAdjustLimit)
		postRAdjustLimit = interpolateCoerced(config.dbhCornerTangentWedgeLow(), config.kickAccuracyWedge(), postRAdjustLimit, tangentWedgeHEps, tangentWedge);

	// Limit the tangent angle corner adjustment values
	postLNegAdjust = std::min(postLNegAdjust, postLAdjustLimit); // The resulting value is assumed to be non-negative
	postRPosAdjust = std::min(postRPosAdjust, postRAdjustLimit); // The resulting value is assumed to be non-negative

	// Wrap the tangent angles to an interval range that is consistent with the calculated tangent wedge
	float postRPosAngle = RT.posAngle;
	float postLNegAngle = postRPosAngle + tangentWedge;

	// Calculate various possible target angles to choose from
	float targetAngleL = postLNegAngle - postLNegAdjust;
	float targetAngleR = postRPosAngle + postRPosAdjust;
	float targetAngleM = 0.5f*(postLNegAngle + postRPosAngle);

	// Decide on a ball target angle
	float targetAngle = targetAngleM;
	if(targetAngleL >= targetAngleR)
	{
		if(targetAngleM > targetAngleL)
			targetAngle = targetAngleL;
		else if(targetAngleR > targetAngleM)
			targetAngle = targetAngleR;
		else
			targetAngle = targetAngleM;
	}
	else if(ballPoseGS.y() >= 0.0f)
		targetAngle = targetAngleL;
	else
		targetAngle = targetAngleR;

	// Adjust the target angle to avoid possible collisions of the robot with the net
	if(ballPoseGS.x() > Lh - postRadius && sin(targetAngle)*ballPoseGS.y() > 0.0f)
	{
		if(tangentWedge > 0.0f)
			targetAngle -= picut(targetAngle); // Note: This is consistent in terms of wrapping with the previous targetAngle!
		else
			targetAngle -= picut(targetAngle + M_PI); // Note: This is consistent in terms of wrapping with the previous targetAngle!
	}

	// Calculate the output ball target unit vector in global coordinates
	Vec2f targetUnitVec(cos(targetAngle), sin(targetAngle));

	// Calculate a target distance
	float targetDist = config.kickMaxDist();
	if(targetUnitVec.x() > 0.0f)
	{
		targetDist = (Lh + field.ballRadius() - ballPoseGS.x()) / targetUnitVec.x();
		float targetY = ballPoseGS.y() + targetDist*targetUnitVec.y();
		if(fabs(targetY) > Gh)
			targetDist = coerceMax(targetDist, config.kickMaxDist());
	}
	targetDist = coerceMin(targetDist, config.minBallToTargetDist());

	// Calculate the distance of the ball from the goal posts/nets
	float distToNetAndPostL = ballToPostLDist;
	float distToNetAndPostR = ballToPostRDist;
	if(ballInEndZone)
	{
		distToNetAndPostL = fabs(ballPoseGS.y() - postL.y());
		distToNetAndPostR = fabs(ballPoseGS.y() - postR.y());
	}
	float distToNetAndPost = std::min(distToNetAndPostL, distToNetAndPostR);

	// Calculate a target wedge
	float targetWedgeL = 2.0f*(postLNegAngle - targetAngle);
	float targetWedgeR = 2.0f*(targetAngle - postRPosAngle);
	float targetWedge = coerce(std::max(targetWedgeL, targetWedgeR), 0.0f, config.maxBallTargetWedge());
	targetWedge = interpolateCoerced(postRadius, postNetRadius, config.dbhNearPostTargetWedge(), targetWedge, distToNetAndPost);

	// Decide whether the robot should kick if possible
	bool kickIfPossible = false;
	if(tangentWedge >= 0.0f && fabs(targetAngle - targetAngleM) < bigEps &&
	  (targetDist >= config.kickMaxDist() || (tangentWedge >= config.kickAccuracyWedge() && targetDist <= config.kickMinDist()) || config.dbhDbZoneDisableAnnulusZone()))
		kickIfPossible = true;
	if(ballPoseGS.norm() <= config.dbhDbZoneCentreRadius() && !config.dbhDbZoneDisableCentreZone())
		kickIfPossible = false;
	if(ballPoseGS.x() >= Lh - config.dbhDbZoneNearOppGoalDepth() && !config.dbhDbZoneDisableGoalZones())
		kickIfPossible = false;
	if(ballPoseGS.x() <= -Lh + config.dbhDbZoneNearOwnGoalDepth() && !config.dbhDbZoneDisableGoalZones())
		kickIfPossible = false;

	// Decide whether a particular foot should be forced due to proximity to the goals/nets/corner (Note: This must come after deciding whether kicking is possible!)
	GameVars::FootSelection suggestFoot = GameVars::FS_EITHER_FOOT;
	bool nearLeftPost = (distToNetAndPostL <= postNetRadius);
	bool nearRightPost = (distToNetAndPostR <= postNetRadius);
	bool allowCornerFootSel = (!kickIfPossible && !config.dbhCornerFootSelDisable());
	if(nearLeftPost && !nearRightPost)                                     // Ball near left goal post
		suggestFoot = GameVars::FS_LEFT_FOOT;
	else if(nearRightPost && !nearLeftPost)                                // Ball near right goal post
		suggestFoot = GameVars::FS_RIGHT_FOOT;
	else if(allowCornerFootSel && ballInCorner(ballPoseGS, postL.y(), +1)) // Dribbling ball in left corner
		suggestFoot = GameVars::FS_RIGHT_FOOT;
	else if(allowCornerFootSel && ballInCorner(ballPoseGS, postR.y(), -1)) // Dribbling ball in right corner
		suggestFoot = GameVars::FS_LEFT_FOOT;

	// Calculate the output ball target pose in global coordinates
	Vec2f ballTargetPoseGS = ballPoseGS + targetDist*targetUnitVec;

	// Convert the ball target pose back into the coordinates of the original problem (i.e. consistent with ballPose)
	Vec2f ballTargetPose = goalSign * ballTargetPoseGS;

	// Convert the ball target pose to a ball target position relative to the robot
	Vec2f ballTargetDir = ballTargetPose - SV.robotPose2D;
	eigenRotateCCW(ballTargetDir, -SV.robotPose.z());

	// Set the game output variables
	GV.forceBehStateByID = WAKBehManager::BS_UNKNOWN;
	GV.suggestFoot = suggestFoot;
	GV.dribbleIfPossible = true;
	GV.kickIfPossible = kickIfPossible;
	GV.diveIfPossible = DD_NONE;
	GV.ballTargetConf = SV.robotPoseConf;
	GV.ballTargetDir = ballTargetDir;
	GV.ballTargetWedge = targetWedge;
	GV.ballTargetType = GameVars::BTT_POSE;
	GV.targetPose.setZero();
	GV.targetPoseTol = -1.0f;
	GV.targetPoseValid = false;
}

// Return whether the ball is in a particular corner (for the purposes of forced foot selection)
bool GameDefaultBallHandling::ballInCorner(const Vec2f& ballPoseGS, float postY, int cornerSign) const
{
	// Retrieve the required field dimensions
	float Lh = field.fieldLengthH();
	float Wh = field.fieldWidthH();

	// Calculate the corner points
	float x1 = Lh - config.dbhCornerFootSelInsideX();
	float x2 = Lh - config.dbhCornerFootSelOutsideX();
	float y1 = postY + cornerSign*config.dbhCornerFootSelPostOffY();
	float y2 = cornerSign*Wh;

	// Return whether the ball is in the corner
	float yDiff = y2 - y1;
	return (cornerSign*(ballPoseGS.y() - y1) >= 0.0f && sign(yDiff)*(yDiff*(ballPoseGS.x() - x1) - (ballPoseGS.y() - y1)*(x2 - x1)) >= 0.0f);
}
// EOF