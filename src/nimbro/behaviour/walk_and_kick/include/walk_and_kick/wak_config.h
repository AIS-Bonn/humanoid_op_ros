// Walk and kick: Config server parameters
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_CONFIG_H
#define WAK_CONFIG_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <config_server/parameter.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class WAKConfig
	* 
	* @brief Configuration struct for the walk and kick node.
	**/
	class WAKConfig
	{
	public:
		//! Constructor
		WAKConfig()
		 : CONFIG_PARAM_PATH("/walk_and_kick/")
		 , CONFIG_SETTINGS_PATH("/settings/")

		 , sIsPenaltyShoot(CONFIG_SETTINGS_PATH + "general/isPenaltyShoot", false)
		 , sKickoffType(CONFIG_SETTINGS_PATH + "general/kickoffType", KT_UNKNOWN + 1, 1, KT_COUNT - 1, KT_DEFAULT)
		 , sListenToGC(CONFIG_SETTINGS_PATH + "general/listenToGC", false)
		 , sListenToTC(CONFIG_SETTINGS_PATH + "general/listenToTC", false)
		 , sUseAutoPositioning(CONFIG_SETTINGS_PATH + "general/useAutoPositioning", true)
		 , sPlayAsCyan(CONFIG_SETTINGS_PATH + "general/playAsCyan", false)
		 , sPlayOnYellow(CONFIG_SETTINGS_PATH + "general/playOnYellow", true)
		 , sWaitTime(CONFIG_SETTINGS_PATH + "general/waitTime", 0, 5, 20, 15)
		 , sEnableDribble(CONFIG_SETTINGS_PATH + "robot/enableDribble", true)
		 , sEnableKick(CONFIG_SETTINGS_PATH + "robot/enableKick", true)
		 , sEnableObstacles(CONFIG_SETTINGS_PATH + "robot/enableObstacles", true)
		 , sForceKickFoot(CONFIG_SETTINGS_PATH + "robot/forceKickFoot", -1, 1, 1, 0)
		 , sGameRole(CONFIG_SETTINGS_PATH + "robot/gameRole", ROLE_UNKNOWN + 1, 1, ROLE_COUNT - 1, ROLE_DEFAULT)

		 , robotNumber("/game_controller/robotNumber", 0, 1, 5, 0)

		 , enableWAK(CONFIG_PARAM_PATH + "enableWAK", true)
		 , pauseWAK(CONFIG_PARAM_PATH + "pauseWAK", false)
		 , plotData(CONFIG_PARAM_PATH + "plotData", false)
		 , publishTF(CONFIG_PARAM_PATH + "publishTF", false)
		 , publishVis(CONFIG_PARAM_PATH + "publishVis", false)
		 , simpleModeButton(CONFIG_PARAM_PATH + "general/simpleModeButton", true)
		 , globalGcvBwdLimit(CONFIG_PARAM_PATH + "general/globalGcvBwdLimit", 0.0, 0.01, 1.0, 1.0)
		 , globalGcvSpeedLimit(CONFIG_PARAM_PATH + "general/globalGcvSpeedLimit", 0.0, 0.01, 1.0, 1.0)
		 , globalGcvShearXY(CONFIG_PARAM_PATH + "general/globalGcvShearXY", 0.5, 0.01, 2.0, 1.0)
		 , forceKickIfChoice(CONFIG_PARAM_PATH + "general/forceKickIfChoice", false)
		 , forceDribbleIfChoice(CONFIG_PARAM_PATH + "general/forceDribbleIfChoice", false)
		 , reqBallOffsetX(CONFIG_PARAM_PATH + "general/reqBallOffset/x", 0.0, 0.01, 0.5, 0.3)
		 , reqBallOffsetXDribble(CONFIG_PARAM_PATH + "general/reqBallOffset/xDribble", 0.0, 0.01, 0.5, 0.3)
		 , reqBallOffsetYDribbleMag(CONFIG_PARAM_PATH + "general/reqBallOffset/yDribbleMag", 0.01, 0.01, 0.4, 0.08)
		 , reqBallOffsetYLeft(CONFIG_PARAM_PATH + "general/reqBallOffset/yLeft", -0.1, 0.01, 0.4, 0.12)
		 , reqBallOffsetYRight(CONFIG_PARAM_PATH + "general/reqBallOffset/yRight", -0.4, 0.01, 0.1, -0.12)
		 , maxBallTargetWedge(CONFIG_PARAM_PATH + "general/ballTarget/maxBallTargetWedge", 0.6, 0.01, 1.5, 1.1)
		 , minBallTargetWedge(CONFIG_PARAM_PATH + "general/ballTarget/minBallTargetWedge", 0.0, 0.01, 0.5, 0.3)
		 , minBallTargetWedgeExtra(CONFIG_PARAM_PATH + "general/ballTarget/minBallTargetWedgeExtra", 0.0, 0.01, 0.3, 0.1)
		 , minBallToTargetDist(CONFIG_PARAM_PATH + "general/ballTarget/minBallToTargetDist", 0.02, 0.02, 2.0, 0.6)
		 , kickAccuracyWedge(CONFIG_PARAM_PATH + "general/kick/accuracyWedge", 0.1, 0.01, 0.8, 0.4)
		 , kickMaxDist(CONFIG_PARAM_PATH + "general/kick/maxDist", 3.0, 0.05, 6.0, 4.5)
		 , kickMinDist(CONFIG_PARAM_PATH + "general/kick/minDist", 2.0, 0.05, 5.0, 3.5)
		 , kickAngleDriftLeftKick(CONFIG_PARAM_PATH + "general/kick/angleDriftLeftKick", -0.5, 0.01, 0.5, 0.0)
		 , kickAngleDriftRightKick(CONFIG_PARAM_PATH + "general/kick/angleDriftRightKick", -0.5, 0.01, 0.5, 0.0)
		 , dribbleAccuracyWedge(CONFIG_PARAM_PATH + "general/dribble/accuracyWedge", 0.1, 0.01, 0.5, 0.2)
		 , targetPoseReevalTime(CONFIG_PARAM_PATH + "general/targetPoseReevalTime", 0.1, 0.1, 5.0, 2.0)

		 , ballStableMaxDist(CONFIG_PARAM_PATH + "sensors/ballStableMaxDist", 1.0, 0.1, 10.0, 4.0)
		 , ballStableTime(CONFIG_PARAM_PATH + "sensors/ballStableTime", 1.0, 0.5, 15.0, 5.0)
		 , maxBallDist(CONFIG_PARAM_PATH + "sensors/maxBallDist", 3.0, 0.1, 13.0, 10.0)
		 , confDecayTime(CONFIG_PARAM_PATH + "sensors/confDecay/timeToDecay", 0.1, 0.05, 5.0, 1.0)
		 , confDecayHalfLifeBall(CONFIG_PARAM_PATH + "sensors/confDecay/halfLifeBall", 0.1, 0.05, 5.0, 1.0)
		 , confDecayHalfLifeGoal(CONFIG_PARAM_PATH + "sensors/confDecay/halfLifeGoal", 0.1, 0.05, 5.0, 1.0)
		 , confDecayHalfLifePose(CONFIG_PARAM_PATH + "sensors/confDecay/halfLifeRobotPose", 0.1, 0.05, 5.0, 1.0)
		 , confDecayHalfLifeObst(CONFIG_PARAM_PATH + "sensors/confDecay/halfLifeObstacle", 0.1, 0.05, 5.0, 1.0)
		 , confLimitBall(CONFIG_PARAM_PATH + "sensors/confLimit/ball", 0.0, 0.01, 1.0, 0.2)
		 , confLimitBallTarget(CONFIG_PARAM_PATH + "sensors/confLimit/ballTarget", 0.0, 0.01, 1.0, 0.2)
		 , confLimitRobotPose(CONFIG_PARAM_PATH + "sensors/confLimit/robotPose", 0.0, 0.01, 1.0, 0.2)
		 , ballHasMovedDisable(CONFIG_PARAM_PATH + "sensors/ballHasMoved/disable", false)
		 , ballHasMovedMaxRobotDist(CONFIG_PARAM_PATH + "sensors/ballHasMoved/maxRobotDist", 0.5, 0.05, 4.0, 2.0)
		 , ballHasMovedCentreRadius(CONFIG_PARAM_PATH + "sensors/ballHasMoved/centreRadius", 0.1, 0.01, 1.0, 0.4)
		 , ballHasMovedWormTime(CONFIG_PARAM_PATH + "sensors/ballHasMoved/wormTime", 0.1, 0.02, 4.0, 1.5)
		 , locHintTimeout(CONFIG_PARAM_PATH + "sensors/localisation/locHintTimeout", 1.0, 0.2, 20.0, 5.0)
		 , locSetToGoalsEnable(CONFIG_PARAM_PATH + "sensors/localisation/setToGoalsEnable", true)
		 , locSetToGoalsHeadingTol(CONFIG_PARAM_PATH + "sensors/localisation/setToGoalsHeadingTol", 0.0, 0.01, 0.5, 0.3)
		 , locSetToSideEnable(CONFIG_PARAM_PATH + "sensors/localisation/setToSideEnable", true)
		 , locSetToSideHeadingTol(CONFIG_PARAM_PATH + "sensors/localisation/setToSideHeadingTol", 0.0, 0.01, 0.5, 0.2)
		 , locSetToSideOffsetX(CONFIG_PARAM_PATH + "sensors/localisation/setToSideOffsetX", 0.0, 0.01, 1.0, 0.4)
		 , waitTimeReduction(CONFIG_PARAM_PATH + "sensors/waitTimeReduction", 0.0, 0.1, 8.0, 0.0)
		 , timeoutBallInPlay(CONFIG_PARAM_PATH + "sensors/timeoutBallInPlay", 1.0, 0.2, 20.0, 10.0)
		 , timeoutDirectGoal(CONFIG_PARAM_PATH + "sensors/timeoutDirectGoal", 1.0, 0.5, 50.0, 20.0)

		 , gcEnable(CONFIG_PARAM_PATH + "gameController/enableGameController", false)
		 , gcSmoothingTime(CONFIG_PARAM_PATH + "gameController/smoothingTime", 0.0, 0.02, 2.0, 0.75)
		 , gcFreshTime(CONFIG_PARAM_PATH + "gameController/gcFreshTime", 0.1, 0.1, 10.0, 5.0)
		 , gcIgnoreSetIfIllegalPose(CONFIG_PARAM_PATH + "gameController/ignoreSetIfIllegalPose", true)
		 , gcTimeoutKickoffType(CONFIG_PARAM_PATH + "gameController/timeoutKickoffType", 1.0, 0.5, 20.0, 12.0)
		 , gcTimeoutReadyFirst(CONFIG_PARAM_PATH + "gameController/timeoutReadyFirst", 5.0, 1.0, 50.0, 30.0)
		 , gcTimeoutSetFirstNormal(CONFIG_PARAM_PATH + "gameController/timeoutSetFirstNormal", 1.0, 0.5, 20.0, 10.0)
		 , gcTimeoutSetFirstPenalty(CONFIG_PARAM_PATH + "gameController/timeoutSetFirstPenalty", 1.0, 0.5, 20.0, 10.0)
		 , gcTimeoutSetLast(CONFIG_PARAM_PATH + "gameController/timeoutSetLast", 1.0, 0.5, 20.0, 5.0)
		 , gcTimeoutTimeoutLast(CONFIG_PARAM_PATH + "gameController/timeoutTimeoutLast", 1.0, 0.5, 20.0, 5.0)

		 , tcEnable(CONFIG_PARAM_PATH + "teamComms/enableTeamComms", false)
		 , tcFreshTime(CONFIG_PARAM_PATH + "teamComms/tcFreshTime", 0.1, 0.1, 10.0, 5.0)
		 
		 , gazeYawAbsMax(CONFIG_PARAM_PATH + "gazeLimits/yawAbsMax", 0.0, 0.05, 3.0, 1.0)
		 , gazePitchMin(CONFIG_PARAM_PATH + "gazeLimits/pitchMin", -0.5, 0.01, 1.0, 0.0)
		 , gazePitchMax(CONFIG_PARAM_PATH + "gazeLimits/pitchMax", -0.5, 0.01, 1.0, 1.0)
		 , gazePitchNeutral(CONFIG_PARAM_PATH + "gazeLimits/pitchNeutral", -0.5, 0.01, 1.0, 0.0)
		 , gazeVelLimit(CONFIG_PARAM_PATH + "gazeLimits/velLimit", 0.5, 0.05, 5.0, 2.5)
 
		 , obhEnableObstBallHandling(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/enableObstacleBallHandling", true)
		 , obhObstBeforeBallBuf(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/obstBeforeBallBuf", 0.0, 0.01, 1.0, 0.5)
		 , obhObstBeyondTargetBuf(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/obstBeyondTargetBuf", 0.0, 0.01, 1.0, 0.6)
		 , obhObstClearanceLow(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/obstClearanceLow", 0.0, 0.01, 0.8, 0.2)
		 , obhObstClearanceHigh(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/obstClearanceHigh", 0.0, 0.01, 0.8, 0.4)
		 , obhClearanceAngleLowMax(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/clearanceAngleLowMax", 0.0, 0.01, 1.0, 0.3)
		 , obhClearanceAngleHighMax(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/clearanceAngleHighMax", 0.5, 0.01, 1.5, 1.2)
		 , obhAngleAdjustForFootSel(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/angleAdjustForFootSel", 0.0, 0.01, 1.0, 0.3)
		 , obhAngleAdjustWedgeRatio(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/angleAdjustWedgeRatio", 0.0, 0.01, 1.0, 0.6)
		 , obhFootSelObstBallDistMax(CONFIG_PARAM_PATH + "gameFSM/obstacleBallHandling/footSelObstBallDistMax", 0.0, 0.02, 2.0, 1.0)

		 , posCommonTargetEnable(CONFIG_PARAM_PATH + "gameFSM/positioning/commonTarget/enable", false)
		 , posCommonTargetX(CONFIG_PARAM_PATH + "gameFSM/positioning/commonTarget/targetX", -3, 1, 3, 0)
		 , posCommonTargetY(CONFIG_PARAM_PATH + "gameFSM/positioning/commonTarget/targetY", -2, 1, 2, 0)
		 , posCommonTargetRot(CONFIG_PARAM_PATH + "gameFSM/positioning/commonTarget/targetRot", -8, 1, 8, 0)
		 , posPoseAttackingX(CONFIG_PARAM_PATH + "gameFSM/positioning/poseAttackingX", -1.0, 0.02, 1.0, -0.4)
		 , posPoseAttackingY(CONFIG_PARAM_PATH + "gameFSM/positioning/poseAttackingY", -1.0, 0.02, 1.0, 0.0)
		 , posPoseAttackingT(CONFIG_PARAM_PATH + "gameFSM/positioning/poseAttackingT", -M_PI, 0.05, M_PI, 0.0)
		 , posPoseDefendingX(CONFIG_PARAM_PATH + "gameFSM/positioning/poseDefendingX", -1.0, 0.02, 1.0, -0.2)
		 , posPoseDefendingY(CONFIG_PARAM_PATH + "gameFSM/positioning/poseDefendingY", -1.0, 0.02, 1.0, 0.0)
		 , posPoseDefendingT(CONFIG_PARAM_PATH + "gameFSM/positioning/poseDefendingT", -M_PI, 0.05, M_PI, 0.0)
		 , posPoseGoalieX(CONFIG_PARAM_PATH + "gameFSM/positioning/poseGoalieX", -1.0, 0.02, 1.0, 0.2)
		 , posPoseGoalieY(CONFIG_PARAM_PATH + "gameFSM/positioning/poseGoalieY", -1.0, 0.02, 1.0, 0.0)
		 , posPoseGoalieT(CONFIG_PARAM_PATH + "gameFSM/positioning/poseGoalieT", -M_PI, 0.05, M_PI, 0.0)
		 , posArrivedCostMax(CONFIG_PARAM_PATH + "gameFSM/positioning/arrivedCostMax", 0.0, 0.02, 2.0, 0.5)
		 , posPoseLegalityBuffer(CONFIG_PARAM_PATH + "gameFSM/positioning/poseLegalityBuffer", 0.0, 0.01, 1.0, 0.2)
		 , posPoseLegalWormTime(CONFIG_PARAM_PATH + "gameFSM/positioning/poseLegalWormTime", 0.1, 0.02, 4.0, 1.5)
		 
		 , dbhGoalPostRadius(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/goalPostRadius", 0.01, 0.01, 0.5, 0.25)
		 , dbhGoalPostRadiusExtra(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/goalPostRadiusExtra", 0.01, 0.01, 1.0, 0.35)
		 , dbhCornerAdjustmentAngle(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/corner/adjustmentAngle", 0.01, 0.01, 1.2, 0.8)
		 , dbhCornerTangentAngleLow(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/corner/tangentAngleLow", 0.0, 0.01, 0.5, 0.2)
		 , dbhCornerTangentAngleHigh(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/corner/tangentAngleHigh", 0.6, 0.01, 1.5, 1.0)
		 , dbhCornerTangentWedgeLow(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/corner/tangentWedgeLow", -0.6, 0.01, 0.0, -0.3)
		 , dbhCornerFootSelDisable(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/corner/footSelDisable", false)
		 , dbhCornerFootSelInsideX(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/corner/footSelInsideX", 0.0, 0.02, 1.5, 0.5)
		 , dbhCornerFootSelOutsideX(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/corner/footSelOutsideX", 0.0, 0.02, 1.5, 1.0)
		 , dbhCornerFootSelPostOffY(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/corner/footSelPostOffY", 0.5, 0.01, 1.5, 0.8)
		 , dbhNearPostTargetWedge(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/nearPostTargetWedge", 0.0, 0.02, 1.5, 0.8)
		 , dbhDbZoneCentreRadius(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/dribbleZone/centreRadius", 0.0, 0.01, 1.5, 0.4)
		 , dbhDbZoneNearOppGoalDepth(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/dribbleZone/nearOppGoalDepth", 0.0, 0.01, 1.0, 0.7)
		 , dbhDbZoneNearOwnGoalDepth(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/dribbleZone/nearOwnGoalDepth", 0.0, 0.01, 2.0, 0.8)
		 , dbhDbZoneDisableAnnulusZone(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/dribbleZone/disableAnnulusZone", false)
		 , dbhDbZoneDisableCentreZone(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/dribbleZone/disableCentreZone", false)
		 , dbhDbZoneDisableGoalZones(CONFIG_PARAM_PATH + "gameFSM/defaultBallHandling/dribbleZone/disableGoalZones", false)
		 
		 , dgChaseBallIfClose(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/chaseBallIfClose", true)
		 , dgArrivedCostMax(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/arrivedCostMax", 0.0, 0.02, 2.0, 0.5)
		 , dgScoringDistLow(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/scoringDistLow", 0.5, 0.05, 5.0, 2.0)
		 , dgScoringDistHigh(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/scoringDistHigh", 0.5, 0.05, 5.0, 3.5)
		 , dgScoringDistChaseBall(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/scoringDistChaseBall", 0.5, 0.05, 5.0, 2.2)
		 , dgBallDistChaseBall(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/ballDistChaseBall", 0.1, 0.05, 5.0, 1.8)
		 , dgTargetRadiusLow(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/targetRadiusLow", 0.0, 0.01, 1.0, 0.2)
		 , dgTargetRadiusHigh(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/targetRadiusHigh", 0.0, 0.02, 2.0, 1.0)
		 , dgLookForBallRadius(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/lookForBallRadius", 0.0, 0.02, 2.0, 0.6)
		 , dgDistBufFromPosts(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/distBufFromPosts", 0.0, 0.02, 2.0, 0.6)
		 , dgTargetTimeAgoTimeout(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/targetTimeAgoTimeout", 5.0, 0.5, 60.0, 30.0)
		 , dgChaseBallWormTime(CONFIG_PARAM_PATH + "gameFSM/defaultGoalie/chaseBallWormTime", 1.0, 0.1, 10.0, 3.0)
		 
		 , gazeBallDistNear(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/distNear", 0.0, 0.05, 2.0, 0.5)
		 , gazeBallDistFar(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/distFar", 1.0, 0.05, 4.0, 1.5)
		 , gazeBallDistHorizon(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/distHorizon", 1.0, 0.05, 6.0, 3.0)
		 , gazeBallPitchFar(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/pitchFar", -0.5, 0.01, 1.0, 0.5)
		 , gazeBallPitchHorizon(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/pitchHorizon", -0.5, 0.01, 1.0, 0.0)
		 , gazeBallTsNear(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/TsNear", 0.1, 0.05, 5.0, 1.5)
		 , gazeBallTsFar(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/TsFar", 0.1, 0.05, 5.0, 0.5)
		 , gazeBallTsMin(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/TsMin", 0.1, 0.05, 5.0, 0.1)
		 , gazeBallTsMax(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/TsMax", 0.1, 0.05, 5.0, 3.0)
		 , gazeBallBoxXMin(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/boxXMin", -0.1, 0.01, 0.5, 0.0)
		 , gazeBallBoxXMax(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/boxXMax", -0.1, 0.01, 0.5, 0.1)
		 , gazeBallBoxYMin(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/boxYMin", -0.4, 0.01, 0.1, -0.1)
		 , gazeBallBoxYMax(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/boxYMax", -0.1, 0.01, 0.4, 0.1)
		 , gazeBallYawDeadband(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/yawDeadband", 0.0, 0.05, 3.0, 1.0)
		 , gazeBallYawFreeLook(CONFIG_PARAM_PATH + "behFSM/gazeAtBall/yawFreeLook", 0.0, 0.05, 3.0, 2.0)
		 
		 , oaEnableObstacleAvoidance(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/enableObstacleAvoidance", true)
		 , oaAngleAtObstacleHigh(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/angleAtObstacleHigh", 0.2, 0.01, 1.4, 0.8)
		 , oaMaxRadialGcvLow(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/maxRadialGcvLow", -1.0, 0.01, 0.0, -1.0)
		 , oaMaxRadialGcvHigh(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/maxRadialGcvHigh", 0.0, 0.01, 1.0, 1.0)
		 , oaObstacleRadiusLow(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/radiusLow", 0.0, 0.01, 0.5, 0.0)
		 , oaObstacleRadiusMid(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/radiusMid", 0.2, 0.01, 1.0, 0.5)
		 , oaObstacleRadiusHigh(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/radiusHigh", 0.2, 0.01, 1.0, 0.7)
		 , oaAdjustAngleXYHigh(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/adjustAngleXYHigh", 0.1, 0.02, 1.5, 1.0)
		 , oaAdjustGcvZHigh(CONFIG_PARAM_PATH + "behFSM/obstacleAvoidance/adjustGcvZHigh", 0.0, 0.01, 1.0, 0.7)
		 
		 , wtgpAngleErrorCost(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/angleErrorCost", 0.0, 0.05, 5.0, 2.0)
		 , wtgpAngleLimitNear(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/angleLimitNear", 0.0, 0.01, 1.5, 0.5)
		 , wtgpAngleLimitFar(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/angleLimitFar", 0.0, 0.01, 1.5, 0.8)
		 , wtgpDistNear(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/distNear", 0.1, 0.01, 1.0, 0.3)
		 , wtgpDistFar(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/distFar", 0.5, 0.02, 3.0, 0.8)
		 , wtgpSpeedNearXY(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/speedNearXY", 0.0, 0.01, 1.0, 0.7)
		 , wtgpSpeedNearZ(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/speedNearZ", 0.0, 0.01, 1.0, 0.7)
		 , wtgpSpeedFarX(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/speedFarX", 0.0, 0.01, 1.0, 0.7)
		 , wtgpSpeedFarZ(CONFIG_PARAM_PATH + "behFSM/walkToGlobalPose/speedFarZ", 0.0, 0.01, 1.0, 0.7)
		 
		 , wtpArrivedCostMaxDefault(CONFIG_PARAM_PATH + "behFSM/walkToPose/arrivedCostMaxDefault", 0.0, 0.02, 2.0, 0.5)
		 , wtpArrivedWormTime(CONFIG_PARAM_PATH + "behFSM/walkToPose/arrivedWormTime", 0.1, 0.1, 10.0, 3.0)
		 
		 , paTimeout(CONFIG_PARAM_PATH + "behFSM/panicAttack/timeout", 0.0, 0.5, 30.0, 10.0)
		 
		 , llrGazeFreqScaler(CONFIG_PARAM_PATH + "behFSM/lookLeftRight/gazeFreqScaler", 0.1, 0.02, 2.0, 0.5)
		 , llrGazePitch(CONFIG_PARAM_PATH + "behFSM/lookLeftRight/gazePitch", -0.5, 0.01, 1.0, 0.0)
		 
		 , laGazeDownFirstFactor(CONFIG_PARAM_PATH + "behFSM/lookAround/gazeDownFirstFactor", 0.0, 0.01, 1.0, 0.5)
		 , laGazeMagInitial(CONFIG_PARAM_PATH + "behFSM/lookAround/magInitial", 0.0, 0.02, 2.0, 0.3)
		 , laGazeMagInc(CONFIG_PARAM_PATH + "behFSM/lookAround/magInc", 0.0, 0.02, 2.0, 0.3)
		 , laGazePitchLookUp(CONFIG_PARAM_PATH + "behFSM/lookAround/pitchLookUp", -0.5, 0.01, 1.0, 0.0)
		 , laGazePitchLookDown(CONFIG_PARAM_PATH + "behFSM/lookAround/pitchLookDown", -0.5, 0.01, 1.0, 0.5)
		 , laGazeSplineAccMax(CONFIG_PARAM_PATH + "behFSM/lookAround/splineAccMax", 1.0, 0.1, 10.0, 4.0)
		 , laGazeSplineVelMax(CONFIG_PARAM_PATH + "behFSM/lookAround/splineVelMax", 0.2, 0.02, 3.0, 1.0)
		
		 , sfbTCBallPoseEnable(CONFIG_PARAM_PATH + "behFSM/searchForBall/tcBallPose/enable", true)
		 , sfbTCBallPoseDebounce(CONFIG_PARAM_PATH + "behFSM/searchForBall/tcBallPose/debounce", 0.0, 0.05, 5.0, 1.5)
		 , sfbStateRequestTimeout(CONFIG_PARAM_PATH + "behFSM/searchForBall/stateRequestTimeout", 1.0, 0.2, 20.0, 5.0)
		 , sfbResumeCycleTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/resumeCycleTime", 0.1, 0.1, 10.0, 5.0)
		 , sfbStayCoolTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/stayCoolTime", 0.1, 0.1, 10.0, 5.0)
		 , sfbBackupEnabled(CONFIG_PARAM_PATH + "behFSM/searchForBall/backup/enabled", true)
		 , sfbBackupMargin(CONFIG_PARAM_PATH + "behFSM/searchForBall/backup/margin", 0.2, 0.02, 2.0, 0.6)
		 , sfbBackupWalkGcvX(CONFIG_PARAM_PATH + "behFSM/searchForBall/backup/walkGcvX", -1.0, 0.01, 0.0, -0.5)
		 , sfbBackupWalkTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/backup/walkTime", 0.1, 0.1, 10.0, 3.0)
		 , sfbBackupWaitTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/backup/waitTime", 0.1, 0.1, 10.0, 1.5)
		 , sfbLBPFreshTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/ballPoseFreshTime", 0.5, 0.5, 10.0, 3.0)
		 , sfbLBPRadiusNear(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/radiusNear", 0.0, 0.05, 2.5, 1.5)
		 , sfbLBPRadiusFar(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/radiusFar", 3.0, 0.05, 8.0, 6.0)
		 , sfbLBPMinBallPoseDurNear(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/minBallPoseDurNear", 0.0, 0.1, 5.0, 2.0)
		 , sfbLBPMinBallPoseDurFar(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/minBallPoseDurFar", 0.0, 0.1, 5.0, 0.3)
		 , sfbLBPMaxBallPoseDurNear(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/maxBallPoseDurNear", 0.0, 0.1, 10.0, 10.0)
		 , sfbLBPMaxBallPoseDurFar(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/maxBallPoseDurFar", 0.0, 0.1, 10.0, 1.0)
		 , sfbLBPMinTimeToWalkNear(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/minTimeToWalkNear", 1.0, 0.5, 15.0, 2.0)
		 , sfbLBPMinTimeToWalkFar(CONFIG_PARAM_PATH + "behFSM/searchForBall/lastBallPose/minTimeToWalkFar", 1.0, 0.5, 15.0, 8.0)
		 , sfbSpinInsteadRadiusBwd(CONFIG_PARAM_PATH + "behFSM/searchForBall/spinInsteadRadiusBwd", 0.0, 0.05, 4.0, 2.0)
		 , sfbSpinInsteadRadiusFwd(CONFIG_PARAM_PATH + "behFSM/searchForBall/spinInsteadRadiusFwd", 0.0, 0.05, 4.0, 1.5)
		 , sfbWtgpDoneRadius(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkToGlobalPose/doneRadius", 0.0, 0.02, 2.0, 0.4)
		 , sfbWtgpDoneTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkToGlobalPose/doneTime", 0.1, 0.1, 6.0, 2.0)
		 , sfbWtgpFailTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkToGlobalPose/failTime", 0.1, 0.1, 6.0, 2.0)
		 , sfbWtgpDistanceLow(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkToGlobalPose/distanceLow", 0.0, 0.05, 5.0, 1.0)
		 , sfbWtgpDistanceHigh(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkToGlobalPose/distanceHigh", 3.0, 0.05, 10.0, 7.0)
		 , sfbWtgpTimeoutLow(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkToGlobalPose/timeoutLow", 3.0, 0.5, 50.0, 10.0)
		 , sfbWtgpTimeoutHigh(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkToGlobalPose/timeoutHigh", 3.0, 1.0, 100.0, 50.0)
		 , sfbSpinGcvZ(CONFIG_PARAM_PATH + "behFSM/searchForBall/spinGcvZ", 0.0, 0.01, 1.0, 0.7)
		 , sfbSpinGcvZSlow(CONFIG_PARAM_PATH + "behFSM/searchForBall/spinGcvZSlow", 0.0, 0.01, 1.0, 0.35)
		 , sfbSpinTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/spinTime", 0.1, 0.1, 15.0, 8.0)
		 , sfbSpinDebounceTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/spinDebounceTime", 0.0, 0.5, 20.0, 8.0)
		 , sfbWalkFwdGcvX(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkFwdGcvX", 0.0, 0.01, 1.0, 0.7)
		 , sfbWalkFwdTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkFwdTime", 0.1, 0.1, 15.0, 6.0)
		 , sfbWalkFwdWaitTime(CONFIG_PARAM_PATH + "behFSM/searchForBall/walkFwdWaitTime", 0.1, 0.1, 10.0, 1.0)
		 
		 , gbbAbsBetaHigh(CONFIG_PARAM_PATH + "behFSM/goBehindBall/absBetaHigh", 0.0, 0.02, 2.0, 1.5)
		 , gbbAbsBetaLow(CONFIG_PARAM_PATH + "behFSM/goBehindBall/absBetaLow", 0.0, 0.02, 2.0, 0.5)
		 , gbbAngleLimitFar(CONFIG_PARAM_PATH + "behFSM/goBehindBall/angleLimitFar", 0.1, 0.01, 1.5, 0.8)
		 , gbbAngleLimitNear(CONFIG_PARAM_PATH + "behFSM/goBehindBall/angleLimitNear", 0.1, 0.01, 1.5, 1.0)
		 , gbbFootChangeMinConf(CONFIG_PARAM_PATH + "behFSM/goBehindBall/footSel/changeFootMinConf", 0.0, 0.01, 1.0, 0.5)
		 , gbbFootChangeWormTime(CONFIG_PARAM_PATH + "behFSM/goBehindBall/footSel/changeFootWormTime", 0.1, 0.1, 10.0, 2.0)
		 , gbbFootSelBallPoseYRatio(CONFIG_PARAM_PATH + "behFSM/goBehindBall/footSel/ballPoseYRatio", 0.0, 0.01, 1.0, 0.4)
		 , gbbFootSelWeightBallPose(CONFIG_PARAM_PATH + "behFSM/goBehindBall/footSel/weightBallPose", 0.0, 0.02, 2.0, 1.0)
		 , gbbFootSelWeightLessDist(CONFIG_PARAM_PATH + "behFSM/goBehindBall/footSel/weightLessDist", 0.0, 0.02, 2.0, 1.0)
		 , gbbFootSelMinLRDist(CONFIG_PARAM_PATH + "behFSM/goBehindBall/footSel/minLRDist", 0.0, 0.005, 0.5, 0.1)
		 , gbbUnforceFootTime(CONFIG_PARAM_PATH + "behFSM/goBehindBall/footSel/unforceFootTime", 0.1, 0.05, 5.0, 2.0)
		 , gbbFullPathLen(CONFIG_PARAM_PATH + "behFSM/goBehindBall/fullPathLen", 0.5, 0.02, 2.0, 0.9)
		 , gbbMinBallDistLeft(CONFIG_PARAM_PATH + "behFSM/goBehindBall/minBallDistLeft", 0.1, 0.01, 1.0, 0.5)
		 , gbbMinBallDistRight(CONFIG_PARAM_PATH + "behFSM/goBehindBall/minBallDistRight", 0.1, 0.01, 1.0, 0.5)
		 , gbbProximityBetaDistTol(CONFIG_PARAM_PATH + "behFSM/goBehindBall/proximity/betaDistTol", 0.01, 0.002, 0.2, 0.08)
		 , gbbProximityRadiusTol(CONFIG_PARAM_PATH + "behFSM/goBehindBall/proximity/radiusTol", 0.01, 0.002, 0.2, 0.08)
		 , gbbProximityValueMax(CONFIG_PARAM_PATH + "behFSM/goBehindBall/proximity/valueMax", 0.1, 0.01, 1.0, 0.7)
		 , gbbPsiAwayFromBallBuf(CONFIG_PARAM_PATH + "behFSM/goBehindBall/psiAwayFromBallBuf", 0.01, 0.01, 0.5, 0.15)
		 , gbbPsiAwayFromBallMax(CONFIG_PARAM_PATH + "behFSM/goBehindBall/psiAwayFromBallMax", 0.5, 0.01, 1.5, 1.0)
		 , gbbRadiusFar(CONFIG_PARAM_PATH + "behFSM/goBehindBall/radiusFar", 1.3, 0.01, 2.5, 1.5)
		 , gbbRadiusNear(CONFIG_PARAM_PATH + "behFSM/goBehindBall/radiusNear", 0.5, 0.01, 1.2, 1.0)
		 , gbbReqBallOffFadingAngle(CONFIG_PARAM_PATH + "behFSM/goBehindBall/reqBallOffsetFadingAngle", 0.0, 0.01, 1.0, 0.6)
		 , gbbSpeedLimitXYZ(CONFIG_PARAM_PATH + "behFSM/goBehindBall/speed/speedLimitXYZ", 0.0, 0.01, 1.0, 1.0)
		 , gbbSpeedFarX(CONFIG_PARAM_PATH + "behFSM/goBehindBall/speed/speedFarX", 0.0, 0.01, 1.0, 1.0)
		 , gbbSpeedFarZ(CONFIG_PARAM_PATH + "behFSM/goBehindBall/speed/speedFarZ", 0.0, 0.01, 1.0, 1.0)
		 , gbbSpeedNearXY(CONFIG_PARAM_PATH + "behFSM/goBehindBall/speed/speedNearXY", 0.0, 0.01, 1.0, 1.0)
		 , gbbSpeedNearXYMin(CONFIG_PARAM_PATH + "behFSM/goBehindBall/speed/speedNearXYMin", 0.0, 0.01, 0.6, 0.05)
		 , gbbSpeedNearZ(CONFIG_PARAM_PATH + "behFSM/goBehindBall/speed/speedNearZ", 0.0, 0.01, 1.0, 1.0)
		 , gbbSpeedNearZMin(CONFIG_PARAM_PATH + "behFSM/goBehindBall/speed/speedNearZMin", 0.0, 0.01, 0.4, 0.05)
		 , gbbStuckMaxBallDist(CONFIG_PARAM_PATH + "behFSM/goBehindBall/stuck/maxBallDist", 0.0, 0.02, 2.0, 1.0)
		 , gbbStuckMaxGcv(CONFIG_PARAM_PATH + "behFSM/goBehindBall/stuck/maxGcv", 0.0, 0.01, 1.0, 0.4)
		 , gbbStuckTime(CONFIG_PARAM_PATH + "behFSM/goBehindBall/stuck/time", 3.0, 0.5, 30.0, 10.0)
		 , gbbVisRobotHalo(CONFIG_PARAM_PATH + "behFSM/goBehindBall/vis/showRobotHalo", false)
		 , gbbVisSimple(CONFIG_PARAM_PATH + "behFSM/goBehindBall/vis/simpleVis", false)
		 
		 , dbAppFootChangeWormTime(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/changeFootWormTime", 0.1, 0.1, 5.0, 1.0)
		 , dbAppAdjustPathAngleHigh(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/adjustPathAngleHigh", 0.1, 0.02, 1.5, 0.6)
		 , dbAppAdjustWalkAngleMax(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/adjustWalkAngleMax", 0.0, 0.01, 1.0, 0.0)
		 , dbAppAngleErrLimit(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/angleErrLimit", 0.2, 0.01, 1.2, 0.8)
		 , dbAppFunnelCurveRadius(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/funnelCurveRadius", 0.2, 0.01, 1.2, 0.5)
		 , dbAppFunnelNeckMag(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/funnelNeckMag", 0.0, 0.005, 0.3, 0.1)
		 , dbAppFunnelEdgeExtra(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/funnelEdgeExtra", 0.1, 0.01, 0.8, 0.2)
		 , dbAppMinPathTargetX(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/minPathTargetX", 0.1, 0.01, 1.0, 0.5)
		 , dbAppMaxPathTargetSlope(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/maxPathTargetSlope", 0.3, 0.01, 1.3, 0.6)
		 , dbAppPathTargetLineOffsetX(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/pathTargetLineOffsetX", 0.0, 0.005, 0.5, 0.2)
		 , dbAppPathOverdriveY(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/pathOverdriveY", 1.0, 0.01, 2.0, 1.0)
		 , dbAppGcvSpeedX(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/gcvSpeedX", 0.0, 0.01, 1.0, 1.0)
		 , dbAppGcvSpeedY(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/gcvSpeedY", 0.0, 0.01, 1.0, 1.0)
		 , dbAppGcvSpeedZ(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/gcvSpeedZ", 0.0, 0.01, 1.0, 1.0)
		 , dbAppGcvSpeedLimit(CONFIG_PARAM_PATH + "behFSM/dribbleBall/approach/gcvSpeedLimit", 0.0, 0.01, 1.0, 1.0)
		 , dbBallMaxX(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/ballMaxX", 0.5, 0.05, 3.0, 2.0)
		 , dbBallMaxYMag(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/ballMaxYMag", 0.0, 0.01, 0.4, 0.15)
		 , dbBallMaxYMagExtra(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/ballMaxYMagExtra", 0.0, 0.01, 0.2, 0.05)
		 , dbBallSpreadSlope(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/ballSpreadSlope", 0.0, 0.01, 0.6, 0.15)
		 , dbBallSpreadSlopeExtra(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/ballSpreadSlopeExtra", 0.0, 0.01, 0.3, 0.05)
		 , dbBallSpreadAcc(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/ballSpreadAcc", 0.0, 0.02, 2.0, 0.0)
		 , dbBallSpreadAccExtra(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/ballSpreadAccExtra", 0.0, 0.02, 2.0, 0.0)
		 , dbTargetMinWedgeHigh(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/targetMinWedgeHigh", 0.4, 0.01, 1.2, 0.8)
		 , dbTargetDistForMinWedge(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/targetDistForMinWedge", 0.5, 0.01, 1.5, 1.0)
		 , dbTargetDistForMinWedgeHigh(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/targetDistForMinWedgeHigh", 1.6, 0.05, 5.0, 3.0)
		 , dbTargetWedgeForFarBall(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/targetWedgeForFarBall", 0.5, 0.05, 3.0, 2.0)
		 , dbBallDistForFarBall(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/ballDistForFarBall", 0.1, 0.02, 2.0, 0.5)
		 , dbOkToDribbleWormTime(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/okToDribbleWormTime", 0.1, 0.02, 2.0, 0.4)
		 , dbStillOkToDribbleWormTime(CONFIG_PARAM_PATH + "behFSM/dribbleBall/conditions/stillOkToDribbleWormTime", 0.1, 0.02, 2.0, 0.4)
		 
		 , kbBallErrorXFwd(CONFIG_PARAM_PATH + "behFSM/kickBall/conditions/ballErrorXFwd", 0.0, 0.01, 0.2, 0.1)
		 , kbBallErrorXFwdExtra(CONFIG_PARAM_PATH + "behFSM/kickBall/conditions/ballErrorXFwdExtra", 0.0, 0.01, 0.2, 0.05)
		 , kbBallErrorYIwd(CONFIG_PARAM_PATH + "behFSM/kickBall/conditions/ballErrorYIwd", 0.0, 0.01, 0.2, 0.05)
		 , kbBallErrorYIwdExtra(CONFIG_PARAM_PATH + "behFSM/kickBall/conditions/ballErrorYIwdExtra", 0.0, 0.01, 0.2, 0.02)
		 , kbBallErrorYOwd(CONFIG_PARAM_PATH + "behFSM/kickBall/conditions/ballErrorYOwd", 0.0, 0.01, 0.2, 0.05)
		 , kbBallErrorYOwdExtra(CONFIG_PARAM_PATH + "behFSM/kickBall/conditions/ballErrorYOwdExtra", 0.0, 0.01, 0.2, 0.02)
		 , kbOkToKickWormTime(CONFIG_PARAM_PATH + "behFSM/kickBall/conditions/okToKickWormTime", 0.1, 0.02, 2.0, 0.4)
		 
		 , visFootLength(CONFIG_PARAM_PATH + "visualisation/footLength", 0.01, 0.005, 0.4, 0.2)
		 , visFootWidth(CONFIG_PARAM_PATH + "visualisation/footWidth", 0.01, 0.005, 0.2, 0.1)
		 , visFootOffsetX(CONFIG_PARAM_PATH + "visualisation/footOffsetX", -0.2, 0.005, 0.2, 0.0)
		 , visFootOffsetY(CONFIG_PARAM_PATH + "visualisation/footOffsetY", 0.0, 0.005, 0.4, 0.15)
		 
		 , debugBlockGCPackets(CONFIG_PARAM_PATH + "debug/blockGCPackets", false)
		 , debugForceHalt(CONFIG_PARAM_PATH + "debug/forceHalt", true)
		 , debugForceNoBall(CONFIG_PARAM_PATH + "debug/forceNoBall", false)
		 , debugForceNoPose(CONFIG_PARAM_PATH + "debug/forceNoPose", false)
		 , debugForceNoKick(CONFIG_PARAM_PATH + "debug/forceNoKick", false)
		 , debugForceNoDive(CONFIG_PARAM_PATH + "debug/forceNoDive", false)
		 , debugMsgSensors(CONFIG_PARAM_PATH + "debug/msgSensors", false)
		 , debugMsgSFB(CONFIG_PARAM_PATH + "debug/msgSFB", false)
		 , debugMsgGBB(CONFIG_PARAM_PATH + "debug/msgGBB", false)
		 , debugMsgKB(CONFIG_PARAM_PATH + "debug/msgKB", false)
		 , debugMsgDB(CONFIG_PARAM_PATH + "debug/msgDB", false)
		 , debugMsgWTP(CONFIG_PARAM_PATH + "debug/msgWTP", false)
		 , debugMsgWTGP(CONFIG_PARAM_PATH + "debug/msgWTGP", false)
		 , debugMsgROSTopics(CONFIG_PARAM_PATH + "debug/msgROSTopics", false)
		 , debugNoStoppedGcv(CONFIG_PARAM_PATH + "debug/noStoppedGcv", false)
		 
		 , forceKickLeftFoot(false)
		 , forceKickRightFoot(false)
		 , confDecayFactorBall(0.0)
		 , confDecayFactorGoal(0.0)
		 , confDecayFactorPose(0.0)
		 , confDecayFactorObst(0.0)
		{
			// Note: The following parameters have callbacks set outside of the WAKConfig class:
			//       sListenToGC:                WAKRosInterface::updateModeStateText
			//       plotData:                   WAKRosInterface::callbackPlotData
			//       debugBlockGCPackets:        WAKRosInterface::handleBlockGCPackets
			//       debugNoStoppedGcv:          WAKRosInterface::handleNoStoppedGcv
			//       ballHasMovedWormTime:       TheWorm::updateWormTime
			//       dgChaseBallWormTime:        TheWorm::updateWormTime
			//       wtpArrivedWormTime:         TheWorm::updateWormTime
			//       gbbFootChangeWormTime:      TheWorm::updateWormTime
			//       dbAppFootChangeWormTime:    TheWorm::updateWormTime
			//       kbOkToKickWormTime:         TheWorm::updateWormTime
			//       dbOkToDribbleWormTime:      TheWorm::updateWormTime
			//       dbStillOkToDribbleWormTime: TheWorm::updateWormTime
			//       visFootLength:              WAKBagMarkerMan::WAKBagMarkerMan
			//       visFootWidth:               WAKBagMarkerMan::WAKBagMarkerMan
			//       visFootOffsetX:             WAKBagMarkerMan::WAKBagMarkerMan
			//       visFootOffsetY:             WAKBagMarkerMan::WAKBagMarkerMan

			// Set up config parameter callbacks
			sForceKickFoot.setCallback(boost::bind(&WAKConfig::handleForceKickFoot, this), true);
			confDecayHalfLifeBall.setCallback(boost::bind(&WAKConfig::handleHalfLife, this, &confDecayHalfLifeBall, &confDecayFactorBall), true);
			confDecayHalfLifeGoal.setCallback(boost::bind(&WAKConfig::handleHalfLife, this, &confDecayHalfLifeGoal, &confDecayFactorGoal), true);
			confDecayHalfLifePose.setCallback(boost::bind(&WAKConfig::handleHalfLife, this, &confDecayHalfLifePose, &confDecayFactorPose), true);
			confDecayHalfLifeObst.setCallback(boost::bind(&WAKConfig::handleHalfLife, this, &confDecayHalfLifeObst, &confDecayFactorObst), true);
			forceKickIfChoice.setCallback(boost::bind(&WAKConfig::handleForceKickIfChoice, this), false);
			forceDribbleIfChoice.setCallback(boost::bind(&WAKConfig::handleForceDribbleIfChoice, this), true);
		}

		//! @name Class constants
		///@{
		const std::string CONFIG_PARAM_PATH;                         //!< @brief Path for the walk and kick configuration parameters on the config server.
		const std::string CONFIG_SETTINGS_PATH;                      //!< @brief Path for the robot settings on the config server.
		///@}

		//! @name Robot settings
		///@{
		config_server::Parameter<bool>  sIsPenaltyShoot;
		config_server::Parameter<int>   sKickoffType;
		config_server::Parameter<bool>  sListenToGC;
		config_server::Parameter<bool>  sListenToTC;
		config_server::Parameter<bool>  sUseAutoPositioning;
		config_server::Parameter<bool>  sPlayAsCyan;
		config_server::Parameter<bool>  sPlayOnYellow;
		config_server::Parameter<int>   sWaitTime;
		config_server::Parameter<bool>  sEnableDribble;
		config_server::Parameter<bool>  sEnableKick;
		config_server::Parameter<bool>  sEnableObstacles;
		config_server::Parameter<int>   sForceKickFoot;
		config_server::Parameter<int>   sGameRole;
		///@}

		//! @name External parameters
		///@{
		config_server::Parameter<int> robotNumber;
		///@}

		//! @name General parameters
		///@{
		config_server::Parameter<bool>  enableWAK;
		config_server::Parameter<bool>  pauseWAK;
		config_server::Parameter<bool>  plotData;
		config_server::Parameter<bool>  publishTF;
		config_server::Parameter<bool>  publishVis;
		config_server::Parameter<bool>  simpleModeButton;
		config_server::Parameter<float> globalGcvBwdLimit;           //!< @brief The global limit to the allowed backwards walking speed, the whole GCV is rescaled if necessary to satisfy this limit.
		config_server::Parameter<float> globalGcvSpeedLimit;         //!< @brief The master commanded GCV norm limit, effectively globally capping the speed that can be commanded by the walk and kick.
		config_server::Parameter<float> globalGcvShearXY;            //!< @brief The desired multiplicative scaling of y over x walking velocities to ensure that the robot walks in the angular direction it is commanded (a value of 1.0 is neutral and has no effect on the GCV).
		config_server::Parameter<bool>  forceKickIfChoice;
		config_server::Parameter<bool>  forceDribbleIfChoice;
		config_server::Parameter<float> reqBallOffsetX;
		config_server::Parameter<float> reqBallOffsetXDribble;
		config_server::Parameter<float> reqBallOffsetYDribbleMag;    //!< @brief The required dribble y ball offsets are calculated as the mean of the normal y left/right ball offsets +- this value.
		config_server::Parameter<float> reqBallOffsetYLeft;
		config_server::Parameter<float> reqBallOffsetYRight;
		config_server::Parameter<float> maxBallTargetWedge;          //!< @brief The maximum allowed ball target wedge size.
		config_server::Parameter<float> minBallTargetWedge;          //!< @brief The minimum allowed ball target wedge size.
		config_server::Parameter<float> minBallTargetWedgeExtra;     //!< @brief The minimum extra allowed ball target wedge size for "still ok" checks.
		config_server::Parameter<float> minBallToTargetDist;
		config_server::Parameter<float> kickAccuracyWedge;           //!< @brief The angular accuracy of a normal kick, expressed as a wedge angle.
		config_server::Parameter<float> kickMaxDist;                 //!< @brief The maximum distance the robot can normally ever kick.
		config_server::Parameter<float> kickMinDist;                 //!< @brief The minimum distance that the robot can normally kick.
		config_server::Parameter<float> kickAngleDriftLeftKick;      //!< @brief The average angle that the ball drifts CCW from straight ahead when being kicked by the left foot.
		config_server::Parameter<float> kickAngleDriftRightKick;     //!< @brief The average angle that the ball drifts CCW from straight ahead when being kicked by the right foot.
		config_server::Parameter<float> dribbleAccuracyWedge;        //!< @brief The angular accuracy of normal dribbling, expressed as a wedge angle.
		config_server::Parameter<float> targetPoseReevalTime;
		///@}

		//! @name Sensor parameters
		///@{
		config_server::Parameter<float> ballStableMaxDist;
		config_server::Parameter<float> ballStableTime;
		config_server::Parameter<float> maxBallDist;                 //!< @brief Maximum believable reported distance to the ball.
		config_server::Parameter<float> confDecayTime;
		config_server::Parameter<float> confDecayHalfLifeBall;
		config_server::Parameter<float> confDecayHalfLifeGoal;
		config_server::Parameter<float> confDecayHalfLifePose;
		config_server::Parameter<float> confDecayHalfLifeObst;
		config_server::Parameter<float> confLimitBall;
		config_server::Parameter<float> confLimitBallTarget;
		config_server::Parameter<float> confLimitRobotPose;
		config_server::Parameter<bool>  ballHasMovedDisable;
		config_server::Parameter<float> ballHasMovedMaxRobotDist;
		config_server::Parameter<float> ballHasMovedCentreRadius;
		config_server::Parameter<float> ballHasMovedWormTime;
		config_server::Parameter<float> locHintTimeout;
		config_server::Parameter<bool>  locSetToGoalsEnable;
		config_server::Parameter<float> locSetToGoalsHeadingTol;
		config_server::Parameter<bool>  locSetToSideEnable;
		config_server::Parameter<float> locSetToSideHeadingTol;
		config_server::Parameter<float> locSetToSideOffsetX;
		config_server::Parameter<float> waitTimeReduction;
		config_server::Parameter<float> timeoutBallInPlay;
		config_server::Parameter<float> timeoutDirectGoal;
		///@}

		//! @name Game controller parameters
		///@{
		config_server::Parameter<bool>  gcEnable;
		config_server::Parameter<float> gcSmoothingTime;
		config_server::Parameter<float> gcFreshTime;
		config_server::Parameter<bool>  gcIgnoreSetIfIllegalPose;
		config_server::Parameter<float> gcTimeoutKickoffType;
		config_server::Parameter<float> gcTimeoutReadyFirst;
		config_server::Parameter<float> gcTimeoutSetFirstNormal;
		config_server::Parameter<float> gcTimeoutSetFirstPenalty;
		config_server::Parameter<float> gcTimeoutSetLast;
		config_server::Parameter<float> gcTimeoutTimeoutLast;
		///@}

		//! @name Team communications parameters
		///@{
		config_server::Parameter<bool>  tcEnable;
		config_server::Parameter<float> tcFreshTime;
		///@}

		//! @name Gaze parameters
		///@{
		config_server::Parameter<float> gazeYawAbsMax;
		config_server::Parameter<float> gazePitchMin;
		config_server::Parameter<float> gazePitchMax;
		config_server::Parameter<float> gazePitchNeutral;
		config_server::Parameter<float> gazeVelLimit;
		///@}

		//! @name GameFSM: Obstacle ball handling parameters
		///@{
		config_server::Parameter<bool>  obhEnableObstBallHandling;
		config_server::Parameter<float> obhObstBeforeBallBuf;
		config_server::Parameter<float> obhObstBeyondTargetBuf;
		config_server::Parameter<float> obhObstClearanceLow;
		config_server::Parameter<float> obhObstClearanceHigh;
		config_server::Parameter<float> obhClearanceAngleLowMax;
		config_server::Parameter<float> obhClearanceAngleHighMax;
		config_server::Parameter<float> obhAngleAdjustForFootSel;
		config_server::Parameter<float> obhAngleAdjustWedgeRatio;
		config_server::Parameter<float> obhFootSelObstBallDistMax;
		///@}

		//! @name GameFSM: Positioning parameters
		///@{
		config_server::Parameter<bool>  posCommonTargetEnable;       //! @brief Flag whether to force the use of a common target as the destination of positioning
		config_server::Parameter<int>   posCommonTargetX;            //! @brief The common target x value to be used as the destination of positioning (-3 = Neg goal line, -2 = Neg penalty mark, -1 = Neg circle edge, 0 = Centre, 1 = Pos circle edge, 2 = Pos penalty mark, 3 = Pos goal line)
		config_server::Parameter<int>   posCommonTargetY;            //! @brief The common target y value to be used as the destination of positioning (-2 = Neg side line, -1 = Neg circle edge, 0 = Centre, 1 = Pos circle edge, 2 = Pos side line)
		config_server::Parameter<int>   posCommonTargetRot;          //! @brief The common target rotation to be used as the destination of positioning (multiplied by 45 degrees, so +-4 is facing the negative goal)
		config_server::Parameter<float> posPoseAttackingX;           //! @brief The global x offset from the centre for taking the kickoff if playing on the yellow goal (centre x is 0.0 in field coordinates).
		config_server::Parameter<float> posPoseAttackingY;           //! @brief The global y offset from the centre for taking the kickoff if playing on the yellow goal (centre y is 0.0 in field coordinates).
		config_server::Parameter<float> posPoseAttackingT;           //! @brief The global \f$\theta\f$ for taking the kickoff if playing on the yellow goal.
		config_server::Parameter<float> posPoseDefendingX;           //! @brief The global x offset from the behind-circle position for defending the kickoff if playing on the yellow goal (behind-circle x is `-field.circleRadius()` in field coordinates).
		config_server::Parameter<float> posPoseDefendingY;           //! @brief The global y offset from the behind-circle position for defending the kickoff if playing on the yellow goal (behind-circle y is 0.0 in field coordinates).
		config_server::Parameter<float> posPoseDefendingT;           //! @brief The global \f$\theta\f$ for defending the kickoff if playing on the yellow goal.
		config_server::Parameter<float> posPoseGoalieX;              //! @brief The global x offset from the goal centre for the goalie if playing on the yellow goal (goal centre x is `-field.fieldLengthH()` in field coordinates).
		config_server::Parameter<float> posPoseGoalieY;              //! @brief The global y offset from the goal centre for the goalie if playing on the yellow goal (goal centre y is 0.0 in field coordinates).
		config_server::Parameter<float> posPoseGoalieT;              //! @brief The global \f$\theta\f$ for the goalie if playing on the yellow goal.
		config_server::Parameter<float> posArrivedCostMax;
		config_server::Parameter<float> posPoseLegalityBuffer;       //! @brief The safety distance that the robot should be away from an illegal pose to ensure that it is not positioned illegally.
		config_server::Parameter<float> posPoseLegalWormTime;
		///@}

		//! @name GameFSM: Default ball handling parameters
		///@{
		config_server::Parameter<float> dbhGoalPostRadius;           //!< @brief The radius from the centre of a goal post at which the ball can roll past the goal post (plus a little extra for buffer).
		config_server::Parameter<float> dbhGoalPostRadiusExtra;      //!< @brief An additive value to the goal post radius, to provide extra buffer distance around the goal post for the robot itself.
		config_server::Parameter<float> dbhCornerAdjustmentAngle;    //!< @brief The maximum angle by which to adjust the ball target to get the ball out of the corner quicker.
		config_server::Parameter<float> dbhCornerTangentAngleLow;    //!< @brief The tangent angle at which the maximum nominal ball target corner adjustment should be dbhCornerAdjustmentAngle (coerced linear interpolation).
		config_server::Parameter<float> dbhCornerTangentAngleHigh;   //!< @brief The tangent angle at which the maximum nominal ball target corner adjustment should be zero (coerced linear interpolation).
		config_server::Parameter<float> dbhCornerTangentWedgeLow;    //!< @brief The tangent wedge at which the tanget wedge should start possibly limiting the ball target corner adjustments.
		config_server::Parameter<bool>  dbhCornerFootSelDisable;     //!< @brief Boolean flag whether to disable corner foot selection.
		config_server::Parameter<float> dbhCornerFootSelInsideX;     //!< @brief The maximum x offset of the corner foot selection area from the goal line towards the centre line at its inside edge (defined by dbhCornerFootSelPostOffY).
		config_server::Parameter<float> dbhCornerFootSelOutsideX;    //!< @brief The maximum x offset of the corner foot selection area from the goal line towards the centre line at the side line.
		config_server::Parameter<float> dbhCornerFootSelPostOffY;    //!< @brief The minimum y offset of the corner foot selection area from the outer goal post.
		config_server::Parameter<float> dbhNearPostTargetWedge;      //!< @brief The fixed target wedge that should be used near the goal posts and nets to ensure that the robot just keeps moving the ball instead of possibly getting stuck going behind it.
		config_server::Parameter<float> dbhDbZoneCentreRadius;       //!< @brief The radius of the dribble zone at the centre of the field.
		config_server::Parameter<float> dbhDbZoneNearOppGoalDepth;   //!< @brief The distance from the opponent's goal line (the one we score in) closer than which there is a dribble zone.
		config_server::Parameter<float> dbhDbZoneNearOwnGoalDepth;   //!< @brief The distance from the own goal line (the one we are defending) closer than which there is a dribble zone.
		config_server::Parameter<bool>  dbhDbZoneDisableAnnulusZone; //!< @brief Boolean flag to disable the annulus-shaped dribble zone that is defined by the kick distance parameters.
		config_server::Parameter<bool>  dbhDbZoneDisableCentreZone;  //!< @brief Boolean flag to disable the circular dribble zone in the centre of the field.
		config_server::Parameter<bool>  dbhDbZoneDisableGoalZones;   //!< @brief Boolean flag to disable the two goal-strip dribble zones.
		///@}

		//! @name GameFSM: Default goalie parameters
		///@{
		config_server::Parameter<bool>  dgChaseBallIfClose;
		config_server::Parameter<float> dgArrivedCostMax;
		config_server::Parameter<float> dgScoringDistLow;
		config_server::Parameter<float> dgScoringDistHigh;
		config_server::Parameter<float> dgScoringDistChaseBall;
		config_server::Parameter<float> dgBallDistChaseBall;
		config_server::Parameter<float> dgTargetRadiusLow;
		config_server::Parameter<float> dgTargetRadiusHigh;
		config_server::Parameter<float> dgLookForBallRadius;
		config_server::Parameter<float> dgDistBufFromPosts;
		config_server::Parameter<float> dgTargetTimeAgoTimeout;
		config_server::Parameter<float> dgChaseBallWormTime;
		///@}

		//! @name BehFSM: Gaze at ball parameters
		///@{
		config_server::Parameter<float> gazeBallDistNear;
		config_server::Parameter<float> gazeBallDistFar;
		config_server::Parameter<float> gazeBallDistHorizon;
		config_server::Parameter<float> gazeBallPitchFar;
		config_server::Parameter<float> gazeBallPitchHorizon;
		config_server::Parameter<float> gazeBallTsNear;
		config_server::Parameter<float> gazeBallTsFar;
		config_server::Parameter<float> gazeBallTsMin;
		config_server::Parameter<float> gazeBallTsMax;
		config_server::Parameter<float> gazeBallBoxXMin;
		config_server::Parameter<float> gazeBallBoxXMax;
		config_server::Parameter<float> gazeBallBoxYMin;
		config_server::Parameter<float> gazeBallBoxYMax;
		config_server::Parameter<float> gazeBallYawDeadband;
		config_server::Parameter<float> gazeBallYawFreeLook;
		///@}

		//! @name BehFSM: Obstacle avoidance parameters
		///@{
		config_server::Parameter<bool>  oaEnableObstacleAvoidance;
		config_server::Parameter<float> oaAngleAtObstacleHigh;
		config_server::Parameter<float> oaMaxRadialGcvLow;
		config_server::Parameter<float> oaMaxRadialGcvHigh;
		config_server::Parameter<float> oaObstacleRadiusLow;
		config_server::Parameter<float> oaObstacleRadiusMid;
		config_server::Parameter<float> oaObstacleRadiusHigh;
		config_server::Parameter<float> oaAdjustAngleXYHigh;
		config_server::Parameter<float> oaAdjustGcvZHigh;
		///@}

		//! @name BehFSM: Walk to global pose parameters
		///@{
		config_server::Parameter<float> wtgpAngleErrorCost;          //!< @brief Scale factor to convert the cost of angular deviations in the final pose to equivalent distance deviations (`total_error = dist error + wtgpAngleErrorCost * angle_error`).
		config_server::Parameter<float> wtgpAngleLimitNear;
		config_server::Parameter<float> wtgpAngleLimitFar;
		config_server::Parameter<float> wtgpDistNear;
		config_server::Parameter<float> wtgpDistFar;
		config_server::Parameter<float> wtgpSpeedNearXY;
		config_server::Parameter<float> wtgpSpeedNearZ;
		config_server::Parameter<float> wtgpSpeedFarX;
		config_server::Parameter<float> wtgpSpeedFarZ;
		///@}

		//! @name BehFSM: Walk to pose parameters
		///@{
		config_server::Parameter<float> wtpArrivedCostMaxDefault;
		config_server::Parameter<float> wtpArrivedWormTime;
		///@}

		//! @name BehFSM: Panic attack parameters
		///@{
		config_server::Parameter<float> paTimeout;
		///@}

		//! @name BehFSM: Look left right parameters
		///@{
		config_server::Parameter<float> llrGazeFreqScaler;           //! @brief Multiplicative scaler for the frequency of the head scan relative to a nominal frequency calculated from the velocity limit.
		config_server::Parameter<float> llrGazePitch;                //! @brief The nominal gaze pitch for head scan motions while looking left and right.
		///@}

		//! @name BehFSM: Look for ball parameters
		///@{
		config_server::Parameter<float> laGazeDownFirstFactor;
		config_server::Parameter<float> laGazeMagInitial;
		config_server::Parameter<float> laGazeMagInc;
		config_server::Parameter<float> laGazePitchLookUp;
		config_server::Parameter<float> laGazePitchLookDown;
		config_server::Parameter<float> laGazeSplineAccMax;
		config_server::Parameter<float> laGazeSplineVelMax;
		///@}

		//! @name BehFSM: Search for ball parameters
		///@{
		config_server::Parameter<bool>  sfbTCBallPoseEnable;
		config_server::Parameter<float> sfbTCBallPoseDebounce;
		config_server::Parameter<float> sfbStateRequestTimeout;
		config_server::Parameter<float> sfbResumeCycleTime;
		config_server::Parameter<float> sfbStayCoolTime;
		config_server::Parameter<bool>  sfbBackupEnabled;
		config_server::Parameter<float> sfbBackupMargin;
		config_server::Parameter<float> sfbBackupWalkGcvX;
		config_server::Parameter<float> sfbBackupWalkTime;
		config_server::Parameter<float> sfbBackupWaitTime;
		config_server::Parameter<float> sfbLBPFreshTime;
		config_server::Parameter<float> sfbLBPRadiusNear;
		config_server::Parameter<float> sfbLBPRadiusFar;
		config_server::Parameter<float> sfbLBPMinBallPoseDurNear;
		config_server::Parameter<float> sfbLBPMinBallPoseDurFar;
		config_server::Parameter<float> sfbLBPMaxBallPoseDurNear;
		config_server::Parameter<float> sfbLBPMaxBallPoseDurFar;
		config_server::Parameter<float> sfbLBPMinTimeToWalkNear;
		config_server::Parameter<float> sfbLBPMinTimeToWalkFar;
		config_server::Parameter<float> sfbSpinInsteadRadiusBwd;
		config_server::Parameter<float> sfbSpinInsteadRadiusFwd;
		config_server::Parameter<float> sfbWtgpDoneRadius;
		config_server::Parameter<float> sfbWtgpDoneTime;
		config_server::Parameter<float> sfbWtgpFailTime;
		config_server::Parameter<float> sfbWtgpDistanceLow;
		config_server::Parameter<float> sfbWtgpDistanceHigh;
		config_server::Parameter<float> sfbWtgpTimeoutLow;
		config_server::Parameter<float> sfbWtgpTimeoutHigh;
		config_server::Parameter<float> sfbSpinGcvZ;
		config_server::Parameter<float> sfbSpinGcvZSlow;
		config_server::Parameter<float> sfbSpinTime;
		config_server::Parameter<float> sfbSpinDebounceTime;
		config_server::Parameter<float> sfbWalkFwdGcvX;
		config_server::Parameter<float> sfbWalkFwdTime;
		config_server::Parameter<float> sfbWalkFwdWaitTime;
		///@}

		//! @name BehFSM: Go behind ball parameters
		///@{
		config_server::Parameter<float> gbbAbsBetaHigh;
		config_server::Parameter<float> gbbAbsBetaLow;
		config_server::Parameter<float> gbbAngleLimitFar;
		config_server::Parameter<float> gbbAngleLimitNear;
		config_server::Parameter<float> gbbFootChangeMinConf;
		config_server::Parameter<float> gbbFootChangeWormTime;
		config_server::Parameter<float> gbbFootSelBallPoseYRatio;    //!< @brief Lateral distance of the ball from the centre of the field (expressed as a ratio of the half field width `field.fieldWidthH()`) at which the distance of the ball from the edge of the field starts to play a role for foot selection (near the edge of the field it is preferential to use the outer foot so that the robot is more between the ball and the goal it is defending).
		config_server::Parameter<float> gbbFootSelWeightBallPose;    //!< @brief The higher this weight, the more the robot chooses a foot that tends to put the robot in a position between the ball and the goal he is defending.
		config_server::Parameter<float> gbbFootSelWeightLessDist;    //!< @brief The higher this weight, the more the robot chooses the foot with the shorter distance to the behind ball pose.
		config_server::Parameter<float> gbbFootSelMinLRDist;         //!< @brief Minimum distance between the required left and right foot poses that is used for the normalisation of difference in distance to the left and right behind ball poses (the normalised difference in distance is used for best foot selection).
		config_server::Parameter<float> gbbUnforceFootTime;
		config_server::Parameter<float> gbbFullPathLen;
		config_server::Parameter<float> gbbMinBallDistLeft;
		config_server::Parameter<float> gbbMinBallDistRight;
		config_server::Parameter<float> gbbProximityBetaDistTol;
		config_server::Parameter<float> gbbProximityRadiusTol;
		config_server::Parameter<float> gbbProximityValueMax;
		config_server::Parameter<float> gbbPsiAwayFromBallBuf;
		config_server::Parameter<float> gbbPsiAwayFromBallMax;
		config_server::Parameter<float> gbbRadiusFar;
		config_server::Parameter<float> gbbRadiusNear;
		config_server::Parameter<float> gbbReqBallOffFadingAngle;
		config_server::Parameter<float> gbbSpeedLimitXYZ;
		config_server::Parameter<float> gbbSpeedFarX;
		config_server::Parameter<float> gbbSpeedFarZ;
		config_server::Parameter<float> gbbSpeedNearXY;
		config_server::Parameter<float> gbbSpeedNearXYMin;
		config_server::Parameter<float> gbbSpeedNearZ;
		config_server::Parameter<float> gbbSpeedNearZMin;
		config_server::Parameter<float> gbbStuckMaxBallDist;
		config_server::Parameter<float> gbbStuckMaxGcv;
		config_server::Parameter<float> gbbStuckTime;
		config_server::Parameter<bool>  gbbVisRobotHalo;
		config_server::Parameter<bool>  gbbVisSimple;
		///@}

		//! @name BehFSM: Dribble ball parameters
		///@{
		config_server::Parameter<float> dbAppFootChangeWormTime;
		config_server::Parameter<float> dbAppAdjustPathAngleHigh;
		config_server::Parameter<float> dbAppAdjustWalkAngleMax;
		config_server::Parameter<float> dbAppAngleErrLimit;
		config_server::Parameter<float> dbAppFunnelCurveRadius;
		config_server::Parameter<float> dbAppFunnelNeckMag;
		config_server::Parameter<float> dbAppFunnelEdgeExtra;
		config_server::Parameter<float> dbAppMinPathTargetX;
		config_server::Parameter<float> dbAppMaxPathTargetSlope;
		config_server::Parameter<float> dbAppPathTargetLineOffsetX;
		config_server::Parameter<float> dbAppPathOverdriveY;
		config_server::Parameter<float> dbAppGcvSpeedX;
		config_server::Parameter<float> dbAppGcvSpeedY;
		config_server::Parameter<float> dbAppGcvSpeedZ;
		config_server::Parameter<float> dbAppGcvSpeedLimit;
		config_server::Parameter<float> dbBallMaxX;
		config_server::Parameter<float> dbBallMaxYMag;
		config_server::Parameter<float> dbBallMaxYMagExtra;
		config_server::Parameter<float> dbBallSpreadSlope;
		config_server::Parameter<float> dbBallSpreadSlopeExtra;
		config_server::Parameter<float> dbBallSpreadAcc;
		config_server::Parameter<float> dbBallSpreadAccExtra;
		config_server::Parameter<float> dbTargetMinWedgeHigh;
		config_server::Parameter<float> dbTargetDistForMinWedge;
		config_server::Parameter<float> dbTargetDistForMinWedgeHigh;
		config_server::Parameter<float> dbTargetWedgeForFarBall;
		config_server::Parameter<float> dbBallDistForFarBall;        //!< @brief The ball distance, in addition to the required ball distance for dribble, for which a ball is considered far.
		config_server::Parameter<float> dbOkToDribbleWormTime;
		config_server::Parameter<float> dbStillOkToDribbleWormTime;
		///@}

		//! @name BehFSM: Kick ball parameters
		///@{
		config_server::Parameter<float> kbBallErrorXFwd;
		config_server::Parameter<float> kbBallErrorXFwdExtra;
		config_server::Parameter<float> kbBallErrorYIwd;
		config_server::Parameter<float> kbBallErrorYIwdExtra;
		config_server::Parameter<float> kbBallErrorYOwd;
		config_server::Parameter<float> kbBallErrorYOwdExtra;
		config_server::Parameter<float> kbOkToKickWormTime;
		///@}

		//! @name Visualisation parameters
		///@{
		config_server::Parameter<float> visFootLength;
		config_server::Parameter<float> visFootWidth;
		config_server::Parameter<float> visFootOffsetX;
		config_server::Parameter<float> visFootOffsetY;
		///@}

		//! @name Debug parameters
		///@{
		config_server::Parameter<bool>  debugBlockGCPackets;
		config_server::Parameter<bool>  debugForceHalt;
		config_server::Parameter<bool>  debugForceNoBall;
		config_server::Parameter<bool>  debugForceNoPose;
		config_server::Parameter<bool>  debugForceNoKick;
		config_server::Parameter<bool>  debugForceNoDive;
		config_server::Parameter<bool>  debugMsgSensors;
		config_server::Parameter<bool>  debugMsgSFB;
		config_server::Parameter<bool>  debugMsgGBB;
		config_server::Parameter<bool>  debugMsgKB;
		config_server::Parameter<bool>  debugMsgDB;
		config_server::Parameter<bool>  debugMsgWTP;
		config_server::Parameter<bool>  debugMsgWTGP;
		config_server::Parameter<bool>  debugMsgROSTopics;
		config_server::Parameter<bool>  debugNoStoppedGcv;
		///@}

		// Extra parameters
		bool forceKickLeftFoot;
		bool forceKickRightFoot;
		float confDecayFactorBall;
		float confDecayFactorGoal;
		float confDecayFactorPose;
		float confDecayFactorObst;

		// Config parameter callbacks
		void handleForceKickFoot() { forceKickLeftFoot = (sForceKickFoot() < 0); forceKickRightFoot = (sForceKickFoot() > 0); }
		void handleHalfLife(const config_server::Parameter<float>* configParam, float* factor) { float cycles = configParam->get()/TINC; *factor = (cycles <= 1e-6f ? 0.0 : std::pow(0.5, 1.0/cycles)); }
		void handleForceKickIfChoice() { if(forceKickIfChoice() && forceDribbleIfChoice()) forceDribbleIfChoice.set(false); }
		void handleForceDribbleIfChoice() { if(forceKickIfChoice() && forceDribbleIfChoice()) forceKickIfChoice.set(false); }
	};
}

#endif
// EOF