// Walk and kick: Sensor variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_sensor_vars.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// SensorVars class
//

// Constructor
SensorVars::SensorVars(WAKConfig& config, WAKRosInterface& RI)
 : config(config)
 , GC(&config, &RI.getPM(), &RI.getMM())
 , TC(RI.TCRI, &config, &RI.getPM(), &RI.getMM())
 , RI(RI)
 , m_updateCycle(0)
{
	// Initialise the class with an initial update
	update(ros::Time());

	// Config parameter callbacks
	config.ballHasMovedWormTime.setCallback(boost::bind(&TheWorm::updateWormTime, &m_ballHasMovedWorm, &config.ballHasMovedWormTime), true);
}

// Update function
void SensorVars::update(const ros::Time& now)
{
	// The walk and kick coordinate system is:
	// - x towards the yellow goal (right one when seen from the bench)
	// - y to the left (away from the bench)
	// - z a CCW rotation from the yellow goal

	// Save the current ROS cycle time
	this->now = now;

	// Update the cycle count
	m_updateCycle++;
	bool init = (m_updateCycle <= 1 || now.isZero());

	// Update the standing count
	if(init)
		m_standingCount = 0;
	if(isStanding())
		m_standingCount++;
	else
		m_standingCount = 0;
	timeStanding = m_standingCount * TINC;

	//
	// Game controller variables
	//

	// Update the game controller variables
	if(config.gcEnable())
	{
		GCDataFresh = GC.update(RI.GCRI.data(), now);
		GCExtraDataFresh = GC.extraDataIsFresh(now);
	}
	else
	{
		GCDataFresh = false;
		GCExtraDataFresh = false;
	}
	listenToGC = (config.gcEnable() && config.sListenToGC() && !init);

	//
	// Configuration variables
	//

	// Decide whether this is a penalty shootout
	if(!listenToGC || init)
		isPenaltyShoot = config.sIsPenaltyShoot();
	if(listenToGC && GCDataFresh && GC.gamePhase != GCVars::GP_TIMEOUT)
		isPenaltyShoot = (GC.gamePhase == GCVars::GP_PENALTY);

	// Retrieve the local game role
	GameRole localGameRole = ROLE_DEFAULT;
	if(gameRoleValid(config.sGameRole()))
		localGameRole = (GameRole) config.sGameRole();

	// Decide whether we are the penalty taker
	if(!listenToGC || init)
		isPenaltyTaker = gameRoleIsFieldPlayer(localGameRole);
	if(listenToGC && GCDataFresh)
		isPenaltyTaker = GC.isPenaltyTaker;

	// Decide on the kickoff type
	if(!listenToGC || init)
		kickoffType = (kickoffTypeValid(config.sKickoffType()) ? (KickoffType) config.sKickoffType() : KT_DEFAULT);
	if(listenToGC && GCDataFresh)
		kickoffType = (kickoffTypeValid(GC.kickoffType) ? GC.kickoffType : KT_DEFAULT);
	if(isPenaltyShoot)
		kickoffType = KT_MANUAL;

	// Set the direction of play
	playOnYellow = config.sPlayOnYellow();
	goalSign = (playOnYellow ? +1 : -1);

	// Set the team colour
	if(!listenToGC || init)
		playAsCyan = config.sPlayAsCyan();
	if(listenToGC && GCDataFresh)
		playAsCyan = GC.ownTeam.isCyan;

	//
	// Play state variables
	//

	// Set the game command based on the button state
	GameCommand lastGameCommand = gameCommand;
	if(RI.button == BTN_STOP)
		gameCommand = CMD_STOP;
	else if(RI.button == BTN_PLAY)
		gameCommand = CMD_PLAY;
	else if(RI.button == BTN_GOALIE)
		gameCommand = CMD_PLAY;
	else if(RI.button == BTN_POS)
		gameCommand = CMD_POS;
	else
		gameCommand = CMD_STOP;
	gameCommandIsNew = (init || gameCommand != lastGameCommand);

	// Decide on a game role
	if(RI.button == BTN_GOALIE)
		gameRole = ROLE_GOALIE;
	else if(isPenaltyShoot)
		gameRole = (isPenaltyTaker ? ROLE_FIELDPLAYER : ROLE_GOALIE);
	else
		gameRole = localGameRole;

	// Update the play state as required
	if(init || gameCommandIsNew)
		setPlayState(PS_STOP);
	PlayState lastPlayState = playState;
	if(!playStateValid(playState))
		setPlayState(PS_STOP);
	if(gameCommand == CMD_STOP)
		setPlayState(PS_STOP);
	else if(gameCommand == CMD_PLAY)
	{
		// Update the play state (NOT listening to game controller)
		if(!listenToGC)
		{
			if(playState != PS_PLAY)
				setPlayState(PS_BEGIN_PLAY, now);
		}

		// Update the play state (listening to game controller)
		if(listenToGC && GCDataFresh)
		{
			ros::Time basestamp = now;
			if(!GC.stampBase.isZero() && GC.stampBase < basestamp)
				basestamp = GC.stampBase;
			if(GC.gamePhase == GCVars::GP_TIMEOUT)
				setPlayState(PS_TIMEOUT, basestamp);
			else if(GC.gamePhase == GCVars::GP_PENALTY)
			{
				if(GC.gameState == GCVars::GS_SET)
					setPlayState(PS_SET, basestamp);
				else if(GC.gameState == GCVars::GS_PLAYING)
					setPlayState(PS_PLAY);
				else // If GS_INITIAL or GS_READY or GS_FINISHED...
					setPlayState(PS_STOP);
			}
			else // If GP_NORMAL or GP_OVERTIME...
			{
				if(GC.gameState == GCVars::GS_READY)
				{
					if(GCExtraDataFresh)
					{
						if(playStateWouldTimeOut(GC.secondaryTime.elapseTime))
							setPlayState(PS_SET);
						else
						{
							setPlayState(PS_READY);
							setPlayStateTimeout(GC.secondaryTime.elapseTime);
						}
					}
					else
					{
						if(playState == PS_READY && havePlayStateTimeout()) {} // If we are already in PS_READY and have previously predicted a time to time out, then we can just sit tight and wait for that to happen.
						else if(playState == PS_SET && (m_playStateFirstTold.isZero() || m_playStateLastTold.isZero()))
						{
							setPlayState(PS_READY); // If the current state is PS_SET, but we were never told to be in PS_SET, then we must have been in PS_READY and timed out by ourself.
							setPlayState(PS_SET);   // As such, we acknowledge that we've just been told to be in PS_READY, but immediately time out again into PS_SET, which effectively just resets the clock on PS_SET timing out.
						}
						else // In all other cases we transition to the PS_READY state and set a manual play state timeout for a fixed time into the future
						{
							setPlayState(PS_READY);
							setPlayStateTimeout(basestamp + ros::Duration(config.gcTimeoutReadyFirst()));
						}
					}
				}
				else if(GC.gameState == GCVars::GS_SET)
					setPlayState(PS_SET, basestamp);
				else if(GC.gameState == GCVars::GS_PLAYING)
				{
					if(playState == PS_PLAY)
						setPlayState(PS_PLAY);
					else
						setPlayState(PS_BEGIN_PLAY, basestamp);
				}
				else // If GS_INITIAL or GS_FINISHED...
					setPlayState(PS_STOP);
			}
		}

		// Update the play state if the current one has timed out
		float timeSinceFirstIn = (m_playStateFirstIn.isZero() ? 0.0 : (now - m_playStateFirstIn).toSec());
		float timeSinceFirstTold = (m_playStateFirstTold.isZero() ? 0.0 : (now - m_playStateFirstTold).toSec());
		float timeSinceLastTold = (m_playStateLastTold.isZero() ? 0.0 : (now - m_playStateLastTold).toSec());
		if(playState == PS_TIMEOUT && timeoutTimeout(timeSinceLastTold))
			setPlayState(PS_PLAY);
		else if(playState == PS_READY && timeoutReady())
			setPlayState(PS_SET);
		else if(playState == PS_SET && timeoutSet(timeSinceFirstIn, timeSinceFirstTold, timeSinceLastTold))
		{
			if(kickoffType == KT_MANUAL)
				setPlayState(PS_PLAY);
			else
				setPlayState(PS_BEGIN_PLAY, now);
		}
		else if(playState == PS_BEGIN_PLAY && timeoutBeginPlay(timeSinceFirstTold))
			setPlayState(PS_PLAY);
	}
	else if(gameCommand == CMD_POS)
		setPlayState(PS_READY);
	else
		setPlayState(PS_STOP);
	playStateIsNew = (init || gameCommandIsNew || playState != lastPlayState);

	// Calculate the time that we have been playing so far
	if(init || gameCommandIsNew || m_lastTimeNotPlaying.isZero() || (playState != PS_BEGIN_PLAY && playState != PS_PLAY))
		m_lastTimeNotPlaying = now;
	timePlaying = (now - m_lastTimeNotPlaying).toSec();

	//
	// Localisation variables
	//

	// Retrieve the heading based on the compass
	if(RI.isFakeRobot())
		compassHeading = RI.robotPoseVec.z(); // The compass heading is overridden with the localisation heading value if this is a fake robot (that actually has no compass)
	else
		compassHeading = RI.robotHeading;     // The compass heading is positive CCW from the positive yellow goal
	compassHeading = picut(compassHeading);

	// Retrieve the confidence of the robot pose
	robotPoseConf = RI.robotPoseConf;
	if(config.debugForceNoPose())
		robotPoseConf = 0.0f;

	// See whether we have a robot pose and whether this is new
	bool hadRobotPose = (init ? false : haveRobotPose);
	haveRobotPose = (robotPoseConf > config.confLimitRobotPose());
	robotPoseIsNew = (!hadRobotPose && haveRobotPose);

	// Update the robot pose timestamps
	if(haveRobotPose)
	{
		if(!hadRobotPose)
			m_robotPoseTimeFirst = now;
		m_robotPoseTimeLast = now;
	}

	// Calculate how long ago, and for how long, the robot pose was last valid
	bool noRobotPoseYet = (init || m_robotPoseTimeLast.isZero());
	robotPoseTimeAgo = (noRobotPoseYet ? INFINITY : (now - m_robotPoseTimeLast).toSec());
	robotPoseDur = (noRobotPoseYet || m_robotPoseTimeFirst.isZero() ? 0.0f : (m_robotPoseTimeLast - m_robotPoseTimeFirst).toSec());

	// Update the robot pose information if we have valid data
	if(init || haveRobotPose)
	{
		robotPose = RI.robotPoseVec;
		robotPose2D = robotPose.head<2>();
		robotPose.z() = picut(robotPose.z());
	}

	//
	// Ball variables
	//

	// Retrieve the confidence of the ball
	ballConf = RI.ballConf;
	if(RI.ballVec.norm() > config.maxBallDist())
		ballConf = 0.0f;
	if(config.debugForceNoBall())
		ballConf = 0.0f;

	// See whether we have a ball and whether this is new
	bool hadBall = (init ? false : haveBall);
	haveBall = (ballConf > config.confLimitBall());
	ballIsNew = (!hadBall && haveBall);

	// Update the ball timestamps
	if(haveBall)
	{
		if(!hadBall)
			m_ballTimeFirst = now;
		m_ballTimeLast = now;
	}

	// Calculate how long ago, and for how long, the ball was last valid
	bool noBallYet = (init || m_ballTimeLast.isZero());
	ballTimeAgo = (noBallYet ? INFINITY : (now - m_ballTimeLast).toSec());
	ballDur = (noBallYet || m_ballTimeFirst.isZero() ? 0.0f : (m_ballTimeLast - m_ballTimeFirst).toSec());

	// Update the ball information if we have valid data
	if(init || haveBall)
	{
		ballDir = RI.ballVec;
		ballAngle = eigenAngleOf(ballDir); // CCW from straight ahead is positive
		ballDist = ballDir.norm();
	}

	// Decide whether we currently have a ball and it is stable
	ballStable = (haveBall && ballDist <= config.ballStableMaxDist() && ballDur >= config.ballStableTime());

	// See whether we have a ball pose and whether this is new
	bool hadBallPose = (init ? false : haveBallPose);
	haveBallPose = (haveBall && haveRobotPose);
	ballPoseIsNew = (!hadBallPose && haveBallPose);

	// Update the ball pose timestamps
	if(haveBallPose)
	{
		if(!hadBallPose)
			m_ballPoseTimeFirst = now;
		m_ballPoseTimeLast = now;
	}

	// Calculate how long ago, and for how long, the ball pose was last valid
	bool noBallPoseYet = (init || m_ballPoseTimeLast.isZero());
	ballPoseTimeAgo = (noBallPoseYet ? INFINITY : (now - m_ballPoseTimeLast).toSec());
	ballPoseDur = (noBallPoseYet || m_ballPoseTimeFirst.isZero() ? 0.0f : (m_ballPoseTimeLast - m_ballPoseTimeFirst).toSec());

	// Update the ball pose information if we have valid data
	if(init || haveBallPose)
		ballPose = robotPose2D + eigenRotatedCCW(ballDir, robotPose.z());

	// Decide whether we currently have a ball pose and it is stable
	ballPoseStable = (haveBallPose && ballDist <= config.ballStableMaxDist() && ballPoseDur >= config.ballStableTime());

	//
	// Goal variables
	//

	// Retrieve the list of goal posts (untouched from what the vision has published)
	goalPosts = RI.goalPostList;

	//
	// Obstacle variables
	//

	// Retrieve the list of obstacles (untouched from what the vision has published)
	obstacles.clear();
	const Obstacle* closest = NULL;
	if(config.sEnableObstacles())
	{
		for(std::size_t i = 0; i < RI.obstacleList.size(); i++)
		{
			if(RI.obstacleList[i].valid())
			{
				obstacles.push_back(RI.obstacleList[i]);
				if(!closest || RI.obstacleList[i].dist < closest->dist)
					closest = &(RI.obstacleList[i]);
			}
		}
	}
	if(closest)
		obstClosest = *closest;
	else
		obstClosest.reset();

	//
	// Extended play state variables
	//

	// See whether the ball has moved since the kickoff
	if(init || gameCommandIsNew || config.ballHasMovedDisable() || (playState != PS_BEGIN_PLAY && playState != PS_PLAY))
	{
		ballHasMoved = false;
		m_ballHasMovedWorm.reset(false);
	}
	else if(haveBallPose && ballDist <= config.ballHasMovedMaxRobotDist())
		m_ballHasMovedWorm.vote(ballPose.norm() > config.ballHasMovedCentreRadius());
	else
		m_ballHasMovedWorm.vote(false);
	if(m_ballHasMovedWorm.unanimousTrue())
		ballHasMoved = true;

	// See whether the ball is in play
	if(init || gameCommandIsNew)
		ballInPlay = false;
	if(playState != PS_BEGIN_PLAY && playState != PS_PLAY)
		ballInPlay = false;
	else if(kickoffType != KT_DEFENDING)
		ballInPlay = true;
	else if(ballHasMoved)
		ballInPlay = true;
	else if(listenToGC && GCExtraDataFresh)
		ballInPlay = (GC.timeToBallInPlay.smooth <= 0.0f);
	else if(timePlaying >= config.timeoutBallInPlay())
		ballInPlay = true;

	// Decide whether a direct goal is allowed
	if(init || gameCommandIsNew)
		directGoalAllowed = false;
	if(playState != PS_BEGIN_PLAY && playState != PS_PLAY)
		directGoalAllowed = false;
	else if(kickoffType != KT_ATTACKING)
		directGoalAllowed = true;
	else if(timePlaying >= config.timeoutDirectGoal())
		directGoalAllowed = true;
	else if(ballHasMoved)
		directGoalAllowed = true;

	//
	// Team communication variables
	//

	// Update the team communication variables
	if(config.tcEnable())
		TCDataAvailable = TC.update(now, this);
	else
		TCDataAvailable = false;
	listenToTC = (config.tcEnable() && config.sListenToTC() && !init);

	//
	// Miscellaneous variables
	//

	// Update the diving variables
	diveDecision = (isStanding() && timeStanding >= 2.5f && !config.debugForceNoDive() ? RI.diveDecision : DD_NONE); // TODO: Make time a config

	//
	// Plotting and visualisation
	//

	// Print the sensor variables state
	if(config.debugMsgSensors() && m_updateCycle % 20 == 1)
	{
		printf("[%s] %s(%5.2f, %5.2f)  %s(%5.2f, %5.2f)  %s(%d)  %s %s %s\n",
		       gameCommandName(gameCommand).c_str(), (haveBall ? "BALL" : "ball"), ballDir.x(), ballDir.y(),
		       (haveRobotPose ? "POSE" : "pose"), robotPose.x(), robotPose.y(), (playOnYellow ? "YELLOW" : "BLUE"),
		       goalSign, (listenToGC ? "EXTERN" : "LOCAL"), (playAsCyan ? "CYAN" : "MAGENTA"), gameRoleName(gameRole).c_str());
	}

	// Plotting
	if(config.plotData())
	{
		RI.PM->plotScalar(isPenaltyShoot * PMSCALE_PENALTY, PM_SV_ISPENALTYSHOOT);
		RI.PM->plotScalar(isPenaltyTaker * PMSCALE_PENALTY, PM_SV_ISPENALTYTAKER);
		RI.PM->plotScalar(kickoffType, PM_SV_KICKOFFTYPE);
		RI.PM->plotScalar(goalSign, PM_SV_GOALSIGN);
		RI.PM->plotScalar(playAsCyan * PMSCALE_COLOR, PM_SV_PLAYASCYAN);
		RI.PM->plotScalar(gameCommand, PM_SV_GAMECOMMAND);
		RI.PM->plotScalar(gameRole, PM_SV_GAMEROLE);
		RI.PM->plotScalar(playState, PM_SV_PLAYSTATE);
		RI.PM->plotScalar((m_playStateFirstIn.isZero() ? 0.0 : (now - m_playStateFirstIn).toSec()), PM_SV_PLAYSTATEFIRSTIN);
		RI.PM->plotScalar((m_playStateFirstTold.isZero() ? 0.0 : (now - m_playStateFirstTold).toSec()), PM_SV_PLAYSTATEFIRSTTOLD);
		RI.PM->plotScalar((m_playStateLastTold.isZero() ? 0.0 : (now - m_playStateLastTold).toSec()), PM_SV_PLAYSTATELASTTOLD);
		RI.PM->plotScalar((m_playStateTimeout.isZero() ? 0.0 : (m_playStateTimeout - now).toSec()), PM_SV_PLAYSTATETIMEOUT);
		RI.PM->plotScalar(timePlaying, PM_SV_TIMEPLAYING);
		RI.PM->plotScalar(compassHeading, PM_SV_COMPASSHEADING);
		RI.PM->plotScalar(robotPoseConf, PM_SV_ROBOTPOSECONF);
		RI.PM->plotScalar(haveRobotPose * PMSCALE_HAVE, PM_SV_HAVEROBOTPOSE);
		RI.PM->plotScalar(robotPoseTimeAgo, PM_SV_ROBOTPOSETIMEAGO);
		RI.PM->plotScalar(robotPoseDur, PM_SV_ROBOTPOSEDUR);
		RI.PM->plotScalar(robotPose.x(), PM_SV_ROBOTPOSEX);
		RI.PM->plotScalar(robotPose.y(), PM_SV_ROBOTPOSEY);
		RI.PM->plotScalar(robotPose.z(), PM_SV_ROBOTPOSEZ);
		RI.PM->plotScalar(ballConf, PM_SV_BALLCONF);
		RI.PM->plotScalar(haveBall * PMSCALE_HAVE, PM_SV_HAVEBALL);
		RI.PM->plotScalar(ballTimeAgo, PM_SV_BALLTIMEAGO);
		RI.PM->plotScalar(ballDur, PM_SV_BALLDUR);
		RI.PM->plotScalar(ballDir.x(), PM_SV_BALLX);
		RI.PM->plotScalar(ballDir.y(), PM_SV_BALLY);
		RI.PM->plotScalar(ballAngle, PM_SV_BALLANGLE);
		RI.PM->plotScalar(ballDist, PM_SV_BALLDIST);
		RI.PM->plotScalar(ballStable * PMSCALE_STABLE, PM_SV_BALLSTABLE);
		RI.PM->plotScalar(haveBallPose * PMSCALE_HAVE, PM_SV_HAVEBALLPOSE);
		RI.PM->plotScalar(ballPoseTimeAgo, PM_SV_BALLPOSETIMEAGO);
		RI.PM->plotScalar(ballPoseDur, PM_SV_BALLPOSEDUR);
		RI.PM->plotScalar(ballPose.x(), PM_SV_BALLPOSEX);
		RI.PM->plotScalar(ballPose.y(), PM_SV_BALLPOSEY);
		RI.PM->plotScalar(ballPoseStable * PMSCALE_STABLE, PM_SV_BALLPOSESTABLE);
		RI.PM->plotScalar(ballHasMoved * PMSCALE_BALLACT, PM_SV_BALLHASMOVED);
		RI.PM->plotScalar(ballInPlay * PMSCALE_KICKOFF, PM_SV_BALLINPLAY);
		RI.PM->plotScalar(directGoalAllowed * PMSCALE_LEGAL, PM_SV_DIRECTGOALALLOWED);
		if(init || playStateIsNew)
			RI.PM->plotEvent("PS " + playStateName(playState));
	}

	// Visualisation markers
	if(RI.MM->willPublish())
	{
		RI.MM->updateMarkerXY(RI.MM->Ball, haveBall, ballDir.x(), ballDir.y(), ballConf);
		if(obstacles.empty())
		{
			RI.MM->updateMarkerXY(RI.MM->ObstacleA, false, 0.0, 0.0, 0.0);
			RI.MM->updateMarkerXY(RI.MM->ObstacleB, false, 0.0, 0.0, 0.0);
		}
		else if(obstacles.size() == 1)
		{
			RI.MM->updateMarkerXY(RI.MM->ObstacleA, true, obstacles[0].vec.x(), obstacles[0].vec.y(), obstacles[0].conf);
			RI.MM->updateMarkerXY(RI.MM->ObstacleB, false, 0.0, 0.0, 0.0);
		}
		else
		{
			std::size_t A, B;
			if(obstacles[0].dist < obstacles[1].dist) { A = 0; B = 1; }
			else { A = 1; B = 0; }
			for(std::size_t i = 2; i < obstacles.size(); i++)
			{
				if(obstacles[i].dist < obstacles[A].dist) { B = A; A = i; }
				else if(obstacles[i].dist < obstacles[B].dist) { B = i; }
			}
			RI.MM->updateMarkerXY(RI.MM->ObstacleA, true, obstacles[A].vec.x(), obstacles[A].vec.y(), obstacles[A].conf);
			RI.MM->updateMarkerXY(RI.MM->ObstacleB, true, obstacles[B].vec.x(), obstacles[B].vec.y(), obstacles[B].conf);
		}
		if(goalSign > 0)
			RI.MM->GoalSign.setColor(0.8f, 0.8f, 0.0f);
		else
			RI.MM->GoalSign.setColor(0.0f, 0.3f, 1.0f);
		RI.MM->GoalSign.marker.pose.position.x = (RI.MM->GoalSign.marker.pose.position.x >= 0.0 ? goalSign : -goalSign) * RI.MM->GoalSign.marker.pose.position.x;
		RI.MM->GoalSign.updateAdd();
		Vec3f northVec(cos(compassHeading), -sin(compassHeading), 0.25f);
		RI.MM->CompassHeading.update(1.0*northVec.x(), 1.0*northVec.y(), northVec.z(), 1.5*northVec.x(), 1.5*northVec.y(), northVec.z());
		RI.MM->CompassHeadingText.update(1.55*northVec.x(), 1.55*northVec.y(), northVec.z());
	}

	// TF transforms
	if(config.publishTF())
		RI.sendTransform(robotPose);
}

// Function to set the play state
void SensorVars::setPlayState(PlayState state, const ros::Time& toldTime)
{
	// Check whether this is a change in play state
	if(state != playState)
	{
		m_playStateFirstIn = now;
		zeroRosTime(m_playStateFirstTold);
		zeroRosTime(m_playStateLastTold);
	}

	// Check whether this state has been told to us
	if(!toldTime.isZero())
	{
		if(m_playStateFirstTold.isZero())
			m_playStateFirstTold = toldTime;
		m_playStateLastTold = toldTime;
	}

	// Clear any pending absolute play state timeout
	clearPlayStateTimeout();

	// Set the play state
	playState = state;
}

// Decide whether SET should time out or not
bool SensorVars::timeoutSet(float timeSinceFirstIn, float timeSinceFirstTold, float timeSinceLastTold) const
{
	// Carefully decide whether the SET play state should time out
	float timeoutFirst = (isPenaltyShoot ? config.gcTimeoutSetFirstPenalty() : config.gcTimeoutSetFirstNormal());
	float timeoutLast = config.gcTimeoutSetLast();
	if((m_playStateFirstTold.isZero() || m_playStateLastTold.isZero()) && timeSinceFirstIn >= timeoutFirst) return true;
	return (timeSinceFirstTold >= timeoutFirst && timeSinceLastTold >= timeoutLast);
}
// EOF