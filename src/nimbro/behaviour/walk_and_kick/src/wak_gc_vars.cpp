// Walk and kick: Game controller variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_gc_vars.h>
#include <rc_utils/ros_time.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// GCVars class
//

// Constructor
GCVars::GCVars(WAKConfig* config, plot_msgs::PlotManagerFS* PM, WAKMarkerMan* MM)
 : config(config)
 , timeRemaining(config)
 , secondaryTime(config)
 , timeToBallInPlay(config)
 , ownPenaltyTimeRemaining(config)
 , PM(PM)
 , MM(MM)
{
	// Initialise the class
	update(rcup_game_controller::GCData(), ros::Time());
}

// Update function
bool GCVars::update(const rcup_game_controller::GCData& data, const ros::Time& now)
{
	// Work out whether this is an initialisation
	bool init = now.isZero();

	// Update the basic message data
	seqID = (unsigned int) data.seq;
	stampBase = data.stampBase;   // Note: Refer to GCData.msg to see which parts of the message are considered to be base
	stampExtra = data.stampExtra; // Note: Refer to GCData.msg to see which parts of the message are considered to be extra
	extraOutOfDate = (data.extraOutOfDate != 0); // True if the state has changed since extra information was received, so it is most likely incorrect, even if it is fresh

	// Update the players per team
	playersPerTeam = data.playersPerTeam;

	// Update the game phase
	GamePhase oldGamePhase = (init ? GP_NORMAL : gamePhase);
	if(data.secondaryState == GP_NORMAL || data.secondaryState == GP_PENALTY || data.secondaryState == GP_OVERTIME || data.secondaryState == GP_TIMEOUT)
		gamePhase = (GamePhase) data.secondaryState;
	else
		gamePhase = GP_NORMAL;
	bool gamePhaseChanged = (init || gamePhase != oldGamePhase);

	// Update the game state
	GameState oldGameState = (init ? GS_INITIAL : gameState);
	if(data.state == GS_INITIAL || data.state == GS_READY || data.state == GS_SET || data.state == GS_PLAYING || data.state == GS_FINISHED)
		gameState = (GameState) data.state;
	else
		gameState = GS_INITIAL;
	bool gameStateChanged = (init || gameState != oldGameState);

	// Work out how long the game controller has been commanding the playing state
	float timePlaying = 0.0f;
	if(gameState == GS_PLAYING)
	{
		if(gameStateChanged || gamePhaseChanged)
			m_stampStartPlaying = now;
		if(!init && !m_stampStartPlaying.isZero())
			timePlaying = (now - m_stampStartPlaying).toSec();
	}
	else
		zeroRosTime(m_stampStartPlaying);

	// Update the kick-off type and whether we are the attacker in a penalty situation
	if(gamePhase == GP_PENALTY || gamePhase == GP_TIMEOUT)
		kickoffType = KT_MANUAL;
	else if(gameState == GS_PLAYING && ((data.secondaryTime <= 0 && !extraOutOfDate) || (timePlaying >= (config ? config->gcTimeoutKickoffType() : DEFAULT_TIMEOUT_KICKOFF_TYPE))))
		kickoffType = KT_MANUAL;
	else if(data.isDropBall != 0)
		kickoffType = KT_DROPBALL;
	else if(data.ownKickoff != 0)
		kickoffType = KT_ATTACKING;
	else
		kickoffType = KT_DEFENDING;
	isPenaltyTaker = (data.ownKickoff != 0);

	// Update the time variables
	timeRemaining.setValue(data.secsRemaining, stampBase, now);
	secondaryTime.setValue(data.secondaryTime, stampExtra, now);
	if(gameState == GS_PLAYING && (gamePhase == GP_NORMAL || gamePhase == GP_OVERTIME) && kickoffType == KT_DEFENDING && data.secondaryTime > 0 && !extraOutOfDate)
		timeToBallInPlay.setValue(data.secondaryTime, stampExtra, now);
	else
		timeToBallInPlay.reset(now);

	// Update the robot state
	ownPenaltyState = (PenaltyState) data.ownRobot.penaltyState;
	ownPenaltyTimeRemaining.setValue(data.ownRobot.secsUntilUnpenalised, stampBase, now);
	ownIsOnBench = (ownPenaltyState == PS_ON_THE_BENCH);
	ownIsPlaying = (ownPenaltyState == PS_NONE);
	ownIsPenalised = (!ownIsOnBench && !ownIsPlaying);

	// Update the team states
	updateTeamState(ownTeam, data.ownTeam);
	updateTeamState(oppTeam, data.oppTeam);

	// Plotting
	if(config && PM && config->plotData())
	{
		PM->plotScalar((seqID % 11) * 0.1, PM_GC_SEQID);
		PM->plotScalar((stampBase.isZero() ? INFINITY : (now - stampBase).toSec()), PM_GC_TIMESINCEPACKETBASE);
		PM->plotScalar((stampExtra.isZero() ? INFINITY : (now - stampExtra).toSec()), PM_GC_TIMESINCEPACKETEXTRA);
		PM->plotScalar(extraOutOfDate * PMSCALE_ISRESUMED, PM_GC_EXTRAOUTOFDATE);
		PM->plotScalar(gamePhase, PM_GC_GAMEPHASE);
		PM->plotScalar(gameState, PM_GC_GAMESTATE);
		PM->plotScalar(timePlaying, PM_GC_TIMEPLAYING);
		PM->plotScalar(kickoffType, PM_GC_KICKOFFTYPE);
		PM->plotScalar(timeRemaining.raw, PM_GC_TIMEREMAINING_RAW);
		PM->plotScalar(timeRemaining.smooth, PM_GC_TIMEREMAINING_SMOOTH);
		PM->plotScalar(secondaryTime.raw, PM_GC_SECONDARYTIME_RAW);
		PM->plotScalar(secondaryTime.smooth, PM_GC_SECONDARYTIME_SMOOTH);
		PM->plotScalar(timeToBallInPlay.raw, PM_GC_TIMETOBALLINPLAY_RAW);
		PM->plotScalar(timeToBallInPlay.smooth, PM_GC_TIMETOBALLINPLAY_SMOOTH);
		PM->plotScalar(ownPenaltyState, PM_GC_OWNPENALTYSTATE);
		PM->plotScalar(ownPenaltyTimeRemaining.raw, PM_GC_OWNPENALTYTIME_RAW);
		PM->plotScalar(ownPenaltyTimeRemaining.smooth, PM_GC_OWNPENALTYTIME_SMOOTH);
		PM->plotScalar(ownTeam.score, PM_GC_OWNSCORE);
		PM->plotScalar(oppTeam.score, PM_GC_OPPSCORE);
		PM->plotScalar(ownTeam.numPlaying, PM_GC_OWNNUMPLAYING);
		PM->plotScalar(oppTeam.numPlaying, PM_GC_OPPNUMPLAYING);
	}

	// Return whether the game controller base data is fresh
	return baseDataIsFresh(now);
}

// Update function for a team state
void GCVars::updateTeamState(GCVars::TeamState& team, const rcup_game_controller::GCTeamInfo& data)
{
	// Update the team variables
	team.teamNumber = data.teamNumber;
	team.isCyan = (data.teamColour == rcup_game_controller::GCTeamInfo::TEAM_CYAN);
	team.score = data.score;
	team.numNotOnBench = 0;
	team.numPenalised = 0;
	team.numPlaying = 0;
	for(std::size_t i = 0; i < data.players.size(); ++i)
	{
		int penaltyState = data.players[i].penaltyState;
		bool isOnBench = (penaltyState == PS_ON_THE_BENCH);
		bool isPlaying = (penaltyState == PS_NONE);
		bool isPenalised = (!isOnBench && !isPlaying);
		if(!isOnBench) team.numNotOnBench++;
		if(isPenalised) team.numPenalised++;
		if(isPlaying) team.numPlaying++;
	}
}

//
// SmoothTime class
//

// Reset function
void GCVars::SmoothTime::reset(const ros::Time& now)
{
	// Reset the time data variables
	raw = 0;
	smooth = 0.0f;
	zeroRosTime(timestamp);
	elapseTime = now;
	zeroRosTime(m_rawTimestamp);
}

// Set function that updates the smooth time value based on the raw one
void GCVars::SmoothTime::setValue(int timeRemaining, const ros::Time& timestamp, const ros::Time& now)
{
	// Set the raw time value and update the smooth one
	if(timestamp.isZero() || now.isZero())
		reset(now);
	else
	{
		this->timestamp = timestamp;
		if(timeRemaining != raw)
			m_rawTimestamp = timestamp;
		if(m_rawTimestamp.isZero() || (timestamp - m_rawTimestamp).toSec() > 1.1 || timeRemaining == 0)
		{
			raw = timeRemaining;
			smooth = timeRemaining;
			elapseTime = now + ros::Duration(timeRemaining);
		}
		else
		{
			raw = timeRemaining;
			ros::Time newElapseTime = timestamp + ros::Duration(timeRemaining);
			if(elapseTime.isZero())
				elapseTime = newElapseTime;
			else
			{
				float timeDiff = (newElapseTime - elapseTime).toSec();
				if(timeDiff <= 0.0f || timeDiff >= (config ? config->gcSmoothingTime() : DEFAULT_SMOOTHING_TIME))
					elapseTime = newElapseTime;
			}
			smooth = (elapseTime - now).toSec();
		}
	}
}
// EOF