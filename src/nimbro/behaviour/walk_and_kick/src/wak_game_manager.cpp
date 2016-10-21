// Walk and kick: Class for management of the game states
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_game_manager.h>
#include <rc_utils/math_vec_mat.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// WAKGameManager class
//

// Game state names
const std::string WAKGameManager::GameStateName[GS_COUNT] = {
	"UnknownGameState",
	"Stopped",
	"PanicAttack",
	"Positioning",
	"GazeForBall",
	"WaitForBallInPlay",
	"DefaultBallHandling",
	"PenaltyBallHandling",
	"DefaultGoalie",
	"PenaltyGoalie"
};

// Constructor
WAKGameManager::WAKGameManager(WAKConfig& config, const SensorVars& SV, const WAKRosInterface& RI)
 : config(config)
 , SV(SV)
 , RI(RI)
 , WGS(config, SV, RI, *this)
 , m_curState(NULL)
 , m_wakCycle(0)
 , m_stateCycle(0)
{
	// Reset the class variables
	resetVars();

	// Construct the game states
	UnknownState = new GameUnknownState(config, SV, WGS, GS_UNKNOWN);
	Stopped = new GameStopped(config, SV, WGS, GS_STOPPED);
	PanicAttack = new GamePanicAttack(config, SV, WGS, GS_PANIC_ATTACK);
	Positioning = new GamePositioning(config, SV, WGS, GS_POSITIONING);
	GazeForBall = new GameGazeForBall(config, SV, WGS, GS_GAZE_FOR_BALL);
	WaitForBallInPlay = new GameWaitForBallInPlay(config, SV, WGS, GS_WAIT_FOR_BALL_IN_PLAY);
	DefaultBallHandling = new GameDefaultBallHandling(config, SV, WGS, GS_DEFAULT_BALL_HANDLING);
	PenaltyBallHandling = new GamePenaltyBallHandling(config, SV, WGS, GS_PENALTY_BALL_HANDLING);
	DefaultGoalie = new GameDefaultGoalie(config, SV, WGS, GS_DEFAULT_GOALIE);
	PenaltyGoalie = new GamePenaltyGoalie(config, SV, WGS, GS_PENALTY_GOALIE);

	// Reset the class
	reset();

	// Config parameter callbacks
	config.posPoseLegalWormTime.setCallback(boost::bind(&TheWorm::updateWormTime, &m_poseLegalWorm, &config.posPoseLegalWormTime), true);
}

// Destructor
WAKGameManager::~WAKGameManager()
{
	// Delete the game states
	delete UnknownState;
	delete Stopped;
	delete PanicAttack;
	delete Positioning;
	delete GazeForBall;
	delete WaitForBallInPlay;
	delete DefaultBallHandling;
	delete PenaltyBallHandling;
	delete DefaultGoalie;
	delete PenaltyGoalie;
}

// Reset function
void WAKGameManager::reset()
{
	// Reset the walk and kick cycle count
	m_wakCycle = 0;

	// Reset the state machine
	resetStateMachine();
}

// Reset variables function
void WAKGameManager::resetVars()
{
	// Reset the game variable outputs
	m_GV.reset();
	m_lastGV.reset();

	// Reset the decide state variables
	m_poseLegalWorm.reset();
	m_poseLegal = false;

	// Reset the miscellaneous variables
	m_lastPlayState = PS_COUNT;
}

// Reset state machine function
void WAKGameManager::resetStateMachine()
{
	// Deactivate the currently active state
	if(m_curState)
		m_curState->deactivate();
	m_curState = UnknownState;

	// Reset the state cycle counter (this should precede the deactivation below)
	m_stateCycle = 0;

	// Reset whether the state is new
	m_stateIsNew = true;

	// Deactivate every state just for good measure
	for(std::map<int, WAKGameState*>::iterator it = m_stateMap.begin(); it != m_stateMap.end(); ++it)
		it->second->deactivate();

	// Reset the class variables
	resetVars();
}

// Game state registration function
void WAKGameManager::registerState(WAKGameState* state, int ID, const std::string& name)
{
	// Ignore null state pointers
	if(!state) return;

	// Insert the state into our state maps
	m_stateMap[ID] = state;
	m_stateNameMap[ID] = name;
}

// Update function
void WAKGameManager::updateManager(cycle_t wakCycle)
{
	// Update the walk and kick cycle number
	m_wakCycle = wakCycle;

	// Update the shared game variables
	WGS.updateShared();
}

// Execute function
void WAKGameManager::execute()
{
	// Initialise the game variable outputs
	m_lastGV = m_GV;
	m_GV.reset();

	// Special handling if walk and kick was just activated
	if(m_wakCycle <= 1)
	{
		// Reset the state machine
		resetStateMachine();

		// Initialise the last game variable outputs
		m_lastGV.forceBehStateByID = WAKBehManager::BS_UNKNOWN;
		m_lastGV.suggestFoot = GameVars::FS_EITHER_FOOT;
		m_lastGV.dribbleIfPossible = false;
		m_lastGV.kickIfPossible = false;
		m_lastGV.diveIfPossible = DD_NONE;
		m_lastGV.ballTargetConf = 0.0f;
		m_lastGV.ballTargetDir.setZero();
		m_lastGV.ballTargetWedge = 0.0f;
		m_lastGV.ballTargetType = GameVars::BTT_UNKNOWN;
		m_lastGV.targetPose.setZero();
		m_lastGV.targetPoseTol = -1.0f;
		m_lastGV.targetPoseValid = false;
	}

	// Evaluate the finite state machine to decide on a next state
	WAKGameState* oldState = m_curState;
	m_curState = decideState();
	bool stateChanged = (m_curState != oldState);
	m_stateIsNew = (stateChanged || m_wakCycle <= 1);

	// Execute state change actions if the state changed
	if(stateChanged)
	{
		// Execute the callback for the state that just deactivated
		oldState->deactivate();

		// Reset the state cycle counter
		m_stateCycle = 0;

		// Execute the callback for the state that just activated
		m_curState->activate();

		// Display the new game state
		ROS_WARN("Game state ==> %s", m_curState->name().c_str());
	}

	// Increment the state cycle counter
	m_stateCycle++;

	// Execute the currently active state
	m_curState->execute(m_GV, m_lastGV, stateChanged);

	// Protect against numerical corruption by errant states
	if(!std::isfinite(m_GV.ballTargetDir.x()) || !std::isfinite(m_GV.ballTargetDir.y()) || !std::isfinite(m_GV.ballTargetWedge) || !std::isfinite(m_GV.ballTargetConf))
	{
		ROS_ERROR_THROTTLE(0.5, "The %s game state returned a non-finite ball target (%.3f, %.3f) of wedge %.3f and conf %.3f => Forcibly adjusting, but this should not happen!", m_curState->name().c_str(), m_GV.ballTargetDir.x(), m_GV.ballTargetDir.y(), m_GV.ballTargetWedge, m_GV.ballTargetConf);
		m_GV.ballTargetConf = 0.0f;
		m_GV.ballTargetDir.setZero();
		m_GV.ballTargetWedge = 0.0f;
	}
	if(!std::isfinite(m_GV.targetPose.x()) || !std::isfinite(m_GV.targetPose.y()) || !std::isfinite(m_GV.targetPose.z()))
	{
		ROS_ERROR_THROTTLE(0.5, "The %s game state returned a non-finite target pose (%.3f, %.3f, %.3f) indicated as %s => Forcibly adjusting, but this should not happen!", m_curState->name().c_str(), m_GV.targetPose.x(), m_GV.targetPose.y(), m_GV.targetPose.z(), (m_GV.targetPoseValid ? "valid" : "invalid"));
		m_GV.targetPose.setZero();
		m_GV.targetPoseValid = false;
	}
	if(!std::isfinite(m_GV.targetPoseTol))
	{
		ROS_ERROR_THROTTLE(0.5, "The %s game state returned a non-finite target pose tolerance (%.3f) => Forcibly adjusting, but this should not happen!", m_curState->name().c_str(), m_GV.targetPoseTol);
		m_GV.targetPoseTol = -1.0f;
	}

	// Ensure that the enum parameters are in their correct ranges
	if(m_GV.coerceEnums())
		ROS_WARN_THROTTLE(0.5, "The %s game state returned bad enum values => Forcibly adjusting, but this should not happen!", m_curState->name().c_str());

	// Ensure that the ball target is not too close to the ball
	if(SV.haveBall)
	{
		Vec2f ballToTargetVec = m_GV.ballTargetDir - SV.ballDir;
		float ballToTargetDist = ballToTargetVec.norm();
		if(ballToTargetDist <= 0.0f)
		{
			if(m_GV.ballTargetConf > 0.0f)
				ROS_WARN_THROTTLE(0.5, "The %s game state returned a ball target exactly equal to the ball position => Forcibly adjusting, but this should not happen!", m_curState->name().c_str());
			m_GV.ballTargetDir = SV.ballDir + config.minBallToTargetDist() * eigenNormalized(SV.ballDir);
		}
		else if(ballToTargetDist < config.minBallToTargetDist())
			m_GV.ballTargetDir = SV.ballDir + config.minBallToTargetDist() * eigenNormalized(ballToTargetVec);
	}

	// Ensure that the commanded ball target wedge is in range
	m_GV.ballTargetWedge = coerce(m_GV.ballTargetWedge, config.minBallTargetWedge(), config.maxBallTargetWedge());

	// Plotting
	if(config.plotData())
	{
		RI.PM->plotScalar(m_stateCycle / ((double) PMSCALE_CYCLE), PM_GM_STATECYCLE);
		RI.PM->plotScalar(m_curState->id(), PM_GM_CURSTATE);
		if(m_stateIsNew)
			RI.PM->plotEvent("Game " + m_curState->nameRef());
	}

	// Visualisation markers
	if(RI.MM->willPublish())
	{
		if(m_stateIsNew || SV.playState != m_lastPlayState)
		{
			RI.MM->GameStateText.setText(playStateName(SV.playState) + ": " + m_curState->nameRef());
			m_lastPlayState = SV.playState;
		}
		RI.MM->GameStateText.updateAdd();
	}
}

// State decision function
WAKGameState* WAKGameManager::decideState()
{
	// Declare variables
	WAKGameState* newState = m_curState;

	// Enter a default next state in any case if the state is currently unknown
	if(!gameStateValid(newState->id()))
		newState = Stopped;

	// Check whether our current pose is legal for a kickoff
	if(SV.haveRobotPose)
		m_poseLegalWorm.vote(WGS.poseIsLegalForKickoff(SV.robotPose));
	else
		m_poseLegalWorm.vote(true);
	if(m_poseLegalWorm.unanimous())
		m_poseLegal = m_poseLegalWorm.decision();

	// Decide on the game state we want to be in
	if(SV.playState == PS_STOP)
		newState = Stopped;
	else if(SV.playState == PS_TIMEOUT)
		newState = Stopped;
	else if(SV.playState == PS_READY)
	{
		if(SV.kickoffType == KT_MANUAL || (!config.sUseAutoPositioning() && SV.gameCommand != CMD_POS))
			newState = Stopped;
		else
			newState = Positioning;
	}
	else if(SV.playState == PS_SET)
	{
		if(SV.isGoalie()) // TODO: But what if a goalie has been completely activated as a field player? Should this ever happen? Then somehow isGoalie() should be false because he's really not a goalie anymore?
			newState = Stopped;
		else
			newState = GazeForBall;
		if(config.gcIgnoreSetIfIllegalPose() && !m_poseLegal && SV.kickoffType != KT_MANUAL && config.sUseAutoPositioning())
			newState = Positioning;
	}
	else if(SV.playState == PS_BEGIN_PLAY)
		newState = GazeForBall;
	else if(SV.playState == PS_PLAY)
	{
		if(SV.isPenaltyShoot)
		{
			if(SV.isPenaltyTaker)
				newState = PenaltyBallHandling;
			else
				newState = PenaltyGoalie;
		}
		else
		{
			if(SV.isGoalie())
				newState = DefaultGoalie;
			else if(!SV.ballInPlay)
				newState = WaitForBallInPlay;
			else
				newState = DefaultBallHandling;
		}
	}
	else
		newState = Stopped;

	// Plotting
	if(config.plotData())
		RI.PM->plotScalar(m_poseLegal * PMSCALE_LEGAL, PM_GM_POSELEGAL);

	// Return the state that has been decided on
	return newState;
}

// Retrieve a game state pointer by ID
WAKGameState* WAKGameManager::stateForID(int ID)
{
	// Return the required game state pointer, or the unknown state if the ID is not recognised
	std::map<int, WAKGameState*>::iterator it = m_stateMap.find(ID);
	if(it != m_stateMap.end())
		return it->second;
	else
		return UnknownState;
}

// Retrieve a game state name by ID
std::string WAKGameManager::stateNameForID(int ID)
{
	// Return the required game state name, or the unknown state name if the ID is not recognised
	std::map<int, std::string>::iterator it = m_stateNameMap.find(ID);
	if(it != m_stateNameMap.end())
		return it->second;
	else
		return UnknownState->name();
}
// EOF