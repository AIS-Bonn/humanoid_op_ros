// Walk and kick: Class for management of the behaviour states
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_beh_manager.h>
#include <rc_utils/ros_time.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// WAKBehManager class
//

// Behaviour state names
const std::string WAKBehManager::BehStateName[BS_COUNT] = {
	"UnknownBehState",
	"Stopped",
	"PANIC PANIC PANIC PANIC",
	"SearchForBall",
	"GoBehindBall",
	"DribbleBall",
	"KickBall",
	"DiveForBall",
	"LookAround",
	"LookAtBall",
	"LookDown",
	"LookForBall",
	"LookLeftRight",
	"WalkToPose",
	"WalkToPoseLookAround",
	"WalkToPoseLookAtBall",
	"WalkToPoseLookForBall",
	"WalkToPoseLookLeftRight"
};

// Constructor
WAKBehManager::WAKBehManager(WAKConfig& config, const SensorVars& SV, const WAKRosInterface& RI, const WAKGameShared& WGS)
 : config(config)
 , SV(SV)
 , RI(RI)
 , WGS(WGS)
 , WBS(config, SV, RI, *this)
 , m_curState(NULL)
 , m_wakCycle(0)
 , m_stateCycle(0)
{
	// Reset the class variables
	resetVars();

	// Construct the behaviour states
	UnknownState = new BehUnknownState(config, SV, WBS, WGS, BS_UNKNOWN);
	Stopped = new BehStopped(config, SV, WBS, WGS, BS_STOPPED);
	PanicAttack = new BehPanicAttack(config, SV, WBS, WGS, BS_PANIC_ATTACK);
	SearchForBall = new BehSearchForBall(config, SV, WBS, WGS, BS_SEARCH_FOR_BALL);
	GoBehindBall = new BehGoBehindBall(config, SV, WBS, WGS, BS_GO_BEHIND_BALL);
	DribbleBall = new BehDribbleBall(config, SV, WBS, WGS, BS_DRIBBLE_BALL);
	KickBall = new BehKickBall(config, SV, WBS, WGS, BS_KICK_BALL);
	DiveForBall = new BehDiveForBall(config, SV, WBS, WGS, BS_DIVE_FOR_BALL);
	LookAround = new GazeBehLookAround(config, SV, WBS, WGS, BS_LOOK_AROUND);
	LookAtBall = new GazeBehLookAtBall(config, SV, WBS, WGS, BS_LOOK_AT_BALL);
	LookDown = new GazeBehLookDown(config, SV, WBS, WGS, BS_LOOK_DOWN);
	LookForBall = new GazeBehLookForBall(config, SV, WBS, WGS, BS_LOOK_FOR_BALL);
	LookLeftRight = new GazeBehLookLeftRight(config, SV, WBS, WGS, BS_LOOK_LEFT_RIGHT);
	WalkToPose = new WalkBehWalkToPose(config, SV, WBS, WGS, BS_WALK_TO_POSE);
	WalkToPoseLookAround = new BehWalkToPoseLookAround(config, SV, WBS, WGS, BS_WALK_TO_POSE_LOOK_AROUND);
	WalkToPoseLookAtBall = new BehWalkToPoseLookAtBall(config, SV, WBS, WGS, BS_WALK_TO_POSE_LOOK_AT_BALL);
	WalkToPoseLookForBall = new BehWalkToPoseLookForBall(config, SV, WBS, WGS, BS_WALK_TO_POSE_LOOK_FOR_BALL);
	WalkToPoseLookLeftRight = new BehWalkToPoseLookLeftRight(config, SV, WBS, WGS, BS_WALK_TO_POSE_LOOK_LEFT_RIGHT);

	// Reset the class
	reset();

	// Config parameter callbacks
	config.kbOkToKickWormTime.setCallback(boost::bind(&TheWorm::updateWormTime, &m_kickWorm, &config.kbOkToKickWormTime), true);
	config.dbOkToDribbleWormTime.setCallback(boost::bind(&TheWorm::updateWormTime, &m_dribbleWorm, &config.dbOkToDribbleWormTime), true);
	config.dbStillOkToDribbleWormTime.setCallback(boost::bind(&TheWorm::updateWormTime, &m_stillDribbleWorm, &config.dbStillOkToDribbleWormTime), true);
}

// Destructor
WAKBehManager::~WAKBehManager()
{
	// Delete the behaviour states
	delete UnknownState;
	delete Stopped;
	delete PanicAttack;
	delete SearchForBall;
	delete GoBehindBall;
	delete DribbleBall;
	delete KickBall;
	delete DiveForBall;
	delete LookAround;
	delete LookAtBall;
	delete LookDown;
	delete LookForBall;
	delete LookLeftRight;
	delete WalkToPose;
	delete WalkToPoseLookAround;
	delete WalkToPoseLookAtBall;
	delete WalkToPoseLookForBall;
	delete WalkToPoseLookLeftRight;
}

// Reset function
void WAKBehManager::reset()
{
	// Reset the game variable inputs
	m_GV.reset();

	// Reset the walk and kick cycle count
	m_wakCycle = 0;

	// Reset the state machine
	resetStateMachine();
}

// Reset variables function
void WAKBehManager::resetVars()
{
	// Reset the actuator variable outputs
	m_AV.reset();
	m_lastAV.reset();

	// Reset the decide state variables
	m_ballAction = BA_DEFAULT;
	m_kickWorm.reset();
	m_dribbleWorm.reset();
	m_stillDribbleWorm.reset();
	m_allowBreakDribble = false;

	// Reset the walking target variables
	m_walkingTarget.setZero();
	m_walkingTargetTol = -1.0f;
	m_walkingTargetValid = false;
}

// Reset state machine function
void WAKBehManager::resetStateMachine()
{
	// Deactivate the currently active state
	if(m_curState)
		m_curState->deactivate();
	m_curState = UnknownState;

	// Reset the state cycle counter (this should precede the deactivation below)
	m_stateCycle = 0;

	// Deactivate every state just for good measure
	for(std::map<int, WAKBehState*>::iterator it = m_stateMap.begin(); it != m_stateMap.end(); ++it)
		it->second->deactivate();

	// Reset the class variables
	resetVars();
}

// Behaviour state registration function
void WAKBehManager::registerState(WAKBehState* state, int ID, const std::string& name)
{
	// Ignore null state pointers
	if(!state) return;

	// Insert the state into our state maps
	m_stateMap[ID] = state;
	m_stateNameMap[ID] = name;
}

// Update function
void WAKBehManager::updateManager(const GameVars& GV, cycle_t wakCycle)
{
	// Update the game variable inputs
	m_GV = GV;

	// Update the walk and kick cycle number
	m_wakCycle = wakCycle;

	// If we have just started playing then process the search for ball hint in the configs
	if(SV.playStateIsNew && SV.playState == PS_PLAY)
	{
		SearchForBall->resetSearch();
		if(SV.kickoffType == KT_MANUAL)
			SearchForBall->requestSfbState(BehSearchForBall::SFB_WS_GOTOCENTRE);
		else if(SV.kickoffType == KT_DEFENDING)
			SearchForBall->requestSfbState(BehSearchForBall::SFB_WS_WALKTOMARK, (SV.goalSign > 0 ? +1 : -1)); // TODO: Robocup hack (used to be spin) to ensure that a bad localisation doesn't make a manually placed defending robot walk back into his own goals
		else // If KT_ATTACKING or KT_DROPBALL...
			SearchForBall->requestSfbState(BehSearchForBall::SFB_WS_GOTOCENTRE);
	}

	// If we are playing but the ball is not in play yet then hold on to any state requests
	if(SV.playState == PS_PLAY && !SV.ballInPlay)
		SearchForBall->refreshSfbStateRequest();

	// Update the shared behaviour variables
	WBS.updateShared(GV);
}

// Execute function
void WAKBehManager::execute()
{
	// Initialise the actuator variable outputs
	m_lastAV = m_AV;
	m_AV.reset();

	// Special handling if walk and kick was just activated
	if(m_wakCycle <= 1)
	{
		// Reset the state machine
		resetStateMachine();

		// Initialise the last actuator variable outputs
		m_lastAV.halt = false;
		m_lastAV.GCV.setZero();
		m_lastAV.doKick = false;
		m_lastAV.rightKick = true;
		m_lastAV.doDive = DD_NONE;
		m_lastAV.gazeYaw = 0.0f;
		m_lastAV.gazePitch = config.gazePitchNeutral();
	}

	// Evaluate the finite state machine to decide on a next state
	WAKBehState* oldState = m_curState;
	m_curState = decideState();
	bool stateChanged = (m_curState != oldState);

	// Execute state change actions if the state changed
	if(stateChanged)
	{
		// Execute the callback for the state that just deactivated
		oldState->deactivate();

		// Reset the state cycle counter
		m_stateCycle = 0;

		// Execute the callback for the state that just activated
		m_curState->activate();

		// Display the new behaviour state
		ROS_WARN("Behaviour state --> %s", m_curState->name().c_str());
	}

	// Increment the state cycle counter
	m_stateCycle++;

	// Invalidate the walking target
	invalidateWalkingTarget();

	// Execute the currently active state
	m_curState->execute(m_AV, m_lastAV, stateChanged);

	// Protect against numerical corruption by errant states
	if(!std::isfinite(m_AV.GCV.x()) || !std::isfinite(m_AV.GCV.y()) || !std::isfinite(m_AV.GCV.z()))
	{
		ROS_ERROR_THROTTLE(0.5, "The %s behaviour state returned a non-finite GCV (%.3f, %.3f, %.3f) => Forcibly adjusting, but this should not happen!", m_curState->name().c_str(), m_AV.GCV.x(), m_AV.GCV.y(), m_AV.GCV.z());
		m_AV.GCV.setZero();
	}
	if(!std::isfinite(m_AV.gazePitch) || !std::isfinite(m_AV.gazeYaw))
	{
		ROS_ERROR_THROTTLE(0.5, "The %s behaviour state returned a non-finite gaze target (%.3f, %.3f) => Forcibly adjusting, but this should not happen!", m_curState->name().c_str(), m_AV.gazePitch, m_AV.gazeYaw);
		m_AV.gazePitch = config.gazePitchNeutral();
		m_AV.gazeYaw = 0.0f;
	}

	// Plotting
	if(config.plotData())
	{
		RI.PM->plotScalar(m_stateCycle / ((double) PMSCALE_CYCLE), PM_BM_STATECYCLE);
		RI.PM->plotScalar(m_curState->id(), PM_BM_CURSTATE);
		RI.PM->plotScalar(m_AV.gazePitch, PM_AV_GAZEPITCH);
		RI.PM->plotScalar(m_AV.gazeYaw, PM_AV_GAZEYAW);
		RI.PM->plotScalar(m_AV.GCV.x(), PM_AV_GCVX);
		RI.PM->plotScalar(m_AV.GCV.y(), PM_AV_GCVY);
		RI.PM->plotScalar(m_AV.GCV.z(), PM_AV_GCVZ);
		RI.PM->plotScalar(m_AV.halt * PMSCALE_HALT, PM_AV_HALT);
		RI.PM->plotScalar((m_AV.doKick && !m_AV.rightKick) * PMSCALE_KICK, PM_AV_KICKLEFT);
		RI.PM->plotScalar((m_AV.doKick && m_AV.rightKick) * PMSCALE_KICK, PM_AV_KICKRIGHT);
		RI.PM->plotScalar(m_AV.doDive, PM_AV_DIVE);
		if(stateChanged || m_wakCycle <= 1)
			RI.PM->plotEvent("Beh " + m_curState->nameRef());
	}

	// Visualisation markers
	if(RI.MM->willPublish())
	{
		if(stateChanged || m_wakCycle <= 1)
			RI.MM->BehStateText.setText(m_curState->nameRef());
		RI.MM->BehStateText.updateAdd();
		if(m_walkingTargetValid)
		{
			RI.MM->WalkingTarget.setPosition(m_walkingTarget.x(), m_walkingTarget.y());
			RI.MM->WalkingTarget.show();
			if(m_walkingTargetTol >= 0.0f)
			{
				for(int i = 0; i <= WAKMarkerMan::NumCirclePoints - 1; i++)
				{
					float theta = i * (M_2PI / (WAKMarkerMan::NumCirclePoints - 1));
					float ctheta = cos(theta);
					float stheta = sin(theta);
					RI.MM->WalkingTargetTol.setPoint(i, m_walkingTarget.x() + m_walkingTargetTol*ctheta, m_walkingTarget.y() + m_walkingTargetTol*stheta, 0.0);
				}
				RI.MM->WalkingTargetTol.show();
			}
			else
				RI.MM->WalkingTargetTol.hide();
		}
		else
		{
			RI.MM->WalkingTarget.hide();
			RI.MM->WalkingTargetTol.hide();
		}
		RI.MM->WalkingTarget.updateAdd();
		RI.MM->WalkingTargetTol.updateAdd();
	}
}

// State decision function
WAKBehState* WAKBehManager::decideState()
{
	// Declare variables
	WAKBehState* const oldState = m_curState;
	WAKBehState* newState = m_curState;

	// Enter a default next state in any case if the state is currently unknown
	if(!behStateValid(newState->id()))
		newState = Stopped;

	// See what we are currently doing
	bool currentlyGBB = (oldState->id() == BS_GO_BEHIND_BALL);
	bool currentlyKicking = (oldState->id() == BS_KICK_BALL);
	bool currentlyDribbling = (oldState->id() == BS_DRIBBLE_BALL);
	bool currentlyDiving = (oldState->id() == BS_DIVE_FOR_BALL);

	// Update the kick and dribble worms
	bool okToKick = KickBall->okToKick();
	bool okToDribble = DribbleBall->okToDribble();
	bool okToDribbleStill = DribbleBall->stillOkToDribble();
	m_kickWorm.vote(okToKick);
	m_dribbleWorm.vote(okToDribble);
	m_stillDribbleWorm.vote(okToDribble || okToDribbleStill);

	// TODO: RoboCup hack
	if(SV.haveBall && WBS.haveBallTarget && WBS.ballToTargetDist >= config.kickMinDist() && !m_allowBreakDribble)
		m_allowBreakDribble = true;
	else if(m_allowBreakDribble && currentlyDribbling && SV.haveBall && WBS.haveBallTarget && WBS.ballToTargetDist < config.kickMinDist() - 0.3f)
	{
		currentlyDribbling = false;
		m_allowBreakDribble = false;
	}

	// Work out whether we could/should kick, dribble and/or dive at this very moment
	bool couldKickNow = m_kickWorm.unanimousTrue();
	bool couldDribbleNow = (m_dribbleWorm.unanimousTrue() || (currentlyDribbling && !m_stillDribbleWorm.unanimousFalse()));
	bool shouldDiveNow = (diveDirectionValid(m_GV.diveIfPossible) && m_GV.diveIfPossible != DD_NONE);

	// Check whether we are allowed to kick and/or dribble
	bool allowKick = m_GV.kickIfPossible;
	bool allowDribble = m_GV.dribbleIfPossible;

	// Handle kick and dribble bans (a ban is an unbreakable command from god, unless both kicking and dribbling are banned, in which case the robot can dribble anyway)
	bool banKick = !config.sEnableKick();
	bool banDribble = !config.sEnableDribble();
	if(banKick)
		allowKick = false;
	if(banDribble)
		allowDribble = false;

	// Retrieve the available ball action tips
	BAType ballActionTipGBB = GoBehindBall->ballActionTip(); // This is only ever something different to BA_UNKNOWN if go behind ball is active and has detected a stuck situation, and is trying to suggest an alternative ball action

	// Decide on the action we are looking to perform with the ball
	if(config.forceDribbleIfChoice() && allowDribble)                       // Force dribble if required, and if it is a choice
		m_ballAction = BA_DRIBBLE;
	else if(config.forceKickIfChoice() && allowKick)                        // Force kick if required, and if it is a choice
		m_ballAction = BA_KICK;
	else if(currentlyGBB && ballActionTipGBB == BA_DRIBBLE && allowDribble) // Go behind ball is advising us to dribble
		m_ballAction = BA_DRIBBLE;
	else if(currentlyGBB && ballActionTipGBB == BA_KICK && allowKick)       // Go behind ball is advising us to kick
		m_ballAction = BA_KICK;
	else if(currentlyDribbling && couldDribbleNow && allowDribble)          // We are currently in the middle of dribbling, so don't interrupt it
		m_ballAction = BA_DRIBBLE;
	else if(allowKick)                                                      // Given the remaining free choice we choose kick first
		m_ballAction = BA_KICK;
	else if(allowDribble)                                                   // Given the remaining free choice we choose dribble second
		m_ballAction = BA_DRIBBLE;
	else if(currentlyKicking)                                               // If we're not allowed to do anything then proceed with kicking, if we're already kicking
		m_ballAction = BA_KICK;
	else if(currentlyDribbling)                                             // If we're not allowed to do anything then proceed with dribbling, if we're already dribbling
		m_ballAction = BA_DRIBBLE;
	else if(config.forceDribbleIfChoice() && !banDribble)                   // If we would be forced to dribble if it were allowed, then dribble
		m_ballAction = BA_DRIBBLE;
	else if(config.forceKickIfChoice() && !banKick)                         // If we would be forced to kick if it were allowed, then kick
		m_ballAction = BA_KICK;
	else if(banKick)                                                        // If kicking is fundamentally banned, then by default dribble
		m_ballAction = BA_DRIBBLE;
	else if(banDribble)                                                     // If dribbling is fundamentally banned but kicking isn't, then by default kick
		m_ballAction = BA_KICK;
	else                                                                    // Else, if absolutely everything is banned, choose the lesser of two evils and dribble
		m_ballAction = BA_DRIBBLE;

	// Decide on the behaviour state we want to be in
	if(behStateValid(m_GV.forceBehStateByID))                               // Choose a particular state if we are forced to from above
		newState = stateForID(m_GV.forceBehStateByID);
	else if(currentlyKicking && KickBall->kickLock())                       // If we are currently executing a kick, then don't interrupt it
		newState = KickBall;
	else if(currentlyDiving && DiveForBall->diveLock())                     // If we are currently executing a dive, then don't interrupt it
		newState = DiveForBall;
	else if(shouldDiveNow)                                                  // If we should dive now, then do it
		newState = DiveForBall;
	else if(!SV.haveBall)                                                   // If we don't have a ball then search for it
		newState = SearchForBall;
	else if(m_ballAction == BA_KICK && couldKickNow)                        // If we can kick right now, and we wish to, then do it
		newState = KickBall;
	else if(m_ballAction == BA_DRIBBLE && couldDribbleNow)                  // If we can dribble right now, and we wish to, then do it
		newState = DribbleBall;
	else                                                                    // Else, keep trying to get in position for the desired ball action
		newState = GoBehindBall;

	// Plotting
	if(config.plotData())
	{
		RI.PM->plotScalar(okToKick * PMSCALE_OKTOKDB, PM_BM_OKTOKICK);
		RI.PM->plotScalar(okToDribble * PMSCALE_OKTOKDB, PM_BM_OKTODRIBBLE);
		RI.PM->plotScalar(okToDribbleStill * PMSCALE_OKTOKDB, PM_BM_OKTODRIBBLESTILL);
		RI.PM->plotScalar(couldKickNow * PMSCALE_COULDKDB, PM_BM_COULDKICKNOW);
		RI.PM->plotScalar(couldDribbleNow * PMSCALE_COULDKDB, PM_BM_COULDDRIBBLENOW);
		RI.PM->plotScalar(allowKick * PMSCALE_ALLOWKDB, PM_BM_ALLOWKICK);
		RI.PM->plotScalar(allowDribble * PMSCALE_ALLOWKDB, PM_BM_ALLOWDRIBBLE);
		RI.PM->plotScalar((m_ballAction == BA_KICK) * PMSCALE_BALLACT, PM_BM_BALLACTIONISKICK);
	}

	// Return the state that has been decided on
	return newState;
}

// Retrieve a behaviour state pointer by ID
WAKBehState* WAKBehManager::stateForID(int ID)
{
	// Return the required behaviour state pointer, or the unknown state if the ID is not recognised
	std::map<int, WAKBehState*>::iterator it = m_stateMap.find(ID);
	if(it != m_stateMap.end())
		return it->second;
	else
		return UnknownState;
}

// Retrieve a behaviour state name by ID
std::string WAKBehManager::stateNameForID(int ID)
{
	// Return the required behaviour state name, or the unknown state name if the ID is not recognised
	std::map<int, std::string>::iterator it = m_stateNameMap.find(ID);
	if(it != m_stateNameMap.end())
		return it->second;
	else
		return UnknownState->name();
}
// EOF