// Walk and kick behaviour state: Search for ball
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/beh_states/beh_search_for_ball.h>
#include <rc_utils/math_vec_mat.h>
#include <rc_utils/math_funcs.h>
#include <algorithm>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// BehSearchForBall class
//

// Search for ball walk state enumeration names
const std::string BehSearchForBall::SFBWalkStateName[SFB_WS_COUNT] = {
	"Unknown",
	"Stay Cool",
	"Back Up",
	"Spin",
	"Go To Ball Hyp",
	"Go To Centre",
	"Spin Here",
	"Walk To Mark",
	"Walk Forwards"
};

// Ball hypothesis type enumeration names
const std::string BehSearchForBall::BallHypTypeName[BHT_COUNT] = {
	"None",
	"TeamComms",
};

// Constructor
BehSearchForBall::BehSearchForBall(WAKConfig& config, const SensorVars& SV, const WAKBehShared& WBS, const WAKGameShared& WGS, int ID) : WAKBehState(config, SV, WBS, WGS, ID), GazeBehLookForBall(config, SV, WBS, WGS, ID)
{
	// Initialise the state request variables
	clearSfbStateRequest();

	// Initialise the last activated state variables
	m_lastActState = SFB_WS_UNKNOWN;
	zeroRosTime(m_lastActTime);
	zeroRosTime(m_lastActSpinTime);
	m_lastActElapsed = 0.0f;

	// Initialise the current walk state and search for ball variables
	changeSfbState(SFB_WS_STAYCOOL);

	// Initialise the resume variables
	clearSuspendedState();

	// Initialise the team communications variables
	m_lastBallHypPose.setZero();
	m_lastBallHypPoseValid = false;

	// Initialise the visualisation variables
	m_visBallHypType = BHT_COUNT;

	// Initialise the activation of the state
	handleActivation(isActive());
}

// Constructor of persistent variables struct
void BehSearchForBall::PersistentVars::reset()
{
	// Initialise the member variables
	nextState = SFB_WS_UNKNOWN;
	ballHypType = BHT_NONE;
	timeout = -1.0f;
	target.setZero();
	dirn = 0;
}

// Reset search function
void BehSearchForBall::resetSearch()
{
	// Reset the search for the ball
	clearSfbStateRequest();
	clearSuspendedState();
	zeroRosTime(m_lastActSpinTime);
}

// Search for ball state change function
void BehSearchForBall::changeSfbState(SFBWalkState newSfbState, SFBWalkState nextState)
{
	// Update the current search for ball walk state
	m_walkState = newSfbState;
	m_walkStateTime = SV.now;
	m_walkStateIsRequest = false;
	m_walkStateIsResumed = false;

	// Reset the persistent search for ball variables
	m_PV.reset();
	m_PV.nextState = nextState;

	// Reset the search for ball variables
	m_factor = -1.0f;
	m_walkFwdTime = config.sfbWalkFwdTime();
	m_doneCounter.reset();
	m_failCounter.reset();

	// Indicate that the walk state is new
	m_walkStateIsNew = true;
}

// Suspend the current walk state
void BehSearchForBall::suspendState()
{
	// Save the current state so that it can be resumed later
	m_resumeWalkState = m_lastActState;
	m_resumeElapsed = m_lastActElapsed;

	// Save the persistent search for ball variables
	m_resumePV = m_PV;
}

// Resume the current suspended walk state
void BehSearchForBall::resumeState()
{
	// Resume the required suspended state
	if(m_resumeWalkState == SFB_WS_BACKUP) // Resuming backing up would rarely ever be a better option than spinning on the spot...
		changeSfbState(SFB_WS_SPIN);
	else if(m_resumeWalkState == SFB_WS_WALKFWDS) // Blindly walking forwards again after our heading has possibly totally changed would not be intelligent...
		changeSfbState(SFB_WS_STAYCOOL);
	else
	{
		// Resume the suspended search for ball state
		changeSfbState(m_resumeWalkState);

		// Fake a walk state time to give the same elapsed time as the cycle when the state was suspended
		m_walkStateTime = SV.now - ros::Duration(m_resumeElapsed);

		// Restore the persistent search for ball variables
		m_PV = m_resumePV;
	}

	// Clear the suspended state
	clearSuspendedState();

	// Indicate that the current state was the result of a resume (we set this even if the exact suspended state wasn't actually resumed, because we want to be more careful about finding ball blips anyway)
	m_walkStateIsResumed = true;
}

// Clear the current suspended state
void BehSearchForBall::clearSuspendedState()
{
	// Clear the current suspended state
	m_resumeWalkState = SFB_WS_UNKNOWN;
	m_resumeElapsed = 0.0f;
}

// Return whether there currently is a suspended state
bool BehSearchForBall::haveSuspendedState() const
{
	// Return whether there currently is a suspended state
	return sfbWalkStateValid(m_resumeWalkState);
}

// Handle activation function
void BehSearchForBall::handleActivation(bool nowActive)
{
	// Handle activation of the base class
	GazeBehLookForBall::handleActivation(nowActive);

	// Suspend the current search for ball state if search for ball is deactivating
	if(!nowActive)
		suspendState();

	// Change to the default search for ball state
	changeSfbState(SFB_WS_STAYCOOL);

	// Handle a pending search for ball state request (if there is one)
	if(nowActive && haveSfbStateRequest())
	{
		// Change to the requested search for ball state
		changeSfbState(m_reqState);

		// Handle the search for ball state request data parameter
		if(m_reqState == SFB_WS_SPIN || m_reqState == SFB_WS_SPINHERE) // Requested data parameter: -1 => Spin CW, 0 => Auto, 1 => Spin CCW
		{
			if(m_reqData != 0)
				m_PV.dirn = (m_reqData > 0 ? +1 : -1);
		}
		else if(m_reqState == SFB_WS_WALKTOMARK) // Requested data parameter: -1 => Walk to defending penalty mark, 0 => Auto, 1 => Walk to attacking penalty mark
		{
			if(m_reqData != 0)
				m_PV.dirn = (m_reqData > 0 ? +1 : -1);
		}
		else if(m_reqState == SFB_WS_WALKFWDS) // Requested data parameter: Walk forward for this many units of time (where each unit is given by config.sfbWalkFwdTime() seconds)
			m_walkFwdTime = config.sfbWalkFwdTime() * coerceMin(m_reqData, 0);
		else if(m_reqState == SFB_WS_GOTOBALLHYP)
		{
			// Initialise the persistent variables for the go to ball hypothesis state
			m_PV.reset();
		}

		// Clear the search for ball state request
		clearSfbStateRequest();

		// Indicate that the activated state was the result of a request
		m_walkStateIsRequest = true;
	}
}

// Execute function
void BehSearchForBall::execute(ActuatorVars& AV, const ActuatorVars& lastAV, bool justActivated)
{
	//
	// Gaze control
	//

	// Look for the ball
	GazeBehLookForBall::execute(AV, lastAV, justActivated);

	//
	// Walking control
	//

	// By default we walk with a zero GCV
	AV.GCV.setZero();

	// Special processing if search for ball just activated
	if(justActivated)
	{
		// Clear all suspended states if it has been too long since search for ball was last active, or if we have received a new walk state request
		if((SV.now - m_lastActTime).toSec() > config.sfbResumeCycleTime() || m_walkStateIsRequest)
		{
			clearSuspendedState();
			m_lastBallHypPoseValid = false;
		}

		// Special processing if we currently do not have a new walk state request
		if(!m_walkStateIsRequest)
		{
			// Resume the current suspended state if we have one
			if(haveSuspendedState())
				resumeState();
		}
	}

	// Retrieve the last activated walk state and see whether the current one is new
	SFBWalkState lastActState = m_lastActState;
	bool walkStateIsNew = (justActivated || m_walkStateIsNew); // Note: This is true even if a state has just been resumed, or a state changes to itself through a call to changeSfbState()!
	m_walkStateIsNew = false;

	// Plot events for state changes
	const std::string& curStateName = sfbWalkStateName(m_walkState);
	if(config.plotData() && walkStateIsNew)
		PM.plotEvent("SFB " + curStateName);

	// Calculate how much time has passed since the current SFB state was activated
	float elapsed = (SV.now - m_walkStateTime).toSec();

	// Decide whether any attempt to spin should be aborted
	bool abortSpin = (walkStateIsNew && !m_walkStateIsRequest && !m_walkStateIsResumed && (SV.now - m_lastActSpinTime).toSec() < config.sfbSpinDebounceTime());

	// Make a note of the last activated SFB state
	m_lastActState = m_walkState;
	m_lastActTime = SV.now;
	m_lastActElapsed = elapsed;

	// See whether we have a suggested next state lined up
	bool haveNextState = sfbWalkStateValid(m_PV.nextState);

	// Initialise the walking target
	WBS.setWalkingTarget(Vec2f::Zero());

	// Decide on where to walk to find the ball
	if(m_walkState == SFB_WS_STAYCOOL)
	{
		// Keep the current walking state while staying cool to avoid unnecessarily toggling the walking state
		AV.halt = lastAV.halt;

		// If enough time has elapsed then try something else
		if(elapsed >= config.sfbStayCoolTime())
			changeSfbState(config.sfbBackupEnabled() ? SFB_WS_BACKUP : SFB_WS_SPIN);
	}
	else if(m_walkState == SFB_WS_BACKUP)
	{
		// We wish to walk
		AV.halt = false;

		// Choose to what extent to back up (if we haven't already)
		bool abortBackup = false;
		if(m_factor < 0.0f)
		{
			float maxBallTimeAgo = config.sfbBackupWaitTime() + (lastActState == SFB_WS_STAYCOOL ? config.sfbStayCoolTime() : 0.0f);
			abortBackup = (SV.ballTimeAgo > maxBallTimeAgo || SV.ballDist > config.sfbBackupMargin());
			if(!SV.haveRobotPose)
				m_factor = 0.5f;
			else
			{
				// Assign local variables
				float margin = config.sfbBackupMargin();
				float Lh = field.fieldLengthH();
				float Wh = field.fieldWidthH();
				float Gh = field.goalWidthH();
				float RPx = SV.robotPose.x();
				float RPy = SV.robotPose.y();
				float theta = SV.robotPose.z();
				float cosTheta = cos(theta);
				float sinTheta = sin(theta);

				// Compute the required backup factor
				float factors[8] = {0};
				factors[0] = backupFactorFromLine(margin, Lh - RPx - (SV.goalSign > 0 ? 0.0f : margin), cosTheta);  // Positive field edge (yellow goal)
				factors[1] = backupFactorFromLine(margin, Lh + RPx - (SV.goalSign > 0 ? margin : 0.0f), -cosTheta); // Negative field edge (blue goal)
				factors[2] = backupFactorFromLine(margin, Wh - RPy, sinTheta);                                      // Far side edge (away from bench)
				factors[3] = backupFactorFromLine(margin, Wh + RPy, -sinTheta);                                     // Near side edge (next to bench)
				factors[4] = backupFactorFromPoint(margin, Lh - RPx, Gh - RPy, theta);                              // Left positive yellow goal post
				factors[5] = backupFactorFromPoint(margin, Lh - RPx, -Gh - RPy, theta);                             // Right positive yellow goal post
				factors[6] = backupFactorFromPoint(margin, -Lh - RPx, Gh - RPy, theta);                             // Right negative blue goal post
				factors[7] = backupFactorFromPoint(margin, -Lh - RPx, -Gh - RPy, theta);                            // Left negative blue goal post
				m_factor = coerce(*std::min_element(factors, factors + 8), 0.0f, 1.0f);                             // Choose the minimum (most constraining) backup factor
			}
		}

		// Walk backwards for a short amount of time
		if(elapsed < m_factor * config.sfbBackupWalkTime())
		{
			AV.GCV.x() = config.sfbBackupWalkGcvX();
			WBS.setWalkingTarget(Vec2f(-config.sfbBackupMargin(), 0.0f));
		}
		else
			AV.GCV.x() = 0.0f;

		// Change the search for ball state if necessary
		if(lastAV.halt || abortBackup || !config.sfbBackupEnabled()) // Backing up makes no sense if we just started walking or shouldn't be doing it anyway
			changeSfbState(SFB_WS_SPIN);
		else if(elapsed >= m_factor * config.sfbBackupWalkTime() + config.sfbBackupWaitTime()) // If enough time has elapsed then try something else
			changeSfbState(SFB_WS_SPIN);
	}
	else if(m_walkState == SFB_WS_SPIN)
	{
		// We wish to walk
		AV.halt = false;

		// Choose a direction to spin (if we haven't already) where +1 is CCW and -1 is CW
		if(m_PV.dirn == 0)
		{
			if(SV.haveRobotPose)
				m_PV.dirn = sign(SV.robotPose.x()*sin(SV.robotPose.z()) - SV.robotPose.y()*cos(SV.robotPose.z())); // This evaluates the sign of the z value of the cross product (px, py, 0) x (cos(pz), sin(pz), 0), where p is the robot pose. This calculates the direction to turn to most quickly bring the centre of the field into view.
			else
				m_PV.dirn = -sign(sin(SV.compassHeading + (SV.goalSign > 0 ? 0.0f : M_PI))); // This calculates the direction to turn to most quickly reach the compass direction that we score in.
		}

		// Command turning on the spot
		if(!abortSpin)
		{
			AV.GCV.z() = m_PV.dirn * (m_walkStateIsResumed ? config.sfbSpinGcvZSlow() : config.sfbSpinGcvZ());
			m_lastActSpinTime = SV.now;
		}

		// If enough time has elapsed then try something else
		if(elapsed >= config.sfbSpinTime() || abortSpin)
			changeSfbState(SFB_WS_GOTOBALLHYP, SFB_WS_GOTOCENTRE);
	}
	else if(m_walkState == SFB_WS_GOTOBALLHYP)
	{
		// We wish to walk
		AV.halt = false;

		// Choose a ball hypothesis to investigate if you haven't already got one
		if(m_PV.ballHypType == BHT_NONE)
		{
			// Initialise the state variables (don't set the next state in case one was passed)
			m_PV.timeout = -1.0f;
			m_PV.target << 0.0f, 0.0f;
			m_PV.dirn = 0;

			// Use a ball hypothesis from the team communications, if we have a reasonable one
			if(config.sfbTCBallPoseEnable() && SV.listenToTC && SV.TCDataAvailable && SV.haveRobotPose)
			{
				// Retrieve the best available ball hypothesis from the team communications (if any)
				bool haveTarget = false;
				float bestBallHypValue = INFINITY;
				for(TCRobotDataMap::const_iterator it = SV.TC.robotDataMap.begin(); it != SV.TC.robotDataMap.end(); ++it)
				{
					// Do not listen to invalid data
					if(!it->second->dataValid) continue;

					// Do not listen to balls that aren't stable detections
					if(!it->second->data.ballPoseStable) continue;

					// Retrieve the communicated stable ball pose
					Vec2f ballHypPose(coerceAbs(it->second->data.ballPoseX, field.fieldLengthH()), coerceAbs(it->second->data.ballPoseY, field.fieldWidthH()));

					// Avoid investigating a ball pose again that we investigated in the recent past
					if(m_lastBallHypPoseValid && (ballHypPose - m_lastBallHypPose).norm() < config.sfbTCBallPoseDebounce()) continue;

					// Calculate the value of the ball hypothesis
					float ballHypValue = calcBallHypValue(ballHypPose);

					// Save this as the current best ball hypothesis if its value is better (i.e. lower) than our previous best ball hypothesis
					if(ballHypValue < bestBallHypValue)
					{
						m_PV.target = ballHypPose;
						haveTarget = true;
					}
				}

				// Investigate the best team communications ball hypothesis that we have
				if(haveTarget)
				{
					// Either spin or walk towards the calculated ball hypothesis to attempt to find the ball
					m_lastBallHypPose = m_PV.target;
					m_lastBallHypPoseValid = true;
					m_PV.ballHypType = BHT_TEAM_COMMS;
					if(spinningToTargetIsBetter(m_PV.target))
					{
						m_PV.timeout = -1.0f;
						m_PV.dirn = calcSpinDirnToTarget(m_PV.target);
						if(!haveNextState)
							m_PV.nextState = SFB_WS_GOTOCENTRE;
					}
					else
					{
						m_PV.timeout = calcTimeout(m_PV.target);
						m_PV.dirn = 0;
						if(!haveNextState)
							m_PV.nextState = SFB_WS_WALKTOMARK;
					}
				}
			}
		}

		// Don't spin if that's what we just did
		if(m_PV.dirn != 0 && abortSpin)
			m_PV.ballHypType = BHT_NONE;

		// Calculate a suitable GCV for investigating the ball hypothesis
		if(m_PV.ballHypType != BHT_NONE)
		{
			if(m_PV.dirn != 0)
			{
				// Set the timeout if we haven't got one yet
				if(m_PV.timeout < 0.0f)
					m_PV.timeout = config.sfbSpinTime();

				// Command turning on the spot
				AV.GCV.z() = m_PV.dirn * (m_walkStateIsResumed ? config.sfbSpinGcvZSlow() : config.sfbSpinGcvZ());
				m_lastActSpinTime = SV.now;
			}
			else
			{
				// Set a GCV suitable for walking to the given target
				float dist = WBS.walkToGlobalPose(AV, m_PV.target.x(), m_PV.target.y());
				WBS.setWalkingTargetTol(config.sfbWtgpDoneRadius());

				// Update counters
				m_failCounter.add(dist == INVALID_DIST);
				m_doneCounter.add(dist < config.sfbWtgpDoneRadius());
			}
		}

		// Change the search for ball state if necessary
		SFBWalkState nextState = (sfbWalkStateValid(m_PV.nextState) ? m_PV.nextState : SFB_WS_GOTOCENTRE); // This default value is used or example if a spinning last ball pose search finished and no TC target was available
		if(m_PV.ballHypType == BHT_NONE) // If we have no ball hypothesis to investigate then fall through to the next state, without spinning as we did nothing
			changeSfbState(nextState);
		else if(m_doneCounter.reached(TO_COUNT(config.sfbWtgpDoneTime()))) // Check whether we have arrived at our destination
			changeSfbState(SFB_WS_SPINHERE, nextState);
		else if(m_failCounter.reached(TO_COUNT(config.sfbWtgpFailTime()))) // Handle the case where walk to global pose failed (e.g. because the robot is not localised)
			changeSfbState(SFB_WS_SPIN);
		else if(elapsed >= m_PV.timeout) // Proceed to the next state if this is taking too long
		{
			if(m_PV.dirn != 0)
				changeSfbState(nextState);
			else
				changeSfbState(SFB_WS_SPINHERE, nextState);
		}
	}
	else if(m_walkState == SFB_WS_GOTOCENTRE)
	{
		// We wish to walk
		AV.halt = false;

		// Select a timeout if we don't have one already
		if(m_PV.timeout < 0.0f)
		{
			m_PV.target << 0.0f, 0.0f; // Location of centre circle
			m_PV.timeout = calcTimeout(m_PV.target);
		}

		// Set a GCV suitable for walking to the chosen target
		float dist = WBS.walkToGlobalPose(AV, m_PV.target.x(), m_PV.target.y());
		WBS.setWalkingTargetTol(config.sfbWtgpDoneRadius());

		// Update counters
		m_failCounter.add(dist == INVALID_DIST);
		m_doneCounter.add(dist < config.sfbWtgpDoneRadius());

		// Change the search for ball state if necessary
		if(m_doneCounter.reached(TO_COUNT(config.sfbWtgpDoneTime()))) // Check whether we have arrived at our destination
			changeSfbState(SFB_WS_SPINHERE, SFB_WS_GOTOBALLHYP);
		else if(m_failCounter.reached(TO_COUNT(config.sfbWtgpFailTime()))) // Handle the case where walk to global pose failed (e.g. because the robot is not localised)
			changeSfbState(SFB_WS_SPIN);
		else if(elapsed >= m_PV.timeout) // Proceed to spinning on the spot if this is taking too long
			changeSfbState(SFB_WS_SPINHERE, SFB_WS_GOTOBALLHYP);
	}
	else if(m_walkState == SFB_WS_SPINHERE)
	{
		// We wish to walk
		AV.halt = false;

		// Choose a direction to spin if we don't have one already
		if(m_PV.dirn == 0)
		{
			if(SV.haveRobotPose)
				m_PV.dirn = sign(-(SV.robotPose.x() + 0.5f*SV.goalSign*(field.fieldLengthH() - field.penaltyMarkDist())) * sin(SV.robotPose.z())); // Turn in the direction that most quickly brings the closest goals into view according to the localisation, with some bias towards choosing the goal that we score in
			else
				m_PV.dirn = -sign(sin(SV.compassHeading + (SV.goalSign > 0 ? 0.0f : M_PI))); // This calculates the direction to turn to most quickly reach the compass direction that we score in.
		}

		// Command turning on the spot
		if(!abortSpin)
		{
			AV.GCV.z() = m_PV.dirn * (m_walkStateIsResumed ? config.sfbSpinGcvZSlow() : config.sfbSpinGcvZ());
			m_lastActSpinTime = SV.now;
		}

		// If enough time has elapsed then try something else
		if(elapsed >= config.sfbSpinTime() || abortSpin)
		{
			if(m_PV.nextState == SFB_WS_GOTOBALLHYP)
				changeSfbState(SFB_WS_GOTOBALLHYP, SFB_WS_WALKTOMARK);
			else
				changeSfbState(haveNextState ? m_PV.nextState : SFB_WS_WALKTOMARK);
		}
	}
	else if(m_walkState == SFB_WS_WALKTOMARK)
	{
		// We wish to walk
		AV.halt = false;

		// Choose which penalty mark we wish to walk to (if we haven't already)
		if(m_PV.dirn == 0)
		{
			if(SV.haveRobotPose)
				m_PV.dirn = sign(0.5f*SV.goalSign*(field.fieldLengthH() - field.penaltyMarkDist()) - SV.robotPose.x()); // Walk to the penalty mark that is not in the half of the field that we're currently in, with some bias towards choosing the penalty mark in the half of the field we want to score in
			else
				m_PV.dirn = SV.goalSign; // Walk to the penalty mark next to the opponent's goal (the one we wish to score in)
		}

		// Select a timeout if we don't have one already
		if(m_PV.timeout < 0.0f)
		{
			m_PV.target << m_PV.dirn * (field.fieldLengthH() - field.penaltyMarkDist()), 0.0f; // Location of the required penalty mark
			m_PV.timeout = calcTimeout(m_PV.target);
		}

		// Set a GCV suitable for walking to the chosen target
		float dist = WBS.walkToGlobalPose(AV, m_PV.target.x(), m_PV.target.y());
		WBS.setWalkingTargetTol(config.sfbWtgpDoneRadius());

		// Update counters
		m_failCounter.add(dist == INVALID_DIST);
		m_doneCounter.add(dist < config.sfbWtgpDoneRadius());

		// Change the search for ball state if necessary
		if(m_doneCounter.reached(TO_COUNT(config.sfbWtgpDoneTime()))) // Check whether we have arrived at our destination
			changeSfbState(SFB_WS_SPINHERE, SFB_WS_GOTOBALLHYP);
		else if(m_failCounter.reached(TO_COUNT(config.sfbWtgpFailTime()))) // Handle the case where walk to global pose failed (e.g. because the robot is not localised)
			changeSfbState(SFB_WS_SPIN);
		else if(elapsed >= m_PV.timeout) // Return to spinning on the spot if this is taking too long
			changeSfbState(SFB_WS_SPINHERE, SFB_WS_GOTOBALLHYP);
	}
	else if(m_walkState == SFB_WS_WALKFWDS)
	{
		// We wish to walk
		AV.halt = false;

		// Walk forwards for a given amount of time
		if(elapsed < m_walkFwdTime)
			AV.GCV.x() = config.sfbWalkFwdGcvX();
		else
			AV.GCV.x() = 0.0f;

		// Apply obstacle avoidance and set the walking target
		Vec2f walkingTarget(2.0f, 0.0f);
		WBS.obstacleAvoidance(AV.GCV, walkingTarget);
		WBS.setWalkingTarget(walkingTarget);

		// If enough time has elapsed then try something else
		if(elapsed >= m_walkFwdTime + config.sfbWalkFwdWaitTime())
			changeSfbState(SFB_WS_SPIN);
	}
	else // Should never happen...
	{
		// Keep the current walking state to avoid unnecesarily toggling the walking state
		AV.halt = lastAV.halt;

		// Change to the default initial search state
		changeSfbState(SFB_WS_STAYCOOL);
	}

	// Print info about the search for ball state
	if(config.debugMsgSFB() && WBS.stateCycle() % 20 == 1)
		printf("SFB: State %d, %.3f elapsed, GCV %.2f %.2f %.2f, Halt %d\n", m_walkState, elapsed, AV.GCV.x(), AV.GCV.y(), AV.GCV.z(), AV.halt);

	// Plotting
	if(config.plotData())
	{
		PM.plotScalar(m_walkState, PM_SFB_SFBSTATE);
		PM.plotScalar(m_walkStateIsRequest * PMSCALE_ISRESUMED, PM_SFB_ISREQUEST);
		PM.plotScalar(m_walkStateIsResumed * PMSCALE_ISRESUMED, PM_SFB_ISRESUMED);
		PM.plotScalar(elapsed, PM_SFB_SFBSTATETIME);
		PM.plotScalar(m_PV.ballHypType, PM_SFB_BALLHYPTYPE);
		PM.plotScalar(m_PV.timeout, PM_SFB_TIMEOUT);
		PM.plotScalar(m_PV.target.x(), PM_SFB_TARGETX);
		PM.plotScalar(m_PV.target.y(), PM_SFB_TARGETY);
		PM.plotScalar(m_PV.dirn, PM_SFB_DIRN);
		PM.plotScalar(m_factor, PM_SFB_FACTOR);
		PM.plotScalar(m_failCounter.count() * PMSCALE_COUNTER, PM_SFB_FAILCOUNTER);
		PM.plotScalar(m_doneCounter.count() * PMSCALE_COUNTER, PM_SFB_DONECOUNTER);
	}

	// Visualisation markers
	if(MM.willPublish())
	{
		if(walkStateIsNew || m_PV.ballHypType != m_visBallHypType)
		{
			m_visBallHypType = m_PV.ballHypType;
			if(m_walkState == SFB_WS_GOTOBALLHYP)
				MM.SubStateText.setText(curStateName + ": " + ballHypTypeName(m_PV.ballHypType));
			else
				MM.SubStateText.setText(curStateName);
		}
		MM.SubStateText.updateAdd();
	}
}

// Calculate the value of investigating a particular ball hypothesis (lower is better!)
float BehSearchForBall::calcBallHypValue(const Vec2f& ballHypPose)
{
	// Better ball hypotheses are closer to the current pose of the robot
	return (ballHypPose - SV.robotPose2D).norm();
}

// Calculate the required timeout for walking to a particular global pose target
float BehSearchForBall::calcTimeout(const Vec2f& target)
{
	// Calculate and return the required timeout
	if(SV.haveRobotPose)
	{
		float targetDist = (target - SV.robotPose2D).norm();
		return interpolateCoerced(config.sfbWtgpDistanceLow(), config.sfbWtgpDistanceHigh(), config.sfbWtgpTimeoutLow(), config.sfbWtgpTimeoutHigh(), targetDist);
	}
	else
		return config.sfbWtgpTimeoutLow();
}

// Calculate the desired direction to spin so that a particular global pose target is faced the most quickly
int BehSearchForBall::calcSpinDirnToTarget(const Vec2f& target)
{
	// Calculate and return the requires spin direction
	if(SV.haveRobotPose)
	{
		float angleToTarget = picut(eigenAngleOf<float, 2>(target - SV.robotPose2D) - SV.robotPose.z());
		return sign(angleToTarget);
	}
	else
		return 0; // Leave it up to the state to choose a spin direction
}

// Decide whether it would be better to spin towards a ball hypothesis instead of walking towards it
bool BehSearchForBall::spinningToTargetIsBetter(const Vec2f& target)
{
	// See whether the target is close enough that spinning to see it would probably be better than walking to it
	if(SV.haveRobotPose)
	{
		float circleRadius = 0.5f*(config.sfbSpinInsteadRadiusFwd() + config.sfbSpinInsteadRadiusBwd());
		float circleCentreBias = 0.5f*(config.sfbSpinInsteadRadiusFwd() - config.sfbSpinInsteadRadiusBwd());
		Vec2f circleCentre = SV.robotPose2D + eigenVecOf(SV.robotPose.z(), circleCentreBias);
		float targetRadius = (target - circleCentre).norm();
		return (targetRadius < circleRadius);
	}
	else
		return false;
}

// Calculate the dimensionless backup factor in the range [0,1] given a particular directionally dependent line to avoid backing up across (has an 'inside' and 'outside' side)
// margin:     The distance margin to use (should nominally be at least the distance that the robot walks backwards when it backs up fully)
// distToLine: The distance to the line (positive if inside and negative if outside)
// cosAlpha:   The cos of the alpha angle, where alpha is the angle between the direction the robot is facing and the direction perpendicular to the line from inside to outside
float BehSearchForBall::backupFactorFromLine(float margin, float distToLine, float cosAlpha)
{
	// Calculate the required backup factor
	float perpDist = margin * -cosAlpha;
	if(perpDist <= distToLine)
		return 1.0f;
	else if(distToLine <= 1e-6f)
		return 0.0f;
	else
		return coerce(distToLine / perpDist, 0.0f, 1.0f); // Logically we must have perpDist > 1e-6 here, so the division is ok
}

// Calculate the dimensionless backup factor in the range [0,1] given a point to avoid backing up into
// margin:     The distance margin to use (should nominally be at least the distance that the robot walks backwards when it backs up fully)
// vecToPoint: The vector from the robot to the point to avoid backing up into in some coordinate system (e.g. the global coordinate system)
// theta:      The angle that the robot is facing CCW from the +ve x axis in the same coordinate system as used by vecToPoint (e.g. the global heading of the robot)
float BehSearchForBall::backupFactorFromPoint(float margin, float vecToPointX, float vecToPointY, float theta)
{
	// Construct a tangential line to the point to avoid and compute the factor using this
	float distToLine = sqrt(vecToPointX*vecToPointX + vecToPointY*vecToPointY) - margin;
	float cosAlpha = cos(theta - atan2(vecToPointY, vecToPointX));
	return backupFactorFromLine(margin, distToLine, cosAlpha);
}
// EOF