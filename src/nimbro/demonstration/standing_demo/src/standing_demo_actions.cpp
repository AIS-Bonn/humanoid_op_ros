// Actions for the standing demo motion demonstration state machine
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Includes
#include <standing_demo/standing_demo_actions.h>
#include <standing_demo/standing_demo_fsm.h>
#include <standing_demo/standing_demo.h>
#include <cstdlib>

// Namespaces
using namespace standing_demo;

// Functions
inline double randBetween(double LBnd, double UBnd);
inline int randBetweenInt(int LBnd, int UBnd);
inline int randMotion(StandingDemoSC* sc);

//
// StandingDemoAction class
//

// Action: Do nothing
void StandingDemoAction::ActDoNothing(StandingDemoSC* sc, StateQueue* Q)
{
	// Wait for a random delay
	Q->append(NewStateInstance<WaitForTimeState>(sc, randBetween(MIN_NOTHING_DELAY, MAX_NOTHING_DELAY)));
}


// Action: Head idle
void StandingDemoAction::ActHeadIdle(StandingDemoSC* sc, StateQueue* Q)
{
	// Execute background head idling for a random amount of time
	Q->append(NewStateInstance<StartHeadIdlingState>(sc));
	Q->append(NewStateInstance<WaitForTimeState>(sc, randBetween(MIN_HEAD_IDLE_TIME, MAX_HEAD_IDLE_TIME)));
	Q->append(NewStateInstance<StopHeadIdlingState>(sc));
}


// Action: Play motions
void StandingDemoAction::ActPlayMotions(StandingDemoSC* sc, StateQueue* Q)
{
	// Random parameters
	int numMotions = randBetweenInt(MIN_NUM_MOTIONS, MAX_NUM_MOTIONS);
	
	// Add the required number of motions to the queue
	for(int i = 0; i < numMotions; i++)
	{
		// Trigger a random motion and wait for a random delay
		Q->append(NewStateInstance<StartMotionPlaybackState>(sc, randMotion(sc)));
		Q->append(NewStateInstance<StopMotionPlaybackState>(sc));
		Q->append(NewStateInstance<WaitForTimeState>(sc, randBetween(MIN_POST_MOTION_DELAY, MAX_POST_MOTION_DELAY)));
	}
}

// Action: Play motions and head idle
void StandingDemoAction::ActPlayMotionsHI(StandingDemoSC* sc, StateQueue* Q)
{
	// Start background head idling
	Q->append(NewStateInstance<StartHeadIdlingState>(sc));
	
	// Play some motions
	ActPlayMotions(sc, Q);
	
	// Stop background head idling
	Q->append(NewStateInstance<StopHeadIdlingState>(sc));
}


//
// Helper functions
//

// Return a random double between two bounds (inclusive)
inline double randBetween(double LBnd, double UBnd)
{
	// Return a random double in the required range
	return LBnd + (((double) rand()) / RAND_MAX) * (UBnd - LBnd);
}

// Return a random integer between two bounds (inclusive)
inline int randBetweenInt(int LBnd, int UBnd)
{
	// Return a random integer in the required range
	if(UBnd >= LBnd)
		return LBnd + (rand() % (UBnd - LBnd + 1));
	else
		return UBnd + (rand() % (LBnd - UBnd + 1));
	
}

// Return a random motion index
inline int randMotion(StandingDemoSC* sc)
{
	// Return a random motion index
	return rand() % sc->m_SD->numMotions();
}
// EOF