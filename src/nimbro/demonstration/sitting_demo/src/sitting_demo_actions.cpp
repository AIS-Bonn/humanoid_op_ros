// Actions for the sitting demo motion demonstration state machine
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <sitting_demo/sitting_demo_actions.h>
#include <sitting_demo/sitting_demo_fsm.h>
#include <sitting_demo/sitting_demo.h>
#include <cstdlib>

// Namespaces
using namespace sitting_demo;

// Functions
inline double randBetween(double LBnd, double UBnd);
inline int randBetweenInt(int LBnd, int UBnd);
inline int randMotion(SittingDemoSC* sc);

//
// SittingDemoAction class
//

// Action: Do nothing
void SittingDemoAction::ActDoNothing(SittingDemoSC* sc, StateQueue* Q)
{
	// Wait for a random delay
	Q->append(NewStateInstance<WaitForTimeState>(sc, randBetween(MIN_NOTHING_DELAY, MAX_NOTHING_DELAY)));
}

// Action: Leg dangle
void SittingDemoAction::ActLegDangle(SittingDemoSC* sc, StateQueue* Q)
{
	// Execute background leg dangling for a random amount of time
	Q->append(NewStateInstance<StartLegDanglingState>(sc));
	Q->append(NewStateInstance<WaitForTimeState>(sc, randBetween(MIN_LEG_DANGLE_TIME, MAX_LEG_DANGLE_TIME)));
	Q->append(NewStateInstance<StopLegDanglingState>(sc));
}

// Action: Head idle
void SittingDemoAction::ActHeadIdle(SittingDemoSC* sc, StateQueue* Q)
{
	// Execute background head idling for a random amount of time
	Q->append(NewStateInstance<StartHeadIdlingState>(sc));
	Q->append(NewStateInstance<WaitForTimeState>(sc, randBetween(MIN_HEAD_IDLE_TIME, MAX_HEAD_IDLE_TIME)));
	Q->append(NewStateInstance<StopHeadIdlingState>(sc));
}

// Action: Leg dangle and head idle
void SittingDemoAction::ActLegDangleHeadIdle(SittingDemoSC* sc, StateQueue* Q)
{
	// TODO: How to have both leg dangling and head idling??
}

// Action: Play motions
void SittingDemoAction::ActPlayMotions(SittingDemoSC* sc, StateQueue* Q)
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

// Action: Play motions and leg dangle
void SittingDemoAction::ActPlayMotionsLD(SittingDemoSC* sc, StateQueue* Q)
{
	// Start background leg dangling
	Q->append(NewStateInstance<StartLegDanglingState>(sc));

	// Play some motions
	ActPlayMotions(sc, Q);

	// Stop background leg dangling
	Q->append(NewStateInstance<StopLegDanglingState>(sc));
}

// Action: Play motions and head idle
void SittingDemoAction::ActPlayMotionsHI(SittingDemoSC* sc, StateQueue* Q)
{
	// Start background head idling
	Q->append(NewStateInstance<StartHeadIdlingState>(sc));

	// Play some motions
	ActPlayMotions(sc, Q);

	// Stop background head idling
	Q->append(NewStateInstance<StopHeadIdlingState>(sc));
}

// Action: Play motions and leg dangle and head idle
void SittingDemoAction::ActPlayMotionsLDHI(SittingDemoSC* sc, StateQueue* Q)
{
	// TODO: How to have both leg dangling and head idling??
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
inline int randMotion(SittingDemoSC* sc)
{
	// Return a random motion index
	return rand() % sc->m_SD->numMotions();
}
// EOF