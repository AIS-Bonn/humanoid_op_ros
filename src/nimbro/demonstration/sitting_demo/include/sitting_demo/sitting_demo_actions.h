// Actions for the sitting demo motion demonstration state machine
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef SITTING_DEMO_ACTIONS_H
#define SITTING_DEMO_ACTIONS_H

// Defines - Joint names
#define NAME_HEAD_PITCH            "head_pitch"
#define NAME_HEAD_YAW              "neck_yaw"
#define NAME_LEFT_SHOULDER_PITCH   "left_shoulder_pitch"
#define NAME_LEFT_SHOULDER_ROLL    "left_shoulder_roll"
#define NAME_LEFT_ELBOW_PITCH      "left_elbow_pitch"
#define NAME_RIGHT_SHOULDER_PITCH  "right_shoulder_pitch"
#define NAME_RIGHT_SHOULDER_ROLL   "right_shoulder_roll"
#define NAME_RIGHT_ELBOW_PITCH     "right_elbow_pitch"
#define NAME_LEFT_KNEE_PITCH       "left_knee_pitch"
#define NAME_LEFT_ANKLE_PITCH      "left_ankle_pitch"
#define NAME_LEFT_ANKLE_ROLL       "left_ankle_roll"
#define NAME_RIGHT_KNEE_PITCH      "right_knee_pitch"
#define NAME_RIGHT_ANKLE_PITCH     "right_ankle_pitch"
#define NAME_RIGHT_ANKLE_ROLL      "right_ankle_roll"

// Defines - Default seated position
#define DSP_HEAD_PITCH             0.00
#define DSP_HEAD_YAW               0.00
#define DSP_LEFT_SHOULDER_PITCH   -0.26
#define DSP_LEFT_SHOULDER_ROLL     0.00
#define DSP_LEFT_ELBOW_PITCH      -0.74
#define DSP_RIGHT_SHOULDER_PITCH  -0.26
#define DSP_RIGHT_SHOULDER_ROLL    0.00
#define DSP_RIGHT_ELBOW_PITCH     -0.74
#define DSP_LEFT_KNEE_PITCH        0.80
#define DSP_LEFT_ANKLE_PITCH       0.00
#define DSP_LEFT_ANKLE_ROLL        0.00
#define DSP_RIGHT_KNEE_PITCH       0.80
#define DSP_RIGHT_ANKLE_PITCH      0.00
#define DSP_RIGHT_ANKLE_ROLL       0.00

// Defines - Random parameter ranges
#define MIN_NOTHING_DELAY          2.5
#define MAX_NOTHING_DELAY          10.0
#define MIN_LEG_DANGLE_TIME        5.0
#define MAX_LEG_DANGLE_TIME        20.0
#define MIN_HEAD_IDLE_TIME         5.0
#define MAX_HEAD_IDLE_TIME         20.0
#define MIN_POST_MOTION_DELAY      1.0
#define MAX_POST_MOTION_DELAY      4.0
#define MIN_NUM_MOTIONS            1
#define MAX_NUM_MOTIONS            4

// Defines - Misc
#define MAIN_FN_DELAY              3.0   // Time to wait at the beginning of the main function before doing anything
#define INIT_DELAY                 5.0   // Time to wait during initialisation before planning the first action
#define PRE_ACTION_DELAY           1.0   // Time to wait (after cessation of all previous motion) before executing a new action
#define DEFAULT_EFFORT             0.25  // Default effort to use for normal sitting-based motions
#define RETURN_SITTING_VELOCITY    0.60  // Angular velocity with which to return to the default sitting position in the return to sitting state
#define RETURN_SITTING_EFFORT      0.10  // Effort with which to return to the default sitting position in the return to sitting state
#define LIMB_CONTROL_TIME_BIAS     0.05  // Number of seconds to bias limbControlFinishTime by in order to make reasonably sure limb control is really finished when we query it again

// Includes
#include <string>

// Class forward-declarations
namespace statecontroller
{
	class StateQueue;
}

// Sitting demo namespace
namespace sitting_demo
{
	// Class forward-declarations
	class SittingDemoSC;

	// Using declarations
	using statecontroller::StateQueue;

	// Demo actions
	enum DemoAction
	{
		ACT_DO_NOTHING = 0,
		ACT_LEG_DANGLE,
		ACT_HEAD_IDLE,
		ACT_LEG_DANGLE_HEAD_IDLE,
		ACT_PLAY_MOTIONS,
		ACT_PLAY_MOTIONS_LD,
		ACT_PLAY_MOTIONS_HI,
		ACT_PLAY_MOTIONS_LD_HI,
		NUM_ACTIONS
	};

	// String names of demo actions
	const std::string DemoActionName[NUM_ACTIONS + 1] =
	{
		"DO_NOTHING",
		"LEG_DANGLE",
		"HEAD_IDLE",
		"LEG_DANGLE_HEAD_IDLE",
		"PLAY_MOTIONS",
		"PLAY_MOTIONS_LD",
		"PLAY_MOTIONS_HI",
		"PLAY_MOTIONS_LD_HI",
		"NUM_ACTIONS"
	};

	// Sitting demo action class
	class SittingDemoAction
	{
	public:
		// Static action functions
		static void ActDoNothing(SittingDemoSC* sc, StateQueue* Q);
		static void ActLegDangle(SittingDemoSC* sc, StateQueue* Q);
		static void ActHeadIdle(SittingDemoSC* sc, StateQueue* Q);
		static void ActLegDangleHeadIdle(SittingDemoSC* sc, StateQueue* Q);
		static void ActPlayMotions(SittingDemoSC* sc, StateQueue* Q);
		static void ActPlayMotionsLD(SittingDemoSC* sc, StateQueue* Q);
		static void ActPlayMotionsHI(SittingDemoSC* sc, StateQueue* Q);
		static void ActPlayMotionsLDHI(SittingDemoSC* sc, StateQueue* Q);
	};
}

#endif /* SITTING_DEMO_ACTIONS_H */
// EOF