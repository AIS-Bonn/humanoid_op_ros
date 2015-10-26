// Walk and Kick node
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes - ROS
#include <ros/ros.h>
#include <ros/console.h>

// Includes - Other
#include <math.h>
#include <motiontimer.h>
#include <gait/gait_common.h>
#include <nimbro_utils/slope_limiter.h>

// Includes - Local
#include <walk_and_kick/walk_and_kick.h>

// Namespaces
using namespace walkandkick;
using namespace nimbro_utils;
using namespace gait;

// Debugging
// #define DEBUG_FORCE_HALT 1 // Uncomment to disallow walking of the robot
// #define DEBUG_FORCE_NOBALL 1 // Uncomment to ignore all ball detections
#define DEBUG_FORCE_NOGOAL 1 // Uncomment to ignore all goal detections
// #define DEBUG_FORCE_NOPOSE 1 // Uncomment to ignore the localisation output
#define DEBUG_FORCE_NOKICK 1 // Uncomment to disallow kicking of the robot
// #define DEBUG_FORCE_BALLACTION BA_DRIBBLE // Uncomment with BA_KICK or BA_DRIBBLE to force exclusive use of that strategy
// #define DEBUG_SENSORS 1 // Uncomment to debug the input sensor variables
// #define DEBUG_POS 1 // Uncomment to debug positioning
// #define DEBUG_SFB 1 // Uncomment to debug search for ball
#define DEBUG_GBB 1 // Uncomment to debug go behind ball
// #define DEBUG_KB 1 // Uncomment to debug kick ball (i.e. the activation and confirmation of kick)
// #define DEBUG_DB 1 // Uncomment to debug dribble ball (i.e. the activation and deactivation of dribble)
// #define DEBUG_DB_APP 1 // Uncomment to debug dribble ball approach (i.e. GCV once dribble is activated)
// #define DEBUG_WTGP 1 // Uncomment to debug walk to global pose

// Fake globals
#define gKickEnabled false

// General
#define TINC                        0.050     // System iteration time (related to velocities and accelerations)
#define ETERNITY                    100000.0f // A very very long time (e.g. for initialising that the kickoff was not recent)
#define INVALID_DIST                100000.0f // Used to signify an invalid distance
#define MAX_BALL_DIST               12.0f     // Maximum believable reported distance to the ball
#define MIN_GOAL_XDIST              0.30f     // If right in front of the goals then the goal centre starts getting moved back into the goals if you are closer than this
#define MIN_BALL_TO_TARGET_DIST     0.6f      // Minimum allowed distance between the ball and the ball target, to avoid angular and positioning sensitivity
#define FAKE_BALL_TARGET_DIST       2.5f      // The distance in front of the ball that fake ball targets are projected (e.g. for compass ball target, shouldn't make much difference what this value is as long as it's big enough)
#define COMPASS_BALL_TARGET_WEDGE   0.35f     // The wedge angle for the compass ball target
#define GOALIE_WAKEUP_BALLX         2.0f
#define GOALIE_WAKEUP_BALLY         2.0f
#define MAX_GCV_NORM                0.8       // Master limiter of GCV norm

// Walk to global pose
#define WTGP_NEAR_DIST              0.5f
#define WTGP_NEAR_SPEEDXY           0.7f
#define WTGP_NEAR_SPEEDZ            0.7f
#define WTGP_NEAR_ANGLIMIT          0.5f
#define WTGP_FAR_DIST               1.5f
#define WTGP_FAR_SPEEDX             0.7f
#define WTGP_FAR_SPEEDZ             0.7f
#define WTGP_FAR_ANGLIMIT           0.8f
#define WTGP_ANGLE_DEV_COST         2.0f // Convert the cost of angular deviations in the final pose into equivalent distance deviations (dist error = PARAM * angle error)

// Positioning
#define POS_GAZE_FREQ_SCALER        0.5f   // Multiplicative scaler for the frequency of the positioning head scan relative to a nominal frequency calculated from the velocity limit
#define POS_KICKOFF_X               -0.4f  // X position for taking the kickoff if playing on the yellow goal
#define POS_KICKOFF_Y               0.0f   // Y position for taking the kickoff if playing on the yellow goal
#define POS_KICKOFF_Z               0.0f   // Z position for taking the kickoff if playing on the yellow goal
#define POS_DEFEND_X                (-field.circleRadius() - 0.2f)  // X position for defending the kickoff if playing on the yellow goal
#define POS_DEFEND_Y                0.0f   // Y position for defending the kickoff if playing on the yellow goal
#define POS_DEFEND_Z                0.0f   // Z position for defending the kickoff if playing on the yellow goal
#define POS_ARRIVED_THRESHOLD       0.4f
#define POS_ARRIVED_DONE_LIMIT      ((int)3.0f/TINC)

// Search for ball
#define SFB_RESUME_CYCLE_CNT        ((int)5.0f/TINC)
#define SFB_GAZE_MAG_INITIAL        0.2f
#define SFB_GAZE_MAG_SLOPE          0.5f
#define SFB_STAYCOOL_TIME           5.0f
#define SFB_BACKUP_MARGIN           0.5f
#define SFB_BACKUP_MARGIN_EXTRA     1.5f
#define SFB_BACKUP_WALK_TIME        1.5f
#define SFB_BACKUP_WALK_GCVX        -0.4f
#define SFB_BACKUP_WAIT_TIME        1.0f
#define SFB_SPIN_TIME               7.0f
#define SFB_SPIN_GCVZ               0.7f
#define SFB_WALKTOPOSE_DONE_LIMIT   ((int)2.0f/TINC)
#define SFB_WALKTOPOSE_FAIL_LIMIT   ((int)4.0f/TINC)
#define SFB_GOTOCENTRE_RADIUS       0.8f
#define SFB_GOTOCENTRE_TIMEOUT      20.0f
#define SFB_WALKTOMARK_RADIUS       0.8f
#define SFB_WALKTOMARK_TIMEOUT      20.0f
#define SFB_FORWARD_WALK_TIME       6.0f
#define SFB_FORWARD_WALK_GCVX       0.8f
#define SFB_FORWARD_WAIT_TIME       1.0f

// Go behind ball
#define GBB_MIN_REQBALLDIR_LRDIST   0.10f // Minimum distance between required left/right foot poses that is used for normalisation of difference in distance to the left and right behind ball poses
#define GBB_FOOTSEL_BALLPOSE_DBR    (0.4f*field.fieldWidthH()) // Lateral distance from the centre of the field at which the localisation starts to play a role for foot selection
#define GBB_FOOTSEL_LESSDIST_WEIGHT 1.5f // The higher this weight the more the robot chooses the foot with the shorter distance to the behind ball pose
#define GBB_FOOTSEL_BALLPOSE_WEIGHT 1.0f // The higher this weight the more the robot chooses a foot that tends to put the robot in a position between the ball and the goal he is defending
#define GBB_CHANGEFOOT_LIMIT        ((int)2.0f/TINC)
#define GBB_CHANGEFOOT_MINCONF      0.5f
#define GBB_STUCK_MAXBALLDIST       1.0f
#define GBB_STUCK_MAXGCV            0.4f
#define GBB_STUCK_LIMIT             ((int)10.0f/TINC)

// Dribble ball
#define DB_OKTODRIBBLE_COUNT        ((int)0.4f/TINC)
#define DB_STILLOKTODRIBBLE_COUNT   ((int)0.4f/TINC)

// Kick ball
#define KB_OKTOKICK_COUNT           ((int)0.4f/TINC)

//
// Configs
//

// Config variable values
const float WalkAndKick::Config::forceUseFoot = 0.0;
const float WalkAndKick::Config::gazeAngleLimit = 1.0;
const float WalkAndKick::Config::gazeVelLimit = 2.5;
const float WalkAndKick::Config::gazeBallFarDist = 1.0;
const float WalkAndKick::Config::gazeBallFarTs = 0.5;
const float WalkAndKick::Config::gazeBallNearTs = 1.5;
const float WalkAndKick::Config::sfbGazeSplineAccMax = 4.0;
const float WalkAndKick::Config::sfbGazeSplineVelMax = 1.0;
const float WalkAndKick::Config::gbbGcvSWalk = 0.7;
const float WalkAndKick::Config::gbbGcvSTipToe = 0.2;
const float WalkAndKick::Config::gbbGcvSStop = 0.0;
const float WalkAndKick::Config::gbbGcvSReverse = -0.5;
const float WalkAndKick::Config::gbbDesiredSLambda = 0.05;
const float WalkAndKick::Config::gbbDesiredSPhi = 0.3;
const float WalkAndKick::Config::gbbReqBallOffX = 0.18;
const float WalkAndKick::Config::gbbReqBallOffYRK = -0.02;
const float WalkAndKick::Config::gbbReqBallOffYLK = 0.10;
const float WalkAndKick::Config::gbbSlowDownOffX = 0.6;
const float WalkAndKick::Config::gbbSpeedLimit = 0.7;
const float WalkAndKick::Config::gbbAlphaGainZ = 1.5;
const float WalkAndKick::Config::gbbBetaGainY = 4.0;
const float WalkAndKick::Config::dbAppWalkToXDist = 0.3;
const float WalkAndKick::Config::dbAppWalkToYScaler = 1.1;
const float WalkAndKick::Config::dbAppSpeedLimit = 0.7;
const float WalkAndKick::Config::dbAppXYSpeed = 0.7;
const float WalkAndKick::Config::dbAppZGain = 1.5;
const float WalkAndKick::Config::dbAppMaxBallToTargetAngle = 0.6;
const float WalkAndKick::Config::dbTargetAnglePrecision = 0.7;
const float WalkAndKick::Config::dbMinTargetAngleTolerance = 0.16;
const float WalkAndKick::Config::dbBallDistXMax = 2.0;
const float WalkAndKick::Config::dbBallErrorYIwd = 0.07;
const float WalkAndKick::Config::dbBallErrorYOwd = 0.07;
const float WalkAndKick::Config::dbBallSpreadAngle = 0.15;
const float WalkAndKick::Config::dbBallSpreadAngleMore = 0.2;
const float WalkAndKick::Config::dbReqBallOffXExtra = 0.0;
const float WalkAndKick::Config::kbTargetAnglePrecision = 0.7;
const float WalkAndKick::Config::kbMinTargetAngleTolerance = 0.16;
const float WalkAndKick::Config::kbBallErrorXFwd = 0.09;
const float WalkAndKick::Config::kbBallErrorXFwdExtra = 0.05;
const float WalkAndKick::Config::kbBallErrorYIwd = 0.07;
const float WalkAndKick::Config::kbBallErrorYOwd = 0.08;
const float WalkAndKick::Config::kbBallErrorYOwdExtra = 0.02;
const float WalkAndKick::Config::zoneDbCentreXMax = 0.9;
const float WalkAndKick::Config::zoneDbCentreXMin = -0.5;
const float WalkAndKick::Config::zoneDbNearOppGoalXDist = 0.6;

//
// WalkAndKick class
//

// Constructor
WalkAndKick::WalkAndKick()
 : SV(this)
 , m_enable_wak("/walk_and_kick/enable_wak",true)
{
	// Create node handle
	ros::NodeHandle nh("~");

	// ROS subscribers
	m_sub_button = nh.subscribe("/button", 1, &WalkAndKick::handleButtonData, this);
	m_sub_ballvec = nh.subscribe("/vision/ballTarget", 1, &WalkAndKick::handleBallData, this);
	m_sub_goalvec = nh.subscribe("/goal/ego_goal_posts", 1, &WalkAndKick::handleGoalData, this);
	m_sub_robotpose = nh.subscribe("/vision/robotPose", 1, &WalkAndKick::handleRobotPoseData, this);
	m_sub_heading = nh.subscribe("/robotmodel/robot_heading", 1, &WalkAndKick::handleHeadingData, this);
	m_sub_robotstate = nh.subscribe("/robotcontrol/state", 1, &WalkAndKick::handleStateData, this);
	
	// ROS publishers
	m_pub_gaitcmd = nh.advertise<gait_msgs::GaitCommand>("/gaitCommand", 1);
	m_pub_headCmd = nh.advertise<head_control::LookAtTarget>("/robotcontrol/headcontrol/target", 1);
	m_pub_leds = nh.advertise<nimbro_op_interface::LEDCommand>("/led", 1);
	
	// Reset the class
	reset();
}

// Destructor
WalkAndKick::~WalkAndKick()
{
}

// Reset function
void WalkAndKick::reset()
{
	// Reset data variables
	resetDataVariables();
	
	// Reset state machine
	resetVars();
	
	// Reset persistent variables
	cycle = 0;
	targetCycle = 0;
	stateCycle = 0;
	targetTime = 0.0f;
	stateTime = 0.0f;
	lastActFact = 0.0f;
	
	// Send a gait command that we're not supposed to walk
	gait_msgs::GaitCommand cmd;
	cmd.gcvX = 0.0;
	cmd.gcvY = 0.0;
	cmd.gcvZ = 0.0;
	cmd.walk = false;
	m_pub_gaitcmd.publish(cmd);
}

// Initialisation function
bool WalkAndKick::init()
{
	// Return successful initialisation
	return true;
}

// Cycle step function
void WalkAndKick::step()
{
	// Simulate the behaviour management
	float actFact = aktivierungsfunktion();
	if(actFact > 0.0f)
		targetfunction();
	lastActFact = actFact;
}

// Update function for layer variables
void WalkAndKick::updateLayer()
{
	// Update variables
	ReqBallDirLeft.x = config.gbbReqBallOffX;
	ReqBallDirLeft.y = config.gbbReqBallOffYLK;
	ReqBallDirRight.x = config.gbbReqBallOffX;
	ReqBallDirRight.y = config.gbbReqBallOffYRK;
}

// ######################################################################
// #######################     Helper Structs     #######################
// ######################################################################

// Constants
const char WalkAndKick::SensorVars::BTTChar[BTT_COUNT] = { 'U', 'G', 'P', 'C' };

// SensorVars struct
WalkAndKick::SensorVars::SensorVars(WalkAndKick* wak) : wak(wak)
{
	// Update variables
	update();
}
void WalkAndKick::SensorVars::update()
{
	// Convention:
	// The walk and kick coordinate system is: x towards the yellow goal (right one when seen from the bench), y to the left (away from the bench), z a CCW rotation from the yellow goal

	// Retrieve command variables
	Role = ROLE_FIELDPLAYER;
	if(wak->m_button == BTN_HALT)
		Command = CMD_STOP;
	else if(wak->m_button == BTN_PLAY)
		Command = CMD_PLAY;
	else if(wak->m_button == BTN_POS)
		Command = CMD_POSE;
	else if(wak->m_button == BTN_GOALIE)
		Command = CMD_PLAY;
	else
		Command = CMD_STOP;
	PlayOnYellow = true;
	GoalSign = (PlayOnYellow ? +1 : -1);
	PlayAsCyan = false;
	Extern = false;

	// Retrieve robot pose variables
	RobotPose.x = wak->m_robot_pose_vec.x();
	RobotPose.y = wak->m_robot_pose_vec.y();
	RobotPose.z = wak->m_robot_pose_vec.z();
	RobotPoseConf = wak->m_robot_pose_conf;
	CompassHeading = wak->m_heading; // Our compass heading is positive CCW from the positive yellow goal

	// Ignore the robot pose if requires
#ifdef DEBUG_FORCE_NOPOSE
	RobotPose.setZero();
	RobotPoseConf = 0.0f;
#endif

	// Retrieve ball variables
	BallConf = wak->m_ball_conf;
	BallDir.x = wak->m_ball_vec.x();
	BallDir.y = wak->m_ball_vec.y();
	BallAssumedConf = wak->m_ball_conf; // We have nothing better at the moment
	BallAssumedDir.x = wak->m_ball_vec.x();
	BallAssumedDir.y = wak->m_ball_vec.y();

	// Ignore balls if required
#ifdef DEBUG_FORCE_NOBALL
	BallConf = 0.0f;
	BallDir.setZero();
	BallAssumedConf = 0.0f;
	BallAssumedDir.setZero();
#endif

	// Compute dependent ball variables
	BallAngle = atan2(BallDir.y, BallDir.x); // CCW from straight ahead is positive
	BallDist = sqrt(BallDir.x*BallDir.x + BallDir.y*BallDir.y);
	BallPose = RobotPose.getXYVec() + BallDir.rotated(RobotPose.z);
	BallAssumedAngle = atan2(BallAssumedDir.y, BallAssumedDir.x); // CCW from straight ahead is positive
	BallAssumedDist = sqrt(BallAssumedDir.x*BallAssumedDir.x + BallAssumedDir.y*BallAssumedDir.y);
	BallAssumedPose = RobotPose.getXYVec() + BallAssumedDir.rotated(RobotPose.z);

	// Ignore balls that are too far away to be believable
	if (BallDist > MAX_BALL_DIST)
		BallConf = 0.0f;
	if (BallAssumedDist > MAX_BALL_DIST)
		BallAssumedConf = 0.0f;

	// Retrieve goal variables
	GoalConf = wak->m_goal_conf;
	GoalDir.x = wak->m_goal_vec.x();
	GoalDir.y = wak->m_goal_vec.y();
	GoalWedge = 0.5f; // Don't have anything, this is a random value

	// Ignore goals if required
#ifdef DEBUG_FORCE_NOGOAL
	GoalConf = 0.0f;
	GoalDir.setZero();
	GoalWedge = 0.0f;
#endif

	// Compute the ball target
	bool useCompass = true;
	BallTargetType = BTT_UNKNOWN;
	if (GoalConf >= 0.2f)
	{
		// Use the goals if we can see them
		BallTargetConf = GoalConf;
		BallTargetDir = GoalDir;
		BallTargetWedge = GoalWedge;
		BallTargetType = BTT_GOAL;
		useCompass = false;
	}
	else if (RobotPoseConf >= 0.2f)
	{
		// Calculate the goal position and use it if we are localised
		float globalXToGoal = GoalSign*wak->field.fieldLengthH() - RobotPose.x;
		if (GoalSign*globalXToGoal < MIN_GOAL_XDIST && fabs(RobotPose.y) < wak->field.goalWidthH()) // Move the goal centre back into the goals if we are right in front of them or in them
			globalXToGoal = GoalSign * MIN_GOAL_XDIST;
		Vec2f globalVecToGoal(globalXToGoal, -RobotPose.y);
		Vec2f globalVecToPostL(globalXToGoal, GoalSign*wak->field.goalWidthH() - RobotPose.y);
		Vec2f globalVecToPostR(globalXToGoal, -GoalSign*wak->field.goalWidthH() - RobotPose.y);
		Vec2f localVecToGoal = globalVecToGoal.rotated(-RobotPose.z);
		BallTargetConf = RobotPoseConf;
		BallTargetDir = localVecToGoal;
		BallTargetWedge = acos(coerceAbs(globalVecToPostL.getNormalized() * globalVecToPostR.getNormalized(), 1.0));
		BallTargetType = BTT_POSE;
		useCompass = false;
	}
	
	// Sanity check the ball target with the compass
	if (!useCompass)
	{
		BallToTargetAngle = atan2(BallTargetDir.y - BallDir.y, BallTargetDir.x - BallDir.x);
		if (GoalSign*cos(BallToTargetAngle + CompassHeading) < -0.2f)
			useCompass = true; // With two disagreeing sources we would rather trust the compass
	}

	// Use the compass to generate a ball target if required
	if (useCompass)
	{
		// Kick in the direction that the compass says is the direction of the goal
		BallTargetConf = 1.0f;
		BallTargetDir.x = BallDir.x + GoalSign*FAKE_BALL_TARGET_DIST*cos(CompassHeading);
		BallTargetDir.y = BallDir.y - GoalSign*FAKE_BALL_TARGET_DIST*sin(CompassHeading);
		BallTargetWedge = COMPASS_BALL_TARGET_WEDGE;
		BallTargetType = BTT_COMPASS;
	}

	// Make sure that the ball target is not too close to the ball to avoid unnecessary sensitivity that can draw out the positioning
	if ((BallTargetDir - BallDir).norm() < MIN_BALL_TO_TARGET_DIST && BallTargetDir.norm() > BallDir.norm())
		BallTargetDir += MIN_BALL_TO_TARGET_DIST * BallTargetDir.getNormalized();

	// Compute dependent ball target variables
	BallTargetAngle = atan2(BallTargetDir.y, BallTargetDir.x); // CCW from straight ahead is positive
	BallTargetDist = sqrt(BallTargetDir.x*BallTargetDir.x + BallTargetDir.y*BallTargetDir.y);
	BallToTargetAngle = atan2(BallTargetDir.y - BallDir.y, BallTargetDir.x - BallDir.x);
}

// ActuatorVars struct
void WalkAndKick::ActuatorVars::init()
{
	Halt = true;
	GCV.setZero();
	DoKick = false;
	KickFoot = 1;
	GazeAngle = 0.0;
}

// #####################################################################
// #######################     Walk and Kick     #######################
// #####################################################################

// Reset variables
void WalkAndKick::resetVars()
{
	// Deactivate the currently active state
	deactivateState(state);

	// Reset the state cycle counter
	stateCycle = 0;
	stateTime = 0.0f;

	// Deactivate every state for good measure
	for (int stateID = STATE_UNKNOWN; stateID < NUM_STATES; stateID++)
		deactivateState((WAKState) stateID);

	// Reset layer variables
	state = STATE_UNKNOWN;
	nextState = STATE_UNKNOWN;

	// Reset actuator output variables
	AV.init();
	lastAV.init();

	// Reset state machine variables
	dsBallAction = BA_DEFAULT;
	dsKick.reset();
	dsKick.setRange(KB_OKTOKICK_COUNT);
	dsDribble.reset();
	dsDribble.setRange(DB_OKTODRIBBLE_COUNT);
	dsStillDribble.reset();
	dsStillDribble.setRange(DB_STILLOKTODRIBBLE_COUNT);
	dsGoalieAlive = false;

	// Reset search for ball state request
	sfbReqState = SFB_WS_COUNT;
	sfbReqData = 0;
	sfbLastActCycle = -1000000;
	sfbLastActState = SFB_WS_STAYCOOL;
}

// Set whether the behaviour is active or not
float WalkAndKick::setActive(bool active)
{
	// Log activation of walk and kick
	if (targetCycle > 0 && (!active || getActivationFactor() <= 0.0f))
		ROS_WARN("Walk and Kick was just DEACTIVATED!");

	// Reset variables if the walk and kick behaviour was just activated or deactivated
	if (targetCycle > 0 && active != (getActivationFactor() > 0.0f))
	{
		resetVars();
		targetCycle = 0;
		targetTime = 0.0f;
	}

	// Return the required float activation value
	return (active ? 1.0 : 0.0);
}

// Activation function
float WalkAndKick::aktivierungsfunktion()
{
	// Increment the cycle counter
	cycle++;

	// Update the layer variables
	updateLayer();

	// Update the sensor variables
	SV.update();

	// Print the sensor variables state
#ifdef DEBUG_SENSORS
	if (cycle % 20 == 1)
		printf("[%s] %s(%5.2f, %5.2f)  %s(%5.2f, %5.2f)  %s%c(%5.2f, %5.2f)  %s(%5.2f, %5.2f)  %s(%d)  %s %s %s\n", (SV.Command == CMD_PLAY ? "PLAY" : "STOP"), (SV.BallConf >= 0.2f ? "BALL" : "ball"), SV.BallDir.x, SV.BallDir.y, (SV.BallAssumedConf > 0.1f ? "BALLASS" : "ballass"), SV.BallAssumedDir.x, SV.BallAssumedDir.y, (SV.BallTargetConf >= 0.2f ? "BALLTGT" : "balltgt"), SV.BallTargetChar(), SV.BallTargetDir.x, SV.BallTargetDir.y, (SV.RobotPoseConf >= 0.2f ? "POSE" : "pose"), SV.RobotPose.x, SV.RobotPose.y, (SV.PlayOnYellow ? "YELLOW" : "BLUE"), SV.GoalSign, (SV.Extern ? "EXTERN" : "LOCAL"), (SV.PlayAsCyan ? "CYAN" : "MAGENTA"), (SV.Role == ROLE_SOCCER_GOALIE ? "GOALIE" : "FIELDPLAYER"));
#endif

	// Otherwise walk and kick is active
	return setActive(true);
}

// Target function
void WalkAndKick::targetfunction()
{
	// Increment the target cycle counter
	targetCycle++;
	targetTime = targetCycle * TINC;

	// Initialise actuator output variables
	lastAV = AV;
	AV.init();

	// Special handling if the behaviour was just activated
	if (targetCycle <= 1)
	{
		ROS_WARN("Walk and Kick was just ACTIVATED!");
		resetVars();
		lastAV.GazeAngle = 0.0;
		lastAV.Halt = false;
		lastAV.GCV.setZero();
	}
	
	// Evaluate our state machine to decide on a next state (written into 'state')
	WAKState oldState = state;
	decideState();
	bool stateChanged = (state != oldState);

	// Execute state change actions if the state changed
	if (stateChanged)
	{
		// Execute callback for the state that just deactivated
		deactivateState(oldState);

		// Log behaviour state
		ROS_WARN("State is now --> %s", getStateName(state));
		
		// Reset the state cycle counter
		stateCycle = 0;
		stateTime = 0.0f;

		// Execute callback for the state that just activated
		activateState(state);
	}

	// Increment state cycle counter
	stateCycle++;
	stateTime = stateCycle * TINC;

	// Execute the currently active state
	executeState(state, stateChanged);

	// Write to our actuators
	writeActuators(AV);
}

// #####################################################################
// #######################     State Machine     #######################
// #####################################################################

// Decide on the next state
bool WalkAndKick::decideState()
{
	// Remember our old state
	WAKState oldState = state;
	if (oldState <= STATE_UNKNOWN || oldState >= NUM_STATES)
		oldState = STATE_UNKNOWN;

	// If a next state was requested by the current state then try to respect that
	if (nextState != STATE_UNKNOWN)
		state = nextState; // Note: This feature is currently not used, as the state gets overwritten below in any case
	nextState = STATE_UNKNOWN;

	// Enter a default next state if it is currently unknown
	if (state <= STATE_UNKNOWN || state >= NUM_STATES)
		state = STATE_SEARCH_FOR_BALL;

	// Update the kick and dribble worms
	bool OkToKick = okToKick();
	bool OkToDribble = okToDribble();
	bool OkToDribbleStill = stillOkToDribble();
	dsKick.vote(OkToKick);
	dsDribble.vote(OkToDribble);
	dsStillDribble.vote(OkToDribble || OkToDribbleStill);

	// Work out whether we can kick and/or dribble at this very moment
	bool canKickNow = dsKick.unanimousTrue();
	bool canDribbleNow = (dsDribble.unanimousTrue() || (oldState == STATE_DRIBBLE_BALL && !dsStillDribble.unanimousFalse()));

	// Unset the dribbling lock if we can't dribble right now
	if (!canDribbleNow)
		dbLock = false;

	// Decide which ball actions are allowed in the current state
	bool allowDribble = true;
	bool allowKick = true;
	if (SV.BallConf >= 0.2f && SV.RobotPoseConf >= 0.2f)
	{
		if (SV.GoalSign*SV.BallPose.x > config.zoneDbCentreXMin && SV.GoalSign*SV.BallPose.x < config.zoneDbCentreXMax) // Dribble zone near the centre line
			allowKick = false;
		else if (field.fieldLengthH() - SV.GoalSign*SV.BallPose.x < config.zoneDbNearOppGoalXDist) // Dribble zone near the goal you score in
			allowKick = false;
	}
	if (dbLock)
		allowKick = false;

	// Handle kick and dribble bans (a ban is unbreakable, even if GBB is trying to suggest otherwise)
	bool banKick = (!gKickEnabled);
	bool banDribble = false;
	if (banKick)
		allowKick = false;
	if (banDribble)
		allowDribble = false;

	// Decide on a ball action for when we are at the ball (we are not necessarily there yet, but we may approach the ball differently depending on what we wish to do with it)
	if (gbbBallActionTip != BA_COUNT && !(banKick && gbbBallActionTip == BA_KICK) && !(banDribble && gbbBallActionTip == BA_DRIBBLE))
		dsBallAction = gbbBallActionTip; // Used by GBB to get out of stuck situations
	else if (allowKick)
		dsBallAction = BA_KICK;
	else if (allowDribble)
		dsBallAction = BA_DRIBBLE;
	else
		dsBallAction = BA_DRIBBLE; // We don't have another choice anyway
#ifdef DEBUG_FORCE_BALLACTION
	dsBallAction = DEBUG_FORCE_BALLACTION;
#endif

	// Decide on the state we want to be in
	if (SV.BallConf < 0.2f)
		state = STATE_SEARCH_FOR_BALL;
	else if (dsBallAction == BA_KICK && canKickNow)
		state = STATE_KICK_BALL;
	else if (dsBallAction == BA_DRIBBLE && canDribbleNow)
		state = STATE_DRIBBLE_BALL;
	else
		state = STATE_GO_BEHIND_BALL; // This takes dsBallAction into consideration to try to end in a suitable position for the attempted action
	
	// If playing as a goalie then don't move until the situation gets hot
	if(m_button == BTN_GOALIE && SV.BallConf >= 0.2f && SV.BallDir.x < GOALIE_WAKEUP_BALLX && fabs(SV.BallDir.y) < GOALIE_WAKEUP_BALLY)
		dsGoalieAlive = true;
	if(m_button != BTN_GOALIE)
		dsGoalieAlive = false;

	// Special commands take priority over everything else
	if (SV.Command == CMD_POSE)
		state = STATE_POSITIONING;
	else if (SV.Command != CMD_PLAY)
		state = STATE_STOPPED;
	
	// Return whether the state was changed by this function
	return (state != oldState);
}

// #####################################################################
// ##########################     Stopped     ##########################
// #####################################################################

// State change function
void WalkAndKick::changedSTP(bool nowActive)
{
}

// State execution functions
void WalkAndKick::executeSTP(bool justActivated)
{
	// Ensure we are halted and looking straight ahead
	AV.Halt = true;
	AV.GCV.setZero();
	AV.GazeAngle = 0.0f;
}

// #####################################################################
// ########################     Positioning     ########################
// #####################################################################

// State change function
void WalkAndKick::changedPOS(bool nowActive)
{
	// Reset variables
	posGazeFreq = 0.0f;
	posGazePhaseOff = 0.0f;
	posArrived = false;
	posDoneCounter.reset();
}

// State execution functions
void WalkAndKick::executePOS(bool justActivated)
{
	//
	// Gaze control
	//

	// Calculate the gaze oscillation parameters if we haven't yet
	if (posGazeFreq <= 0.0f)
	{
		posGazeFreq = coerce(POS_GAZE_FREQ_SCALER * config.gazeVelLimit / config.gazeAngleLimit, 0.5, 5.0);
		posGazePhaseOff = asin(coerceAbs(lastAV.GazeAngle / config.gazeAngleLimit, 1.0));
	}

	// Look around as much as possible to try to localise as best possible
	AV.GazeAngle = config.gazeAngleLimit * sin(posGazeFreq*stateTime + posGazePhaseOff);
	
	// No positioning head movements for a static goalie
	if(m_button == BTN_GOALIE && !dsGoalieAlive)
		AV.GazeAngle = 0.0;

	//
	// Walking control
	//

	// We wish to walk if we have not arrived yet
	AV.Halt = posArrived;

	// Calculate the required global pose to walk to
	Vec3f target;
	if (false) // Should be SV.HaveKickoff
	{
		target.x = SV.GoalSign * POS_KICKOFF_X;
		target.y = SV.GoalSign * POS_KICKOFF_Y;
		target.z = picut(POS_KICKOFF_Z + (SV.GoalSign > 0 ? 0.0 : M_PI));
	}
	else
	{
		target.x = SV.GoalSign * POS_DEFEND_X;
		target.y = SV.GoalSign * POS_DEFEND_Y;
		target.z = picut(POS_DEFEND_Z + (SV.GoalSign > 0 ? 0.0 : M_PI));
	}

	// Set a GCV suitable for walking to the target pose
	float dist = walkToGlobalPose(target.x, target.y, target.z);

	// Check whether we have arrived at our destination
	posDoneCounter.add(dist < POS_ARRIVED_THRESHOLD);
	if (posDoneCounter.reached(POS_ARRIVED_DONE_LIMIT))
		posArrived = true;

	// Print info about the positioning state
#ifdef DEBUG_POS
	if (stateCycle % 20 == 1)
		printf("POS: %s TARGET(%.2f, %.2f, %.2f) %s(%.2f, %.2f, %.2f) %s(%.2f) GAZE(%.2f) COUNT(%d) GCV(%.2f, %.2f, %.2f)\n", (false ? "KICKOFF" : "DEFEND"), target.x, target.y, target.z, (SV.RobotPoseConf >= 0.2f ? "POSE" : "pose"), SV.RobotPose.x, SV.RobotPose.y, SV.RobotPose.z, (dist < POS_ARRIVED_THRESHOLD ? "DIST" : "dist"), dist, AV.GazeAngle, posDoneCounter.count(), AV.GCV.x, AV.GCV.y, AV.GCV.z);
#endif
}

// #####################################################################
// ######################     Search for Ball     ######################
// #####################################################################

// State change function
void WalkAndKick::changedSFB(bool nowActive)
{
	// Reset variables
	sfbGazeSpline.reset();
	sfbGazeSpline.setState(lastAV.GazeAngle, 0.0);
	if (nowActive && sfbReqState != SFB_WS_COUNT)
	{
		changeSfbState(sfbReqState);
		if (sfbReqState == SFB_WS_WALKFWDS)
			sfbWalkFwdTime = SFB_FORWARD_WALK_TIME * sfbReqData;
		sfbReqState = SFB_WS_COUNT;
		sfbReqData = 0;
	}
	else
		changeSfbState(SFB_WS_STAYCOOL);
	sfbWSTime = 0.0;
}

// Search for ball state change function
void WalkAndKick::changeSfbState(SFBWalkState newSfbState)
{
	sfbWalkState = newSfbState;
	sfbWSTime = stateTime;
	sfbSpinDirn = 0;
	sfbFactor = -1.0f;
	sfbDoneCounter.reset();
	sfbFailCounter.reset();
	sfbWalkFwdTime = SFB_FORWARD_WALK_TIME;
}

// Search for ball initial state request function
void WalkAndKick::requestSfbState(SFBWalkState state, int data)
{
	sfbReqState = state;
	sfbReqData = data;
}

// Execution function
void WalkAndKick::executeSFB(bool justActivated)
{
	//
	// Gaze control
	//

	// Desired behaviour:
	// - If we have an assumed ball position then center the ball search around that if the state is SFB_WS_STAYCOOL or SFB_WS_BACKUP
	// - Otherwise center the ball search about the zero position and go limit to limit

	// Recalculate the gaze spline if required
	if (sfbGazeSpline.finished())
	{
		// Decide on a gaze angle to center the search about
		float gazeAngleGuess = 0.0f;
		if (SV.BallAssumedConf > 0.1f && (sfbWalkState == SFB_WS_STAYCOOL || sfbWalkState == SFB_WS_BACKUP))
			gazeAngleGuess = SV.BallAssumedAngle;
		gazeAngleGuess = coerceAbs(gazeAngleGuess, 0.98f*config.gazeAngleLimit); // The scale factor is to ensure that the spline curX can numerically reach both sides of the gaze angle guess (otherwise we risk getting stuck at a limit)

		// Calculate a next gaze angle target
		float gazeAngleMag = coerceAbs(SFB_GAZE_MAG_INITIAL + SFB_GAZE_MAG_SLOPE*stateTime, 2.0f*config.gazeAngleLimit);
		float angleTarget = coerceAbs(gazeAngleGuess + sign(gazeAngleGuess - sfbGazeSpline.curX())*gazeAngleMag, config.gazeAngleLimit);

		// Update the gaze spline
		sfbGazeSpline.newTarget(angleTarget, 0.0, config.sfbGazeSplineVelMax, config.sfbGazeSplineAccMax, true);

	}

	// Evaluate the gaze spline
	AV.GazeAngle = sfbGazeSpline.forward(TINC);
	
	// No search for ball head movements for a static goalie
	if(m_button == BTN_GOALIE && !dsGoalieAlive)
		AV.GazeAngle = 0.0;

	//
	// Walking control
	//

	// Desired behaviour:
	// - INIT: In the first few seconds just walk or not (depending on whether you already were or not) on the spot
	// - THEN: Try walking backwards a short distance (sanity check with the localisation that that won't make us walk off the field) and wait a few seconds
	// - THEN: Turn on the spot for a larger number of seconds to look around (choose the direction once at the start based on assumed ball direction (if conf > 0.1), or if not available, turn in the direction that most quickly brings the goals you are playing on into view)
	// - THEN: Walk to the center circle
	// - THEN: Turn on the spot there for some time (choose the direction once at the start that most quickly brings your goals into view according to the localisation)
	// - THEN: Walk to the penalty mark that is on the other half of the field and go back to the previous spin state

	// By default we walk with a zero GCV
	AV.GCV.setZero();

	// Calculate how much time has passed since the current SFB state was activated
	float elapsed = stateTime - sfbWSTime;
	
	// Go back to the old search for ball state if not much time has passed since it was active
	if (justActivated && (cycle - sfbLastActCycle < SFB_RESUME_CYCLE_CNT))
		sfbWalkState = sfbLastActState;
	
	// Make a note of the SFB walk state
	sfbLastActState = sfbWalkState;
	sfbLastActCycle = cycle;

	// Decide on where to walk to find the ball
	if (sfbWalkState == SFB_WS_STAYCOOL)
	{
		// Keep the current walking state while staying cool to avoid unnecesarily toggling the walking state
		AV.Halt = lastAV.Halt;

		// If enough time has elapsed then try something else
		if (elapsed >= SFB_STAYCOOL_TIME)
			changeSfbState(SFB_WS_BACKUP);
	}
	else if (sfbWalkState == SFB_WS_BACKUP)
	{
		// Backing up makes no sense if we just started walking
		if (lastAV.Halt)
			changeSfbState(SFB_WS_SPIN);

		// We wish to walk
		AV.Halt = false;

		// Choose to what extent to back up (if we haven't already)
		if (sfbFactor < 0.0f)
		{
			if (SV.RobotPoseConf < 0.2f)
				sfbFactor = 0.5f;
			else
			{
				float sideFactor = coerce((field.fieldWidthH() - fabs(SV.RobotPose.y)) / SFB_BACKUP_MARGIN, 0.0, 1.0); // Drops to zero the the side edges of the field
				float oppGoalFactor = coerce((field.fieldLengthH() + SV.GoalSign*SV.RobotPose.x) / SFB_BACKUP_MARGIN, 0.0, 1.0); // Drops to zero near the opponent's goal (the one we score in)
				float ourGoalFactor = coerce((field.fieldLengthH() - SV.GoalSign*SV.RobotPose.x) / SFB_BACKUP_MARGIN_EXTRA, 0.0, 1.0); // Drops to zero near our goal (the one we're protecting, we use an extra margin to make sure we don't reverse the ball into our own goal by accident)
				sfbFactor = std::min(std::min(sideFactor, oppGoalFactor), ourGoalFactor); // Should be in range 0.0 -> 1.0
			}
		}

		// Walk backwards for a short amount of time
		if (elapsed < sfbFactor * SFB_BACKUP_WALK_TIME)
			AV.GCV.x = SFB_BACKUP_WALK_GCVX;
		else
			AV.GCV.x = 0.0;

		// If enough time has elapsed then try something else
		if (elapsed >= sfbFactor * SFB_BACKUP_WALK_TIME + SFB_BACKUP_WAIT_TIME)
			changeSfbState(SFB_WS_SPIN);
	}
	else if (sfbWalkState == SFB_WS_SPIN)
	{
		// We wish to walk
		AV.Halt = false;

		// Choose a direction to spin (if we haven't already) where +1 is CCW and -1 is CW
		if (sfbSpinDirn == 0)
		{
			if (SV.BallAssumedConf > 0.1f)
				sfbSpinDirn = sign(SV.BallAssumedDir.y); // Turn in the direction of the assumed ball
			else
				sfbSpinDirn = sign(-SV.GoalSign * sin(SV.RobotPose.z)); // Turn in the direction that most quickly brings the opponent's goals into view according to the localisation
		}

		// Command turning on the spot
		AV.GCV.z = sfbSpinDirn * SFB_SPIN_GCVZ;

		// If enough time has elapsed then try something else
		if (elapsed >= SFB_SPIN_TIME)
			changeSfbState(SFB_WS_GOTOCENTRE);
	}
	else if (sfbWalkState == SFB_WS_GOTOCENTRE)
	{
		// We wish to walk
		AV.Halt = false;

		// Set a GCV suitable for walking to the centre circle
		float dist = walkToGlobalPose(0.0f, 0.0f);

		// Handle the case where walk to global pose failed
		sfbFailCounter.add(dist == INVALID_DIST);
		if (sfbFailCounter.reached(SFB_WALKTOPOSE_FAIL_LIMIT))
			changeSfbState(SFB_WS_SPIN);

		// Go back to spinning on the spot if this is taking too long
		if (elapsed >= SFB_GOTOCENTRE_TIMEOUT)
			changeSfbState(SFB_WS_SPIN);

		// Check whether we have arrived at our destination
		sfbDoneCounter.add(dist < SFB_GOTOCENTRE_RADIUS);
		if (sfbDoneCounter.reached(SFB_WALKTOPOSE_DONE_LIMIT))
			changeSfbState(SFB_WS_SPINHERE);
	}
	else if (sfbWalkState == SFB_WS_SPINHERE)
	{
		// We wish to walk
		AV.Halt = false;

		// Choose a direction to spin if we don't have one already
		if (sfbSpinDirn == 0)
			sfbSpinDirn = sign(SV.GoalSign * sin(SV.RobotPose.z)); // Turn in the direction that most quickly brings our goals into view according to the localisation

		// Command turning on the spot
		AV.GCV.z = sfbSpinDirn * SFB_SPIN_GCVZ;

		// If enough time has elapsed then try something else
		if (elapsed >= SFB_SPIN_TIME)
			changeSfbState(SFB_WS_WALKTOMARK);
	}
	else if (sfbWalkState == SFB_WS_WALKTOMARK)
	{
		// We wish to walk
		AV.Halt = false;

		// Choose which penalty mark we wish to walk to (if we haven't already)
		if (sfbSpinDirn == 0)
		{
			if (SV.RobotPoseConf >= 0.2f)
				sfbSpinDirn = sign(-(SV.RobotPose.x - 2.0f*SV.GoalSign)); // Walk to the penalty mark that is not in the half of the field we're currently in
			else
				sfbSpinDirn = SV.GoalSign; // Walk to the penalty mark next to the opponent's goal
		}

		// Set a GCV suitable for walking to the required penalty mark
		float dist = walkToGlobalPose(sfbSpinDirn * (field.fieldLengthH() - field.penaltyMarkDist()), 0.0f);

		// Handle the case where walk to global pose failed
		sfbFailCounter.add(dist == INVALID_DIST);
		if (sfbFailCounter.reached(SFB_WALKTOPOSE_FAIL_LIMIT))
			changeSfbState(SFB_WS_SPIN);

		// Go back to spinning on the spot if this is taking too long
		if (elapsed >= SFB_WALKTOMARK_TIMEOUT)
			changeSfbState(SFB_WS_SPIN);

		// Check whether we have arrived at our destination
		sfbDoneCounter.add(dist < SFB_WALKTOMARK_RADIUS);
		if (sfbDoneCounter.reached(SFB_WALKTOPOSE_DONE_LIMIT))
			changeSfbState(SFB_WS_SPINHERE);
	}
	else if (sfbWalkState == SFB_WS_WALKFWDS)
	{
		// We wish to walk
		AV.Halt = false;

		// Walk forwards for a given amount of time
		if (elapsed < sfbWalkFwdTime)
			AV.GCV.x = SFB_FORWARD_WALK_GCVX;
		else
			AV.GCV.x = 0.0;

		// If enough time has elapsed then try something else
		if (elapsed >= sfbWalkFwdTime + SFB_FORWARD_WAIT_TIME)
			changeSfbState(SFB_WS_SPIN);
	}
	else // Should never happen...
	{
		// Keep the current walking state to avoid unnecesarily toggling the walking state
		AV.Halt = lastAV.Halt;

		// Change to the default initial search state
		changeSfbState(SFB_WS_STAYCOOL);
	}

	// Print info about the search for ball state
#ifdef DEBUG_SFB
	if (stateCycle % 20 == 1)
		printf("SFB: State %d, %.3f elapsed, GCV %.2f %.2f %.2f, Halt %d\n", sfbWalkState, elapsed, AV.GCV.x, AV.GCV.y, AV.GCV.z, AV.Halt);
#endif
}

// ####################################################################
// ######################     Go Behind Ball     ######################
// ####################################################################

// State change function
void WalkAndKick::changedGBB(bool nowActive)
{
	// Reset variables
	gbbUseRightFoot = true;
	gbbChangeToFoot.reset(GBB_CHANGEFOOT_LIMIT);
	gbbStuck.reset();
	gbbBallActionTip = BA_COUNT;
}

// State execution functions
void WalkAndKick::executeGBB(bool justActivated)
{
	// Note: This behaviour assumes that BallConf >= 0.2f. It does not assume however that BallTargetConf >= 0.2f.
	// TODO: Obstacle avoidance

	//
	// Gaze control
	//

	// Look at the ball
	gazeAtBall();

	//
	// Walking control
	//

	// Required gait control vector
	Vec3f GCV(0.0, 0.0, 0.0); // x forwards, y left, z CCW

	// Save the relative offsets to the ball and target in local variables
	Vec2f ball = SV.BallDir;         // Vector from robot to ball in body-fixed coordinates (see also the use of SV.BallPose etc below)
	Vec2f target = SV.BallTargetDir; // Vector from robot to target in body-fixed coordinates
	if (SV.BallTargetConf < 0.2f)    // If we don't have a target then take one that's right in front of us (further down we also disable beta control)
	{
		target.x = FAKE_BALL_TARGET_DIST;
		target.y = 0.0f;
	}

	// Calculate the vector from the ball to the target in body-fixed coordinates
	Vec2f E = target - ball;
	Vec2f unitEx = E.getNormalized();
	Vec2f unitEy = unitEx.rotatedCCW90();

	// Retrieve the required ball dir vectors for kicking
	Vec2f usedRBDLeft = ReqBallDirLeft;
	Vec2f usedRBDRight = ReqBallDirRight;

	// Adjust the required ball dir vectors if our aim is to dribble (want to be further back so that we can pick up speed initially)
	if (dsBallAction == BA_DRIBBLE)
	{
		float angle = acos(coerceAbs(unitEx * ball.getNormalized(), 1.0)); // Note: Dot product
		float extraRBDX = config.dbReqBallOffXExtra * coerce(1.0 - angle/M_PI_2, 0.0, 1.0);
		usedRBDLeft.x += extraRBDX;
		usedRBDRight.x += extraRBDX;
	}

	// Calculate the distance between the two possible behind ball poses
	float ReqBallDirDistLR = (usedRBDLeft - usedRBDRight).norm();

	// Calculate the distance to the two possible behind ball poses from the robot
	float DistToReqLeft = (ball - usedRBDLeft.x*unitEx - usedRBDLeft.y*unitEy).norm();
	float DistToReqRight = (ball - usedRBDRight.x*unitEx - usedRBDRight.y*unitEy).norm();
	float footDueToLessDist = coerceAbs((DistToReqLeft - DistToReqRight) / coerceMin(ReqBallDirDistLR, GBB_MIN_REQBALLDIR_LRDIST), 1.0); // 1 = Definitely use right foot, -1 = Definitely use left foot

	// Decide whether we should reconsider the foot we're currently trying to line up
	if (footDueToLessDist > GBB_CHANGEFOOT_MINCONF) // The right foot would be better right now
		gbbChangeToFoot.vote(true);
	if (footDueToLessDist < -GBB_CHANGEFOOT_MINCONF) // The left foot would be better right now
		gbbChangeToFoot.vote(false);
	bool reconsiderFoot = (gbbChangeToFoot.unanimous() && gbbChangeToFoot.decision() != gbbUseRightFoot);

	// Decide which foot to line up with the ball and the ball target
	if (justActivated || reconsiderFoot)
	{
		// See which foot the ball pose suggests (try to use the outer foot near the sides of the field)
		float footDueToBallPose = 0.0f;
		if (SV.RobotPoseConf >= 0.2f)
			footDueToBallPose = -SV.GoalSign * sign(SV.BallPose.y) * coerce(fabs(SV.BallPose.y) - GBB_FOOTSEL_BALLPOSE_DBR, 0.0, 1.0); // 1 = Definitely use right foot, -1 = Definitely use left foot

		// Decide on a foot to line up the ball with
		gbbUseRightFoot = ((GBB_FOOTSEL_BALLPOSE_WEIGHT*footDueToBallPose + GBB_FOOTSEL_LESSDIST_WEIGHT*footDueToLessDist) >= 0.0f);
	}

	// Force the use of a particular foot if the config commands it
	if (config.forceUseFoot > 0.0f)
		gbbUseRightFoot = true;
	if (config.forceUseFoot < 0.0f)
		gbbUseRightFoot = false;

	// Target relative offset from the robot to the ball that corresponds to good positioning behind the ball
	Vec2f ReqBallDir = (gbbUseRightFoot ? usedRBDRight : usedRBDLeft);

	// Calculate the length squared of the offset ray vector
	float BLen_sq = ball.x*ball.x + ball.y*ball.y;
	float SLen_sq = BLen_sq - ReqBallDir.y*ReqBallDir.y;

	// Calculate the required GCV, ensuring that we avoid roots of negative numbers if the ball is too close
	float alpha, beta, SLen;
	Vec3f PolarGCV(0.0, 0.0, 0.0); // x is in the direction of S (positive towards the ball), y is in the direction perp to S (positive left from the direction to the ball), z is CCW rotation
	if (SLen_sq > 0.0f)
	{
		// Calculate the offset ray angle alpha (CCW angle from the body-fixed x direction to the angle of the S vector)
		SLen = sqrt(SLen_sq);
		alpha = atan2(SLen*ball.y - ball.x*ReqBallDir.y, SLen*ball.x + ball.y*ReqBallDir.y);

		// Calculate the offset ray to goal angle beta (CCW angle from the S vector to the ball to target vector)
		float Sx = SLen * cos(alpha);
		float Sy = SLen * sin(alpha);
		beta = atan2(Sx*E.y - Sy*E.x, Sx*E.x + Sy*E.y);

		// If we don't actually have a proper target then disable beta control
		if (SV.BallTargetConf < 0.2f)
			beta = 0.0f;

		// Calculate the desired S distance to the ball (lambda is how much the desired S distance gets increased near the left/right sides of the robot to avoid running into the ball, phi is a trimming factor to make it symmetrical, refer to calcBallOffX.m)
		float sinalphi = sin(alpha + config.gbbDesiredSPhi); // TODO: Automatic calculation of a suitable config.gbbDesiredSPhi based on ReqBallDir.x and config.gbbDesiredSLambda (HARD, but should be possible)
		float sinphi = sin(config.gbbDesiredSPhi);
		float desiredS = ReqBallDir.x + config.gbbDesiredSLambda*(sinalphi*sinalphi - sinphi*sinphi);

		// Decide on an appropriate action in polar coordinates
		if (SLen > desiredS + config.gbbSlowDownOffX)
		{
			// Far enough away, focus on turning towards the ball and walking towards it
			PolarGCV.x = config.gbbGcvSWalk;
			PolarGCV.y = 0.0f;
			PolarGCV.z = config.gbbAlphaGainZ * alpha;
		}
		else if (SLen > desiredS)
		{
			// Getting closer to the ball, so start backing off the forwards speed
			float u = (SLen - desiredS) / config.gbbSlowDownOffX;
			PolarGCV.x = config.gbbGcvSTipToe + u * (config.gbbGcvSWalk - config.gbbGcvSTipToe); // Ramp down x velocity for distance control
			PolarGCV.y = (1 - u) * (config.gbbBetaGainY * -beta); // Ramp up y velocity for beta control
			PolarGCV.z = config.gbbAlphaGainZ * alpha; // Alpha control
		}
		else
		{
			// Nice and close to the ball, so perform remaining fine adjustments and make sure you don't run into the ball
			float u = SLen / desiredS;
			PolarGCV.x = config.gbbGcvSReverse + u * (config.gbbGcvSStop - config.gbbGcvSReverse); // Keep distance the same
			PolarGCV.y = config.gbbBetaGainY * -beta; // Beta control
			PolarGCV.z = config.gbbAlphaGainZ * alpha; // Alpha control
		}
	}
	else // Ball is closer to the robot than the abs value of the desired ball Y offset...
	{
		// Define alpha as the angle directly to the ball in this case
		alpha = atan2(ball.y, ball.x);

		// Try to back away from the ball and turn towards it (you're probably just going to kick it away by accident anyway)
		PolarGCV.x = config.gbbGcvSReverse; // Back away from the ball
		PolarGCV.y = 0.0f;
		PolarGCV.z = config.gbbAlphaGainZ * alpha; // Alpha control
	}

	// Convert polar coordinates to the desired GCV
	Vec2f LinVel = PolarGCV.getXYVec().rotated(alpha);
	GCV.x = LinVel.x;
	GCV.y = LinVel.y;
	GCV.z = PolarGCV.z;

	// Normalise the GCV to our desired maximum walking speed
	float gcvNorm = GCV.norm();
	float speedLimit = fabs(config.gbbSpeedLimit);
	if (gcvNorm > speedLimit)
		GCV *= speedLimit / gcvNorm;

	// Detect whether go behind ball is somehow stuck
	bool ballClose = (SV.BallDist < GBB_STUCK_MAXBALLDIST);
	bool nearZeroGCV = (GCV.norm() < GBB_STUCK_MAXGCV);
	gbbStuck.add(ballClose && nearZeroGCV);
	if (gbbStuck.reached(GBB_STUCK_LIMIT))
	{
		if (dsBallAction == BA_DRIBBLE)
			gbbBallActionTip = BA_KICK;
		else
			gbbBallActionTip = BA_DRIBBLE;
		gbbStuck.reset();
	}

	// Set our walking GCV
	AV.GCV = GCV;
	AV.Halt = false;

	// Print info about the go behind ball state
#ifdef DEBUG_GBB
	if (stateCycle % 20 == 1)
		printf("GBB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), GCV(%.2f, %.2f, %.2f) FOR %s\n", (SV.BallConf >= 0.2f ? "BALL" : "ball"), ball.x, ball.y, (SV.BallTargetConf >= 0.2f ? "BALLTGT" : "balltgt"), SV.BallTargetChar(), target.x, target.y, AV.GCV.x, AV.GCV.y, AV.GCV.z, (dsBallAction == BA_DRIBBLE ? "DRIBBLE" : "KICK"));
#endif
}

// ####################################################################
// #######################     Dribble Ball     #######################
// ####################################################################

// State change function
void WalkAndKick::changedDB(bool nowActive)
{
	// Reset variables
	dbLock = nowActive;
}

// Decide whether we are in a position to dribble
bool WalkAndKick::okToDribble() const
{
	// Don't dribble if we don't have a ball or ball target to dribble to
	if (SV.BallConf < 0.2f || SV.BallTargetConf < 0.2f)
		return false;

	// Calculate whether we are facing the target
	float angularTolerance = coerceMin(fabs(config.dbTargetAnglePrecision*0.5f*SV.BallTargetWedge), config.dbMinTargetAngleTolerance); // Note: The wedge angle is the complete angular range from the left side of the target to the right side...
	bool facingTarget = (fabs(SV.BallToTargetAngle) < angularTolerance); // The ball target is in front of the ball from our perspective (within an angular tolerance dependent on how wide the target is)

	// Calculate whether the ball is suitably in front of our left or right foot (allowed y deviation increases linearly with distance)
	Vec2f ballErrorLeft = SV.BallDir - ReqBallDirLeft;
	Vec2f ballErrorRight = SV.BallDir - ReqBallDirRight;
	float spreadSlope = tan(config.dbBallSpreadAngle);
	float leftSpread = coerceMin(ballErrorLeft.x*spreadSlope, 0.0);
	float rightSpread = coerceMin(ballErrorRight.x*spreadSlope, 0.0);
	bool leftFootInPosition = (SV.BallDir.x > 0.0f && SV.BallDir.x < config.dbBallDistXMax && (ballErrorLeft.y < config.dbBallErrorYOwd + leftSpread) && (ballErrorLeft.y > -config.dbBallErrorYIwd - leftSpread));
	bool rightFootInPosition = (SV.BallDir.x > 0.0f && SV.BallDir.x < config.dbBallDistXMax && (ballErrorRight.y < config.dbBallErrorYIwd + rightSpread) && (ballErrorRight.y > -config.dbBallErrorYOwd - rightSpread));

	// Decide whether we are in the position to dribble
	bool inDribblePosition = facingTarget && (leftFootInPosition || rightFootInPosition);

	// Print information about checking whether we can dribble
#ifdef DEBUG_DB
	if (stateCycle % 20 == 1 && state != STATE_DRIBBLE_BALL)
		printf("DB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f), COMPASS(%.2f), %s, %s\n", (SV.BallConf >= 0.2f ? "BALL" : "ball"), SV.BallDir.x, SV.BallDir.y, (SV.BallTargetConf >= 0.2f ? "BALLTGT" : "balltgt"), SV.BallTargetChar(), SV.BallTargetDir.x, SV.BallTargetDir.y, (leftFootInPosition ? "ERR_LEFT" : "err_left"), ballErrorLeft.x, ballErrorLeft.y, (rightFootInPosition ? "ERR_RIGHT" : "err_right"), ballErrorRight.x, ballErrorRight.y, (facingTarget ? "ANGLE" : "angle"), SV.BallToTargetAngle, SV.CompassHeading, (facingTarget ? "AIMED" : "not aimed"), (inDribblePosition ? "CAN DRIBBLE" : "no"));
#endif

	// Return whether we can dribble
	return inDribblePosition;
}

// Decide whether we are still in good dribbling form, assuming that okToDribble() returned true in the recent past
bool WalkAndKick::stillOkToDribble() const
{
	// Not ok if we don't have a ball or ball target to dribble to
	if (SV.BallConf < 0.2f || SV.BallTargetConf < 0.2f)
		return false;

	// Calculate whether we are very approximately still facing the target
	bool stillFacingTarget = (fabs(SV.BallToTargetAngle) < config.dbAppMaxBallToTargetAngle);

	// Calculate the ball position errors from the standard kicking position (not a typo)
	Vec2f ballErrorLeft = SV.BallDir - ReqBallDirLeft;
	Vec2f ballErrorRight = SV.BallDir - ReqBallDirRight;

	// Calculate the allowed ball spread (allowed y deviation increases linearly with distance)
	float spreadSlopeMore = tan(config.dbBallSpreadAngleMore);
	float leftSpread = coerceMin(ballErrorLeft.x*spreadSlopeMore, 0.0);
	float rightSpread = coerceMin(ballErrorRight.x*spreadSlopeMore, 0.0);

	// Calculate whether the ball is still suitably in front of us
	bool ballXOk = (SV.BallDir.x > 0.0f && SV.BallDir.x < config.dbBallDistXMax);
	bool ballYOk = ((ballErrorLeft.y < config.dbBallErrorYOwd + leftSpread) && (ballErrorRight.y > -config.dbBallErrorYOwd - rightSpread));
	bool ballStillOk = (ballXOk && ballYOk);

	// Put together whether we are still ok to dribble
	bool dribbleStillOk = (ballStillOk && stillFacingTarget);

	// Print information about checking whether we can still dribble
#ifdef DEBUG_DB
	if (stateCycle % 20 == 1 && state == STATE_DRIBBLE_BALL)
		printf("DB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), %s(%.2f), %s %s %s\n", (SV.BallConf >= 0.2f ? "BALL" : "ball"), SV.BallDir.x, SV.BallDir.y, (SV.BallTargetConf >= 0.2f ? "BALLTGT" : "balltgt"), SV.BallTargetChar(), SV.BallTargetDir.x, SV.BallTargetDir.y, (stillFacingTarget ? "TGTANGLE" : "tgtangle"), SV.BallToTargetAngle, (ballXOk ? "X_OK" : "notx"), (ballYOk ? "Y_OK" : "noty"), (dribbleStillOk ? "PASS" : "FAIL"));
#endif

	// Return whether we are still ok to dribble
	return dribbleStillOk;
}

// State execution functions
void WalkAndKick::executeDB(bool justActivated)
{
	//
	// Gaze control
	//

	// Look at the ball
	gazeAtBall();

	//
	// Walking control
	//

	// Required gait control vector
	Vec3f GCV(0.0, 0.0, 0.0); // x forwards, y left, z CCW

	// Save the relative offsets to the ball and target in local variables
	Vec2f ball = SV.BallDir;         // Vector from robot to ball in body-fixed coordinates (see also the use of SV.BallPose etc below)
	Vec2f target = SV.BallTargetDir; // Vector from robot to target in body-fixed coordinates
	if (SV.BallTargetConf < 0.2f)    // If we don't have a target then take one that's right in front of us (further down we also disable beta control)
	{
		target.x = FAKE_BALL_TARGET_DIST;
		target.y = 0.0f;
	}

	// Calculate the vector from the ball to the target in body-fixed coordinates
	Vec2f E = target - ball;
	Vec2f unitEx = E.getNormalized();
	Vec2f unitEy = unitEx.rotatedCCW90();

	// Transform the ball vector into the E basis
// 	float ballEx = ball * unitEx; // Component of the robot-ball vector parallel to the ball-target vector
	float ballEy = ball * unitEy; // Component of the robot-ball vector perpendicular to the ball-target vector

	// Calculate the instantaneous point we want to walk towards to do our dribble approach
	float walkToEx = config.dbAppWalkToXDist;
	float walkToEy = config.dbAppWalkToYScaler * (ballEy - (ballEy > 0.5f*(config.gbbReqBallOffYLK + config.gbbReqBallOffYRK) ? config.gbbReqBallOffYLK : config.gbbReqBallOffYRK));
	Vec2f walkTo = walkToEx*unitEx + walkToEy*unitEy;
	float turnTo = atan2(E.y, E.x); // We turn to be in line with the ball-target vector

	// Calculate the GCV we want to walk with
	Vec2f gcvXY = config.dbAppXYSpeed * walkTo.getNormalized();
	GCV.x = gcvXY.x;
	GCV.y = gcvXY.y;
	GCV.z = config.dbAppZGain * turnTo;

	// Normalise the GCV to our desired maximum walking speed
	float gcvNorm = GCV.norm();
	float speedLimit = fabs(config.dbAppSpeedLimit);
	if (gcvNorm > speedLimit)
		GCV *= speedLimit / gcvNorm;

	// Set our walking GCV
	AV.GCV = GCV;
	AV.Halt = false;

	// Print information about the dribbling approach
#ifdef DEBUG_DB_APP
	if (stateCycle % 20 == 1)
		printf("DB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), WALKTO_EXY(%.2f, %.2f), TURNTO(%.2f), GCV(%.2f, %.2f, %.2f) %s\n", (SV.BallConf >= 0.2f ? "BALL" : "ball"), SV.BallDir.x, SV.BallDir.y, (SV.BallTargetConf >= 0.2f ? "BALLTGT" : "balltgt"), SV.BallTargetChar(), SV.BallTargetDir.x, SV.BallTargetDir.y, walkToEx, walkToEy, turnTo, AV.GCV.x, AV.GCV.y, AV.GCV.z, (dbLock ? "LOCK" : "nolock"));
#endif
}

// #####################################################################
// #########################     Kick Ball     #########################
// #####################################################################

// State change function
void WalkAndKick::changedKB(bool nowActive)
{
}

// Decide whether we are in a position to kick
bool WalkAndKick::okToKick() const
{
	// Don't kick if we don't have a ball or ball target to kick to
	if (SV.BallConf < 0.2f || SV.BallTargetConf < 0.2f)
		return false;

	// Calculate whether we are facing the target
	float angularTolerance = coerceMin(fabs(config.kbTargetAnglePrecision*0.5f*SV.BallTargetWedge), config.kbMinTargetAngleTolerance); // Note: The wedge angle is the complete angular range from the left side of the target to the right side...
	bool facingTarget = (fabs(SV.BallToTargetAngle) < angularTolerance); // The ball target is in front of the ball from our perspective (within an angular tolerance dependent on how wide the target is)

	// Calculate whether the ball is in front of our left or right foot
	Vec2f ballErrorLeft = SV.BallDir - ReqBallDirLeft;
	Vec2f ballErrorRight = SV.BallDir - ReqBallDirRight;
	bool leftFootInPosition = (SV.BallDir.x > 0.0f && ballErrorLeft.x < config.kbBallErrorXFwd && ballErrorLeft.y < config.kbBallErrorYOwd && ballErrorLeft.y > -config.kbBallErrorYIwd); // The ball is within a rectangle of where it should be in front of the left foot
	bool rightFootInPosition = (SV.BallDir.x > 0.0f && ballErrorRight.x < config.kbBallErrorXFwd && ballErrorRight.y < config.kbBallErrorYIwd && ballErrorRight.y > -config.kbBallErrorYOwd); // The ball is within a rectangle of where it should be in front of the right foot

	// Decide whether we are in the position to kick
	bool inKickPosition = facingTarget && (leftFootInPosition || rightFootInPosition);

	// Print information about checking whether we can kick
#ifdef DEBUG_KB
	if (stateCycle % 20 == 1)
		printf("KB: %s(%.2f, %.2f), %s%c(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f, %.2f), %s(%.2f), COMPASS(%.2f), %s, %s\n", (SV.BallConf >= 0.2f ? "BALL" : "ball"), SV.BallDir.x, SV.BallDir.y, (SV.BallTargetConf >= 0.2f ? "BALLTGT" : "balltgt"), SV.BallTargetChar(), SV.BallTargetDir.x, SV.BallTargetDir.y, (leftFootInPosition ? "ERR_LEFT" : "err_left"), ballErrorLeft.x, ballErrorLeft.y, (rightFootInPosition ? "ERR_RIGHT" : "err_right"), ballErrorRight.x, ballErrorRight.y, (facingTarget ? "ANGLE" : "angle"), SV.BallToTargetAngle, SV.CompassHeading, (facingTarget ? "AIMED" : "not aimed"), (inKickPosition ? "CAN KICK" : "no"));
#endif

	// Return whether we can kick
	return inKickPosition;
}

// Decide whether we are still in an acceptable position to kick, assuming that okToKick() returned true in the recent past
bool WalkAndKick::stillOkToKick() const
{
	// Note: We don't check SV.BallConf in case going to the kick halt pose consistently affects seeing the ball (change in height, knees come out)
	//       We don't check SV.BallTargetConf as we were sure when we stopped walking and we haven't moved since then
	
	// Calculate the allowed bounds on the ball
	float minBallX = 0.0;
	float maxBallX = std::max(ReqBallDirLeft.x, ReqBallDirRight.x) + config.kbBallErrorXFwd + config.kbBallErrorXFwdExtra;
	float minBallY = ReqBallDirRight.y - config.kbBallErrorYOwd - config.kbBallErrorYOwdExtra;
	float maxBallY = ReqBallDirLeft.y + config.kbBallErrorYOwd + config.kbBallErrorYOwdExtra;

	// Check whether the ball is still ok for kicking
	bool ballXOk = (minBallX <= SV.BallDir.x && SV.BallDir.x <= maxBallX);
	bool ballYOk = (minBallY <= SV.BallDir.y && SV.BallDir.y <= maxBallY);
	bool ballStillOk = (ballXOk && ballYOk);

	// Print information about checking whether we can still kick
#ifdef DEBUG_KB
	printf("CONFIRMING KICK: %s, %s(%.2f, %.2f), %s(%.2f -> %.2f), %s(%.2f -> %.2f), %s%c(%.2f, %.2f), COMPASS(%.2f)\n", (ballStillOk ? "PASS" : "FAIL"), (SV.BallConf >= 0.2f ? "BALL" : "ball"), SV.BallDir.x, SV.BallDir.y, (ballXOk ? "XRANGE" : "xrange"), minBallX, maxBallX, (ballYOk ? "YRANGE" : "yrange"), minBallY, maxBallY, (SV.BallTargetConf >= 0.2f ? "BALLTGT" : "balltgt"), SV.BallTargetChar(), SV.BallTargetDir.x, SV.BallTargetDir.y, SV.CompassHeading);
#endif

	// Return whether we are still ok to kick
	return ballStillOk;
}

// Decision of best kick foot
int WalkAndKick::bestKickFoot() const
{
	// Return which kick foot is currently more appropriate
	float centrey = 0.5*(ReqBallDirLeft.y + ReqBallDirRight.y);
	return sign(centrey - SV.BallDir.y); // 1 = Right, -1 = Left, 0 = Avoid this value
}

// State execution functions
void WalkAndKick::executeKB(bool justActivated)
{
	//
	// Gaze control
	//

	// Look at the ball
	gazeAtBall();

	//
	// Walking control
	//

	// Stop walking
	AV.Halt = true;
	AV.GCV.setZero();

	//
	// Kicking control
	//

	// We wish to kick
	AV.DoKick = true;
	AV.KickFoot = bestKickFoot();
}

// #####################################################################
// #####################     Behaviour Helpers     #####################
// #####################################################################

// Set AV.GazeAngle to try to gaze at the ball (returns false if no ball)
bool WalkAndKick::gazeAtBall()
{
	// If we don't have a ball then we can't gaze at it
	if (SV.BallConf < 0.2f)
	{
		AV.GazeAngle = lastAV.GazeAngle;
		return false;
	}

	// Decide on a gaze settling time based on the distance to the ball
	float Ts = config.gazeBallFarTs; // Ts = 90% settling time
	if (SV.BallDist < config.gazeBallFarDist)
		Ts = config.gazeBallNearTs + (SV.BallDist / config.gazeBallFarDist) * (config.gazeBallFarTs - config.gazeBallNearTs);
	Ts = coerce(Ts, 0.1f, 3.0f);

	// Update the gaze angle to look towards the ball
	float beta = 1.0f - pow(0.1f, TINC / Ts);
	AV.GazeAngle = lastAV.GazeAngle + beta*(SV.BallAngle - lastAV.GazeAngle);
	AV.GazeAngle = coerceAbs(AV.GazeAngle, config.gazeAngleLimit);

	// Return that we successfully commanded to gaze at the ball
	return true;
}

// Set AV.GCV to try to walk to a particular global pose (returns a metric of proximity to the target: dist error + WTGP_ANGLE_DEV_COST * angle error)
float WalkAndKick::walkToGlobalPose(float targetX, float targetY, float targetZ, bool useZ)
{
	// If we are not localised then we don't know which direction to walk
	if (SV.RobotPoseConf < 0.2f)
	{
		AV.GCV.setZero();
		return INVALID_DIST;
	}

	// Coerce the desired global location to the field boundary
	targetX = coerceAbs(targetX, field.fieldLengthH());
	targetY = coerceAbs(targetY, field.fieldWidthH());

	// Calculate the XY vector to the target in body-fixed coordinates
	Vec2f relativeTarget(targetX - SV.RobotPose.x, targetY - SV.RobotPose.y); // relativeTarget is in global coordinates here
	relativeTarget.rotate(-SV.RobotPose.z); // relativeTarget is now in body-fixed coordinates

	// Calculate the target direction (unit vector), distance and angle
	Vec2f targetDir = relativeTarget.getNormalized();
	float targetDist = relativeTarget.norm();
	float targetAngle = atan2(relativeTarget.y, relativeTarget.x);

	// Calculate the angle deviation to the global Z target
	float targetZDeviation = picut(targetZ - SV.RobotPose.z);

	// Calculate the gait command for a far target
	Vec3f gcvFar;
	gcvFar.x = coerceMin(WTGP_FAR_SPEEDX*(1.0 - fabs(targetAngle)/WTGP_FAR_ANGLIMIT), 0.0);
	gcvFar.y = 0.0f;
	gcvFar.z = WTGP_FAR_SPEEDZ * coerceAbs(targetAngle / WTGP_FAR_ANGLIMIT, 1.0);

	// Calculate the gait command for a near target
	Vec3f gcvNear;
	float nearSpeed = WTGP_NEAR_SPEEDXY * coerce(targetDist / WTGP_NEAR_DIST, 0.0, 1.0);
	gcvNear.x = nearSpeed * targetDir.x;
	gcvNear.y = nearSpeed * targetDir.y;
	if (useZ)
		gcvNear.z = WTGP_NEAR_SPEEDZ * coerceAbs(targetZDeviation / WTGP_NEAR_ANGLIMIT, 1.0);
	else
		gcvNear.z = 0.0f;

	// Choose a gait command to work with
	if (targetDist <= WTGP_NEAR_DIST)
		AV.GCV = gcvNear;
	else if (targetDist <= WTGP_FAR_DIST)
		AV.GCV = gcvNear + ((targetDist - WTGP_NEAR_DIST) / (WTGP_FAR_DIST - WTGP_NEAR_DIST)) * (gcvFar - gcvNear);
	else
		AV.GCV = gcvFar;

	// Calculate the current distance cost to the target
	float distCost = (useZ ? targetDist + WTGP_ANGLE_DEV_COST * fabs(targetZDeviation) : targetDist);

	// Print info about the positioning state
#ifdef DEBUG_WTGP
	if (stateCycle % 20 == 1)
		printf("WTGP: %s %s GBLTGT(%.2f, %.2f, %.2f) %s(%.2f, %.2f, %.2f) LCLTGT(%.2f, %.2f) ROTERR(%.2f) GCV(%.2f, %.2f, %.2f)\n", (useZ ? "USE Z" : "NO Z"), (targetDist <= WTGP_NEAR_DIST ? "NEAR" : (targetDist <= WTGP_FAR_DIST ? "MIX" : "FAR")), targetX, targetY, targetZ, (SV.RobotPoseConf >= 0.2f ? "POSE" : "pose"), SV.RobotPose.x, SV.RobotPose.y, SV.RobotPose.z, relativeTarget.x, relativeTarget.y, targetZDeviation, AV.GCV.x, AV.GCV.y, AV.GCV.z);
#endif

	// Return the current distance cost to the global walking target
	return distCost;
}

// ####################################################################
// #####################     Actuator Outputs     #####################
// ####################################################################

// Write the actuator outputs to the actuators
void WalkAndKick::writeActuators(const ActuatorVars& ActVar)
{
	// Walking gait command
	Vec3f GCV = ActVar.GCV;
	float gcvNorm = GCV.norm();
	float speedLimit = MAX_GCV_NORM;
	if (gcvNorm > speedLimit)
		GCV *= speedLimit / gcvNorm;
	gait_msgs::GaitCommand cmd;
#ifdef DEBUG_FORCE_HALT
	cmd.gcvX = 0.0;
	cmd.gcvY = 0.0;
	cmd.gcvZ = 0.0;
	cmd.walk = false;
#else
	cmd.gcvX = GCV.x;
	cmd.gcvY = GCV.y;
	cmd.gcvZ = GCV.z;
	cmd.walk = !ActVar.Halt;
#endif
	if(m_button == BTN_GOALIE && !dsGoalieAlive)
		cmd.walk = false;

	// Kicking
#ifdef DEBUG_FORCE_NOKICK
	if(false)
#else
	if(ActVar.DoKick)
#endif
	{
		if(ActVar.KickFoot > 0) // 1 = Right, -1 = Left
			cmd.motion = MID_KICK_RIGHT;
		else
			cmd.motion = MID_KICK_LEFT;
		cmd.walk = false;
	}

	// Publish the walking/kicking gait command
	m_pub_gaitcmd.publish(cmd);

	// Gaze angle
	double GazeAngle = SlopeLimiter::eval(ActVar.GazeAngle, lastAV.GazeAngle, config.gazeVelLimit * TINC);
	GazeAngle = coerceAbs(GazeAngle, config.gazeAngleLimit);
	
	// Publish the head control command
	head_control::LookAtTarget headCmd;
	headCmd.enabled = true;
	headCmd.is_angular_data = true;
	headCmd.is_relative = false;
	headCmd.pitchEffort = 0.0; // Use the default pitch effort
	headCmd.yawEffort = 0.0; // Use the default yaw effort
	headCmd.vec.x = 0.0;
	headCmd.vec.y = 0.3;
	headCmd.vec.z = GazeAngle;
	m_pub_headCmd.publish(headCmd);
}

// ####################################################################
// #####################     Helper Functions     #####################
// ####################################################################

// Array of state names (should match up with WAKState enumeration in header)
const char* const WalkAndKick::StateName[NUM_STATES] = {
	"Unknown State",
	"Stopped",
	"Positioning",
	"Search for Ball",
	"Go Behind Ball",
	"Dribble Ball",
	"Kick Ball"
};

// Get name of state
const char* WalkAndKick::getStateName(WAKState stateID)
{
	// Return the required state name
	if (stateID <= STATE_UNKNOWN || stateID >= NUM_STATES)
		stateID = STATE_UNKNOWN;
	return StateName[stateID];
}

// Run the required changed handler for the deactivation of a state
void WalkAndKick::deactivateState(WAKState stateID)
{
	// Call the required changed handler with false
	switch (stateID)
	{
	case STATE_STOPPED:         changedSTP(false); break;
	case STATE_POSITIONING:     changedPOS(false); break;
	case STATE_SEARCH_FOR_BALL: changedSFB(false); break;
	case STATE_GO_BEHIND_BALL:  changedGBB(false); break;
	case STATE_DRIBBLE_BALL:    changedDB (false); break;
	case STATE_KICK_BALL:       changedKB (false); break;
	default: break;
	}
}

// Run the required changed handler for the activation of a state
void WalkAndKick::activateState(WAKState stateID)
{
	// Call the required changed handler with true
	switch (stateID)
	{
	case STATE_STOPPED:         changedSTP(true); break;
	case STATE_POSITIONING:     changedPOS(true); break;
	case STATE_SEARCH_FOR_BALL: changedSFB(true); break;
	case STATE_GO_BEHIND_BALL:  changedGBB(true); break;
	case STATE_DRIBBLE_BALL:    changedDB (true); break;
	case STATE_KICK_BALL:       changedKB (true); break;
	default: break;
	}
}

// Execute the required state action
void WalkAndKick::executeState(WAKState stateID, bool justActivated)
{
	// Call the required execute function
	switch (stateID)
	{
	case STATE_STOPPED:         executeSTP(justActivated); break;
	case STATE_POSITIONING:     executePOS(justActivated); break;
	case STATE_SEARCH_FOR_BALL: executeSFB(justActivated); break;
	case STATE_GO_BEHIND_BALL:  executeGBB(justActivated); break;
	case STATE_DRIBBLE_BALL:    executeDB (justActivated); break;
	case STATE_KICK_BALL:       executeKB (justActivated); break;
	default: break;
	}
}

// #####################################################################
// #######################     Data Handlers     #######################
// #####################################################################

// Reset function
void WalkAndKick::resetDataVariables()
{
	// Reset variables
	m_button = BTN_HALT;
	m_ball_vec.setZero();
	m_ball_conf = 0.0;
	m_ball_time.fromSec(0);
	m_goal_vec.setZero();
	m_goal_conf = 0.0;
	m_goal_time.fromSec(0);
	m_robot_pose_vec.setZero();
	m_robot_pose_conf = 0.0;
	m_robot_pose_time.fromSec(0);
	m_heading = 0.0;
	m_heading_time.fromSec(0);
	m_standing = false;
	m_walking = false;
	m_kicking = false;
}

// New button data callback
void WalkAndKick::handleButtonData(const nimbro_op_interface::ButtonConstPtr& msg)
{
	// Check whether the middle button has been pressed
	if((msg->button == 1) && m_enable_wak())
	{
		// Wrapped increment the button state
		int nextButtonState = m_button + 1;
		if(nextButtonState < BTN_FIRST || nextButtonState >= BTN_COUNT)
			nextButtonState = BTN_FIRST;
		m_button = (ButtonState) nextButtonState;
		
		// Handle the new state
		switch(m_button)
		{
			default:
			case BTN_HALT:
				m_led.rgb6.r = 1.0;
				m_led.rgb6.g = 0.0;
				m_led.rgb6.b = 0.0;
				ROS_INFO("Button 1 pressed: State is now BTN_HALT");
				break;
			case BTN_PLAY:
				m_led.rgb6.r = 0.0;
				m_led.rgb6.g = 1.0;
				m_led.rgb6.b = 0.0;
				ROS_INFO("Button 1 pressed: State is now BTN_PLAY");
				break;
			case BTN_GOALIE:
				m_led.rgb6.r = 0.0;
				m_led.rgb6.g = 0.0;
				m_led.rgb6.b = 1.0;
				ROS_INFO("Button 1 pressed: State is now BTN_GOALIE");
				break;
			case BTN_POS:
				m_led.rgb6.r = 0.5;
				m_led.rgb6.g = 0.0;
				m_led.rgb6.b = 0.5;
				ROS_INFO("Button 1 pressed: State is now BTN_POS");
				break;
		}
		
		// Change the LED state
		m_led.mask = nimbro_op_interface::LEDCommand::LED6;
		m_pub_leds.publish(m_led);
	}
}

// New ball data callback
void WalkAndKick::handleBallData(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	// Store the data received locally
	m_ball_time = msg->header.stamp;
	m_ball_vec << msg->point.x, msg->point.y, msg->point.z;
	m_ball_conf = 0.5*msg->point.z; // TODO: Hafez wanted the ball conf cutoff to be 0.4, but the code uses the 0.2 standard, hence scale

	// Debug
// 	ROS_INFO_THROTTLE(1,"%.1f -------> Data on the ball topic: %.3f %.3f with confidence %.2f",m_ball_time.toSec(),m_ball_vec.x(),m_ball_vec.y(),m_ball_vec.z());
}

// New goal data callback
void WalkAndKick::handleGoalData(const geometry_msgs::PolygonStampedConstPtr& msg)
{
	// If two goal posts are seen, calculate the vector to the centre of the goal
	if(msg->polygon.points.size() == 2)
	{
		m_goal_vec << (msg->polygon.points[0].x + msg->polygon.points[1].x)/2, (msg->polygon.points[0].y + msg->polygon.points[1].y)/2, (msg->polygon.points[0].z + msg->polygon.points[1].z)/2;
// 		ROS_INFO_THROTTLE(1,"---> Goal data on topic: %lf %lf %lf          (ANGLE %lf)",m_goal_vec.x(),m_goal_vec.y(),m_goal_vec.z(),(180/M_PI)*atan2(m_goal_vec.y(),m_goal_vec.x()));
	}
	else
	{
		ROS_INFO_THROTTLE(1,"---> In goal topic but not two points!");
	}
}

// New robot pose data callback
void WalkAndKick::handleRobotPoseData(const geometry_msgs::PointStampedConstPtr& msg)
{
	// Store the data received locally
	m_robot_pose_time = msg->header.stamp;
	m_robot_pose_vec << msg->point.x, msg->point.y, picut(msg->point.z);
	m_robot_pose_conf = 1.0;

	// Debug
// 	ROS_INFO_THROTTLE(1,"%.1f -------> Data on the loc topic: %.3f %.3f at angle %.3f",m_robot_pose_time.toSec(),m_robot_pose_vec.x(),m_robot_pose_vec.y(),m_robot_pose_vec.z());
}

// New robot heading data
void WalkAndKick::handleHeadingData( const robotcontrol::RobotHeadingConstPtr& msg)
{
	m_heading_time = msg->stamp;
	m_heading = picut(msg->heading);
}

// New robot state data callback
void WalkAndKick::handleStateData(const robotcontrol::StateConstPtr& msg)
{
	// Return whether walking gait is active
	m_standing = (msg->label == "standing");
	m_walking  = (msg->label == "walking");
	m_kicking  = (msg->label == "kicking");
}

// #####################################################################
// #######################     Main Function     #######################
// #####################################################################

// Main function
int main(int argc,char **argv)
{
	// Process ROS command line arguments
	ros::init(argc,argv,"walk_and_kick");

	// Create an instance of the WalkAndKick class and initialise it
	WalkAndKick WAK;
	if(!WAK.init())
	{
		ROS_ERROR("Could not initialise walk_and_kick node!");
		return 1;
	}
	else
	{
		ROS_INFO("Initialisation of WalkAndKick complete!");
	}

	// Initialise timer object for loop rate timing
	MotionTimer timer(TINC);

	// Keep looping while everything is fine and dandy...
	while(ros::ok())
	{
		// Do all ROS callbacks before sleeping so that the next step starts immediately after the timer tick
		ros::spinOnce();

		// Sleep for the required duration
		uint64_t expirations = timer.sleep();

		// If enabled, perform a cycle step
		if(WAK.m_enable_wak())
			WAK.step();
		else
			WAK.reset();

		// Check whether any cycles were missed
		if(expirations > 1)
			ROS_WARN("Missed %lu timer cycles",expirations-1);
	}

	// Return value
	return 0;
}
// EOF
