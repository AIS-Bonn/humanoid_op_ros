// Fall protection for the robot
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// INIT POSE MOTION
// ----------------
// Motion name:   init_pose_*
// Motion states: relaxed --> setting_pose --> relaxed
// Description:   Every init pose motion must consist of two identical keyframes corresponding to the pose to initialise to.
//                The former should have duration 0.0, and the latter should have duration 1.0.
//                The init pose motion is played immediately by the motion player every time the robot enters the relaxed state.
//                The init pose motion is uninterruptible, and a warning is issued if the robotcontrol state is changed while the motion is playing.
//
// INIT MOTION
// -----------
// Motion name:   init_*
// Motion states: init --> initializing --> {standing, sitting, etc...}
// Description:   After a completed fade in via the fade torque server, the state defaults to init. The motion player detects this and automatically
//                plays the init motion, which should bring the robot to the pose it needs to be in to complete the tasks that are required of it.
//                The pose commanded by the corresponding init pose motion should be the starting point of the init motion. Thus during a fade in,
//                the robot first fades to the init pose, and once the torque is completely faded in, plays the init motion, during which the state
//                changes to initializing.
//
// PRONE GET UP MOTION
// -------------------
// Motion name:   getup_prone
// Motion states: lying_prone --> getting_up --> init
// Description:   When the robot enters the lying_prone state, the motion player automatically plays the prone get up motion. This should transfer
//                the robot to the init pose (the one immediately after a fade in!), after which the init motion kicks in and gets the robot back
//                into position. Thus in the normal use case of a humanoid robot, the prone get up motion simply needs to bring the robot back into
//                the sitting-on-your-heels pose (the 'normal' init pose), from which the init motion can make the robot stand up by extending the legs.
//
// SUPINE GET UP MOTION
// --------------------
// Motion name:   getup_supine
// Motion states: lying_supine --> getting_up --> init
// Description:   When the robot enters the lying_supine state, the motion player automatically plays the supine get up motion. This should transfer
//                the robot to the init pose (the one immediately after a fade in!), after which the init motion kicks in and gets the robot back
//                into position. Thus in the normal use case of a humanoid robot, the supine get up motion simply needs to bring the robot back into
//                the sitting-on-your-heels pose (the 'normal' init pose), from which the init motion can make the robot stand up by extending the legs.
//
// LEFT/RIGHT PRONE/SUPINE GET UP MOTIONS
// --------------------------------------
// Motion name:   getup_left_prone, getup_left_supine, getup_right_prone, getup_right_supine
// Motion states: lying_side --> getting_up --> lying_biased
// Description:   When the robot is detected to be lying on its side (lying_side state), one of the four side get up motions is played to try to tip the
//                robot over into either the prone or supine position, based on what is deemed to be the easiest action. On completion of the motion,
//                the state becomes lying_biased (the post-state of each of the four possible side get up motions), which is immediately re-evaluated as
//                lying_prone or lying_supine based on the current orientation of the robot. The normal get up motions kick in at this stage.
//
// STATES
// ------
// relaxed      => State when the robot is hardware relaxed (i.e. via the fade torque action server)
// setting_pose => Standardised play state during the init pose motion
// init         => State immediately after a fade to a stiffness of 1.0 (larger than 0.5) via the fade torque server
//                 The pose of the robot must always be the same as at the end of the init pose motion when in this state
// initializing => Standardised play state during the init motion
// standing     => State when the robot is standing
// sitting      => State when the robot is sitting
// kicking      => State while the robot is performing a kick
// falling      => State whilst fall protection is triggered (time-limited, after which the state is set to either lying_prone or lying_supine)
// lying_prone  => State when the robot is lying on the ground in a prone position (face down)
// lying_supine => State when the robot is lying on the ground in a supine position (face up)
// lying_side   => State when the robot is lying on the ground on its left or right side (face approximately parallel to ground)
// lying_biased => State when the robot used to be lying on its side, but has played a motion to attempt to force the prone or supine lying position
// getting_up   => Standardised play state during the prone, supine and side get up motions

// Includes
#include "fallprotection.h"
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <robotcontrol/model/robotmodel.h>

// Defines
#define FALL_PROTECTION_TIME 2.0

// Namespaces
using namespace fall_protection;

// Class constants
const std::string FallProtection::m_lyingPlayStatePrefix = "lying_side|play_";

// Constructor
FallProtection::FallProtection()
 : m_fallProtectionEnabled("/fallProtection/fallProtectionEnabled", true)
 , m_sideGetupsEnabled("/fallProtection/sideGetupsEnabled", true)
 , m_fallTriggerAngle("/fallProtection/triggerAngle", 0.1, 0.01, M_PI, 0.65)
 , m_fallTriggerAngleKick("/fallProtection/triggerAngleKick", 0.1, 0.01, M_PI, 0.65)
 , m_sideGetupRollAngle("/fallProtection/sideGetupRollAngle", 0.1, 0.01, 1.60, M_PI_4)
 , m_fadeTorqueAction("/robotcontrol/fade_torque")
 , m_enabled(true)
 , m_fall_triggered(false)
 , m_triggerTime(0,0)
{
}

// Initialisation function
bool FallProtection::init(robotcontrol::RobotModel* model)
{
	// Initialise the motion module
	robotcontrol::MotionModule::init(model);

	// Send zero joint commands to all joints
	for(size_t i = 0; i < model->numJoints(); ++i)
		setJointCommand(i, 0, 0, false);

	// Retrieve a node handle
	ros::NodeHandle nh("~");

	// Retrieve the side getup motion names
	std::string getupLeftProneMotion, getupLeftSupineMotion, getupRightProneMotion, getupRightSupineMotion;
	nh.param<std::string>("GetupLeftProneMotion", getupLeftProneMotion, "");
	nh.param<std::string>("GetupLeftSupineMotion", getupLeftSupineMotion, "");
	nh.param<std::string>("GetupRightProneMotion", getupRightProneMotion, "");
	nh.param<std::string>("GetupRightSupineMotion", getupRightSupineMotion, "");

	// Register the required robot model states
	m_state_relaxed      = model->registerState("relaxed");
	m_state_setting_pose = model->registerState("setting_pose");
	m_state_init         = model->registerState("init");
	m_state_sitting      = model->registerState("sitting");
	m_state_kicking      = model->registerState("kicking");
	m_state_falling      = model->registerState("falling");
	m_state_getting_up   = model->registerState("getting_up");
	m_state_lying_prone  = model->registerState("lying_prone");
	m_state_lying_supine = model->registerState("lying_supine");
	m_state_lying_side   = model->registerState("lying_side");
	m_state_lying_biased = model->registerState("lying_biased");
	m_state_lying_side_left_prone   = (getupLeftProneMotion.empty()   ? m_state_lying_prone  : model->registerState(m_lyingPlayStatePrefix + getupLeftProneMotion));
	m_state_lying_side_left_supine  = (getupLeftSupineMotion.empty()  ? m_state_lying_supine : model->registerState(m_lyingPlayStatePrefix + getupLeftSupineMotion));
	m_state_lying_side_right_prone  = (getupRightProneMotion.empty()  ? m_state_lying_prone  : model->registerState(m_lyingPlayStatePrefix + getupRightProneMotion));
	m_state_lying_side_right_supine = (getupRightSupineMotion.empty() ? m_state_lying_supine : model->registerState(m_lyingPlayStatePrefix + getupRightSupineMotion));

	// Return that initialisation was successful
	return true;
}

// Trigger function
bool FallProtection::isTriggered()
{
	// Don't do anything if fall protection is not enabled
	if(!m_enabled)
	{
		ROS_WARN_THROTTLE(60.0, "Fall protection is disabled via the config server");
		m_enabled = m_fallProtectionEnabled();
		return false;
	}

	// Retrieve the current state of the robot
	robotcontrol::RobotModel::State state = model()->state();
	std::string stateLabel = model()->stateLabel(state);

	// If the robot is lying biased (attempted to force prone or supine) then trigger the suitable getup motion
	if(state == m_state_lying_biased)
	{
		if(model()->robotFPitchPR() >= 0)
			model()->setState(m_state_lying_prone);
		else
			model()->setState(m_state_lying_supine);
		state = model()->state();
		stateLabel = model()->stateLabel(state);
	}

	// Work out whether the getup procedure is currently in action
	bool playingSideMotion = (stateLabel.compare(0, m_lyingPlayStatePrefix.size(), m_lyingPlayStatePrefix) == 0);
	bool gettingUp = (playingSideMotion
	               || state == m_state_getting_up
	               || state == m_state_lying_prone
	               || state == m_state_lying_supine
	               || state == m_state_lying_side
	               || state == m_state_lying_biased);

	// Update the fall protection enabled state if fall protection isn't currently triggered
	if(!m_fall_triggered && !gettingUp)
		m_enabled = m_fallProtectionEnabled();

	// Calculate the angle deviation from vertical
	const Eigen::Quaterniond& q = model()->robotOrientationPR();
	double zGz = 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
	if(zGz <= -1.0) zGz = -1.0;
	double angle = acos(zGz);

	// Decide on the angular limit
	double angLimit;
	if(state == m_state_kicking)
		angLimit = m_fallTriggerAngleKick();
	else
		angLimit = m_fallTriggerAngle();

	// Unset the robot model relaxed flag if we were interrupted before the falling timeout elapsed (we assume that whoever interrupted us has no reason to want a relaxed robot)
	if(m_fall_triggered && state != m_state_falling)
	{
		ROS_WARN_THROTTLE(0.4, "Fall protection interrupted as someone wrote '%s' into the robot state", model()->currentStateLabel().c_str());
		model()->setRelaxed(false);
		m_fall_triggered = false;
	}

	// Check whether fall protection should trigger
	if(fabs(angle) > angLimit && !gettingUp
	   && state != m_state_relaxed
	   && state != m_state_setting_pose
	   && state != m_state_init
	   && state != m_state_sitting
	   && state != m_state_falling)
	{
		ROS_WARN_THROTTLE(0.4, "Fall protection triggered! Angle deviation from vertical was %.3f deg (limit %.1f)", angle * 180.0 / M_PI, angLimit * 180.0 / M_PI);
		model()->setState(m_state_falling);
		m_triggerTime = ros::Time::now();
		m_fall_triggered = true;
	}

	// Return whether fall protection is triggered
	return m_fall_triggered;
}

// Step function
void FallProtection::step()
{
	// Get the elapsed time since the trigger time
	ros::Duration duration = ros::Time::now() - m_triggerTime;

	// Request a relax of the robot
	model()->setRelaxed(true);

	// If the fall protection time has elapsed then transition into a lying state and unrelax the robot
	if(duration.toSec() > FALL_PROTECTION_TIME)
	{
		// Retrieve the current fused pitch and roll
		double fPitch = model()->robotFPitchPR();
		double fRoll = model()->robotFRollPR();
		
		// Detect whether the robot is lying left or right
		bool lyingLeft = (fRoll <= -m_sideGetupRollAngle());
		bool lyingRight = (fRoll >= m_sideGetupRollAngle());
		bool allowSide = m_sideGetupsEnabled();

		// Transition into the suitable lying state
		if(fPitch >= 0) // Prone position...
		{
			if(lyingLeft && allowSide)
				model()->setState(m_state_lying_side_left_prone);
			else if(lyingRight && allowSide)
				model()->setState(m_state_lying_side_right_prone);
			else
				model()->setState(m_state_lying_prone);
		}
		else // Supine position...
		{
			if(lyingLeft && allowSide)
				model()->setState(m_state_lying_side_left_supine);
			else if(lyingRight && allowSide)
				model()->setState(m_state_lying_side_right_supine);
			else
				model()->setState(m_state_lying_supine);
		}

		// Display which state has been entered to the user
		ROS_WARN("Starting getup motion from %s state", model()->currentStateLabel().c_str());

		// Unrelax the robot
		model()->setRelaxed(false);
		m_fall_triggered = false;

		// Trigger a fade in while the suitable getup motion starts playing
		robotcontrol::FadeTorqueGoal goal;
		goal.torque = 1.0;
		m_fadeTorqueAction.cancelAllGoals();
		m_fadeTorqueAction.sendGoal(goal);
	}
}

PLUGINLIB_EXPORT_CLASS(fall_protection::FallProtection, robotcontrol::MotionModule)
// EOF