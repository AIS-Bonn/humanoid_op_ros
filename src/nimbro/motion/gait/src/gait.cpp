// Generic gait motion module
// File: gait.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <gait/gait.h>
#include <ros/console.h>
#include <nimbro_utils/math_funcs.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>

// Defines - Timing
#define MIN_TRUEDT_FACTOR      0.8  // Lower coercion limit for in.truedT (Units: Multiples of dT)
#define MAX_TRUEDT_FACTOR      2.2  // Upper coercion limit for in.truedT (Units: Multiples of dT)

// Defines - Reach halt pose
#define REACH_HALT_POSE_VEL    0.5  // Maximum velocity to use in reaching the halt pose (Units: rad/s)
#define REACH_HALT_POSE_ACC    1.0  // Acceleration to use in reaching the halt pose (Units: rad/s^2)
#define REACH_HALT_POSE_DELAY  0.3  // Time to wait after sending the last reach halt pose command before passing control to the gait engine (Units: s)
#define REACH_HALT_POSE_TOL    1e-3 // Joint command tolerance to accept in reaching the halt pose (difference between the current joint *commands* and the target halt pose commands)
#define REACH_HALT_EFFORT_TOL  1e-3 // Joint effort tolerance to accept in reaching the halt pose (difference between the currently commanded joint efforts and the target halt pose efforts)

// Defines - Robot states
#define ROBOT_STANDING_STATE_NAME  "standing"   // Standing state from which the gait can trigger
#define ROBOT_WALKING_STATE_NAME   "walking_"   // State during walking (active gait)
#define PLAY_MOTION_TOKEN          "|play_"     // State prefix to make the motion player play a motion

// Namespaces
using namespace gait;
using namespace nimbro_utils;

//
// Gait class
//

// Constructor
Gait::Gait()
 : CONFIG_PARAM_PATH("/gait/")
 , m_enableJoystick(CONFIG_PARAM_PATH + "enableJoystick", true)
 , m_plotData(CONFIG_PARAM_PATH + "plotData", false)
 , m_publishOdometry(CONFIG_PARAM_PATH + "publishOdometry", true)
 , m_publishTransforms(CONFIG_PARAM_PATH + "publishTransforms", true)
 , m_gaitCmdVecNormP(CONFIG_PARAM_PATH + "gaitCmdVecNormP", 0.5, 0.10, 10.0, 2.0)
 , m_gaitCmdVecNormMax(CONFIG_PARAM_PATH + "gaitCmdVecNormMax", 0.5, 0.05, 1.5, 1.0)
 , m_now(0, 0)
 , m_lastNow(0, 0)
 , m_enginePluginLoader("gait", "gait::GaitEngine")
 , m_motionPending(MID_NONE)
 , m_oldMotionPending(MID_NONE)
 , m_motionStance(STANCE_DEFAULT)
 , m_motionAdjustLeftFoot(true)
 , m_motionAdjustRightFoot(true)
 , m_reachDuration(0.0)
 , m_reachedHalt(false)
 , m_updatedHalt(false)
 , m_joystickEnabled(true)
 , m_joystickGaitCmdLock(false)
 , m_joystickButton0Pressed(false)
 , m_joystickButton1Pressed(false)
 , m_joystickButton2Pressed(false)
 , m_joystickButton3Pressed(false)
 , m_gaitState(GS_INACTIVE)
 , m_PM(PM_COUNT, "/gait")
{
	// Configure the TF transforms
	configureTransforms();

	// Configure the plot manager
	configurePlotManager();

	// Set configuration parameter callbacks
	m_enableJoystick.setCallback(boost::bind(&Gait::callbackEnableJoystick, this));
	m_plotData.setCallback(boost::bind(&Gait::callbackPlotData, this));

	// Initial call to configuration parameter callbacks
	callbackEnableJoystick();
	callbackPlotData();
}

// Destructor
Gait::~Gait()
{
	// Explicitly delete the instance of the gait engine before the plugin loader object gets destroyed
	m_engine.reset();
}

// Initialisation function
bool Gait::init(robotcontrol::RobotModel* model)
{
	// Initialise the base class
	if(!MotionModule::init(model)) return false;

	// Retrieve ROS node handle
	ros::NodeHandle nh("~");

	// Save the robot model
	m_model = model;

	// Retrieve the gait name
	m_gaitName = getParamString();
	size_t dblcolon = m_gaitName.find("::");
	if(dblcolon != std::string::npos)
		m_gaitName = m_gaitName.erase(0, dblcolon + 2);
	if(m_gaitName.empty())
		m_gaitName = "<null>";

	// Create configuration parameter for the enabled state of the gait
	m_enableGait = boost::make_shared<config_server::Parameter<bool> >(CONFIG_PARAM_PATH + "enable" + m_gaitName, true);

	// Retrieve the robotcontrol timer duration
	m_dT = m_model->timerDuration();

	// Register the RobotModel states required by the gait
	m_state_standing   = m_model->registerState(ROBOT_STANDING_STATE_NAME);
	m_state_walking    = m_model->registerState(ROBOT_WALKING_STATE_NAME + m_gaitName);

	// Retrieve the URDF links that will be used for setting support coefficients
	m_trunkLink     = m_model->urdf()->getLink("trunk_link");
	m_leftFootLink  = m_model->urdf()->getLink("left_foot_plane_link");
	m_rightFootLink = m_model->urdf()->getLink("right_foot_plane_link");

	// Construct a map for the required joints (requires m_model)
	if(!constructJointMap()) return false;

	// Reset the gait command
	m_gaitCmd.reset();

	// Advertise the required services
	m_srv_resetOdom = nh.advertiseService("/gait/resetOdom", &Gait::handleResetOdometry, this);
	m_srv_setOdom = nh.advertiseService("/gait/setOdom", &Gait::handleSetOdometry, this);

	// Subscribe to the required ROS topics
	m_sub_gaitCommand = nh.subscribe("/gaitCommand", 1, &Gait::handleGaitCommand, this);
	m_sub_joystickData = nh.subscribe("/joy", 1, &Gait::handleJoystickData, this);

	// Load the required gait engine (needs m_gaitName and getParamString())
	if(!loadGaitEngine())
		return false;

	// Reset the gait
	resetGait();

	// Initialise that no motion is pending
	clearPendingMotion();

	// Return that initialisation was successful
	return true;
}

// Trigger function
bool Gait::isTriggered()
{
	// Save the current ROS time
	m_lastNow = m_now;
	m_now = ros::Time::now();

	// Set the plot manager timestamp
	m_PM.setTimestamp(m_now);

	// Retrieve the robotcontrol timer duration
	m_dT = m_model->timerDuration();

	// Reset the halt pose update flag
	m_updatedHalt = false;

	// Reset the gait command to stop walking if a motion is pending
	if(m_motionPending < MID_NONE || m_motionPending >= MID_COUNT)
		clearPendingMotion();
	if(m_motionPending != MID_NONE && m_oldMotionPending == MID_NONE)
		m_gaitCmd.reset();

	// Save the current gait state
	GaitState oldGaitState = m_gaitState;

	// Get the current situation
	bool enabled = m_enableGait->get();
	bool isWalking = m_engine->out.walking;
	bool shouldWalk = m_gaitCmd.walk;
	bool walkingState = (m_model->state() == m_state_walking);
	bool standingState = (m_model->state() == m_state_standing);

	// Update the gait state machine
	if(m_gaitState == GS_INACTIVE)
	{
		if(enabled && standingState && shouldWalk)    // Are the preconditions for walking met?
		{
			m_gaitState = GS_REACHING_HALT_POSE;
			m_model->setState(m_state_walking);
			startReachHaltPose();
		}
	}
	else // <-- m_gaitState != GS_INACTIVE
	{
		if(!enabled) resetGait();                     // Has the gait been disabled?
		else if(m_gaitState == GS_REACHING_HALT_POSE) // Are we still in the process of reaching our halt pose?
		{
			if(!walkingState) resetGait();
			else if(m_reachedHalt)
			{
				if(shouldWalk) m_gaitState = GS_STARTING_WALKING;
				else resetGait();
			}
		}
		else // <-- enabled && m_gaitState == GS_STARTING_WALKING / GS_WALKING / GS_STOPPING_WALKING
		{
			if(!walkingState) resetGait();
			else if(!shouldWalk && !isWalking) resetGait();
			else if(!shouldWalk &&  isWalking) m_gaitState = GS_STOPPING_WALKING;
			else if( shouldWalk && !isWalking) m_gaitState = GS_STARTING_WALKING;
			else if( shouldWalk &&  isWalking) m_gaitState = GS_WALKING;
		}
	}

	// Increment the gait odometry ID if we are just about to start walking
	if(m_gaitState != oldGaitState && m_gaitState == GS_STARTING_WALKING)
		m_gait_odom.ID++;

	// Trigger a motion if one is pending
	if(m_motionPending != MID_NONE)
	{
		if(m_model->state() == m_state_standing) // A call to resetGait() above sets the state to standing and the gait state to GS_INACTIVE simultaneously. If the gait state changes without returning to the standing state (e.g. continuance of walking) then the pending motion is forgotten below.
		{
			const std::string& motion = motionName[m_motionPending];
			m_model->setState(m_model->registerState(ROBOT_STANDING_STATE_NAME PLAY_MOTION_TOKEN + motion));
			m_PM.plotEvent("play_" + motion);
			clearPendingMotion();
		}
		if(m_gaitState != GS_STOPPING_WALKING) // This ensures that if a non-motion gait command is received before the robot has stopped walking, and the robot starts walking again, the motion is aborted and not played. It also normally executes in the same cycle as a motion is triggered above.
			clearPendingMotion();
	}

	// Save the current motion pending state
	m_oldMotionPending = m_motionPending;

	// Plot data
	if(m_PM.getEnabled())
	{
		// Plot the current gait state
		m_PM.plotScalar(m_gaitState, PM_GAIT_STATE);
		
		// Plot events for gait state transitions
		if(m_gaitState != oldGaitState)
			plotGaitStateEvent();
	}

	// Trigger the motion module if the gait state is active
	bool trigger = (m_gaitState != GS_INACTIVE);

	// Publish the plot data now if the motion module isn't going to trigger
	if(!trigger)
	{
		m_PM.publish();
		m_PM.clear();
	}

	// Return whether the gait is active
	return trigger;
}

// Step function
// Note: It is assumed that this function is called straight after the trigger function if the trigger function returns true.
//       What is thereby really being assumed is that no ROS callbacks are executed between the trigger function and the step
//       function, giving the assurance that variables such as m_gaitCmd haven't changed.
void Gait::step()
{
	// Step the robot gait as required
	if(m_gaitState == GS_REACHING_HALT_POSE)
	{
		// Continue blending the robot to its halt pose
		updateGaitEngineInputs(m_engine->in);
		continueReachHaltPose();
	}
	else // <-- m_gaitState == GS_STARTING_WALKING / GS_WALKING / GS_STOPPING_WALKING
	{
		// Update and plot the inputs to the gait engine
		updateGaitEngineInputs(m_engine->in);
		plotGaitEngineInputs  (m_engine->in);

		// Execute a step of the gait engine
		m_engine->step();
	}

	// Plot and process the outputs from the gait engine
	plotGaitEngineOutputs   (m_engine->out);
	processGaitEngineOutputs(m_engine->out);

	// Publish the data stored in the plot manager
	m_PM.publish();
	m_PM.clear();
}

// Publish transforms function
void Gait::publishTransforms()
{
	// Save the current ROS time
	ros::Time now = ros::Time::now();

	// Publish the gait odometry if required to do so
	if(m_publishOdometry())
	{
		// Update the odometry timestamp
		m_gait_odom.header.stamp = now;
		
		// Publish the required odometry
		m_pub_odom.publish(m_gait_odom);
	}

	// Publish the gait transforms if required to do so
	if(m_publishTransforms())
	{
		// Update the TF frame timestamps (Note: m_tf_ego_floor and m_tf_odom point to elements of m_tf_transforms, so implicitly m_tf_transforms is updated here)
		m_tf_ego_floor->stamp_ = now;
		m_tf_odom->stamp_ = now;

		// Publish the required transforms
		m_tf_broadcaster.sendTransform(m_tf_transforms);
	}
}

// Update the transforms based on the information returned from the gait engine
void Gait::updateTransforms()
{
	// Update the ego_floor frame
	m_tf_ego_floor->setOrigin(tf::Vector3(0.0, 0.0, m_engine->out.odomPosition[2]));

	// Update the odometry frame position
	m_tf_odom->setOrigin(tf::Vector3(m_engine->out.odomPosition[0], m_engine->out.odomPosition[1], 0.0));

	// Update the odometry frame orientation (take the fused yaw component only)
	double w = m_engine->out.odomOrientation[0];
	double z = m_engine->out.odomOrientation[3];
	double wznorm = w*w + z*z;
	if(wznorm >= 1e-24)
	{
		wznorm = sqrt(wznorm);
		w /= wznorm;
		z /= wznorm;
	}
	else
	{
		w = 1.0;
		z = 0.0;
	}
	m_tf_odom->setRotation(tf::Quaternion(0.0, 0.0, z, w));

	// Calculate the fused yaw of the orientation
	double fyaw = 2.0*atan2(z,w);
	if(fyaw >   M_PI) fyaw -= M_2PI; // fyaw is now in [-2*pi,pi]
	if(fyaw <= -M_PI) fyaw += M_2PI; // fyaw is now in (-pi,pi]

	// Update the 2D gait odometry
	m_gait_odom.odom2D.x = m_engine->out.odomPosition[0];
	m_gait_odom.odom2D.y = m_engine->out.odomPosition[1];
	m_gait_odom.odom2D.theta = fyaw;

	// Update the 3D gait odometry
	m_gait_odom.odom.position.x = m_engine->out.odomPosition[0];
	m_gait_odom.odom.position.y = m_engine->out.odomPosition[1];
	m_gait_odom.odom.position.z = m_engine->out.odomPosition[2];
	m_gait_odom.odom.orientation.w = m_engine->out.odomOrientation[0];
	m_gait_odom.odom.orientation.x = m_engine->out.odomOrientation[1];
	m_gait_odom.odom.orientation.y = m_engine->out.odomOrientation[2];
	m_gait_odom.odom.orientation.z = m_engine->out.odomOrientation[3];
}

// Load the required gait engine dynamically via pluginlib
bool Gait::loadGaitEngine()
{
	// Load the requested gait engine
	ROS_INFO("Loading gait engine of name '%s' and type '%s'", m_gaitName.c_str(), getParamString().c_str());
	try
	{
		// Dynamically load an instance of the required gait engine
		m_engine = m_enginePluginLoader.createInstance(getParamString());
		if(!m_engine)
		{
			ROS_ERROR("Failed to load gait engine '%s': Null pointer returned", getParamString().c_str());
			return false;
		}

		// Pass the robot model to the gait engine
		m_engine->model = m_model;
		
		// Pass a read-only copy of the Gait class to the gait engine
		m_engine->setGaitOwner(this);
	}
	catch(pluginlib::PluginlibException& e)
	{
		ROS_ERROR("Failed to load gait engine '%s': %s", getParamString().c_str(), e.what());
		return false;
	}

	// Return that the engine was successfully loaded
	return true;
}

// Update the gait engine halt pose
void Gait::updateHaltPose()
{
	// Allow the gait engine to update its halt pose
	if(!m_updatedHalt)
	{
		m_engine->updateHaltPose();
		m_updatedHalt = true;
	}
}

// Update the gait engine inputs with the latest gait commands and settings
void Gait::updateGaitEngineInputs(GaitEngineInput& in)
{
	// Update the joint positions
	for(int i = 0; i < NUM_JOINTS; i++)
		in.jointPos[i] = m_model->joint(m_jointMap[i])->feedback.pos;

	// Update the nominal and true elapsed cycle times
	in.timestamp = m_now.toSec();
	in.nominaldT = m_dT;
	if(!m_lastNow.isZero())
		in.truedT = coerce((m_now - m_lastNow).toSec(), MIN_TRUEDT_FACTOR * in.nominaldT, MAX_TRUEDT_FACTOR * in.nominaldT);
	else
		in.truedT = in.nominaldT;
	
	// Update the gait command
	in.gaitCmd = m_gaitCmd;
	if(in.gaitCmd.walk)
	{
		float p = m_gaitCmdVecNormP();
		float norm = std::pow(std::pow(std::fabs(in.gaitCmd.linVelX), p) + std::pow(std::fabs(in.gaitCmd.linVelY), p) + std::pow(std::fabs(in.gaitCmd.angVelZ), p), 1/p);
		if(norm > m_gaitCmdVecNormMax())
		{
			in.gaitCmd.linVelX /= norm;
			in.gaitCmd.linVelY /= norm;
			in.gaitCmd.angVelZ /= norm;
		}
	}
	else
	{
		in.gaitCmd.linVelX = 0.0;
		in.gaitCmd.linVelY = 0.0;
		in.gaitCmd.angVelZ = 0.0;
	}

	// Update the motion parameters
	in.motionPending = (m_motionPending > MID_NONE && m_motionPending < MID_COUNT);
	if(in.motionPending)
	{
		in.motionID = (in.motionPending ? m_motionPending : MID_NONE);
		in.motionStance = m_motionStance;
		in.motionAdjustLeftFoot = m_motionAdjustLeftFoot;
		in.motionAdjustRightFoot = m_motionAdjustRightFoot;
	}
	else
	{
		in.motionID = MID_NONE;
		in.motionStance = STANCE_DEFAULT;
		in.motionAdjustLeftFoot = false;
		in.motionAdjustRightFoot = false;
	}

	// Plot the raw gait command
	plotRawGaitCommand();
}

// Process the outputs provided by the gait engine
void Gait::processGaitEngineOutputs(const GaitEngineOutput& out)
{
	// Write the joint commands requested by the gait engine to the RobotModel joint structs
	writeJointCommands(out);

	// Set the link support coefficients in RobotModel
	setSupportCoefficients(out.supportCoeffLeftLeg, out.supportCoeffRightLeg);

	// Update the TF transforms
	updateTransforms();
}

// Write the gait engine output joint commands into RobotModel
void Gait::writeJointCommands(const GaitEngineOutput& out)
{
	// Transcribe the joint commands requested by the gait engine to the RobotModel joint structs
	for(int i = 0; i < NUM_JOINTS; i++)
		setJointCommand(m_jointMap[i], out.jointCmd[i], out.jointEffort[i], out.useRawJointCmds);
}

// Write the gait engine output support coefficients into RobotModel
void Gait::setSupportCoefficients(double leftLegCoeff, double rightLegCoeff)
{
	// Calculate the trunk coefficient, attempting to make the coefficients sum to unity (if they do not already)
	double coeffSum = leftLegCoeff + rightLegCoeff;
	double trunkCoeff = (coeffSum < 1.0 ? 1.0 - coeffSum : 0.0);
	if(trunkCoeff < 1e-10) trunkCoeff = 0.0;

	// Write the support coefficients into RobotModel
	m_model->resetSupport();
	m_model->setSupportCoefficient(m_trunkLink, trunkCoeff);
	m_model->setSupportCoefficient(m_leftFootLink, leftLegCoeff);
	m_model->setSupportCoefficient(m_rightFootLink, rightLegCoeff);
}

// Plot the gait engine inputs
void Gait::plotGaitEngineInputs(const GaitEngineInput& in)
{
	// Don't do anything if the plot manager isn't enabled anyway
	if(!m_PM.getEnabled()) return;

	// Plot the required inputs
	m_PM.plotScalar(in.nominaldT,       PM_NOMINAL_DT);
	m_PM.plotScalar(in.truedT,          PM_TRUE_DT);
	m_PM.plotScalar(in.gaitCmd.linVelX, PM_GAITCMD_LIN_VEL_X);
	m_PM.plotScalar(in.gaitCmd.linVelY, PM_GAITCMD_LIN_VEL_Y);
	m_PM.plotScalar(in.gaitCmd.angVelZ, PM_GAITCMD_ANG_VEL_Z);
	m_PM.plotScalar(in.gaitCmd.walk,    PM_GAITCMD_WALK);

	// Note: We do not plot the in.jointPos[] member, as this information is plotted anyway in other more
	//       low level parts of the software framework.
}

// Plot the gait engine outputs
void Gait::plotGaitEngineOutputs(const GaitEngineOutput& out)
{
	// Don't do anything if the plot manager isn't enabled anyway
	if(!m_PM.getEnabled()) return;
	
	// Plot the required outputs
	for(int i = 0; i < NUM_JOINTS; i++)
	{
		m_PM.plotScalar(out.jointCmd[i],      PM_JOINTCMD_FIRST + i);
		m_PM.plotScalar(out.jointEffort[i],   PM_JOINTEFFORT_FIRST + i);
	}
	m_PM.plotScalar(out.useRawJointCmds,      PM_USE_RAW_JOINT_CMDS);
	m_PM.plotScalar(out.walking,              PM_WALKING);
	m_PM.plotScalar(out.supportCoeffLeftLeg,  PM_LEFT_SUPPORT_COEFF);
	m_PM.plotScalar(out.supportCoeffRightLeg, PM_RIGHT_SUPPORT_COEFF);
	m_PM.plotScalar(out.odomPosition[0],      PM_GAIT_ODOM_X);
	m_PM.plotScalar(out.odomPosition[1],      PM_GAIT_ODOM_Y);
	m_PM.plotScalar(out.odomPosition[2],      PM_GAIT_ODOM_Z);
	m_PM.plotScalar(0.1*(((m_gait_odom.ID - 1) % 10) + 1), PM_GAIT_ODOM_ID);
}

// Handle gait command data
void Gait::handleGaitCommand(const gait_msgs::GaitCommandConstPtr& cmd)
{
	// Transcribe the required gait command, if the joystick doesn't currently have a lock on the gait command
	if(!m_joystickGaitCmdLock)
	{
		if(cmd->motion > MID_NONE && cmd->motion < MID_COUNT)
		{
			switch(cmd->motion)
			{
				case MID_KICK_LEFT:  setPendingMotion((MotionID) cmd->motion, STANCE_KICK, false, true); break;
				case MID_KICK_RIGHT: setPendingMotion((MotionID) cmd->motion, STANCE_KICK, true, false); break;
				default:             setPendingMotion((MotionID) cmd->motion, STANCE_DEFAULT, true, true); break;
			}
		}
		else
		{
			m_gaitCmd.linVelX = cmd->gcvX;
			m_gaitCmd.linVelY = cmd->gcvY;
			m_gaitCmd.angVelZ = cmd->gcvZ;
			m_gaitCmd.walk = cmd->walk;
		}
	}
}

// Plot the raw gait command
void Gait::plotRawGaitCommand()
{
	// Don't do anything if the plot manager isn't enabled anyway
	if(!m_PM.getEnabled()) return;

	// Plot the required data
	m_PM.plotScalar(m_gaitCmd.linVelX, PM_GAITCMDRAW_LIN_VEL_X);
	m_PM.plotScalar(m_gaitCmd.linVelY, PM_GAITCMDRAW_LIN_VEL_Y);
	m_PM.plotScalar(m_gaitCmd.angVelZ, PM_GAITCMDRAW_ANG_VEL_Z);
	m_PM.plotScalar(m_gaitCmd.walk,    PM_GAITCMDRAW_WALK);
}

// Set a pending motion
void Gait::setPendingMotion(MotionID ID, MotionStance stance, bool adjustLeft, bool adjustRight)
{
	// Update the motion variables
	m_motionPending = ID;
	m_motionStance = stance;
	m_motionAdjustLeftFoot = adjustLeft;
	m_motionAdjustRightFoot = adjustRight;
}

// Clear a pending motion
void Gait::clearPendingMotion()
{
	// Clear any pending motion
	m_motionPending = MID_NONE;
	m_motionStance = STANCE_DEFAULT;
	m_motionAdjustLeftFoot = true;
	m_motionAdjustRightFoot = true;
}

// Start blending to the robot's halt pose
void Gait::startReachHaltPose()
{
	// Declare variables
	int i;

	// Update the gait engine's halt pose
	updateHaltPose();

	// Construct a list of the commands from the last robotcontrol cycle
	robotcontrol::Joint::Command* lastCmd[NUM_JOINTS];
	for(i = 0; i < NUM_JOINTS; i++)
		lastCmd[i] = &(m_model->joint(m_jointMap[i])->lastCmd);

	// Compute the required joint trajectories to reach the halt pose using a spline-based approach
	double T = 0.0;
	for(i = 0; i < NUM_JOINTS; i++)
	{
		m_jointSpline[i].setParams(lastCmd[i]->pos, lastCmd[i]->vel, m_engine->haltJointCmd[i], 0.0, REACH_HALT_POSE_VEL, REACH_HALT_POSE_ACC);
		if(m_jointSpline[i].T() > T)
			T = m_jointSpline[i].T();
	}
	m_reachDuration = T;

	// Compute the required joint effort trajectories to reach the halt pose using a spline-base approach
	for(i = 0; i < NUM_JOINTS; i++)
		m_jointEffortSpline[i].setParams(lastCmd[i]->effort, m_engine->haltJointEffort[i], m_reachDuration);

	// Save the start time of the motion blend
	m_reachStartTime = m_now;
	
	// Reset the halt pose reached flag
	m_reachedHalt = false;

	// Display a console message
	ROS_INFO("Motion blend to the gait halt pose is starting (expected duration %.3fs)...", m_reachDuration + REACH_HALT_POSE_DELAY);
}

// Continue blending to the robot's halt pose
void Gait::continueReachHaltPose() // This function needs to set *all* gait engine outputs to appropriate values!
{
	// Update the gait engine's halt pose
	updateHaltPose();

	// Calculate the elapsed time since the start of the halt pose motion blending
	double t = (m_now - m_reachStartTime).toSec();

	// Evaluate the spline-based joint trajectories for the current time step
	for(int i = 0; i < NUM_JOINTS; i++)
	{
		m_engine->out.jointCmd[i] = m_jointSpline[i].x(t);
		m_engine->out.jointEffort[i] = m_jointEffortSpline[i].x(t);
	}
	m_engine->out.useRawJointCmds = m_engine->haltUseRawJointCmds;
	m_engine->out.walking = false;
	m_engine->out.supportCoeffLeftLeg = 0.5;
	m_engine->out.supportCoeffRightLeg = 0.5;

	// Check whether we have completed the required motion blend and successfully arrived at the halt pose (it may have changed in the meantime!)
	if(t >= m_reachDuration + REACH_HALT_POSE_DELAY)
	{
		// Compare the currently commanded joint positions and efforts to the required halt pose commands and efforts
		bool reachedHaltPose = true;
		for(int i = 0; i < NUM_JOINTS; i++)
		{
			if((fabs(m_engine->out.jointCmd[i] - m_engine->haltJointCmd[i]) > REACH_HALT_POSE_TOL) ||
			   (fabs(m_engine->out.jointEffort[i] - m_engine->haltJointEffort[i]) > REACH_HALT_EFFORT_TOL))
			{
				reachedHaltPose = false;
				break;
			}
		}

		// Stop or restart the motion blend depending on whether the desired halt pose has been reached
		if(reachedHaltPose)
			stopReachHaltPose();
		else
			startReachHaltPose();
	}
}

// Stop blending to the robot's halt pose
void Gait::stopReachHaltPose()
{
	// Set the halt pose reached flag
	m_reachedHalt = true;

	// Display a console message
	ROS_INFO("Motion blend to the gait halt pose has finished.");
}

// Handle joystick command data
void Gait::handleJoystickData(const sensor_msgs::JoyConstPtr& joy)
{
	// Ignore this message if the use of a joystick is not enabled
	if(!m_joystickEnabled) return;

	// Ignore this message if it doesn't contain enough information
	if(joy->axes.size() < 3 || joy->buttons.size() < 4) return;

	// Toggle the joystick gait command lock if a falling edge is detected on button 0
	if(m_joystickButton0Pressed && !joy->buttons[0])
		setJoystickGaitCmdLock(!m_joystickGaitCmdLock);
	m_joystickButton0Pressed = joy->buttons[0];

	// Nothing more to do if we don't have joystick lock
	if(!m_joystickGaitCmdLock)
	{
		m_joystickButton1Pressed = joy->buttons[1];
		m_joystickButton2Pressed = joy->buttons[2];
		m_joystickButton3Pressed = joy->buttons[3];
		return;
	}

	// Toggle the gait command walk flag if a falling edge is detected on button 1
	if(m_joystickButton1Pressed && !joy->buttons[1])
	{
		m_gaitCmd.reset(!m_gaitCmd.walk); // Toggle and reset the gait command
	}
	m_joystickButton1Pressed = joy->buttons[1];

	// Trigger a right kick motion if a falling edge is detected on button 2
	if(m_joystickButton2Pressed && !joy->buttons[2])
		setPendingMotion(MID_KICK_RIGHT, STANCE_KICK, true, false);
	m_joystickButton2Pressed = joy->buttons[2];

	// Trigger a left kick motion if a falling edge is detected on button 3
	if(m_joystickButton3Pressed && !joy->buttons[3])
		setPendingMotion(MID_KICK_LEFT, STANCE_KICK, false, true);
	m_joystickButton3Pressed = joy->buttons[3];

	// Transcribe the gait command
	if(m_gaitCmd.walk)
	{
		m_gaitCmd.linVelX = joy->axes[1];
		m_gaitCmd.linVelY = joy->axes[0];
		m_gaitCmd.angVelZ = joy->axes[2];
	}
}

// Update the joystick lock status
void Gait::setJoystickGaitCmdLock(bool lock)
{
	// Update the joystick gait command lock variable, checking whether the status is being changed
	if(m_joystickGaitCmdLock != lock)
	{
		m_joystickGaitCmdLock = lock;
		m_gaitCmd.reset(); // This resets m_gaitCmd.walk to false, making the walking stop...
		if(m_joystickGaitCmdLock)
			m_PM.plotEvent("joystickEnable");
		else
			m_PM.plotEvent("joystickDisable");
	}
}

// Callback for when the enableJoystick parameter is updated
void Gait::callbackEnableJoystick()
{
	// Update the joystick enabled variable, checking whether the status is being changed
	if(m_joystickEnabled != m_enableJoystick())
	{
		m_joystickEnabled = m_enableJoystick();
		setJoystickGaitCmdLock(false);
	}
}

// Configure the TF transforms
void Gait::configureTransforms()
{
	// Create ROS node handle
	ros::NodeHandle nh("~");

	// Advertise the gait odometry topic
	m_pub_odom = nh.advertise<gait_msgs::GaitOdom>("/gait/odometry", 1);

	// Initialise the gait odometry information
	m_gait_odom.header.frame_id = gait::gaitOdomFrame;
	m_gait_odom.header.seq = 0;
	m_gait_odom.header.stamp.fromNSec(0);
	m_gait_odom.ID = 0;
	m_gait_odom.odom.position.x = 0.0;
	m_gait_odom.odom.position.y = 0.0;
	m_gait_odom.odom.position.z = 0.0;
	m_gait_odom.odom.orientation.w = 1.0;
	m_gait_odom.odom.orientation.x = 0.0;
	m_gait_odom.odom.orientation.y = 0.0;
	m_gait_odom.odom.orientation.z = 0.0;
	m_gait_odom.odom2D.x = 0.0;
	m_gait_odom.odom2D.y = 0.0;
	m_gait_odom.odom2D.theta = 0.0;

	// Initialise the TF transform broadcaster
	m_tf_transforms.resize(2);
	m_tf_ego_floor = &(m_tf_transforms[0]);
	m_tf_odom = &(m_tf_transforms[1]);

	// Define and initialise the ego_floor frame
	m_tf_ego_floor->frame_id_ = "/ego_floor";
	m_tf_ego_floor->child_frame_id_ = "/ego_rot";
	m_tf_ego_floor->setIdentity();

	// Define and initialise the odometry frame
	m_tf_odom->frame_id_ = gaitOdomFrame;
	m_tf_odom->child_frame_id_ = "/ego_floor";
	m_tf_odom->setIdentity();
}

// Service handler for resetting the gait odometry
bool Gait::handleResetOdometry(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
	// Reset the gait engine's odometry
	m_engine->setOdometry(0.0, 0.0, 0.0);
	m_engine->updateOdometry();

	// Increment the gait odometry ID to safely declare the jump in odometry
	updateTransforms();
	m_gait_odom.ID++;

	// Display a message that this service was called
	ROS_WARN("Odometry has been set to (%.3f, %.3f, %.3f)", 0.0, 0.0, 0.0);

	// Return success
	return true;
}

// Service handler for setting the gait odometry
bool Gait::handleSetOdometry(gait_msgs::SetOdomRequest &req, gait_msgs::SetOdomResponse &res)
{
	// Set the gait engine's odometry
	m_engine->setOdometry(req.posX, req.posY, req.rotZ);
	m_engine->updateOdometry();

	// Increment the gait odometry ID to safely declare the jump in odometry
	updateTransforms();
	m_gait_odom.ID++;

	// Display a message that this service was called
	ROS_WARN("Odometry has been set to (%.3f, %.3f, %.3f)", req.posX, req.posY, req.rotZ);

	// Return success
	return true;
}

// Joint functions
bool Gait::constructJointMap()
{
	// Initialise the joint map
	for(int i = 0; i < NUM_JOINTS; i++)
		m_jointMap[i] = 0;

	// Attempt to locate and map the joints that we need for the gait
	try
	{
		for(int i = 0; i < NUM_JOINTS; i++)
			m_jointMap[i] = m_model->jointIndex(gait::jointName[i]);
	}
	catch(std::logic_error& e)
	{
		ROS_ERROR("Failed to construct joint map for %s gait: %s!", gaitName().c_str(), e.what());
		return false;
	}

	// Return success
	return true;
}

// Reset the gait state machine
void Gait::resetGait()
{
	// Reset the gait state
	m_gaitState = GS_INACTIVE;

	// Reset state-related variables
	m_reachedHalt = false;
	m_gaitCmd.reset();

	// Reset the robot state to standing if it is currently in our unique walking state
	if(m_model->state() == m_state_walking)
		m_model->setState(m_state_standing); // TODO: This is quite dirty as the robot is not necessarily in a correct standing pose at this point (think mid-gait or mid-reach halt pose or simply just halt pose). Introduce a post-gait state that captures this indeterminism?

	// Reset the gait engine
	m_engine->reset();
	m_engine->setOdometry(0.0, 0.0, 0.0);
	m_engine->resetBase();
	m_engine->updateOdometry();

	// Increment the gait odometry ID to safely declare the jump in odometry
	updateTransforms();
	m_gait_odom.ID++;

	// Update the plot data
	plotRawGaitCommand();
	plotGaitEngineInputs (m_engine->in);
	plotGaitEngineOutputs(m_engine->out);
}

// Plot an event for the current gait state
void Gait::plotGaitStateEvent()
{
	// Don't do anything if the plot manager isn't enabled anyway
	if(!m_PM.getEnabled()) return;

	// Plot the required event
	if(m_gaitState == GS_INACTIVE) m_PM.plotEvent("Inactive");
	else if(m_gaitState == GS_WALKING) m_PM.plotEvent("Walking");
	else if(m_gaitState == GS_STARTING_WALKING) m_PM.plotEvent("StartingWalking");
	else if(m_gaitState == GS_STOPPING_WALKING) m_PM.plotEvent("StoppingWalking");
	else if(m_gaitState == GS_REACHING_HALT_POSE) m_PM.plotEvent("ReachingHaltPose");
}

// Configure the plot manager
void Gait::configurePlotManager()
{
	// Gait engine inputs
	m_PM.setName(PM_NOMINAL_DT,        "nominaldT");
	m_PM.setName(PM_TRUE_DT,           "truedT");
	m_PM.setName(PM_GAITCMD_LIN_VEL_X, "gaitCmd/linVelX");
	m_PM.setName(PM_GAITCMD_LIN_VEL_Y, "gaitCmd/linVelY");
	m_PM.setName(PM_GAITCMD_ANG_VEL_Z, "gaitCmd/angVelZ");
	m_PM.setName(PM_GAITCMD_WALK,      "gaitCmd/walk");

	// Gait engine outputs
	for(int i = 0; i < NUM_JOINTS; i++)
	{
		m_PM.setName(PM_JOINTCMD_FIRST + i,    "jointCmd/" + gait::jointName[i]);
		m_PM.setName(PM_JOINTEFFORT_FIRST + i, "jointEffort/" + gait::jointName[i]);
	}
	m_PM.setName(PM_USE_RAW_JOINT_CMDS,  "useRawJointCmds");
	m_PM.setName(PM_WALKING,             "walking");
	m_PM.setName(PM_LEFT_SUPPORT_COEFF,  "supportCoeff/leftLeg");
	m_PM.setName(PM_RIGHT_SUPPORT_COEFF, "supportCoeff/rightLeg");
	m_PM.setName(PM_GAIT_ODOM_X,         "gaitOdom/X");
	m_PM.setName(PM_GAIT_ODOM_Y,         "gaitOdom/Y");
	m_PM.setName(PM_GAIT_ODOM_Z,         "gaitOdom/Z");
	m_PM.setName(PM_GAIT_ODOM_ID,        "gaitOdom/ID");

	// Raw gait command
	m_PM.setName(PM_GAITCMDRAW_LIN_VEL_X, "gaitCmdRaw/linVelX");
	m_PM.setName(PM_GAITCMDRAW_LIN_VEL_Y, "gaitCmdRaw/linVelY");
	m_PM.setName(PM_GAITCMDRAW_ANG_VEL_Z, "gaitCmdRaw/angVelZ");
	m_PM.setName(PM_GAITCMDRAW_WALK,      "gaitCmdRaw/walk");

	// Gait state
	m_PM.setName(PM_GAIT_STATE, "gaitState");
}

// Callback for when the plotData parameter is updated
void Gait::callbackPlotData()
{
	// Enable or disable plotting as required
	if(m_plotData())
		m_PM.enable();
	else
		m_PM.disable();
}

PLUGINLIB_EXPORT_CLASS(gait::Gait, robotcontrol::MotionModule)
// EOF