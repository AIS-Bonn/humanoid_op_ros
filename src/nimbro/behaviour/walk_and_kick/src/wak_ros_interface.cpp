// Walk and kick: ROS interface class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_ros_interface.h>
#include <walk_and_kick/wak_game_state.h>
#include <walk_and_kick/wak_beh_state.h>
#include <walk_and_kick/walk_and_kick.h>
#include <rc_utils/slope_limiter.h>
#include <boost/make_shared.hpp>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// WAKRosInterface class
//

// Constructor
WAKRosInterface::WAKRosInterface(WalkAndKick* wak) : nh("~"), wak(wak), config(wak->config), TCRI(nh), PM(NULL), MM(NULL), MMB(NULL)
{
	// Check whether we are a fake robot
	std::string robotName;
	nh.param<std::string>("/robot_name", robotName, std::string());
	m_isFakeRobot = (robotName == "xs0");

	// Create marker managers
	MM = new WAKMarkerMan(config);
	MMB = new WAKBagMarkerMan(config);

	// One-shot variable for initial publishing of a neutral state
	m_publishedNeutral = false;

	// Reset the data variables
	resetData();

	// Reset the LED variables
	m_lastLEDState = 0x00;
	m_lastLEDMask = 0x00;
	m_lastLEDTime.fromNSec(0);
	m_blinkRGBLED = false;

	// Reset the behaviour state variables
	m_behState.running = false;
	m_behState.modeID = -1;
	m_behState.modeString.clear();
	m_behState.gameStateID = -1;
	m_behState.gameStateString.clear();
	m_behState.behStateID = -1;
	m_behState.behStateString.clear();

	// ROS subscribers
	m_sub_button = nh.subscribe("/button", 1, &WAKRosInterface::handleButtonData, this);
	m_sub_robotState = nh.subscribe("/robotcontrol/state", 1, &WAKRosInterface::handleRobotStateData, this);
	m_sub_robotHeading = nh.subscribe("/robotmodel/robot_heading", 1, &WAKRosInterface::handleRobotHeadingData, this);
	m_sub_stoppedGaitCmd = nh.subscribe("/walk_and_kick/stoppedGaitCmd", 1, &WAKRosInterface::handleStoppedGaitCommand, this);
	m_sub_robotDiagnostics = nh.subscribe("/robotcontrol/diagnostics", 1, &WAKRosInterface::handleRobotDiagnosticsData, this);
	m_sub_visionOutput = nh.subscribe("/vision/outputs", 1, &WAKRosInterface::handleVisionOutputData, this);
	m_sub_diveDecision = nh.subscribe("/dive_predictor/decision", 1, &WAKRosInterface::handleDiveDecision, this);

	// ROS publishers
	m_pub_gaitCmd = nh.advertise<gait_msgs::GaitCommand>("/gaitCommand", 1);
	m_pub_headCmd = nh.advertise<head_control::LookAtTarget>("/robotcontrol/headcontrol/target", 1);
	m_pub_leds = nh.advertise<nimbro_op_interface::LEDCommand>("/led", 5);
	m_pub_state = nh.advertise<walk_and_kick::BehaviourState>("/walk_and_kick/state", 1);
	m_pub_teamComms = nh.advertise<walk_and_kick::TeamCommsData>("/walk_and_kick/teamPacket", 1);

	// ROS service servers
	m_srv_visualiseClear = nh.advertiseService("/walk_and_kick/visualiseClear", &WAKRosInterface::handleVisualiseClear, this);
	m_srv_visualiseDBH = nh.advertiseService("/walk_and_kick/visualiseDBH", &WAKRosInterface::handleVisualiseDBH, this);
	m_srv_visualiseDbApp = nh.advertiseService("/walk_and_kick/visualiseDbApp", &WAKRosInterface::handleVisualiseDbApp, this);
	m_srv_visualiseGcvXY = nh.advertiseService("/walk_and_kick/visualiseGcvXY", &WAKRosInterface::handleVisualiseGcvXY, this);

	// TF transforms
	m_tfBehField.frame_id_ = "/ego_floor";
	m_tfBehField.child_frame_id_ = "/beh_field";
	m_tfBehField.setIdentity();

	// Start listening to the game controller
	GCRI.startListening();

	// Start listening to team communications (this should be the only time the team communications are ever started)
	TCRI.startListening();

	// Create the plot manager
	const int plotsPerRobot = 2; // This should correspond to the number of plots per robot in the for-loop below
	const unsigned int numRobotSlots = TCRI.nextRobotUID();
	PM = new plot_msgs::PlotManagerFS(PM_COUNT + plotsPerRobot*numRobotSlots, "/walk_and_kick");
	PM->setTimestamp(ros::Time(0, 0));

	// Set the names of the dynamically sized team communications plot variables
	const TCRobotListenerList& teamComms = TCRI.teamComms();
	for(TCRobotListenerList::const_iterator rtcIt = teamComms.begin(); rtcIt != teamComms.end(); ++rtcIt)
	{
		const TCRobotListenerConstPtr& TCRL = *rtcIt;
		std::ostringstream ss;
		ss << "TeamComms/" << TCRL->robot << "/";
		std::string prefix = ss.str();
		PM->setName(PM_COUNT + 0*numRobotSlots + TCRL->robotUID, prefix + "reasonInvalid");
		PM->setName(PM_COUNT + 1*numRobotSlots + TCRL->robotUID, prefix + "timeSinceData");
	}

	// Configure the plot manager
	configurePlotManager(PM);

	// Set up config parameter callbacks
	config.sListenToGC.setCallback(boost::bind(&WAKRosInterface::updateModeStateText, this), true);
	config.plotData.setCallback(boost::bind(&WAKRosInterface::callbackPlotData, this), true);
	config.debugBlockGCPackets.setCallback(boost::bind(&WAKRosInterface::handleBlockGCPackets, this), true);
	config.debugNoStoppedGcv.setCallback(boost::bind(&WAKRosInterface::handleNoStoppedGCV, this), true);
}

// Destructor
WAKRosInterface::~WAKRosInterface()
{
	// Delete the plot manager
	delete PM;

	// Delete the marker managers
	delete MM;
	delete MMB;
}

// Reset function
void WAKRosInterface::resetData()
{
	// Reset the game controller data
	GCRI.resetData();

	// Reset the team communications data
	TCRI.resetData();

	// Reset the data variables
	button = BTN_STOP;
	ballVec.setZero();
	ballConf = 0.0f;
	ballDetected = false;
	ballTime.fromNSec(0);
	goalPostList.clear();
	goalPostTime.fromNSec(0);
	obstacleList.clear();
	obstacleTime.fromNSec(0);
	robotPoseVec.setZero();
	robotPoseConf = 0.0f;
	robotPoseTime.fromNSec(0);
	robotHeading = 0.0f;
	robotHeadingTime.fromNSec(0);
	robotcontrolCommsOk = true;
	diveDecision = DD_NONE;

	// Reset the internal variables
	m_relaxed = true;
	m_initing = false;
	m_standing = false;
	m_walking = false;
	m_kicking = false;
	m_fallen = false;
	m_lastButton = BTN_UNKNOWN;
	m_btnHeadCmd = true;
	m_headCmd.enabled = false;
	m_headCmd.vec.y = config.gazePitchNeutral();
	m_headCmd.vec.z = 0.0;
	gait::resetGaitCommand(m_stoppedGaitCmd);
	m_buttonName = buttonStateName(button);

	// Reset the plot manager timestamp to zero
	if(PM)
		PM->setTimestamp(ros::Time(0, 0));
}

// Update function
void WAKRosInterface::update(const ros::Time& now, bool LED4)
{
	// Fade out detection confidences if we haven't seen a ROS message in a while
	if((now - ballTime).toSec() >= config.confDecayTime())
		ballConf *= config.confDecayFactorBall;
	if((now - goalPostTime).toSec() >= config.confDecayTime())
	{
		for(size_t i = 0; i < goalPostList.size(); ++i)
			goalPostList[i].conf *= config.confDecayFactorGoal;
	}
	if((now - obstacleTime).toSec() >= config.confDecayTime())
	{
		for(size_t i = 0; i < obstacleList.size(); ++i)
			obstacleList[i].conf *= config.confDecayFactorObst;
	}
	if((now - robotPoseTime).toSec() >= config.confDecayTime())
		robotPoseConf *= config.confDecayFactorPose;

	// Update the LEDs
	int haveBallNow = (ballDetected ? LEDCommand::LED2 : 0x00);
	int haveBall = (ballConf > config.confLimitBall() ? LEDCommand::LED3 : 0x00);
	int LED4State = (LED4 ? LEDCommand::LED4 : 0x00);
	writeLEDCommand(LEDCommand::LED2 | LEDCommand::LED3 | LEDCommand::LED4, haveBallNow | haveBall | LED4State);

	// Handle changes in the button state
	if(button != m_lastButton)
	{
		m_lastButton = button;
		m_buttonName = buttonStateName(button);
		if(config.plotData())
			PM->plotEvent("Btn " + m_buttonName);
		updateModeStateText();
	}

	// Plotting
	if(config.plotData())
		PM->plotScalar(robotcontrolCommsOk * PMSCALE_COMMSOK, PM_RI_COMMSOK);

	// Visualisation markers
	if(MM->willPublish())
		MM->ModeStateText.updateAdd();
}

// Write the required outputs to the actuators
void WAKRosInterface::writeActuators(const ActuatorVars& actVar)
{
	// Copy out the commanded walking velocity
	Vec3f GCV = actVar.GCV;

	// Adjust the commanded walking velocity for XY shear
	float shear = sqrt(config.globalGcvShearXY());
	GCV.x() /= shear;
	GCV.y() *= shear;

	// Adjust for the maximum allowed backwards walking velocity
	if(GCV.x() < -config.globalGcvBwdLimit())
		GCV *= fabs(config.globalGcvBwdLimit() / GCV.x());

	// Adjust for the maximum allowed total gait velocity
	float gcvNorm = GCV.norm();
	if(gcvNorm > config.globalGcvSpeedLimit())
		GCV *= fabs(config.globalGcvSpeedLimit() / gcvNorm);

	// Construct the walking gait command
	gait_msgs::GaitCommand cmd;
	if(config.debugForceHalt())
	{
		cmd.gcvX = 0.0f;
		cmd.gcvY = 0.0f;
		cmd.gcvZ = 0.0f;
		cmd.walk = false;
	}
	else
	{
		cmd.gcvX = GCV.x();
		cmd.gcvY = GCV.y();
		cmd.gcvZ = GCV.z();
		cmd.walk = !actVar.halt;
	}

	// Trigger a kicking or dive motion if required
	if(actVar.doKick && !config.debugForceNoKick() && !config.debugForceHalt())
	{
		if(actVar.rightKick)
			cmd.motion = gait::MID_KICK_RIGHT;
		else
			cmd.motion = gait::MID_KICK_LEFT;
		cmd.walk = false;
	}
	else if(actVar.doDive != DD_NONE && !config.debugForceNoDive() && !config.debugForceHalt())
	{
		bool disableWalk = true;
		if(actVar.doDive == DD_RIGHT)
			cmd.motion = gait::MID_DIVE_RIGHT;
		else if(actVar.doDive == DD_LEFT)
			cmd.motion = gait::MID_DIVE_LEFT;
		else if(actVar.doDive == DD_SIT)
			cmd.motion = gait::MID_DIVE_SIT;
		else
			disableWalk = false;
		if(disableWalk)
			cmd.walk = false;
	}

	// Override the gait command if the stopped gait command is applicable and telling us to walk
	if(button == BTN_STOP && m_stoppedGaitCmd.walk && !config.debugNoStoppedGcv() && !config.debugForceHalt())
	{
		cmd = m_stoppedGaitCmd;
		ROS_INFO_THROTTLE(5.0, "Walking with GCV (%.2f, %.2f, %.2f) due to messages on stopped state gait command topic!", cmd.gcvX, cmd.gcvY, cmd.gcvZ);
	}

	// Publish the walking/kicking gait command
	m_pub_gaitCmd.publish(cmd);

	// Calculate the required gaze orientation
	double GazeYaw = SlopeLimiter::eval(actVar.gazeYaw, m_headCmd.vec.z, config.gazeVelLimit() * TINC);
	double GazePitch = SlopeLimiter::eval(actVar.gazePitch, m_headCmd.vec.y, config.gazeVelLimit() * TINC);
	GazeYaw = coerceAbs<double>(GazeYaw, config.gazeYawAbsMax());
	GazePitch = coerce<double>(GazePitch, config.gazePitchMin(), config.gazePitchMax());

	// Publish the head control command
	if(button != BTN_STOP || m_btnHeadCmd)
	{
		if(button != BTN_STOP)
			m_btnHeadCmd = true;
		else
			m_btnHeadCmd = (fabs(GazePitch - m_headCmd.vec.y) > 1e-6 || fabs(GazeYaw - m_headCmd.vec.z) > 1e-6);
		m_headCmd.enabled = true;
		m_headCmd.is_angular_data = true;
		m_headCmd.is_relative = false;
		m_headCmd.pitchEffort = 0.0f; // Use the default pitch effort
		m_headCmd.yawEffort = 0.0f; // Use the default yaw effort
		m_headCmd.vec.x = 0.0;
		m_headCmd.vec.y = GazePitch;
		m_headCmd.vec.z = GazeYaw;
		m_pub_headCmd.publish(m_headCmd);
	}

	// Plotting
	if(config.plotData())
	{
		PM->plotScalar(m_headCmd.vec.y, PM_RI_PUBGAZEPITCH);
		PM->plotScalar(m_headCmd.vec.z, PM_RI_PUBGAZEYAW);
		PM->plotScalar(cmd.gcvX, PM_RI_PUBGCVX);
		PM->plotScalar(cmd.gcvY, PM_RI_PUBGCVY);
		PM->plotScalar(cmd.gcvZ, PM_RI_PUBGCVZ);
		PM->plotScalar(cmd.walk * PMSCALE_WALK, PM_RI_PUBWALK);
		PM->plotScalar((cmd.motion == gait::MID_KICK_LEFT) * PMSCALE_MOTION, PM_RI_PUBKICKLEFT);
		PM->plotScalar((cmd.motion == gait::MID_KICK_RIGHT) * PMSCALE_MOTION, PM_RI_PUBKICKRIGHT);
	}

	// Visualisation markers
	if(MM->willPublish())
	{
		MM->GcvXY.update(cmd.gcvX, cmd.gcvY, 0.0);
		if(fabs(cmd.gcvZ) < 0.03f)
		{
			MM->GcvZ.hide();
			MM->GcvZ.updateAdd();
		}
		else
		{
			MM->GcvZ.show();
			MM->GcvZ.update(0.35, 0.0, 0.0, 0.35, 0.7*cmd.gcvZ, 0.0);
		}
	}
}

// Publish a command to put the head into a neutral position
void WAKRosInterface::writeNeutralHead()
{
	// Publish a neutral head control command
	m_headCmd.enabled = true;
	m_headCmd.is_angular_data = true;
	m_headCmd.is_relative = false;
	m_headCmd.pitchEffort = 0.0f; // Use the default pitch effort
	m_headCmd.yawEffort = 0.0f; // Use the default yaw effort
	m_headCmd.vec.x = 0.0;
	m_headCmd.vec.y = config.gazePitchNeutral();
	m_headCmd.vec.z = 0.0;
	m_pub_headCmd.publish(m_headCmd);

	// Plotting
	if(config.plotData())
	{
		PM->plotScalar(m_headCmd.vec.y, PM_RI_PUBGAZEPITCH);
		PM->plotScalar(m_headCmd.vec.z, PM_RI_PUBGAZEYAW);
	}
}

// Publish a zero gait command
void WAKRosInterface::writeZeroGcv()
{
	// Publish a zero gait command
	gait_msgs::GaitCommand cmd;
	gait::resetGaitCommand(cmd);
	m_pub_gaitCmd.publish(cmd);

	// Plotting
	if(config.plotData())
	{
		PM->plotScalar(cmd.gcvX, PM_RI_PUBGCVX);
		PM->plotScalar(cmd.gcvY, PM_RI_PUBGCVY);
		PM->plotScalar(cmd.gcvZ, PM_RI_PUBGCVZ);
		PM->plotScalar(cmd.walk * PMSCALE_WALK, PM_RI_PUBWALK);
		PM->plotScalar((cmd.motion == gait::MID_KICK_LEFT) * PMSCALE_MOTION, PM_RI_PUBKICKLEFT);
		PM->plotScalar((cmd.motion == gait::MID_KICK_RIGHT) * PMSCALE_MOTION, PM_RI_PUBKICKRIGHT);
	}
}

// Publish the current behaviour state
void WAKRosInterface::publishBehaviourState(const WAKGameState* gameState, const WAKBehState* behState)
{
	// Ignore null pointers
	if(!gameState || !behState) return;

	// Get the current ROS time
	ros::Time now = ros::Time::now();

	// See whether we have any new information to publish
	bool newMode = (button != m_behState.modeID);                    // This assumes that normal modes have IDs >= 0 and always have the same string representation
	bool newGameState = (gameState->id() != m_behState.gameStateID); // This assumes that normal game states have IDs >= 0 and always have the same string representation
	bool newBehState = (behState->id() != m_behState.behStateID);    // This assumes that normal behaviour states have IDs >= 0 and always have the same string representation
	bool newTime = ((now - m_behState.header.stamp).toSec() >= 0.4);

	// Publish the required behaviour state if we have reason to
	if(newMode || newGameState || newBehState || newTime)
	{
		m_behState.header.seq++;
		m_behState.header.stamp = now;
		if(newMode)
		{
			m_behState.modeID = button;
			m_behState.modeString = buttonStateName(button);
		}
		if(newGameState)
		{
			m_behState.gameStateID = gameState->id();
			m_behState.gameStateString = gameState->name();
		}
		if(newBehState)
		{
			m_behState.behStateID = behState->id();
			m_behState.behStateString = behState->name();
		}
		m_behState.running = (m_behState.modeID != BTN_STOP);
		m_pub_state.publish(m_behState);
	}
}

// Send the required robot pose TF transform
void WAKRosInterface::sendTransform(const Vec3f& RobotPose)
{
	// Update the required transform
	m_tfBehField.stamp_ = PM->getTimestamp();
	m_tfBehField.setOrigin(tf::Vector3(RobotPose.x(), RobotPose.y(), 0.0));
	m_tfBehField.setRotation(tf::Quaternion(0.0, 0.0, sin(0.5*RobotPose.z()), cos(0.5*RobotPose.z())));
	m_tfBehField.setData(m_tfBehField.inverse());

	// Send the required transform
	m_tfBroadcaster.sendTransform(m_tfBehField);
}

// Plot data configuration variable callback
void WAKRosInterface::callbackPlotData()
{
	// Enable or disable plotting as required
	if(config.plotData())
		PM->enable();
	else
		PM->disable();
}

// Handle new button data
void WAKRosInterface::handleButtonData(const nimbro_op_interface::ButtonConstPtr& msg)
{
	// Check whether the middle button has been pressed
	if((msg->button == 1) && config.enableWAK())
	{
		// Handle the button press differently depending on whether it is a long press or not
		if(msg->longPress != 0)
		{
			// Toggle the listen to game controller config parameter
			if(config.sListenToGC())
				config.sListenToGC.set(false);
			else
			{
				config.sListenToGC.set(true);
				config.gcEnable.set(true);
			}

			// Inform the user as to the current state of the walk and kick
			ROS_INFO("Button 1 long-pressed: Config parameter now says %s", (config.sListenToGC() ? "LISTEN TO GC" : "IGNORE GC"));
		}
		else
		{
			// Wrapped increment the button state
			int nextButtonState = button + 1;
			if(nextButtonState <= BTN_UNKNOWN || nextButtonState >= (config.simpleModeButton() ? BTN_PLAY + 1 : BTN_COUNT))
				nextButtonState = BTN_UNKNOWN + 1;
			button = (ButtonState) nextButtonState;

			// Inform the user as to the current state of the walk and kick
			ROS_INFO("Button 1 pressed: Button state is now %s", buttonStateName(button).c_str());

			// Reset the stopped gait command
			gait::resetGaitCommand(m_stoppedGaitCmd);

			// Update the RGBLED state
			writeRGBLEDState();
		}
	}
}

// Handle new robot state data
void WAKRosInterface::handleRobotStateData(const robotcontrol::StateConstPtr& msg)
{
	// Constants (refer to fallprotection.cpp for state information)
	static const std::string standingPrefix = "standing|play_";
	static const std::string fallingPrefix = "falling|play_";
	static const std::string walkingPrefix = "walking_";
	static const std::string lyingPrefix = "lying_";

	// Return whether walking gait is active
	m_relaxed  = (msg->label == "relaxed" || msg->label == "setting_pose");
	m_initing  = (msg->label == "init" || msg->label == "initializing");
	m_standing = (msg->label == "standing" || msg->label.compare(0, standingPrefix.size(), standingPrefix) == 0);
	m_walking  = (msg->label == "walking" || msg->label.compare(0, walkingPrefix.size(), walkingPrefix) == 0);
	m_kicking  = (msg->label == "kicking");
	m_fallen   = (msg->label == "falling" || msg->label == "getting_up" || msg->label.compare(0, lyingPrefix.size(), lyingPrefix) == 0 || msg->label.compare(0, fallingPrefix.size(), fallingPrefix) == 0);
}

// Handle new robot heading data
void WAKRosInterface::handleRobotHeadingData(const robotcontrol::RobotHeadingConstPtr& msg)
{
	// Store the received data locally
	robotHeading = picut(msg->heading);
	robotHeadingTime = msg->stamp;
}

// Handle a new stopped gait command
void WAKRosInterface::handleStoppedGaitCommand(const gait_msgs::GaitCommandConstPtr& msg)
{
	// Store the received data locally, as long as the robot is stopped
	if(button == BTN_STOP)
	{
		m_stoppedGaitCmd = *msg;
		m_stoppedGaitCmd.motion = gait::MID_NONE;
	}
}

// Handle new robot diagnostics data
void WAKRosInterface::handleRobotDiagnosticsData(const robotcontrol::DiagnosticsConstPtr& msg)
{
	// Store whether the robotcontrol communications are doing ok
	robotcontrolCommsOk = (msg->commsOk != 0);
}

// Handle new vision output data
void WAKRosInterface::handleVisionOutputData(const vision_module::vision_outputsConstPtr& msg)
{
	// Retrieve the ball data
	ballVec << msg->ball.position.x, msg->ball.position.y;
	ballConf = msg->ball.probability;
	ballDetected = (msg->ball.detected != 0);
	ballTime = msg->header.stamp;

	// Retrieve the goal post data
	goalPostList.resize(msg->goals.size());
	for(size_t i = 0; i < msg->goals.size(); ++i)
	{
		goalPostList[i].vec << msg->goals[i].position.x, msg->goals[i].position.y;
		goalPostList[i].conf = msg->goals[i].probability;
	}
	goalPostTime = msg->header.stamp;

	// Retrieve the obstacle data
	obstacleList.resize(msg->obstacles.size());
	for(size_t i = 0; i < msg->obstacles.size(); ++i)
	{
		obstacleList[i].setVec(msg->obstacles[i].position.x, msg->obstacles[i].position.y);
		obstacleList[i].conf = msg->obstacles[i].probability;
		obstacleList[i].id = msg->obstacles[i].id;
	}
	obstacleTime = msg->header.stamp;

	// Retrieve the robot pose data
	robotPoseVec << msg->location.position.x, msg->location.position.y, picut(msg->location.position.z);
	robotPoseConf = msg->location.probability;
	robotPoseTime = msg->header.stamp;

	// Debug ROS topic data
	if(config.debugMsgROSTopics())
	{
		ROS_INFO_THROTTLE(1.0, "%.1f => Ball(%.2f, %.2f @ %.1f) Pose(%.2f, %.2f, %.2f @ %.1f) GoalPosts(%d) Obstacles(%d)", msg->header.stamp.toSec(),
		                  ballVec.x(), ballVec.y(), ballConf, robotPoseVec.x(), robotPoseVec.y(), robotPoseVec.z(), robotPoseConf, (int)goalPostList.size(), (int)obstacleList.size());
	}
}

// Handle a dive decision
void WAKRosInterface::handleDiveDecision(const std_msgs::StringConstPtr& msg)
{
	// Update the latest dive decision
	if(msg->data == "left_dive")
		diveDecision = DD_LEFT;
	else if(msg->data == "right_dive")
		diveDecision = DD_RIGHT;
	else if(msg->data == "sit_dive")
		diveDecision = DD_SIT;
	else
		diveDecision = DD_NONE;
}

// Handle a call to the visualise clear service
bool WAKRosInterface::handleVisualiseClear(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Pass the work on to the main walk and kick class
	return wak->handleVisualiseClear(req, resp);
}

// Handle a call to the visualise default ball handling service
bool WAKRosInterface::handleVisualiseDBH(VisualiseDBHRequest& req, VisualiseDBHResponse& resp)
{
	// Pass the work on to the main walk and kick class
	return wak->handleVisualiseDBH(req, resp);
}

// Handle a call to the visualise dribble approach service
bool WAKRosInterface::handleVisualiseDbApp(VisualiseDbAppRequest& req, VisualiseDbAppResponse& resp)
{
	// Pass the work on to the main walk and kick class
	return wak->handleVisualiseDbApp(req, resp);
}

// Handle a call to the visualise gcv XY service
bool WAKRosInterface::handleVisualiseGcvXY(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	// Pass the work on to the main walk and kick class
	return wak->handleVisualiseGcvXY(req, resp);
}

// Handle the regular LED updates
void WAKRosInterface::updateRGBLED(bool blink)
{
	// Update the LED state
	m_blinkRGBLED = blink;
	writeRGBLEDState();

	// Initially set the current robot state to neutral
	if(!m_publishedNeutral && button == BTN_STOP && m_pub_gaitCmd.getNumSubscribers() > 0)
	{
		writeZeroGcv();
		writeNeutralHead();
		clearLEDState();
		m_publishedNeutral = true;
	}
}

// Write a particular LED command
void WAKRosInterface::writeLEDCommand(int mask, int state)
{
	// Coerce the mask and state
	mask &= (LEDCommand::LED2 | LEDCommand::LED3 | LEDCommand::LED4);
	state &= mask;

	// Get the current ROS time
	ros::Time now = ros::Time::now();

	// Only publish a LED command if it is different to the last LED command we have published
	if((m_lastLEDState & mask) != state || (mask & ~m_lastLEDMask) != 0x00 || (now - m_lastLEDTime).toSec() >= 0.3)
	{
		// Update the last written LED state
		m_lastLEDMask |= mask;
		m_lastLEDState &= ~mask;
		m_lastLEDState |= state;
		m_lastLEDTime = now;

		// Send the required LED command
		LEDCommand led;
		led.mask = m_lastLEDMask;
		led.state = m_lastLEDState;
		m_pub_leds.publish(led);
	}
}

// Clear the state of the RGBLED
void WAKRosInterface::clearRGBLED() const
{
	// Clear the state of the RGBLED
	LEDCommand led;
	led.rgb5.r = 0.0f;
	led.rgb5.g = 0.0f;
	led.rgb5.b = 0.0f;
	led.rgb5Blink = false;
	led.mask = LEDCommand::LED5;
	led.state = LEDCommand::LED5;
	m_pub_leds.publish(led);
}

// Update the state of the RGBLED
void WAKRosInterface::writeRGBLEDState() const
{
	// Declare variables
	LEDCommand led;

	// Set the appropriate colour for the current button state
	led.rgb5.a = 1.0f;
	switch(button)
	{
		default:
		case BTN_STOP:
			led.rgb5.r = 1.0f;
			led.rgb5.g = 0.0f;
			led.rgb5.b = 0.0f;
			break;
		case BTN_PLAY:
			led.rgb5.r = 0.0f;
			led.rgb5.g = 0.0f;
			led.rgb5.b = 1.0f;
			break;
		case BTN_GOALIE:
			led.rgb5.r = 0.0f;
			led.rgb5.g = 0.8f;
			led.rgb5.b = 0.8f;
			break;
		case BTN_POS:
			led.rgb5.r = 0.8f;
			led.rgb5.g = 0.0f;
			led.rgb5.b = 0.8f;
			break;
	}

	// Update the state of the RGBLED
	led.mask = LEDCommand::LED5;
	led.state = LEDCommand::LED5;
	led.rgb5Blink = m_blinkRGBLED;
	m_pub_leds.publish(led);
}
// EOF
