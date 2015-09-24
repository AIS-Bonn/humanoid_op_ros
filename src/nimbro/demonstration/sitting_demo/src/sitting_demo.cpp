// Motion demonstration on a sitting robot
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Includes
#include <sitting_demo/sitting_demo_actions.h>
#include <sitting_demo/sitting_demo.h>
#include <face_tracker/face_tracker.h> // TODO: Where do we want this?
#include <motion_player/PlayMotion.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <ros/console.h>
#include <stdlib.h>
#include <string>
#include <time.h>

// Defines
#define LOOP_FREQ  10.0

// Namespaces
using namespace sitting_demo;

// Constructor
SittingDemo::SittingDemo()
 : m_sc(this)
 , m_robotIsSitting(false)
 , m_buttonPressed(false)
 , m_enabled(false)
 , m_sittingDemoEnabled("sittingDemoEnabled", true)
 , m_haveMotions(false)
{
	// Node handle
	ros::NodeHandle nh("~");

	// Advertise the required ROS topics
	m_pub_FSMState = nh.advertise<demo_msgs::DemoFSMState>("FSMState", 1, true);
	m_pub_action = nh.advertise<demo_msgs::DemoFSMState>("action", 1, true);
	m_pub_LEDCommand = nh.advertise<nimbro_op_interface::LEDCommand>("/led", 1, true);
	
	// Locate the required services
	m_srv_playMotion = nh.serviceClient<motion_player::PlayMotion>("/motion_player/play");
	m_srv_playCommands = nh.serviceClient<limb_control::PlayCommands>("/limb_control/playCommands");

	// Subscribe to the required ROS topics
	m_sub_robotState = nh.subscribe("/robotcontrol/state", 1, &SittingDemo::handleRobotState, this);
	m_sub_button = nh.subscribe("/button", 1, &SittingDemo::handleButton, this);

	// Configure the required configuration server callbacks
	m_sittingDemoEnabled.setCallback(boost::bind(&SittingDemo::handleDemoEnabled, this));

	// Load the required demo motions from the ROS parameter server
	loadDemoMotions();

	// Set the initial state of the state controller
	m_sc.init(NewStateInstance<InitDemoState>(&m_sc));

	// Initialise the LED
	publishLEDCommand();
}

// Load the required list of demo motions from the ROS parameter server
void SittingDemo::loadDemoMotions()
{
	// Node handle
	ros::NodeHandle nh("~");

	// Set the ROS parameter to retrieve the required demo motion names from
	const std::string motionParam = "DemoMotions";
	ROS_INFO("ROS parameter for demo motions: %s/%s", ros::this_node::getName().c_str(), motionParam.c_str());

	// Retrieve the list of required demo motions from the parameter server
	XmlRpc::XmlRpcValue list;
	if(nh.getParam(motionParam, list))
	{
		// Display header
		ROS_INFO("Reading demo motions from parameter server:");
		ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		// Retrieve the individual motion string names
		for(int i = 0; i < list.size(); i++)
		{
			std::string motion = static_cast<std::string>(list[i]);
			ROS_INFO("- %s", motion.c_str());
			m_demoMotions.push_back(motion);
		}

		// Display how many motions were retrieved
		if(m_demoMotions.size() > 0)
		{
			m_haveMotions = true;
			ROS_INFO("Successfully loaded %u motions!", (unsigned) m_demoMotions.size());
		}
		else
		{
			ROS_ERROR("Zero motions were specified on the parameter server!");
		}
	}
	else
	{
		ROS_ERROR("ROS parameter specifying demo motions was not found!");
	}
}

// Main loop step function
bool SittingDemo::step()
{
	// Save the current ROS time
	m_curStepTime = ros::Time::now();

	// Step the demo state controller
	ret_t ret;
	if((ret = m_sc.step()) != SCR_OK)
	{
		ROS_ERROR("Sitting demo state controller exited with code %d!", ret);
		return false;
	}

	// Return success
	return true;
}

// Main loop halt function
bool SittingDemo::halt()
{
	// Print a message why the sitting demo is halted
	if(!m_sittingDemoEnabled())
		ROS_WARN_THROTTLE(10.0, "Sitting demo currently disabled as the sitting demo enabled config parameter is false");
	else if(!m_robotIsSitting)
		ROS_WARN_THROTTLE(10.0, "Waiting for the robot to be in the sitting pose");
	else
		ROS_WARN_THROTTLE(10.0, "Waiting for a middle button press to start the sitting demo");

	// Return whether the halt executed successfully
	return true;
}

// Play a given motion from the demo motion list (uses the motion player motion module)
void SittingDemo::playMotion(int motionID)
{
	// Error checking
	if(motionID < 0 || motionID >= (int) m_demoMotions.size()) return;

	// Call the motion player with the selected motion
	motion_player::PlayMotion motion;
	motion.request.name = m_demoMotions[motionID];
	m_srv_playMotion.call(motion.request, motion.response);
}

// Send a given array of limb commands to the limb control motion module
double SittingDemo::sendLimbCommands(const limb_control::PlayCommandsRequest& PCmdReq)
{
	// Declare variables
	limb_control::PlayCommandsResponse PCmdResp;

	// Send the required limb commands
	m_srv_playCommands.call(PCmdReq, PCmdResp);

	// Estimate the new finish time of limb control
	m_limbControlFinishTime = ros::Time::now() + ros::Duration(PCmdResp.timeleft + LIMB_CONTROL_TIME_BIAS);

	// Return the result of the service call (an estimate of the time in seconds until limb control will be finished with the commands it currently has)
	return PCmdResp.timeleft;
}

// Publish the given current sitting demo FSM state
void SittingDemo::publishFSMState(const demo_msgs::DemoFSMState& state)
{
	// Publish the required message
	m_pub_FSMState.publish(state);
}

// Publish the given current sitting demo action
void SittingDemo::publishAction(const demo_msgs::DemoFSMState& action)
{
	// Publish the required message
	m_pub_action.publish(action);
}

// Send a LED command based on the current sitting demo state
void SittingDemo::publishLEDCommand()
{
	// Construct the required LED command (LED6 only)
	m_LED.mask = nimbro_op_interface::LEDCommand::LED6;
	if(m_enabled)
	{
		m_LED.rgb6.r = 1.0;
		m_LED.rgb6.g = 0.0;
		m_LED.rgb6.b = 1.0;
		m_LED.rgb6.a = 1.0;
	}
	else
	{
		m_LED.rgb6.r = 1.0;
		m_LED.rgb6.g = 0.0;
		m_LED.rgb6.b = 0.0;
		m_LED.rgb6.a = 1.0;
	}

	// Publish the LED command to the required ROS topic
	m_pub_LEDCommand.publish(m_LED);
}

// Handle ROS topic updates for the robot state
void SittingDemo::handleRobotState(const robotcontrol::StateConstPtr& msg)
{
	// Save the robotcontrol robot state name
	m_robotStateName = msg->label;
	m_robotIsSitting = (m_robotStateName == "sitting");
}

// Handle button presses
void SittingDemo::handleButton(const nimbro_op_interface::ButtonConstPtr& msg)
{
	// Check if the middle button was pressed (button 1 as zero-indexed)
	if(msg->button == 1)
	{
		m_buttonPressed = true;
		m_enabled = !m_enabled;
		publishLEDCommand();
		if(m_enabled)
			ROS_WARN("Middle button press detected => Sitting demo now enabled!");
		else
			ROS_WARN("Middle button press detected => Sitting demo now disabled!");
	}
}

// Handle updates from the sittingDemoEnabled config server parameter
void SittingDemo::handleDemoEnabled()
{
	// Perform the appropriate action depending on whether the demo was just enabled or disabled
	if(m_sittingDemoEnabled())
	{
		ROS_WARN("Sitting demo was just enabled!");
	}
	else
	{
		ROS_WARN("Sitting demo was just disabled!");
	}
}

// Main function
int main(int argc, char* argv[])
{
	// Initialise ROS
	ros::init(argc, argv, "sitting_demo");

	// Node handle
	ros::NodeHandle nh("~");

	// Stall for a little bit of time to allow robotcontrol to get started properly (not essential)
	ros::Time start = ros::Time::now();
	while(ros::Time::now() < start + ros::Duration(MAIN_FN_DELAY));
	ROS_INFO("Launching sitting demo...");

	// Seed the random number generation with the system time
	srand(std::time(NULL));

	// Construct an instance of the SittingDemo class
	SittingDemo SD;

	// Exit the node if no demo motions are available
	if(!SD.haveMotions())
	{
		ROS_ERROR("Exiting sitting demo as no demo motions are available!");
		return 1;
	}

	// TODO: Move this to the state controller? Or inside the SittingDemo class?
// 	// Construct an instance of the FaceTracker class
// 	FaceTracker FT(nh);
// 	FT.startTracking();

	// Set up the main loop 
	ros::Rate rate(LOOP_FREQ);

	// Main loop
	bool ok;
	while(ros::ok())
	{
		// Handle all ROS callbacks
		ros::spinOnce();

		// Execute the main loop step
		if(SD.enabled())
			ok = SD.step();
		else
			ok = SD.halt();
		if(!ok) break;

		// Sleep for the required duration
		rate.sleep();
	}

	// Return success
	return 0;
}
// EOF