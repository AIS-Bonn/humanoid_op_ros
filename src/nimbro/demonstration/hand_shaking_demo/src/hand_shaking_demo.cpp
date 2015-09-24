// Hand shaking demonstration on a standing robot
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <hand_shaking_demo/hand_shaking_demo.h>
#include <motion_player/PlayMotion.h>
#include <ros/console.h>

// Defines
#define LOOP_FREQ  10.0

// Namespaces
using namespace hand_shaking_demo;

// Constructor
HandShakingDemo::HandShakingDemo()
 : m_buttonPressed(false)
 , m_robotIsStanding(false)
 , m_robotIsShaking(false)
 , m_handShakingDemoEnabled("handShakingDemoEnabled", true)
{
	// Node handle
	ros::NodeHandle nh("~");

	// Locate the required services
	m_srv_playMotion = nh.serviceClient<motion_player::PlayMotion>("/motion_player/play");

	// Subscribe to the required ROS topics
	m_sub_button = nh.subscribe("/button", 1, &HandShakingDemo::handleButton, this);
	m_sub_robotState = nh.subscribe("/robotcontrol/state", 1, &HandShakingDemo::handleRobotState, this);

	// Configure the required configuration server callbacks
	m_handShakingDemoEnabled.setCallback(boost::bind(&HandShakingDemo::handleDemoEnabled, this));
}

// Main loop step function
bool HandShakingDemo::step()
{
	// Print a heartbeat
	if(m_robotIsStanding)
		ROS_INFO_THROTTLE(10.0, "Waiting for a middle button press to start the hand shake...");
	else if(m_robotIsShaking)
		ROS_INFO_THROTTLE(3.0, "Robot is shaking, so I'm ignoring further middle button presses.");
	else
		ROS_INFO_THROTTLE(10.0, "Robot is not standing, so I'm ignoring middle button presses.");

	// Fire off the hand shake motion if the middle button is pressed
	if(m_robotIsStanding && m_buttonPressed)
	{
		playMotion("shake_hand");
		m_buttonPressed = false;
		ROS_WARN("Just fired off a motion player service call for the hand shaking motion!");
	}

	// Return whether the step executed successfully
	return true;
}

// Main loop halt function
bool HandShakingDemo::halt()
{
	// Return whether the halt executed successfully
	return true;
}

// Play a given motion using the motion player module
void HandShakingDemo::playMotion(const std::string& motionName)
{
	// Call the motion player with the selected motion
	motion_player::PlayMotion motion;
	motion.request.name = motionName;
	m_srv_playMotion.call(motion.request, motion.response);
}

// Handle button presses
void HandShakingDemo::handleButton(const nimbro_op_interface::ButtonConstPtr& msg)
{
	// Check if the middle button was pressed (button 1 as zero-indexed)
	if(msg->button == 1)
		m_buttonPressed = true;
}

// Handle ROS topic updates for the robot state
void HandShakingDemo::handleRobotState(const robotcontrol::StateConstPtr& msg)
{
	// Save whether the robot was last known to be standing
	bool wasStanding = m_robotIsStanding;

	// Save the robotcontrol robot state name
	m_robotStateName = msg->label;
	m_robotIsStanding = (m_robotStateName == "standing");
	m_robotIsShaking = (m_robotStateName == "shaking_hand");
	
	// Let the user know if a transition has occurred
	if(m_robotIsStanding && !wasStanding)
		ROS_WARN("Robot just entered the standing state.");
	if(!m_robotIsStanding && wasStanding)
		ROS_WARN("Robot just left the standing state.");
}

// Handle updates from the standingDemoEnabled config server parameter
void HandShakingDemo::handleDemoEnabled()
{
	// Perform the appropriate action depending on whether the demo was just enabled or disabled
	if(m_handShakingDemoEnabled())
	{
		ROS_WARN("Standing demo was just enabled!");
	}
	else
	{
		ROS_WARN("Standing demo was just disabled!");
	}
}

// Main function
int main(int argc, char* argv[])
{
	// Initialise ROS
	ros::init(argc, argv, "hand_shaking_demo");

	// Node handle
	ros::NodeHandle nh("~");

	// Stall for a little bit of time to allow robotcontrol to get started properly (not essential)
	ros::Time start = ros::Time::now();
	while(ros::Time::now() < start + ros::Duration(2.5));
	ROS_INFO("Launching hand shaking demo...");

	// Construct an instance of the HandShakingDemo class
	HandShakingDemo SD;

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
	
	// Display message if quitting
	ROS_INFO("Terminating hand shaking demo...");

	// Return success
	return 0;
}
// EOF