// Walk and kick: Main function
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/walk_and_kick.h>
#include <motiontimer.h>
#include <ros/ros.h>
#include <signal.h>

// Namespaces
using namespace walk_and_kick;

//
// Main function
//

// Custom SIGINT handler
bool g_shutdown = false;
void sigint_handler(int signal)
{
	// Trigger a shutdown
	g_shutdown = true;
}

// Main function
int main(int argc, char **argv)
{
	// Override the default SIGINT handler
	signal(SIGINT, sigint_handler);

	// Initialise ROS
	ros::init(argc, argv, "walk_and_kick", ros::init_options::NoSigintHandler);

	// Retrieve a ROS node handle
	ros::NodeHandle nh("~");

	// Create an instance of the WalkAndKick class and initialise it
	WalkAndKick WAK;
	if(WAK.init())
		ROS_INFO("Initialisation of walk and kick complete!");
	else
	{
		ROS_ERROR("Could not initialise walk and kick!");
		return 1;
	}

	// Sleep a bit to wait for subscribers
	usleep(500000);

	// Initialise timer object for loop rate timing
	MotionTimer timer(TINC);
	int cycleLED = timer.cyclesForTime(LED_PERIOD);

	// Manually handle an RGBLED update once at the beginning
	WAK.publishLED();

	// Keep looping while everything is fine and dandy...
	int cycleLEDCount = 0;
	bool step = true, oldStep = true, paused = false;
	while(ros::ok())
	{
		// Handle a shutdown if necessary
		if(g_shutdown)
		{
			WAK.resetAll();  // Clean up after walk and kick
			ros::shutdown(); // Shut down ROS nicely
			break;           // Stop the main loop
		}

		// Do all ROS callbacks before sleeping so that the next step starts immediately after the timer tick
		ros::spinOnce();

		// Sleep for the required duration
		uint64_t expirations = timer.sleep();

		// Execute the walk and kick (or not) as required
		oldStep = step;
		step = WAK.config.enableWAK();
		if(step != oldStep)
		{
			WAK.publishNeutral();
			if(step)
				ROS_WARN("Walk and kick was just ACTIVATED!");
			else
				ROS_WARN("Walk and kick was just DEACTIVATED!");
		}
		if(step)
		{
			if(WAK.config.pauseWAK())
			{
				ROS_WARN_THROTTLE(15.0, "Walk and kick is PAUSED");
				paused = true;
			}
			else
			{
				if(paused)
				{
					ROS_WARN("Walk and kick has been UNPAUSED");
					paused = false;
				}
				WAK.step();
			}
		}
		else
			WAK.reset();

		// At regular intervals update the LED state
		cycleLEDCount++;
		if(cycleLEDCount >= cycleLED || WAK.publishLEDPending())
		{
			WAK.publishLED();
			cycleLEDCount = 0;
		}

		// Publish the walk and kick state
		WAK.publishState();

		// Check whether any cycles were missed
		if(expirations > 1)
			ROS_WARN("Walk and kick missed %lu timer cycles", expirations - 1);
	}

	// Return success
	return 0;
}
// EOF