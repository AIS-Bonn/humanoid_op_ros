// Test node for marker manager
// File: marker_manager_test_node.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <vis_utils/marker_manager.h>

// Namespaces
using namespace vis_utils;

// Main function
int main(int argc, char **argv)
{
	// Initialise ROS node
	ros::init(argc, argv, "marker_manager_test_node");

	// Retrieve a ROS node handle
	ros::NodeHandle nh("~");

	// Sleep a bit to wait for subscribers
	usleep(500000);

	// Create a marker manager
	MarkerManager MM("~/markers");

	// Variables
	int index[3] = {12, 18, 9};
	std::string ns[2] = {"myns", "otherns"};
	ros::Time initTime = ros::Time::now();

	// Main loop
	ros::Rate r(3.0); // Hz
	while(ros::ok())
	{
		// ROS spin
		ros::spinOnce();

		// Sleep
		r.sleep();

		// Clear marker manager
		MM.clear();

		// Test marker stuff
		int thisns = (fmod((ros::Time::now() - initTime).toSec(), 10.0) > 5.0);
		for(int i = 0; i < 3; ++i)
		{
			SphereMarker marker(&MM, "beh_field", 0.5, ns[thisns], true);
			marker.setColor(i*0.3, 0.0, i*0.3);
			marker.setPosition(i - 1.0, 0.0, 0.5);
			MM.updateDynamicMarker(index[i], marker);
		}

		// Publish marker manager
		MM.publish();
	}

	// Return success
	return 0;
}
// EOF