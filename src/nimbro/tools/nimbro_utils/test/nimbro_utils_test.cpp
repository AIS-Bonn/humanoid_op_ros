// Unit testing of the nimbro utilities headers that require ROS
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <gtest/gtest.h>
#include <nimbro_utils/ros_timing.h>
#include <config_server/SetParameter.h>
#include <ros/node_handle.h>
#include <ros/init.h>

// Namespaces
using namespace nimbro_utils;

//
// ROS timing tests
//

// Test: RosTimeMarker
TEST(RosTimingTest, test_RosTimeMarker)
{
	// Declare variables
	double elapsed;

	// Create an instance of a RosTimeMarker
	RosTimeMarker RTM;

	// Verify initial state
	EXPECT_FALSE(RTM.haveMarker());
	EXPECT_LT(RTM.getElapsed(), 0.0);
	EXPECT_TRUE(RTM.hasElapsed(1000.0));

	// Set a marker and verify the changes
	ros::Time start = ros::Time::now();
	RTM.setMarker();
	EXPECT_TRUE(RTM.haveMarker());
	EXPECT_GE(RTM.getElapsed(), 0.0);
	EXPECT_FALSE(RTM.hasElapsed(1000.0)); // Assume 1000 seconds hasn't passed yet...!
	
	// Wait for 500ms to elapse
	while(!RTM.hasElapsed(0.5))
	{
		// Avoid infinite looping...
		if((elapsed = (ros::Time::now() - start).toSec()) > 0.7)
			break;
	}

	// Check that the amount of elapsed time is reasonable
	EXPECT_GE(elapsed, 0.49);
	EXPECT_LE(elapsed, 0.70);
	EXPECT_GE(RTM.getElapsed(), 0.49);
	EXPECT_LE(RTM.getElapsed(), 0.69); // Being a wee bit stricter here... (see infinite loop avoidance above)

	// Unset the marker and verify the changes
	RTM.unsetMarker();
	EXPECT_FALSE(RTM.haveMarker());
	EXPECT_LT(RTM.getElapsed(), 0.0);
	EXPECT_TRUE(RTM.hasElapsed(1000.0));
}

// Test: RosTimeTracker
TEST(RosTimingTest, test_RosTimeTracker)
{
	// Declare variables
	double elapsed;
	std::size_t m, N = 3;

	// Create an instance of a RosTimeTracker
	RosTimeTracker RTT(N);

	// Verify initial state
	for(m = 0;m < N;m++)
	{
		EXPECT_FALSE(RTT.haveMarker(m));
		EXPECT_LT(RTT.getElapsed(m), 0.0);
		EXPECT_TRUE(RTT.hasElapsed(m, 1000.0));
	}

	// Set some markers and verify the changes
	ros::Time start = ros::Time::now();
	for(m = 0;m < N;m++)
	{
		RTT.setMarker(m);
		EXPECT_TRUE(RTT.haveMarker(m));
		EXPECT_GE(RTT.getElapsed(m), 0.0);
		EXPECT_FALSE(RTT.hasElapsed(m, 1000.0)); // Assume 1000 seconds hasn't passed yet...!
	}

	// Wait for 500ms to elapse on marker 0
	while(!RTT.hasElapsed(0, 0.5))
	{
		// Avoid infinite looping...
		if((elapsed = (ros::Time::now() - start).toSec()) > 0.7)
			FAIL();
	}
	EXPECT_GE(elapsed, 0.49);

	// Wait for 800ms to elapse on marker 1
	while(!RTT.hasElapsed(1, 0.8))
	{
		// Avoid infinite looping...
		if((elapsed = (ros::Time::now() - start).toSec()) > 1.0)
			FAIL();
	}
	EXPECT_GE(elapsed, 0.79);

	// Wait for 1100ms to elapse on marker 2
	while(!RTT.hasElapsed(2, 1.1))
	{
		// Avoid infinite looping...
		if((elapsed = (ros::Time::now() - start).toSec()) > 1.3)
			FAIL();
	}
	EXPECT_GE(elapsed, 1.09);

	// Unset the markers and verify the changes
	for(m = 0;m < N;m++)
	{
		RTT.unsetMarker(m);
		EXPECT_FALSE(RTT.haveMarker(m));
		EXPECT_LT(RTT.getElapsed(m), 0.0);
		EXPECT_TRUE(RTT.hasElapsed(m, 1000.0));
	}
}

// Test: RosServiceCaller
TEST(RosTimingTest, test_RosServiceCaller)
{
	// Lacking something about the RosServiceCaller class that you can really test, we just test the
	// instantiation thereof and the negative response when a dummy service client is used.
	RosServiceCaller<config_server::SetParameter> RSC(0.6, 0.3);
	EXPECT_FALSE(RSC.callService());
	ros::ServiceClient m_srv_dummy;
	RSC.setServiceClient(m_srv_dummy);
	EXPECT_FALSE(RSC.callService()); // This should be within the 0.3s delay and automatically return false
}

//
// Main function
//
int main(int argc, char **argv)
{
	// Initialise ROS node
	ros::init(argc, argv, "nimbro_utils_test");
	
	// Retrieve a node handle (to ensure ros::Time::now() is successful)
	ros::NodeHandle nh("~");
	
	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF