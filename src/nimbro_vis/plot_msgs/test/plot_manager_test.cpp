// Unit testing of the plot manager header
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <gtest/gtest.h>
#include <plot_msgs/plot_manager.h>

// Namespaces
using namespace plot_msgs;

//
// Plotting tests
//

// Test: PlotManager
TEST(PlottingTest, test_PlotManager)
{
	// Create some sample plot managers
	PlotManager PM1;
	PlotManager PM2("");
	PlotManager PM3("/");
	PlotManager PM4("node1");
	PlotManager PM5("/node2");
	PlotManager PM6("/node3/");
	PlotManager PM7("~");
	PlotManager PM8("~subnode");
	PlotManager PM9("~moresub/");

	// Test the plot manager plot variable naming
	EXPECT_STREQ("/plot_manager_test/", PM1.getBasename().c_str());
	EXPECT_STREQ("/plot_manager_test/", PM2.getBasename().c_str());
	EXPECT_STREQ("/", PM3.getBasename().c_str());
	EXPECT_STREQ("/node1/", PM4.getBasename().c_str());
	EXPECT_STREQ("/node2/", PM5.getBasename().c_str());
	EXPECT_STREQ("/node3/", PM6.getBasename().c_str());
	EXPECT_STREQ("/plot_manager_test/", PM7.getBasename().c_str());
	EXPECT_STREQ("/plot_manager_test/subnode/", PM8.getBasename().c_str());
	EXPECT_STREQ("/plot_manager_test/moresub/", PM9.getBasename().c_str());
}

// Test: PlotManagerFS
TEST(PlottingTest, test_PlotManagerFS)
{
	// Create some sample fixed-size plot managers
	PlotManagerFS PM1(5);
	PlotManagerFS PM2(5, "");
	PlotManagerFS PM3(5, "/");
	PlotManagerFS PM4(5, "node1");
	PlotManagerFS PM5(5, "/node2");
	PlotManagerFS PM6(5, "/node3/");
	PlotManagerFS PM7(5, "~");
	PlotManagerFS PM8(5, "~subnode");
	PlotManagerFS PM9(5, "~moresub/");

	// Test the fixed-size plot manager plot variable naming
	EXPECT_STREQ("/plot_manager_test/", PM1.getBasename().c_str());
	EXPECT_STREQ("/plot_manager_test/", PM2.getBasename().c_str());
	EXPECT_STREQ("/", PM3.getBasename().c_str());
	EXPECT_STREQ("/node1/", PM4.getBasename().c_str());
	EXPECT_STREQ("/node2/", PM5.getBasename().c_str());
	EXPECT_STREQ("/node3/", PM6.getBasename().c_str());
	EXPECT_STREQ("/plot_manager_test/", PM7.getBasename().c_str());
	EXPECT_STREQ("/plot_manager_test/subnode/", PM8.getBasename().c_str());
	EXPECT_STREQ("/plot_manager_test/moresub/", PM9.getBasename().c_str());
}

//
// Main function
//
int main(int argc, char **argv)
{
	// Initialise ROS node
	ros::init(argc, argv, "plot_manager_test");

	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF