// Unit testing of the marker manager header
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <gtest/gtest.h>
#include <vis_utils/marker_manager.h>

// Namespaces
using namespace vis_utils;

//
// Markers tests
//

// Test: MarkerManager
TEST(MarkersTest, test_MarkerManager)
{
	// Create some sample marker managers
	MarkerManager MM1;
	MarkerManager MM2("", 1);
	MarkerManager MM3("/", 2, true);
	MarkerManager MM4("node1_markers", 3, false);
	MarkerManager MM5("/node2_markers");
	MarkerManager MM6("~");
	MarkerManager MM7("~markers");
	
	// Test the marker manager topic naming
	EXPECT_STREQ("/marker_manager_test/vis_marker_array", MM1.getTopicName().c_str());
	EXPECT_STREQ("/marker_manager_test/vis_marker_array", MM2.getTopicName().c_str());
	EXPECT_STREQ("/marker_manager_test/vis_marker_array", MM3.getTopicName().c_str());
	EXPECT_STREQ("node1_markers", MM4.getTopicName().c_str());
	EXPECT_STREQ("/node2_markers", MM5.getTopicName().c_str());
	EXPECT_STREQ("/marker_manager_test/vis_marker_array", MM6.getTopicName().c_str());
	EXPECT_STREQ("/marker_manager_test/markers", MM7.getTopicName().c_str());

	// Test the publish interval
	EXPECT_EQ(1, MM1.getPublishInterval());
	EXPECT_EQ(1, MM2.getPublishInterval());
	EXPECT_EQ(2, MM3.getPublishInterval());
	EXPECT_EQ(3, MM4.getPublishInterval());

	// Test the enabled state
	EXPECT_TRUE(MM1.getEnabled());
	EXPECT_TRUE(MM2.getEnabled());
	EXPECT_TRUE(MM3.getEnabled());
	EXPECT_FALSE(MM4.getEnabled());
	MM2.disable();
	EXPECT_FALSE(MM2.getEnabled());
	MM2.enable();
	EXPECT_TRUE(MM2.getEnabled());

	// Test initial state of marker array
	EXPECT_EQ(0, MM3.getNumMarkers());

	// Check occurrence of publishing
	MM3.clear();
	EXPECT_TRUE(MM3.willPublish());
	MM3.publish();
	MM3.clear();
	EXPECT_FALSE(MM3.willPublish());
	MM3.publish();
	MM3.clear();
	EXPECT_TRUE(MM3.willPublish());
	MM3.publish();
	MM3.clear();
	EXPECT_FALSE(MM3.willPublish());
	MM3.forcePublish();
	EXPECT_TRUE(MM3.willPublish());
	MM3.disable();
	EXPECT_FALSE(MM3.willPublish());
	MM3.enable();
	EXPECT_TRUE(MM3.willPublish());
	MM3.publish();
	MM3.clear();
	EXPECT_FALSE(MM3.willPublish());
	MM3.disable();
	EXPECT_FALSE(MM3.willPublish());
	MM3.enable();
	EXPECT_TRUE(MM3.willPublish());
	MM3.publish();
	EXPECT_FALSE(MM3.willPublish());
	MM3.reset();
	EXPECT_TRUE(MM3.willPublish());
}

// Test: GenMarker
TEST(MarkersTest, test_GenMarker)
{
	// Create a parent marker manager
	MarkerManager MM("~markers");
	
	// Create some sample generic markers
	GenMarker GM1(&MM);
	GenMarker GM2(&MM, "/odom");
	GenMarker GM3(&MM, "/ego_rot", "");
	GenMarker GM4(&MM, "/ego_floor", "~");
	GenMarker GM5(&MM, "/map", "~my_ns");

	// Test the internal parent pointer
	EXPECT_EQ(&MM, GM1.MM);
	EXPECT_EQ(&MM, GM2.MM);

	// Test the frame naming
	EXPECT_STREQ("", GM1.marker.header.frame_id.c_str());
	EXPECT_STREQ("/odom", GM2.marker.header.frame_id.c_str());
	EXPECT_STREQ("/ego_rot", GM3.marker.header.frame_id.c_str());
	EXPECT_STREQ("/ego_floor", GM4.marker.header.frame_id.c_str());
	EXPECT_STREQ("/map", GM5.marker.header.frame_id.c_str());

	// Test the generic marker namespace naming
	EXPECT_STREQ("/marker_manager_test", GM1.marker.ns.c_str());
	EXPECT_STREQ("/marker_manager_test", GM2.marker.ns.c_str());
	EXPECT_STREQ("/marker_manager_test", GM3.marker.ns.c_str());
	EXPECT_STREQ("/marker_manager_test", GM4.marker.ns.c_str());
	EXPECT_STREQ("/marker_manager_test/my_ns", GM5.marker.ns.c_str());

	// Test the internal ID numbers
	EXPECT_EQ(1, GM1.marker.id);
	EXPECT_EQ(2, GM2.marker.id);
	EXPECT_EQ(3, GM3.marker.id);
	EXPECT_EQ(4, GM4.marker.id);
	EXPECT_EQ(5, GM5.marker.id);

	// Change the marker parameters
	GM2.setFrameID("/map");
	GM2.setType(visualization_msgs::Marker::CYLINDER);
	GM2.setPosition(1.0, 2.0, 3.0);
	GM2.setOrientation(0.3, 0.5, 0.7, 0.9);
	GM2.setScale(0.5, 0.5, 0.5);
	GM2.setColor(0.0, 0.5, 1.0);
	GM2.setLifetime(0.6);
	GM2.setFrameLocked(true);

	// Test the change of marker parameters
	EXPECT_STREQ("/map", GM2.marker.header.frame_id.c_str());
	EXPECT_EQ((int) visualization_msgs::Marker::CYLINDER, GM2.marker.type);
	EXPECT_EQ(1.0, GM2.marker.pose.position.x);
	EXPECT_EQ(2.0, GM2.marker.pose.position.y);
	EXPECT_EQ(3.0, GM2.marker.pose.position.z);
	EXPECT_EQ(0.3, GM2.marker.pose.orientation.w);
	EXPECT_EQ(0.5, GM2.marker.pose.orientation.x);
	EXPECT_EQ(0.7, GM2.marker.pose.orientation.y);
	EXPECT_EQ(0.9, GM2.marker.pose.orientation.z);
	EXPECT_EQ(0.5, GM2.marker.scale.x);
	EXPECT_EQ(0.5, GM2.marker.scale.y);
	EXPECT_EQ(0.5, GM2.marker.scale.z);
	EXPECT_EQ(0.0, GM2.marker.color.r);
	EXPECT_EQ(0.5, GM2.marker.color.g);
	EXPECT_EQ(1.0, GM2.marker.color.b);
	EXPECT_EQ(1.0, GM2.marker.color.a);
	EXPECT_DOUBLE_EQ(0.6, GM2.marker.lifetime.toSec());
	EXPECT_TRUE(GM2.marker.frame_locked);

	// Test marker update
	MM.clear();
	EXPECT_TRUE(MM.willPublish());
	EXPECT_EQ(0, MM.getNumMarkers());
	GM2.update();
	EXPECT_EQ(1, MM.getNumMarkers());
	MM.publish();
	MM.clear();
	EXPECT_EQ(0, MM.getNumMarkers());
}

// Test: SphereMarker
TEST(MarkersTest, test_SphereMarker)
{
	// Create a parent marker manager
	MarkerManager MM("~markers");

	// Create a sample sphere marker
	SphereMarker SM1(&MM, "/odom", 0.1);

	// Test marker parameters
	EXPECT_EQ((int) visualization_msgs::Marker::SPHERE, SM1.marker.type);
	EXPECT_EQ(0.1, SM1.marker.scale.x);
	EXPECT_EQ(0.1, SM1.marker.scale.y);
	EXPECT_EQ(0.1, SM1.marker.scale.z);

	// Test default position
	EXPECT_EQ(0.0, SM1.marker.pose.position.x);
	EXPECT_EQ(0.0, SM1.marker.pose.position.y);
	EXPECT_EQ(0.0, SM1.marker.pose.position.z);

	// Test marker update
	MM.clear();
	EXPECT_TRUE(MM.willPublish());
	EXPECT_EQ(0, MM.getNumMarkers());
	SM1.update(0.1, 0.2, 0.3);
	EXPECT_EQ(0.1, SM1.marker.pose.position.x);
	EXPECT_EQ(0.2, SM1.marker.pose.position.y);
	EXPECT_EQ(0.3, SM1.marker.pose.position.z);
	EXPECT_EQ(1, MM.getNumMarkers());
	MM.publish();
	MM.clear();
	EXPECT_EQ(0, MM.getNumMarkers());
}

// Test: CubeMarker
TEST(MarkersTest, test_CubeMarker)
{
	// Create a parent marker manager
	MarkerManager MM("~markers");

	// Create a sample cube marker
	CubeMarker CM1(&MM, "/odom", 0.1);

	// Test marker parameters
	EXPECT_EQ((int) visualization_msgs::Marker::CUBE, CM1.marker.type);
	EXPECT_EQ(0.1, CM1.marker.scale.x);
	EXPECT_EQ(0.1, CM1.marker.scale.y);
	EXPECT_EQ(0.1, CM1.marker.scale.z);

	// Test default position
	EXPECT_EQ(0.0, CM1.marker.pose.position.x);
	EXPECT_EQ(0.0, CM1.marker.pose.position.y);
	EXPECT_EQ(0.0, CM1.marker.pose.position.z);

	// Test marker update
	MM.clear();
	EXPECT_TRUE(MM.willPublish());
	EXPECT_EQ(0, MM.getNumMarkers());
	CM1.update(0.1, 0.2, 0.3);
	EXPECT_EQ(0.1, CM1.marker.pose.position.x);
	EXPECT_EQ(0.2, CM1.marker.pose.position.y);
	EXPECT_EQ(0.3, CM1.marker.pose.position.z);
	EXPECT_EQ(1, MM.getNumMarkers());
	MM.publish();
	MM.clear();
	EXPECT_EQ(0, MM.getNumMarkers());
}

// Test: BoxMarker
TEST(MarkersTest, test_BoxMarker)
{
	// Create a parent marker manager
	MarkerManager MM("~markers");

	// Create a sample box marker
	BoxMarker BM1(&MM, "/odom", 0.6, 0.5, 0.4);

	// Test marker parameters
	EXPECT_EQ((int) visualization_msgs::Marker::CUBE, BM1.marker.type);
	EXPECT_EQ(0.6, BM1.marker.scale.x);
	EXPECT_EQ(0.5, BM1.marker.scale.y);
	EXPECT_EQ(0.4, BM1.marker.scale.z);

	// Test default position and orientation
	EXPECT_EQ(0.0, BM1.marker.pose.position.x);
	EXPECT_EQ(0.0, BM1.marker.pose.position.y);
	EXPECT_EQ(0.0, BM1.marker.pose.position.z);
	EXPECT_EQ(1.0, BM1.marker.pose.orientation.w);
	EXPECT_EQ(0.0, BM1.marker.pose.orientation.x);
	EXPECT_EQ(0.0, BM1.marker.pose.orientation.y);
	EXPECT_EQ(0.0, BM1.marker.pose.orientation.z);

	// Test marker update
	MM.clear();
	EXPECT_TRUE(MM.willPublish());
	EXPECT_EQ(0, MM.getNumMarkers());
	BM1.update(0.1, 0.2, 0.3);
	EXPECT_EQ(0.1, BM1.marker.pose.position.x);
	EXPECT_EQ(0.2, BM1.marker.pose.position.y);
	EXPECT_EQ(0.3, BM1.marker.pose.position.z);
	EXPECT_EQ(1, MM.getNumMarkers());
	BM1.update(0.4, 0.3, 0.2, 0.0, 0.7, 0.9, 0.5);
	EXPECT_EQ(0.4, BM1.marker.pose.position.x);
	EXPECT_EQ(0.3, BM1.marker.pose.position.y);
	EXPECT_EQ(0.2, BM1.marker.pose.position.z);
	EXPECT_EQ(0.0, BM1.marker.pose.orientation.w);
	EXPECT_EQ(0.7, BM1.marker.pose.orientation.x);
	EXPECT_EQ(0.9, BM1.marker.pose.orientation.y);
	EXPECT_EQ(0.5, BM1.marker.pose.orientation.z);
	EXPECT_EQ(2, MM.getNumMarkers());
	MM.publish();
	MM.clear();
	EXPECT_EQ(0, MM.getNumMarkers());
}

//
// Main function
//
int main(int argc, char **argv)
{
	// Initialise ROS node
	ros::init(argc, argv, "marker_manager_test");
	
	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF