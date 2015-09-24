// Dump RBDL tree parsed from URDF to stdout
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <rbdl/rbdl_parser.h>
#include <rbdl/treestream.h>

#include <ros/ros.h>

#include <urdf/model.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dump");

	std::string root;
	if(argc == 2)
		root = argv[1];

	ros::NodeHandle nh;

	rbdl_parser::URDF_RBDL_Model model;

	urdf::Model urdf;

	if(!urdf.initParam("robot_description"))
	{
		ROS_ERROR("Could not load URDF from parameter server");
		return 1;
	}

	if(!model.initFrom(urdf, root))
	{
		ROS_ERROR("Could not parse to RBDL");
		return 1;
	}

	TreeStream stream(&std::cout);
	stream << model;

	return 0;
}
