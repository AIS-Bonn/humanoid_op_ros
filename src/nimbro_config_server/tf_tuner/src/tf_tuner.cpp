// Configurable tf publisher
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <config_server/parameter.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_tuner");

	if(argc != 3)
	{
		fprintf(stderr, "Usage: tf_tuner <parent frame> <child frame>\n");
		return 1;
	}

	ros::NodeHandle nh("~");

	tf::TransformBroadcaster broadcaster;

	tf::StampedTransform transform;
	transform.frame_id_ = argv[1];
	transform.child_frame_id_ = argv[2];

	config_server::Parameter<float> param_trans_x("/tf_tuner/lin/x", -0.4, 0.01, 0.4, 0.0);
	config_server::Parameter<float> param_trans_y("/tf_tuner/lin/y", -0.4, 0.01, 0.4, 0.0);
	config_server::Parameter<float> param_trans_z("/tf_tuner/lin/z", -0.4, 0.01, 0.4, 0.0);

	config_server::Parameter<float> param_rot_r("/tf_tuner/rot/r", -M_PI/4.0, 0.01, M_PI/4.0, 0.0);
	config_server::Parameter<float> param_rot_p("/tf_tuner/rot/p", -M_PI/4.0, 0.01, M_PI/4.0, 0.0);
	config_server::Parameter<float> param_rot_y("/tf_tuner/rot/y", -M_PI/4.0, 0.01, M_PI/4.0, 0.0);

	ros::WallRate rate(10.0);
	while(ros::ok())
	{
		ros::spinOnce();

		transform.setOrigin(tf::Vector3(param_trans_x(), param_trans_y(), param_trans_z()));

		tf::Quaternion rot;
		rot.setRPY(param_rot_r(), param_rot_p(), param_rot_y());
		transform.setRotation(rot);

		transform.stamp_ = ros::Time::now();

		broadcaster.sendTransform(transform);

		rate.sleep();
	}
}

