// Main Loop for the Vision Module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/main.hpp>
#include <iostream>

using namespace cv;
using namespace std;
using namespace boost::timer;


#define NANO 1000000000.0

VisionTimer walTimer(5);


/**
 * Main function of this module
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_module_node");
	ros::NodeHandle nodeHandle;

	bool dummy = false;
	for (int i = 0; i < argc; i++)
	{
		if (strcmp(argv[i], "dummy") == 0)
		{
			dummy = true;
		}
	}
	ros::Publisher fps_pub = nodeHandle.advertise<std_msgs::Int64>(
			"/vision/fps", 10);

	params.Init(nodeHandle);

	VisionRate loop_rate(params.maxrate.get(), true);

	Vision visionObj(dummy);
	if (!visionObj.Init())
	{
		ROS_ERROR("There was a problem initing vision object!");
		loop_rate.Destroy();
		visionObj.DeInit();
		params.Destroy();
		return 0;
	}

	LinearInterpolator fpsCoefInterpolator(Point2d(params.maxrate.get(),0.9),Point2d(1,0.6));
	while (ros::ok())
	{
		if (params.counter.get() != 0)
		{
			cpu_timer timer;
			visionObj.plot_fps(params.fps.get());
			visionObj.update();

			loop_rate.sleep();

			float wallT = NANO / timer.elapsed().wall;
			float tmpWT = wallT;
			boundry_n(tmpWT, 1, params.maxrate.get());
			float coef = fpsCoefInterpolator.Interpolate(tmpWT);
			float coefP = 1 - coef;
			params.fps.set((coef * params.fps.get()) + (coefP * (wallT)));

			std_msgs::Int64 msg;
			msg.data = params.fps.get();
			if (params.fps.get() < 15)
			{
				HAF_WARN_THROTTLE(1, "Low FPS Vision %.1f %s", params.fps.get(),
						 "");
			}
			else
			{
				HAF_INFO_THROTTLE(1, "FPS = %.1f %s", params.fps.get(),
						 "");
			}
			fps_pub.publish(msg);
			params.update();
		}
		ros::spinOnce();
		params.counter.set(params.counter.get() + 1);
	}
	loop_rate.Destroy();
	visionObj.DeInit();
	params.Destroy();
	return 0;
}
