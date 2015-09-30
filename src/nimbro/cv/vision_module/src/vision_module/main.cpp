// Main Loop for the Vision Module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

/** @addtogroup VisionModule */
/*@{*/

#include <vision_module/main.hpp>
using namespace cv;
using namespace std;
using namespace boost::timer;

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
	params.Init();

	const int RATE = 30;
	double fpsData = RATE;
	VisionRate loop_rate(RATE, true);
	Vision visionObj(dummy);
	visionObj.Init();
	int counter = 0;

	while (ros::ok())
	{
		cpu_timer timer;

		visionObj.update();
		loop_rate.sleep();

		std_msgs::Int64 msg;

		msg.data = fpsData;
		if (!dummy)
		{
			ROS_INFO_THROTTLE(1, "fps = %"PRId64, msg.data);
		}
		else if (counter % (int) (1000. / RATE) == 0)
		{
			printf("[ INFO][/vision_module_node->main]: fps = %"PRId64"\r\n",
					msg.data);
		}
		if (fpsData < 15)
		{
			ROS_WARN_THROTTLE(10, "Low FPS Vision %d", (int )fpsData);
		}
		fps_pub.publish(msg);
		ros::spinOnce();
		fpsData = (0.9 * fpsData)
				+ (0.1 * (1000000000l / timer.elapsed().wall));

		counter++;
	}
	return 0;
}
/** @}*/
