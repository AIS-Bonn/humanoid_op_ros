// Main Loop for the bgr2yuyv
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <bgr2yuyv/main.hpp>
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bgr2yuyv");
	ros::NodeHandle nodeHandle;
	CameraDummy cam;
	cam.InitCameraDevice(true);
	while (ros::ok())
	{
		if(cam.TakeCapture()>0.5)
		{

		int pointercounter = 0;

		unsigned char frame_raw_data[WIDTH * HEIGHT * 3];

			for (int i = 0; i < (yuyvSize); i += 4)
			{
				int Y1 = (unsigned char) cam.yuyvImg[i];
				int U = (unsigned char) cam.yuyvImg[i + 1];
				int Y2 = (unsigned char) cam.yuyvImg[i + 2];
				int V = (unsigned char) cam.yuyvImg[i + 3];

				//Storing the YUV data
				frame_raw_data[pointercounter++] = Y1;
				frame_raw_data[pointercounter++] = U;
				frame_raw_data[pointercounter++] = V;

				//Storing the YUV data
				frame_raw_data[pointercounter++] = Y2;
				frame_raw_data[pointercounter++] = U;
				frame_raw_data[pointercounter++] = V;

			}			//END for

			//LOCAL VISIALIZATION ON the RGB image
			unsigned char rgbFrame[WIDTH * HEIGHT * 3];
			yuv2rgb(frame_raw_data, rgbFrame, WIDTH, HEIGHT);
			cv::Mat fullrgbframe = cv::Mat(HEIGHT, WIDTH, CV_8UC3, rgbFrame);

			imshow("RGB", fullrgbframe);
			waitKey(1);
		}
		ros::spinOnce();
		usleep(1000);
	}
	return 0;
}

