// FPS printer
// Author: Max Schwarz <Max@x-quadraht.de>

#ifndef PRINT_FPS_H
#define PRINT_FPS_H

#include <ros/time.h>
#include <ros/subscriber.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

class PrintFPS : public nodelet::Nodelet
{
public:
	PrintFPS();
	virtual ~PrintFPS();

	virtual void onInit();

	void processImage(const sensor_msgs::Image::Ptr& img);
private:
	ros::Subscriber m_sub_input;

	int m_counter;
	ros::Time m_lastTime;
};

#endif /* PRINT_FPS_H */
// EOF