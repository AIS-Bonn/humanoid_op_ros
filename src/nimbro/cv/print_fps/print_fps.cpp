// FPS printer
// Author: Max Schwarz <Max@x-quadraht.de>

#include "print_fps.h"
#include <ros/node_handle.h>
#include <pluginlib/class_list_macros.h>

PrintFPS::PrintFPS()
{
}

PrintFPS::~PrintFPS()
{
}

void PrintFPS::onInit()
{
	m_sub_input = getNodeHandle().subscribe("image", 1, &PrintFPS::processImage, this);

	m_counter = 0;
	m_lastTime = ros::Time::now();
}

void PrintFPS::processImage(const sensor_msgs::Image::Ptr&)
{
	m_counter++;

	if(m_counter == 100)
	{
		ros::Time n = ros::Time::now();
		float fps = 100.0 / (n - m_lastTime).toSec();
		NODELET_INFO("FPS: %f", fps);

		m_counter = 0;
		m_lastTime = n;
	}
}

PLUGINLIB_DECLARE_CLASS(openplatform, print_fps, PrintFPS, nodelet::Nodelet)
// EOF