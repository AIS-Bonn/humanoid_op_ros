//Camera.cpp
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <vision_module/Inputs/CameraDummy.hpp>

bool CameraDummy::InitCameraDevice(bool first)
{
	return true;
}

double CameraDummy::TakeCapture()
{
	if(capNumber <=1)
	{
		return -1;
	}

	if (rawImage.empty())
	{
		ROS_WARN_THROTTLE(10, "Failed to get capture!");
		return -1;
	}

	return 1;
}
