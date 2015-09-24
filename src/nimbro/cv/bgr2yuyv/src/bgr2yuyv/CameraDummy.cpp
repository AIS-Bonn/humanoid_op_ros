//Camera.cpp
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <bgr2yuyv/CameraDummy.hpp>

bool CameraDummy::InitCameraDevice(bool first)
{
	return true;
}

double CameraDummy::TakeCapture()
{
	if(capNumber <=lastCapNumber)
	{
		return -1;
	}
	lastCapNumber=capNumber;

	if (rawImage.empty())
	{
		ROS_WARN_THROTTLE(10, "Failed to get capture!");
		return -1;
	}

	return 1; // TODO: it should corespond with gyro!
}

