//Camera.cpp
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <vision_module/Inputs/Camera.hpp>

bool Camera::InitCameraDevice(bool firstTime)
{
	if (!firstTime)
	{
		delete cap;
		cap = new VideoCapture(devNumber); // open the camera
	}
	if (!cap->isOpened())  // check if we succeeded
	{
		ROS_WARN_THROTTLE(10, "Cant Open Camera Device!");
		return false;
	}

	if (cap->set(CV_CAP_PROP_FRAME_WIDTH, params.camera.width->get())
			| cap->set(CV_CAP_PROP_FRAME_HEIGHT, params.camera.height->get()))
	{
		ROS_WARN_THROTTLE(10, "Cant set image size values!");
	}


	if (-1 == system(paramStr))
	{
		ROS_WARN_THROTTLE(10, "Cant find or set v4l2-ctrl values!");
	}


	return true;
}

double Camera::TakeCapture()
{
	if (params.camera.devNumber->get() != devNumber)
	{
		devNumber = params.camera.devNumber->get();
		sprintf(devStr, "/dev/video%d", devNumber);
		sprintf(paramStr,
				"v4l2ctrl -d /dev/video%d -l /nimbro/share/launch/config/vision/logitechConfig.txt",
				devNumber);
		sprintf(paramDefStr,"v4l2ctrl -d /dev/video%d -l /nimbro/share/launch/config/vision/logitechConfig_default.txt ",devNumber);
		usleep(1000000);
		InitCameraDevice(false);
	}

	int camFd = open(devStr, O_RDONLY);
	if (camFd != -1)
	{
		close(camFd);
	}
	else
	{
		usleep(1000000);
		InitCameraDevice(false);
		return -1;
	}

	*cap >> rawImage;
	if (rawImage.empty())
	{
		ROS_WARN_THROTTLE(10, "Failed to get capture!");
		return -1;
	}
	if (params.camera.flipHor->get() && params.camera.flipVer->get())
	{
		flip(rawImage, rawImage, -1);
	}
	else
	{
		if (params.camera.flipVer->get())
		{
			flip(rawImage, rawImage, 0);
		}
		else if (params.camera.flipHor->get())
		{
			flip(rawImage, rawImage, 1);
		}
	}
	takenImg_pub.publish(rawImage,MatPublisher::bgr);
	return 1;
}


