//Camera.cpp
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <vision_module/Inputs/Camera.hpp>

bool Camera::InitCameraDevice(bool firstTime)
{

	if (!firstTime)
	{
		usleep(200000); //To make sure that camFd is closed!
	}

	bool changed;
	if (!changeDevNum(firstTime, changed))
	{
		usleep(100000);
		cap.release();
		return false;
	}

	if (!firstTime)
	{
		cap.release();
	}
	cap.open(devNumber); // open the camera
	if (!cap.isOpened())  // check if we succeeded
	{
		ROS_WARN_THROTTLE(1, "Can't Open Camera Device!");
		return false;
	}

	bool res = SetCameraProp() & SetCameraSetting();
	ROS_INFO("Found device on /dev/video%d", devNumber);
	return res;
}

double Camera::TakeCapture()
{

	if (devNumber < 0)
	{
		InitCameraDevice(false);
		return -1;
	}
	int camFd = open(devStr, O_RDONLY); //Just for check whether the camera is presented or not
	if (camFd != -1)
	{
		close(camFd); //Camera is ok!
		SetCameraProp();
	}
	else
	{
		failCount = 0;
		if (!InitCameraDevice(false))
		{
			return -1;
		}
	}

	if (successCount > 60) //After at least 60 successful grab (~2 Sec). To make sure that this is intentional
	{
		if (params.camera->freezInput())
		{
			HAF_WARN_THROTTLE(1, "freezInput = True.");
			return 1;
		}
	}
	else
	{
		if (params.camera->freezInput())
		{
			params.camera->freezInput.set(false);
		}
	}

	if (!cap.read(rawImage) || rawImage.empty())
	{
		ROS_WARN_THROTTLE(2, "Failed to get capture!");
		if (failCount++ > 30)
		{
			ROS_ERROR("This node will be shutdown intentionally.");
			cap.release();
			exit(-1);
		}
		return -1;
	}
	ros::Time now = ros::Time::now();
	if (abs((now - futureImageTime).toSec()) > 0.4) //It is the first time or camera was disconnected
	{
		rawImageTime = futureImageTime = now;
	}
	else
	{
		rawImageTime = futureImageTime;
		futureImageTime = now;
	}

	if (params.camera->flipHor() && params.camera->flipVer())
	{
		flip(rawImage, rawImage, -1);
	}
	else
	{
		if (params.camera->flipVer())
		{
			flip(rawImage, rawImage, 0);
		}
		else if (params.camera->flipHor())
		{
			flip(rawImage, rawImage, 1);
		}
	}
	failCount = 0;
	successCount++;
	nh.getParamCached("/vision/imgPublishTime", imgPublishTime);
	if (imgPublishTime <= 0
			|| (ros::WallTime::now() - lastImgPublish).toSec()
					>= imgPublishTime)
	{
		takenImg_pub.publish(rawImage, MatPublisher::bgr);
		lastImgPublish = ros::WallTime::now();
		shouldPublishNow=true;
	}
	else
	{
		shouldPublishNow=false;
	}
	return 1;
}

bool Camera::SetCameraSetting()
{

	bool res = true;
	if (-1 == system(paramDefStr))
	{
		ROS_WARN_THROTTLE(1, "Can't Open Camera Device!");
		res = false;
	}
	usleep(50000);
	for (int i = 0; i < 3; i++)
	{
		if (-1 == system(paramStr))
		{
			ROS_WARN_THROTTLE(1, "Can't Open Camera Device!");
			res = false;
		}
		Mat tmp;
		cap >> tmp;
		usleep(10000);
	}
	return res;
}

bool Camera::SetCameraProp()
{
	bool shouldSet = false;

	if ((width != params.camera->width() || height != params.camera->height()))
	{
		width = params.camera->width();
		height = params.camera->height();
		shouldSet = true;
	}

	if (shouldSet)
	{

		if (cap.set(CV_CAP_PROP_FRAME_WIDTH, width)
				| cap.set(CV_CAP_PROP_FRAME_HEIGHT, height))
		{
			ROS_WARN_THROTTLE(1, "Can't set image size values!");
		}
		usleep(10000);
		int inDevWidth = (int) cap.get(CV_CAP_PROP_FRAME_WIDTH);
		int inDevHeight = (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		if (inDevWidth != width || inDevHeight != height)
		{
			ROS_WARN(
					"Could not set requested resolution [%d, %d] into device, instead [%d, %d] will be used.",
					width, height, inDevWidth, inDevHeight);
		}
		if (inDevWidth != width || inDevHeight != height)
		{
			ROS_WARN(
					"Use 'v4l2-ctl -d %d --list-formats-ext' to see possible settings.",
					devNumber);
		}

		return inDevWidth != 0 && inDevHeight != 0;

	}
	return true;
}

bool Camera::changeDevNum(bool first, bool &changed)
{
	int preDevNumber = devNumber;
	char buf[512];
	string devName;
	nh.param("cameraDev", devName, std::string("/dev/eyeRight"));
	ssize_t count = readlink(devName.c_str(), buf, sizeof(buf));
	buf[count] = '\0';
	string bufStr(buf);
	changed = false;
	if (count >= 0)
	{
		size_t fPos = bufStr.find("video");
		if (fPos != std::string::npos)
		{
			devNumber = atoi(bufStr.substr(fPos + 5, devName.size()).c_str());
		}
		else
		{
			HAF_ERROR_THROTTLE(2,
					"Can't parse device number from symlink %s -> %s.",
					devName.c_str(), buf);
			return false;
		}
	}
	else
	{
		if (errno == EINVAL)
		{
			size_t fPos = devName.find("video");
			if (fPos != std::string::npos)
			{
				devNumber = atoi(
						devName.substr(fPos + 5, devName.size()).c_str());
			}
			else
			{
				HAF_ERROR_THROTTLE(2, "Can't parse device number from %s.",
						devName.c_str());
				return false;
			}
		}
		else
		{
			HAF_ERROR_THROTTLE(2, "No File Was Found in %s.", devName.c_str());
			return false;
		}
	}

	string configPath = ros::package::getPath("launch") + "/config/vision/";
	if (first || preDevNumber != devNumber)
	{
		changed = true;
		sprintf(devStr, "/dev/video%d", devNumber);
		sprintf(paramStr, "v4l2ctrl -d /dev/video%d -l %slogitechConfig.txt",
				devNumber, configPath.c_str());
		sprintf(paramOffStr,
				"v4l2ctrl -d /dev/video%d -l %slogitechConfig_off.txt ",
				devNumber, configPath.c_str());
		sprintf(paramDefStr,
				"v4l2ctrl -d /dev/video%d -l %slogitechConfig_default.txt ",
				devNumber, configPath.c_str());
		if (!first)
		{
			ROS_WARN("Switch device to /dev/video%d requested param =[%s]",
					devNumber, devName.c_str());
		}
	}

	int camFd = open(devStr, O_RDONLY); //Just for check whether the camera is presented or not
	if (camFd != -1)
	{
		close(camFd); //Camera is ok!
		return true;
	}
	HAF_ERROR_THROTTLE(2, "Can't Open File %s.", devStr);
	return false;
}

