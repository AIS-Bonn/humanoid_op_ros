//Camera.hpp
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/MatPublisher.hpp>
#include <vision_module/Inputs/ICamera.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
using namespace cv;

/**
* @ingroup VisionModule
*
* @brief This class is responsible for image acquisition process \n  As an output this class provides the raw image in /vision/takenImg topic
**/
class Camera: public ICamera
{
private:
	bool destroyed;
	VideoCapture cap;
	char devStr[255];
	char paramDefStr[512];
	char paramStr[512];
	char paramOffStr[512];
	int devNumber;
	ros::Time futureImageTime;
	int width;
	int height;
	long int failCount;
	long int successCount;
	ros::NodeHandle nh;
	MatPublisher takenImg_pub;
	ros::WallTime lastImgPublish;
	double imgPublishTime;
	bool shouldPublishNow;
public:
	inline Camera() :destroyed(false),
			devNumber(-1),width(-1),height(-1),failCount(0),successCount(0),nh("~"),takenImg_pub("/vision/takenImg"),shouldPublishNow(false)
	{
		ros::NodeHandle nh;
		lastImgPublish=ros::WallTime::now();
		nh.param("/vision/imgPublishTime",imgPublishTime,1.0);
		rawImageTime = ros::Time::now();
		futureImageTime=ros::Time(0);
	}

	inline void DeInitCameraDevice()
	{
		if (!destroyed)
		{
			if (-1 == system(paramOffStr))
			{
				cout<<"Can't find or set v4l2-ctrl values!"<<endl;
			}
			cap.release();
			destroyed=true;
		}
	}

	/*! @fn virtual void ~Camera()
	 *   @brief Destructor
	 */
	inline virtual ~Camera()
	{
		DeInitCameraDevice();
	}
	inline bool IsReady()
	{
		return true;
	}
	inline bool IsDummy()
	{
		return false;
	}
	bool InitCameraDevice(bool);
	/**
	 * @fn TakeCapture
	 * @brief for capture new image
	 * @return How much this capture is reliable
	 */
	double TakeCapture();

	bool changeDevNum(bool first, bool &changed);
	bool SetCameraProp();
	bool SetCameraSetting();

	inline bool ShouldPublish()
	{
		bool result=shouldPublishNow;
		shouldPublishNow=false;
		return result;
	}
};
