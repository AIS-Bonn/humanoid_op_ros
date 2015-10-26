//Camera.hpp
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/MatPublisher.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_module/Inputs/ICamera.hpp>
using namespace cv;
using namespace boost::timer;

/**
 * @class CameraDummy
 * @brief This class is responsible for imitating image acquisition process as a dummy replacement for Camera Class \n  As an input this class listen the raw image in /vision/takenImg topic
 **/
class CameraDummy:public ICamera
{
private:
	ros::NodeHandle nodeHandle;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it_;
	int capNumber;
public:
	inline CameraDummy() :
			it_(nodeHandle), capNumber(0)
	{
		sub = it_.subscribe("/vision/takenImg", 1, &CameraDummy::imageCallback,
				this);
	}
	/*! @fn virtual void ~CameraDummy()
	*   @brief Destructor
	*/
	inline virtual ~CameraDummy()
	{
	}
	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
			rawImage = cv_ptr->image.clone();
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
			capNumber++;
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}
	inline bool IsReady()
	{
		return capNumber>1;
	}
	inline bool IsDummy()
	{
		return true;
	}
	inline void DeInitCameraDevice()
	{

	}
	bool InitCameraDevice(bool);
	/**
	 * @fn TakeCapture
	 * @brief for capture new image
	 * @return How much this capture is reliable
	 */
	double TakeCapture();
};

