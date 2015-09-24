//Camera.hpp
// Created on: Apr 19, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
#define WIDTH 640
#define HEIGHT 480
#define  yuyvSize  (WIDTH * HEIGHT * 2)

class CameraDummy
{
private:
	ros::NodeHandle nodeHandle;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it_;
	int capNumber;
	int lastCapNumber;
public:
	Mat rawImage;
	unsigned char yuyvImg[yuyvSize];

	inline CameraDummy() :
			it_(nodeHandle), capNumber(0), lastCapNumber(0)
	{
		sub = it_.subscribe("/vision/takenImg", 1, &CameraDummy::imageCallback,
				this);
	}
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

			Mat tmpI;

			cvtColor(rawImage, tmpI, CV_BGR2YCrCb);

			int index = 0;
			for (int i = 0; i < (tmpI.rows * tmpI.cols * 3); i += 6)
			{

				int Y1 = (unsigned char) tmpI.data[i];
				int U = (unsigned char) tmpI.data[i + 1];
				int V = (unsigned char) tmpI.data[i + 2];
				int Y2 = (unsigned char) tmpI.data[i + 3];

				//Storing the YUYV data
				yuyvImg[index++] = Y1;
				yuyvImg[index++] = U;
				yuyvImg[index++] = Y2;
				yuyvImg[index++] = V;
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

	bool InitCameraDevice(bool);
	double TakeCapture();
};
