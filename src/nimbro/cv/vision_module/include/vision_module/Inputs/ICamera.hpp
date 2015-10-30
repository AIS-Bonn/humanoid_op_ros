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

using namespace cv;

class ICamera
{
public:
	Mat rawImage;
	 virtual ~ICamera(){};
	virtual bool IsDummy()=0;
	virtual bool IsReady()=0;
	virtual bool InitCameraDevice(bool)=0;
	virtual double TakeCapture()=0;
};

