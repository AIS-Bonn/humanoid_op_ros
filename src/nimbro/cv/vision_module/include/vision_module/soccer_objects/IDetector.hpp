//IDetector.hpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <algorithm>    // std::sort
using namespace cv;
class IDetector
{
public:
	virtual bool Init()=0;
	virtual ~IDetector(){};
};
