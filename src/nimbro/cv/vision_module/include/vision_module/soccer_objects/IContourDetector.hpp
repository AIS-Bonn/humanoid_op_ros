//IDetector.hpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/soccer_objects/IDetector.hpp>
#include <algorithm>    // std::sort
using namespace cv;
class IContourDetector:public IDetector
{
public:
	virtual bool Init()=0;
	virtual ~IContourDetector(){};
	virtual bool GetContour(const Mat &binaryFrame,
				vector<Point> &resContour)=0;
};
