//LineDetector.hpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/SoccerObjects/IDetector.hpp>
#include <vision_module/Tools/BoundaryLinearInterpolation.hpp>
#include <algorithm>    // std::sort
using namespace cv;
/**
* @ingroup VisionModule
*
* @brief For detecting center circle
**/
class CircleDetector: public IDetector
{
public:
	bool GetCircle(double H2, vector<LineSegment> &clusteredLines,
			bool &confiused, Point2d &resultCircle,
			Point2d &resultCircleRotated, CameraProjections &projection);
	inline bool Init()
	{
		return true;
	}
	inline ~CircleDetector()
	{
	}
};
