// Created on: May 23, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <math.h>
#include <vision_module/Tools/LineSegment.hpp>
/**
* @ingroup VisionModule
*
* @brief A class for linearInterpolation
**/
class LinearInterpolator
{
private:
	LineSegment line;
public:
	inline LinearInterpolator(LineSegment _line):line(_line)
	{

	}

	inline LinearInterpolator(cv::Point2d p1, cv::Point2d p2)
	{
		line.P1=p1;
		line.P2=p2;
	}

	inline double Interpolate(double x)
	{
		return (line.P1.y + (line.P2.y - line.P1.y) * (x - line.P1.x) / (line.P2.x - line.P1.x));
	}

	inline virtual ~LinearInterpolator()
	{

	}
};
