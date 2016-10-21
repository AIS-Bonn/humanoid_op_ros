// Created on: June 30, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <vision_module/Tools/LineSegment.hpp>
#include <math.h>
/**
* @ingroup VisionModule
*
* @brief A class for interpolating between boundaries
**/
class BoundaryLineInterpolation
{
private:
	LineSegment line;
public:
	inline BoundaryLineInterpolation(LineSegment _line) :
		line(_line)
	{

	}

	inline BoundaryLineInterpolation(double nearDistance,double nearValue,double farDistance,double farValue)
	{
		line.P1=Point2d(nearDistance,nearValue);
		line.P2=Point2d(farDistance,farValue);
	}

	inline bool GetValue(double distance, double &value)
	{
		double max_y = std::max(line.P1.y, line.P2.y);
		LineSegment verLine(cv::Point(distance, 0), cv::Point2d(distance, max_y));
		cv::Point2d resBand;
		if (line.Intersect(verLine, resBand))
		{
			value = resBand.y;
			return true;
		}
		return false;
	}

	inline virtual ~BoundaryLineInterpolation()
	{

	}
};
