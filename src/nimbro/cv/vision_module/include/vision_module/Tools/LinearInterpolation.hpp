// Created on: June 30, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <vision_module/Tools/LineSegment.hpp>
#include <math.h>
class LineInterpolation
{
private:
	LineSegment line;
public:
	inline LineInterpolation(LineSegment _line) :
		line(_line)
	{

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

	inline virtual ~LineInterpolation()
	{

	}
};
