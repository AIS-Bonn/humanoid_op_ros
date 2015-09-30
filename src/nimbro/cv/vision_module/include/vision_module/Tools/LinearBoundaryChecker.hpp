//LinearBoundaryChecker.hpp
// Created on: May 29, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once

#include <vision_module/Tools/LineSegment.hpp>
#include <math.h>
class LinearBoundaryChecker
{
private:
	LineSegment LLow, LHigh;
public:
	inline LinearBoundaryChecker(LineSegment _LLow, LineSegment _LHigh) :
			LLow(_LLow), LHigh(_LHigh)
	{

	}

	inline bool Check(double distance, double value)
	{
		double max_y = std::max(LLow.P1.y,
				std::max(LLow.P2.y, std::max(LHigh.P1.y, LHigh.P2.y)));
		LineSegment verLine(Point(distance, 0), Point2d(distance, max_y));
		Point2d resLowerBand, resUpperBand;
		if (LLow.Intersect(verLine, resLowerBand)
				&& LHigh.Intersect(verLine, resUpperBand))
		{
			if (value >= resLowerBand.y && value <= resUpperBand.y)
			{
				return true;
			}
		}
		return false;
	}

	inline virtual ~LinearBoundaryChecker()
	{

	}
};

