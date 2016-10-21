//LinearBoundaryChecker.hpp
// Created on: May 29, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once

#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Tools/LinearInterpolator.hpp>
#include <math.h>

/**
* @ingroup VisionModule
*
* @brief A class for check if a point is between two given line segments
**/
class LinearBoundaryChecker
{
private:
	LineSegment LLow, LHigh;
public:
	inline bool checkValidity()
	{
		Point2d tmp;
		return !LLow.Intersect(LHigh, tmp);
	}

	inline LinearBoundaryChecker(LineSegment _LLow, LineSegment _LHigh) :
			LLow(_LLow), LHigh(_LHigh)
	{
		if (!checkValidity())
		{
			HAF_ERROR_THROTTLE(1,
					"Conflict in input for LinearBoundaryChecker");
		}
	}

	inline LinearBoundaryChecker(double nearDistance, double nearMin,
			double nearMax, double farDistance, double farMin, double farMax)
	{
		LLow.P1 = Point2d(nearDistance, nearMin);
		LLow.P2 = Point2d(farDistance, farMin);

		LHigh.P1 = Point2d(nearDistance, nearMax);
		LHigh.P2 = Point2d(farDistance, farMax);
		if (!checkValidity())
		{
			HAF_ERROR_THROTTLE(1,
					"Conflict in input for LinearBoundaryChecker");
		}
	}

	inline bool CheckExtrapolation(double distance,
			double value)
	{
		LineSegment verLine(Point2d(distance, -10),
				Point2d(distance, 10));
		Point2d resLowerBand, resUpperBand;
		if (LLow.IntersectLineForm(verLine, resLowerBand)
				&& LHigh.IntersectLineForm(verLine, resUpperBand))
		{
			if (value >= resLowerBand.y && value <= resUpperBand.y)
			{
				return true;
			}
		}
		else
		{
			ROS_ERROR("Error in programming %f", distance);
		}

		return false;
	}


	inline bool GetExtrapolation(double distance,
			double &min, double &max)
	{
		LineSegment verLine(Point2d(distance, -10),
				Point2d(distance, 10));
		Point2d resLowerBand, resUpperBand;

		if (LLow.IntersectLineForm(verLine, resLowerBand)
				&& LHigh.IntersectLineForm(verLine, resUpperBand))
		{
			if (resUpperBand.y > resLowerBand.y)
			{
				min = resLowerBand.y;
				max = resUpperBand.y;
				return true;
			}
		}
		else
		{
			ROS_ERROR("Error in programming %f", distance);
		}
		return false;
	}

	inline bool CheckInside(double distance, double value)
	{

		double max_y = std::max(LLow.P1.y,
				std::max(LLow.P2.y, std::max(LHigh.P1.y, LHigh.P2.y)));
		LineSegment verLine(Point2d(distance, 0), Point2d(distance, max_y));
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

