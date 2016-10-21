//LineDetector.cpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/SoccerObjects/CircleDetector.hpp>

bool CircleDetector::GetCircle(double H2,vector<LineSegment> &clusteredLines,bool &confiused,Point2d &resultCircle,Point2d &resultCircleRotated,CameraProjections &projection)
{
	vector<Point2d> circlePoint;
	vector<LineSegment> clusteredLinesImg;
	for (size_t lineI = 0; lineI < clusteredLines.size(); lineI++)
	{
		double lLength = clusteredLines[lineI].GetLength();
		if (lLength > params.circle->maxLineLen()
				|| lLength < params.circle->minLineLen())
		{
			continue;
		}

		LineSegment pls = clusteredLines[lineI].PerpendicularLineSegment();
		for (size_t lineJ = lineI + 1; lineJ < clusteredLines.size(); lineJ++)
		{

			double lLength2 = clusteredLines[lineJ].GetLength();
			if (lLength2 > params.circle->maxLineLen()
					|| lLength2 < params.circle->minLineLen())
			{
				continue;
			}
			if (dist3D_Segment_to_Segment(clusteredLines[lineJ],
					clusteredLines[lineI])
					> params.circle->maxDistBetween2LS())
				continue;
			LineSegment pls2 = clusteredLines[lineJ].PerpendicularLineSegment();
			Point2d intersect;
			if (pls.IntersectLineForm(pls2, intersect))
			{
				double distance1 = clusteredLines[lineI].DistanceFromLine(
						intersect);
				double distance2 = clusteredLines[lineJ].DistanceFromLine(
						intersect);
				if (distance1 < H2 * params.circle->radiusMaxCoef()
						&& distance1 > H2 * params.circle->radiusMinCoef()
						&& distance2 < H2 * params.circle->radiusMaxCoef()
						&& distance2 > H2 * params.circle->radiusMinCoef())
				{
					circlePoint.push_back(intersect);
				}
			}
		}
	}

	if (circlePoint.size() >= (size_t) params.circle->minLineSegmentCount())
	{
		Point2d sum;
		for (size_t cCounter = 0; cCounter < circlePoint.size(); cCounter++)
		{
			sum += circlePoint[cCounter];
			for (size_t cCounter2 = cCounter + 1;
					cCounter2 < circlePoint.size(); cCounter2++)
			{
				if (GetDistance(circlePoint[cCounter], circlePoint[cCounter2])
						> params.circle->confiusedDist())
				{
					confiused = true;
				}
			}
		}
		resultCircle.x = sum.x / circlePoint.size();
		resultCircle.y = sum.y / circlePoint.size();
		resultCircleRotated = projection.RotateTowardHeading(
				resultCircle);
		return true;
	}
	return false;
}
