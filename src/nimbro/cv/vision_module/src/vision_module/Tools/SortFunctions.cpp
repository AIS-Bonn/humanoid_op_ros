//SortFunctions.cpp
// Created on: May 15, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <vision_module/Tools/SortFuntions.hpp>

bool SortFuncDistanceAcending(vector<Point> i, vector<Point> j,
		CameraProjections &projecttion)
{
	Point2f iR, jR;
	bool res = projecttion.GetOnRealCordinate_single(minAreaRect(i).center, iR);
	res &= projecttion.GetOnRealCordinate_single(minAreaRect(j).center, jR);
	if (!res)
	{
		ROS_ERROR("Error in programming!");
	}

	return GetDistance(iR) < GetDistance(jR);
}


bool SortFuncDescending(vector<Point> i, vector<Point> j)
{
	return contourArea(i, false) > contourArea(j, false);
}
