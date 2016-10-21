//ObstacleDetector.hpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/SoccerObjects/IDetector.hpp>
#include <vision_module/Tools/SortFuntions.hpp>
#include <algorithm>    // std::sort
using namespace cv;

class ObstacleC
{
private:

	double confidence;
public:
	Point2d Position;
	long unsigned int id;
	double getConfidence()
	{
		boundry_n(confidence, 0, 1);
		return confidence;
	}

	bool decayConfidence()
	{
		confidence *= params.obstacle->decayConfidence();
		return confidence > 0.1;
	}

	ObstacleC(Point2d position, double confidence = 1.0,int id=0) :
			confidence(confidence), Position(position),id(id)
	{

	}

	bool update(Point2d _pos, double _conf)
	{
		if (GetDistance(_pos, Position) <= params.obstacle->maxPossibleJump())
		{
			boundry_n(_conf, 0, 1);
			lowPass(_pos, Position, _conf * params.obstacle->lowPassCoef());
			confidence = _conf;
			return true; //Input was similar to this obstacle and updated this obstacle
		}
		else
		{
			return false; //Input obstacle is not similar to this obstacle
		}
	}

	virtual ~ObstacleC()
	{
	}
};
/**
* @ingroup VisionModule
*
* @brief For detecting other robots with black feet
**/
class ObstacleDetector: public IDetector
{
public:

	bool GetObstacleContours(Mat &obstacleMask, vector<cv::Point2f> &resInReal,
			Mat &guiImg, CameraProjections &projection, bool SHOWGUI)
	{
		const int minArea = params.obstacle->minArea();
		vector<vector<cv::Point> > obstContours;

		cv::findContours(obstacleMask, obstContours, CV_RETR_EXTERNAL,
				CV_CHAIN_APPROX_NONE);

		resInReal.reserve(obstContours.size());

		for (size_t i = 0; i < obstContours.size(); i++) // iterate through each contour.
		{
			cv::approxPolyDP(obstContours[i], obstContours[i],
					cv::arcLength(obstContours[i], true) * 0.003, true);
			double area = contourArea(obstContours[i]);
			if (area <= minArea)
				continue;
			cv::Rect rec = boundingRect(obstContours[i]);
			cv::Point btnPoint(rec.x + rec.width / 2, rec.y + rec.height);

			cv::Point2f btnInReal;
			if (!projection.GetOnRealCordinate_single(btnPoint, btnInReal))
			{
				ROS_ERROR("Error in programming!");
			}
			if (SHOWGUI && params.obstacle->showResult())
			{
				drawContours(guiImg, obstContours, i, Scalar(240, 250, 250), 2,
						8);
			}
			double distanceToRobot = GetDistance(btnInReal);
			if (distanceToRobot >= params.obstacle->minDistance()
					&& distanceToRobot <= params.obstacle->maxDistance())
			{
				resInReal.push_back(btnInReal);
			}
		}

		return resInReal.size() > 0;
	}

	inline bool Init()
	{
		return true;
	}
};
