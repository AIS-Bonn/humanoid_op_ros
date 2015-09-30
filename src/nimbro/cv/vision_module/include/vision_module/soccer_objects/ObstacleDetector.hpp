//ObstacleDetector.hpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/soccer_objects/IDetector.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/Tools/SortFuntions.hpp>
#include <algorithm>    // std::sort
using namespace cv;

class ObstacleDetector:public IDetector
{
public:

	vector<cv::Point> GetObstacleContours(Mat & obstacleMask,vector<cv::Point2f> &resInReal,vector< vector <cv::Point > > &resInRawContours,CameraProjections &proj)
	{
		const int minArea = 500;
		vector<cv::Point>  res;
		vector<vector<cv::Point> > obstContours;

		cv::findContours(obstacleMask.clone()/*To have binaryFrame after this function*/,
			obstContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		std::sort(obstContours.begin(), obstContours.end(),
			sorter(&proj));

		for (size_t i = 0; i < obstContours.size(); i++) // iterate through each contour.
		{
			cv::Rect rec = boundingRect(obstContours[i]);
			double area = contourArea(obstContours[i]);
			cv::Point btnPoint(rec.x + rec.width / 2, rec.y + rec.height);
			cv::Point2f btnInReal;
			if(!proj.GetOnRealCordinate(btnPoint,btnInReal))
			{
				ROS_ERROR("ERore in programming!");
			}

			cv::Point topPoint(rec.x + rec.width / 2, rec.y);
			if (area < minArea)
				continue;


			vector<cv::Point> tmpContour = obstContours[i];
			cv::approxPolyDP(tmpContour, tmpContour,
				cv::arcLength(tmpContour, true) * 0.003, true);
			resInRawContours.push_back(tmpContour);
			res.push_back(btnPoint);
			resInReal.push_back(btnInReal);
		}

		return res;
	}


	inline bool Init(){return true;};
};
