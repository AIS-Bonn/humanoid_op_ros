//SortFuntions.hpp
// Created on: May 15, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/soccer_objects/IDetector.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <algorithm>    // std::sort
using namespace cv;

bool SortFuncDescending(vector<Point> i, vector<Point> j);

bool SortFuncDistanceAcending(vector<Point> i, vector<Point> j,CameraProjections &projecttion);

class sorter {
	CameraProjections *projecttion;
public:
      sorter(CameraProjections *projecttion) : projecttion(projecttion) {}
      bool operator()(vector<Point> o1, vector<Point> o2) {

    		Point2f iR,jR;
    		bool res=projecttion->GetOnRealCordinate(minAreaRect(o1).center,iR);
    		res&=projecttion->GetOnRealCordinate(minAreaRect(o2).center,jR);
    		if(!res)
    		{
    			ROS_ERROR("Error in programming!");
    		}

    		return GetDistance(iR) < GetDistance(jR);

      }
      bool operator()(Rect o1, Rect o2) {

    		Point2f iR,jR;

    		bool res=projecttion->GetOnRealCordinate(GetCenter(o1),iR);
    		res&=projecttion->GetOnRealCordinate(GetCenter(o2),jR);
    		if(!res)
    		{
    			ROS_ERROR("Error in programming!");
    		}

    		return GetDistance(iR) < GetDistance(jR);

      }
};

