//GoalDetector.hpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/soccer_objects/IDetector.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/Tools/SortFuntions.hpp>
#include <vision_module/Tools/LinearBoundaryChecker.hpp>
#include <algorithm>    // std::sort
using namespace cv;
class GoalDetector: public IDetector
{
public:
	cv::CascadeClassifier object_cascade;

	bool GetPosts(Mat &gray,const Mat &binaryFrame, CameraProjections &projection,
			vector<Point> fieldHull, vector<LineSegment> &resLines,
		 vector<LineSegment> &alllLines,
			vector<Point2f> &goalPosition,bool SHOWGUI,Mat &guiImg)
	{

		if ( binaryFrame.empty())
		{
			return false;
		}
		const int MinLineLength =params.goal.MinLineLength->get();
		const int DistanceToMerge = params.goal.DistanceToMerge->get();
		const int MaxOutField = params.goal.MaxOutField->get();
		const int MinNearFieldUpPoint = -25;


		vector<cv::Vec4i> linesP;
		HoughLinesP(binaryFrame, linesP, 10, M_PI/34, 20,
			MinLineLength, 4);
		vector<LineSegment> allVerLines;
		for (size_t i = 0; i < linesP.size(); i++)
		{
			cv::Vec4i lP = linesP[i];
			LineSegment tmpLine(cv::Point2d(lP[0], lP[1]), cv::Point2d(lP[2], lP[3]));
			if (tmpLine.GetAbsMinAngleDegree(LineSegment(cv::Point(0,0),cv::Point(0,100)))<15)
			{
				allVerLines.push_back(tmpLine);
			}
		}

		cv::Rect rec;
		rec.x = 0;
		rec.y = 0;
		rec.width = params.camera.width->get();
		rec.height = params.camera.height->get();
		vector<LineSegment> allVerLines2;
		MergeLinesMax(allVerLines, 30, DistanceToMerge,
			allVerLines2, rec);

		for (size_t i = 0; i < allVerLines2.size(); i++)
		{
			LineSegment tmpLine = allVerLines2[i];
			cv::Point up = (tmpLine.P1.y > tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;
			cv::Point down =
				(tmpLine.P1.y < tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;

			double verLen = tmpLine.GetLength();
			cv::Point2f downReal;
			if(!projection.GetOnRealCordinate(down, downReal))
			{
				ROS_ERROR("Erorr in programming!");
				return false;
			}

			if (!checkDistance_Box(downReal, verLen,params.goal))
				continue;

			double downDistance2Field = pointPolygonTest(fieldHull, down, true);
			if (downDistance2Field < MaxOutField ) //down point should be near inside
				continue;
			if ( pointPolygonTest(fieldHull, up, true) > MinNearFieldUpPoint)
				continue;
			std::vector<cv::Rect> objects;
			double heigthOfTheGoal = GetDistance(up, down);
			cv::Rect rec(up.x - heigthOfTheGoal/10, up.y, heigthOfTheGoal/6, heigthOfTheGoal);
			double ratioHT = params.goal.BiggerRectHT->get();
			double ratioHB =  params.goal.BiggerRectHB->get();
			double ratioW =  params.goal.BiggerRectW->get();
			int xL = max_n(0, (int)(rec.x - rec.width * ratioW));
			int xR = min_n(params.camera.width->get(), (int)(rec.x + rec.width + rec.width * ratioW));

			int yT = max_n(0, (int)(rec.y - rec.height * ratioHT));
			int yB = min_n(params.camera.height->get(), (int)(rec.y + rec.height + rec.height * ratioHB));

			rec.x = xL;
			rec.y = yT;
			rec.width = xR - xL;
			rec.height = yB - yT;

			cv::Mat imgGray = gray(rec);
			/*			cv::imshow("REC", imgGray);
			cv::waitKey(100);*/

			int minSizeDim = max_n(3, max_n(rec.size().height, rec.size().width) / 13);
			int minNeighborsDim = max_n(5, min_n(rec.size().height, rec.size().width) / 16);
			object_cascade.detectMultiScale(imgGray, objects, 1.1, minNeighborsDim, 0, cv::Size(minSizeDim, minSizeDim),
				rec.size());

			if (objects.size() > 0)
			{
				resLines.push_back(tmpLine);
				goalPosition.push_back(downReal);
			}
		}

		if (SHOWGUI &&params.debug.showGoalD->get())
		{

			cv::Mat darker = cv::Mat::zeros(guiImg.size(), CV_8UC3);
			darker.copyTo(guiImg, 255 - binaryFrame);
			for (size_t i = 0; i < allVerLines2.size(); i++)
			{
				cv::line(guiImg, allVerLines2[i].P1, allVerLines2[i].P2, blueColor(), 2);
			}
		}
		if (SHOWGUI)
		{
			for (size_t i = 0; i < resLines.size(); i++)
			{
				cv::line(guiImg, resLines[i].P1, resLines[i].P2, redColor(), 3);
			}
		}
		return resLines.size() > 0;
	}

	bool checkDistance_Box(Point2f downPointInReal, double length,
			hsvRangeC type
			)
	{

		LineSegment lowerBound(
				Point2f(type.NearestDistance->get(), type.NearMinLen->get()),
				Point2f(type.FarestDistance->get(), type.FarMinLen->get()));
		LineSegment higherBound(
				Point2f(type.NearestDistance->get(), type.NearMaxLen->get()),
				Point2f(type.FarestDistance->get(), type.FarMaxLen->get()));
		LinearBoundaryChecker checker(lowerBound, higherBound);

		double distanceToRobot = GetDistance(downPointInReal);

		return checker.Check(distanceToRobot, length);
	}


	inline bool Init()
	{
		if (!object_cascade.load("/nimbro/share/launch/config/vision/cascadeGoal.xml"))
		{
			ROS_ERROR("Error loading cascade Goal!");
			return false;
		};
		return true;
	}

};

