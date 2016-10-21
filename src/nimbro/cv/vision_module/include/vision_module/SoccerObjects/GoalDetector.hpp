//GoalDetector.hpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/SoccerObjects/IDetector.hpp>
#include <vision_module/Tools/SortFuntions.hpp>
#include <vision_module/Tools/LinearBoundaryChecker.hpp>
#include <algorithm>    // std::sort
using namespace cv;
/**
* @ingroup VisionModule
*
* @brief For detecting potential goal posts
**/
class GoalDetector: public IDetector
{
public:

	bool GetPosts(Mat& cannyImg, Mat& rawHSV, Mat &gray, const Mat &binaryFrame,
			CameraProjections &projection, vector<Point> fieldHull,
			vector<LineSegment> &resLines, vector<LineSegment> &alllLines,
			vector<Point2f> &goalPosition, bool SHOWGUI, Mat &guiImg)
	{

		if (binaryFrame.empty())
		{
			return false;
		}
		cv::Rect rec;
		rec.x = 0;
		rec.y = 0;
		rec.width = params.camera->width();
		rec.height = params.camera->height();
		const int MinLineLength = params.goal->MinLineLength();
		const int DistanceToMerge = params.goal->DistanceToMerge();
		const int MaxOutField = params.goal->MaxOutField();
		const int MinNearFieldUpPoint = -20;

		vector<cv::Vec4i> linesP;

		HoughLinesP(cannyImg, linesP, 1, M_PI / 45, 10, MinLineLength, 20);
		vector<LineSegment> allVerLines;
		for (size_t i = 0; i < linesP.size(); i++)
		{
			cv::Vec4i lP = linesP[i];
			LineSegment tmpLine(cv::Point2d(lP[0], lP[1]),
					cv::Point2d(lP[2], lP[3]));
			double leftAvg = 0;
			double rightAvg = 0;
			if (tmpLine.GetAbsMinAngleDegree(
					LineSegment(cv::Point(0, 0), cv::Point(0, 100))) < 15)
			{
				vector<cv::Point2d> midds = tmpLine.GetMidPoints(5);
				int maxMidPoint = midds.size();
				int vote_for_doubleLeft = 0;
				int vote_for_doubleRight = 0;

				cv::Point down =
						(tmpLine.P1.y < tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;
				double jumpDouble = params.goal->jumpMax();
				cv::Point2f downReal;
				if (!projection.GetOnRealCordinate_single(down, downReal))
				{
					ROS_ERROR("Erorr in programming!");
					return false;
				}
				double downDistance2Field = pointPolygonTest(fieldHull, down,
						true);
				if (downDistance2Field < MaxOutField)
					continue;
				double distance = GetDistance(downReal);
				if (distance < 2)
				{
					jumpDouble = 40;
				}
				else if (distance >= 2 && distance < 3)
				{
					jumpDouble = 23;
				}
				else
				{
					jumpDouble = 15;
				}

				for (size_t j = 0; j < midds.size(); j++)
				{

					LineSegment tocheck = tmpLine.PerpendicularLineSegment(
							jumpDouble, midds[j]);

					cv::Point left =
							(tocheck.P1.x < tocheck.P2.x) ?
									tocheck.P1 : tocheck.P2;
					cv::Point right =
							(tocheck.P1.x < tocheck.P2.x) ?
									tocheck.P2 : tocheck.P1;
					cv::LineIterator itLeft(cannyImg, midds[j], left, 8);
					cv::LineIterator itRight(cannyImg, midds[j], right, 8);
					cv::LineIterator itHSVLeft(rawHSV, midds[j], left, 8);
					cv::LineIterator itHSVRight(rawHSV, midds[j], right, 8);

					int safeToShow = 0;
					if (tocheck.P1.x >= 0 && tocheck.P1.y >= 0
							&& tocheck.P1.x < params.camera->width()
							&& tocheck.P1.y < params.camera->height())
					{
						safeToShow++;
					}
					if (tocheck.P2.x >= 0 && tocheck.P2.y >= 0
							&& tocheck.P2.x < params.camera->width()
							&& tocheck.P2.y < params.camera->height())
					{
						safeToShow++;
					}


					for (int k = 0; k < itLeft.count;
							k++, ++itLeft, ++itHSVLeft)
					{
						if (k < 2)
							continue;
						uchar val = *(*itLeft);
						cv::Vec3b hsvC = (cv::Vec3b) *itHSVLeft;

						if (val > 0 && k > params.goal->minDoubleLength())
						{
							if (safeToShow >= 2 && SHOWGUI
									&& params.goal->showVote())
							{
								cv::line(guiImg, midds[j], itHSVLeft.pos(),
										redColor(), 1);
							}
							leftAvg += k;
							vote_for_doubleLeft++;
							break;
						}

						if (hsvC[0] >= params.goal->h0()
								&& hsvC[0] <= params.goal->h1()
								&& hsvC[1] >= params.goal->s0()
								&& hsvC[1] <= params.goal->s1()
								&& hsvC[2] >= params.goal->v0()
								&& hsvC[2] <= params.goal->v1())
						{

						}
						else
						{
							break;
						}
					}

					for (int k = 0; k < itRight.count;
							k++, ++itRight, ++itHSVRight)
					{
						if (k < 2)
							continue;
						uchar val = *(*itRight);
						cv::Vec3b hsvC = (cv::Vec3b) *itHSVRight;

						if (val > 0 && k > params.goal->minDoubleLength())
						{
							if (safeToShow >= 2 && SHOWGUI
									&& params.goal->showVote())
							{
								cv::line(guiImg, midds[j], itHSVRight.pos(),
										redColor(), 1);
							}
							rightAvg += k;
							vote_for_doubleRight++;
							break;
						}

						if (hsvC[0] >= params.goal->h0()
								&& hsvC[0] <= params.goal->h1()
								&& hsvC[1] >= params.goal->s0()
								&& hsvC[1] <= params.goal->s1()
								&& hsvC[2] >= params.goal->v0()
								&& hsvC[2] <= params.goal->v1())
						{

						}
						else
						{
							break;
						}
					}

				}

				bool leftOK = (vote_for_doubleLeft / (float) maxMidPoint) * 100.
						> params.goal->doubleVote();

				bool rightOk = (vote_for_doubleRight / (float) maxMidPoint)
						* 100. > params.goal->doubleVote();

				if (leftOK || rightOk)
				{
					{
						LineSegment tmpLineChanged = tmpLine;

						if (leftOK)
						{
							int amount = abs(leftAvg / vote_for_doubleLeft)
									/ 2.;
							tmpLineChanged.P1.x -= amount;
							tmpLineChanged.P2.x -= amount;
						}
						else if (rightOk)
						{
							int amount = abs(rightAvg / vote_for_doubleRight)
									/ 2.;
							tmpLineChanged.P1.x += amount;
							tmpLineChanged.P2.x += amount;
						}
						tmpLineChanged.Clip(rec);
						allVerLines.push_back(tmpLineChanged);
					}
				}
			}
		}

		vector<LineSegment> allVerLines2;
		MergeLinesMax(allVerLines, 30, DistanceToMerge, allVerLines2, rec);

		for (size_t i = 0; i < allVerLines2.size(); i++)
		{
			LineSegment tmpLine = allVerLines2[i];
			cv::Point up =
					(tmpLine.P1.y > tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;
			cv::Point down =
					(tmpLine.P1.y < tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;

			double verLen = tmpLine.GetLength();
			if (pointPolygonTest(fieldHull, up, true) > MinNearFieldUpPoint)
				continue;


			cv::Point2f downReal;
			if (!projection.GetOnRealCordinate_single(down, downReal))
			{
				ROS_ERROR("Erorr in programming!");
				return false;
			}

			if (!checkDistance_Box(downReal, verLen))
				continue;
			double downDistance2Field = pointPolygonTest(fieldHull, down, true);
			if (downDistance2Field < MaxOutField)
				continue;
			if (GetDistance(downReal) > 5)
				continue;
			goalPosition.push_back(downReal);
			resLines.push_back(LineSegment(down, up));
		}

		if (SHOWGUI && params.goal->showVote())
		{
			for (size_t i = 0; i < allVerLines2.size(); i++)
			{
				cv::line(guiImg, allVerLines2[i].P1, allVerLines2[i].P2,
						blueColor(), 1);
			}
		}
		if (SHOWGUI && params.goal->showResult())
		{
			for (size_t i = 0; i < resLines.size(); i++)
			{
				cv::line(guiImg, resLines[i].P1, resLines[i].P2, blueColor(),
						2);
			}
		}
		return resLines.size() > 0;
	}

	bool checkDistance_Box(Point2f downPointInReal, double length)
	{

		LineSegment lowerBound(
				Point2f(params.goal->NearestDistance(), params.goal->NearMinLen()),
				Point2f(params.goal->FarestDistance(), params.goal->FarMinLen()));
		LineSegment higherBound(
				Point2f(params.goal->NearestDistance(), params.goal->NearMaxLen()),
				Point2f(params.goal->FarestDistance(), params.goal->FarMaxLen()));
		LinearBoundaryChecker checker(lowerBound, higherBound);

		double distanceToRobot = GetDistance(downPointInReal);

		return checker.CheckInside(distanceToRobot, length);
	}

	inline bool Init()
	{
		bool result=true;
		if(!readFromFile<Point3d>(params.goal->distSizeVecPath, params.goal->distSizeVec))
		{
			ROS_ERROR("Create or modify %s!",params.goal->distSizeVecPath.c_str());
			result=false;
		}
		std::sort(params.goal->distSizeVec.begin(),
				params.goal->distSizeVec.end(), [](const Point3d& a, const Point3d& b)
	            {
	                return a.z < b.z;
	            });
		return result;
	}

};
