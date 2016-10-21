//BallDetector.hpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/SortFuntions.hpp>
#include <vision_module/Tools/LinearBoundaryChecker.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Tools/GuiManager.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/SoccerObjects/IDetector.hpp>
#include <algorithm>    // std::sort
#include <string>     // std::string, std::to_string
#include <boost/range/algorithm_ext/erase.hpp>
using namespace cv;


class BallCircleC: public CircleC
{
public:
	Point2f RealPos;
	bool isCascade;
	bool enable;
	bool isMerged;
	BallCircleC(Point Center, int Radius, Point2f RealPos, bool isCascade,
			bool enable = true, bool isMerged = false) :
			CircleC(Center, Radius), RealPos(RealPos), isCascade(isCascade), enable(
					enable), isMerged(isMerged)
	{

	}
	BallCircleC(const BallCircleC& _in) :
			CircleC(_in), RealPos(_in.RealPos), isCascade(_in.isCascade), enable(
					_in.enable), isMerged(_in.isMerged)
	{
	}
};
/**
* @ingroup VisionModule
*
* @brief For detecting at least 50% white ball
**/
class BallDetector: public IDetector
{
private:
	CascadeClassifier object_cascade;
	bool checkHistogram(const Mat &rawHSV, const Mat &fieldBinaryRaw,
			const vector<cv::Point> &con, double minHistogramDiff[3],
			double histResult[3]);

	bool checkHistogram(const Mat &rawHSV, const Mat &fieldBinaryRaw,
			cv::Point center, int radius, double minHistogramDiff[3],
			double histResult[3]);

	bool checkHistogramInPic(cv::Mat &hsvRoi, cv::Mat &justBallMask,
			double minHistogramDiff[3], double histResult[3]);

	bool checkDistance_Rec(const cv::Rect &rec, CameraProjections &projection,
			Point2f &realPos);

public:
	std::vector<BallCircleC> GetBall(const Mat &rawHSV,
			const vector<Point> &hullField, const Mat &fieldBinaryRaw,
			Mat &rawGray, const Mat &ballMask, const Mat &fieldConvectHull,
			CameraProjections &projection, Mat &cannyImg,
			GuiManager *_guiManager, Mat &guiImg, bool SHOWGUI);

	/**
	 * @brief Init function for ball detection
	 *
	 * This function is called in the vision class
	 **/
	bool Init();
	bool LoadCascade();
	inline ~BallDetector()
	{
	}
};

