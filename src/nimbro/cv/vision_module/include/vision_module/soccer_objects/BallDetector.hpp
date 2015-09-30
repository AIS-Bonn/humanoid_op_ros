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
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/soccer_objects/IDetector.hpp>
#include <algorithm>    // std::sort
using namespace cv;
class BallDetector:public IDetector
{
private:
	CascadeClassifier object_cascade;
	vector<Mat> hist_list;
	vector<cv::Rect> lastBallDetection;
	bool checkHistogram(const Mat &rawHSV,const Mat &fieldBinaryRaw,const vector<cv::Point> &con,double minHistogramDiffH,
		double minHistogramDiffS,
		double minHistogramDiffV);

	bool checkHistogram(const Mat &rawHSV,const Mat &fieldBinaryRaw,cv::Point center, int radius, double minHistogramDiffH ,
		double minHistogramDiffS,
		double minHistogramDiffV );

	bool checkHistogramInPic(cv::Mat &hsvRoi, cv::Mat &justBallMask, double minHistogramDiffH,
		double minHistogramDiffS ,
		double minHistogramDiffV );

	bool checkDistance_Rec(const cv::Rect &rec,CameraProjections &projection);

	bool checkContourRatio(const cv::vector<cv::Point> &con);
public:
	std::vector<cv::Rect> GetBallRect(const Mat &rawHSV,const vector<Point> &hullField, const Mat &fieldBinaryRaw,Mat &rawGray,
			const Mat &ballMask,
			CameraProjections &projection,Mat &guiImg,bool SHOWGUI);


	bool Init();

};


