//LineDetector.hpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/soccer_objects/IDetector.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Tools/LinearInterpolation.hpp>
#include <algorithm>    // std::sort
using namespace cv;
class LineDetector: public IDetector
{
public:
	bool GetLines(Mat &rawHSV, Mat & fieldMask,
			Mat &guiImg, bool SHOWGUI, const Mat &lineBinary,
			vector<LineSegment> &resLines);
	inline bool Init()
	{
		return true;
	}
	;
};
