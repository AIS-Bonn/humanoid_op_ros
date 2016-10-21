//LineDetector.hpp
// Created on: May 14, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <algorithm>    // std::sort
#include <math.h>       /* pow */
#include <vision_module/SoccerObjects/IDetector.hpp>
#include <vision_module/Tools/BoundaryLinearInterpolation.hpp>

using namespace cv;
/**
* @ingroup VisionModule
*
* @brief For detecting field lines
**/
class LineDetector: public IDetector
{
public:
	bool GetLines(Mat &rawHSV, Mat & fieldMask, Mat &guiImg,
			CameraProjections &projection, bool SHOWGUI, const Mat &lineBinary,
			Rect box, vector<LineSegment> &resLines);
	inline ~LineDetector()
	{
	}
	inline bool Init()
	{
		return true;
	}
};
