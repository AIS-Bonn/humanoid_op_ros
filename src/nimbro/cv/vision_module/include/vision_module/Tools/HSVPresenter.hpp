//HSVPresenter.hpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/MatPublisher.hpp>
using namespace cv;
/**
* @ingroup VisionModule
*
* @brief A class for showing HSV color range
**/
class HSVPresenter
{
private:
	int rangeImgWidth;
	int rangeImgHeight;
	Size rangeImgSize;
	bool lastActiveRange[COLORED_OBJECT_COUNT];
	int activeIndex;
	hsvRangeC currecntRanges[COLORED_OBJECT_COUNT];
	Mat rangeImg;
public:
	HSVPresenter() :
			rangeImgWidth(50), rangeImgHeight(200), rangeImgSize(rangeImgWidth,
					rangeImgHeight), activeIndex(-1), rangeImg(rangeImgSize,
					CV_8UC3)
	{
		lastActiveRange[0] = params.field->GetHSVRange().active->get();
		lastActiveRange[1] = params.ball->GetHSVRange().active->get();
		lastActiveRange[2] = params.goal->GetHSVRange().active->get();
		lastActiveRange[3] = params.line->GetHSVRange().active->get();
		lastActiveRange[4] = params.obstacle->GetHSVRange().active->get();
		lastActiveRange[5] = params.igus->GetHSVRange().active->get();

		currecntRanges[0] = params.field->GetHSVRange();
		currecntRanges[1] = params.ball->GetHSVRange();
		currecntRanges[2] = params.goal->GetHSVRange();
		currecntRanges[3] = params.line->GetHSVRange();
		currecntRanges[4] = params.obstacle->GetHSVRange();
		currecntRanges[5] = params.igus->GetHSVRange();
	}
	inline ~HSVPresenter()
	{
	}
	bool Update();
	void DrawOnInputMat(Mat &guiImg, bool SHOWGUI);
};

