//HSVPresenter.hpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/MatPublisher.hpp>
using namespace cv;

class HSVPresenter
{
private:
	int rangeImgWidth;
	int rangeImgHeight;
	Size rangeImgSize;
	Mat rangeImg;
	bool lastActiveRange[COLORED_OBJECT_COUNT];
	int activeIndex;
	hsvRangeC currecntRanges[4];
	MatPublisher rangeImg_pub;
public:
	HSVPresenter() :
			rangeImgWidth(50), rangeImgHeight(200), rangeImgSize(rangeImgWidth,
					rangeImgHeight), rangeImg(rangeImgSize, CV_8UC3),activeIndex(-1),rangeImg_pub("/vision/rangeImg")
	{
		lastActiveRange[0] = params.field.active->get();
		lastActiveRange[1] = params.ball.active->get();
		lastActiveRange[2] = params.goal.active->get();
		lastActiveRange[3] = params.line.active->get();

		currecntRanges[0] = params.field;
		currecntRanges[1] = params.ball;
		currecntRanges[2] = params.goal;
		currecntRanges[3] = params.line;
	}
	void Publish();
	bool Update();
};

