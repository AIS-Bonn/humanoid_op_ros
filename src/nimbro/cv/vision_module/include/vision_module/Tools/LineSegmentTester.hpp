//LineSegmentTester.hpp
// Created on: Jun 1, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Tools/General.hpp>

using namespace std;
using namespace cv;

void CallBackFunc(int event, int x, int y, int flags, void* userdata);

class LineSegmentTester
{
public:
	Mat Image;
	Vector<LineSegment> Lines;
	string name;
	int innerIndex, totalIndex;
	bool enable;
	Point p1;
	size_t Max;
public:
	inline LineSegmentTester(Size box, size_t _max = 2, string _name =
			"LineSegmentTester") :
			name(_name), innerIndex(0), totalIndex(0), enable(false), Max(_max)
	{
		Image = Mat::zeros(box, CV_8UC3);
		namedWindow(name, 1);
		//set the callback function for any mouse event
		setMouseCallback(name, CallBackFunc, this);
	}
	inline bool IsReady(size_t count)
	{
		return Lines.size() >= count;
	}
	inline bool IsReady()
	{
		return Lines.size() >= Max;
	}
	inline void Show(int wait = 1)
	{
		for (size_t i = 0; i < Lines.size(); i++)
		{
			line(Image, Lines[i].P1, Lines[i].P2, redColor(), 1);
		}
		if (innerIndex == 1)
		{
			circle(Image, p1, 2, yellowColor(), 2);
		}
		imshow(name, Image);
		waitKey(wait);
		Image = Mat::zeros(Image.size(), CV_8UC3);
	}
};

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	LineSegmentTester* data = reinterpret_cast<LineSegmentTester*>(userdata);
	if (event == EVENT_LBUTTONDOWN)
	{
		if (data->innerIndex == 1 && data->Lines.size()<data->Max)
		{
			data->Lines.push_back(LineSegment(data->p1, Point(x, y)));
		}
		data->enable = true;
	}
	else if (event == EVENT_RBUTTONDOWN)
	{
		data->innerIndex=0;
		if (data->Lines.size() > 0)
		{
			data->Lines.pop_back();
		}
	}
	else if (event == EVENT_MBUTTONDOWN)
	{
		data->Lines.clear();
		data->innerIndex=0;
	}
	else if (event == EVENT_LBUTTONUP)
	{
		data->enable = false;
		if (data->innerIndex == 0)
		{
			data->p1 = Point(x, y);
		}
		if (data->innerIndex == 1)
		{
			data->Lines[data->Lines.size() - 1].P1 = data->p1;
			data->Lines[data->Lines.size() - 1].P2 = Point(x, y);
		}
		data->innerIndex++;
		if (data->innerIndex > 1)
			data->innerIndex = 0;
	}
	else if (event == EVENT_MOUSEMOVE)
	{
		if (data->enable)
		{
			if (data->innerIndex == 0)
			{
				data->p1 = Point(x, y);
			}
			if (data->innerIndex == 1)
			{
				data->Lines[data->Lines.size() - 1].P1 = data->p1;
				data->Lines[data->Lines.size() - 1].P2 = Point(x, y);
			}
		}
	}
}
