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

void CallBackFuncCT(int event, int x, int y, int flags, void* userdata);

class ContourTester
{
public:
	Mat Image;
	vector<vector<Point> > Contours;
	string name;
	size_t Max;
	vector<Scalar> colors;
public:
	inline ContourTester(Size box, size_t _max = 2, string _name =
			"ContourTester") :
			name(_name), Max(_max)
	{
		Image = Mat::zeros(box, CV_8UC3);
		namedWindow(name, 1);
		//set the callback function for any mouse event
		setMouseCallback(name, CallBackFuncCT, this);
		Contours.push_back(vector<Point>());
		colors.push_back(randColor());
	}
	inline bool IsReady(size_t count)
	{
		return Contours.size() > count
				&& Contours[Contours.size() - 1].size() > 2;
	}
	inline bool IsReady()
	{
		return Contours.size() > Max && Contours[Contours.size() - 1].size() > 2;
	}
	inline void Show(int wait = 1)
	{
		for (size_t i = 0; i < Contours.size(); i++)
		{
			if (Contours.size() - 1 == i)
			{
				drawContours(Image, Contours, i, redColor(), 2);
			}
			else
			{
				drawContours(Image, Contours, i, colors[i], 1);
			}
		}
		if(Contours[Contours.size() - 1].size() == 1)
		{
			circle(Image,Contours[Contours.size() - 1][0],2,redColor(),1);
		}
		imshow(name, Image);
		waitKey(wait);
		Image = Mat::zeros(Image.size(), CV_8UC3);
	}
};

void CallBackFuncCT(int event, int x, int y, int flags, void* userdata)
{
	ContourTester* data = reinterpret_cast<ContourTester*>(userdata);
	if (event == EVENT_RBUTTONUP)
	{
		if (data->Contours[data->Contours.size() - 1].size() > 0)
		{
			data->Contours[data->Contours.size() - 1].pop_back();
		}
		else
		{
			if (data->Contours.size() > 1)
			{
				data->Contours.pop_back();
				data->colors.pop_back();
			}
		}
	}
	else if (event == EVENT_MBUTTONDOWN)
	{
		if (data->Contours[data->Contours.size() - 1].size() < 3)
			return;
		data->Contours.push_back(vector<Point>());
		data->colors.push_back(randColor());
	}
	else if (event == EVENT_LBUTTONUP)
	{
		data->Contours[data->Contours.size() - 1].push_back(Point(x, y));
	}
}
