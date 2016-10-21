//GuiManager.hpp
// Created on: May 7, 2016
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <ros/ros.h>
#include <stdio.h>
#include <stdarg.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/MatPublisher.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <rqt_vision_module/GuiEvent.h>
#include <rqt_vision_module/mouseevent.hpp>
#include <vision_module/Tools/LinearBoundaryChecker.hpp>
#include <rqt_vision_module/ConsoleMsg.h>
using namespace cv;
/**
* @ingroup VisionModule
*
* @brief A container class for distance-size calibration
**/
class DistanceSizeS
{

public:
	double length1;
	double length2;
	Point2d realLocation;
	Point2d imgLocation;
	DistanceSizeS()
	{
		reset();
	}
	bool isValid()
	{
		return ((length1 >= 0 && length2 >= 0) && GetDistance(realLocation) >= 0
				&& imgLocation.x >= 0 && imgLocation.x < IMGWIDTH
				&& imgLocation.y >= 0 && imgLocation.y < IMGHEIGHT);
	}
	void reset()
	{
		length1 = length2 = 0;
		realLocation.x = realLocation.y = 0;
		imgLocation.x = imgLocation.y = -1;
	}
};
/**
* @ingroup VisionModule
*
* @brief A container class for gui rectangles
**/
class GuiRetS
{
public:
	Point2d p1;
	Point2d p2;
	bool hold;
	GuiRetS()
	{
		reset();
	}
	bool isFinished()
	{
		return !hold && isValid();
	}
	bool isValid()
	{
		return p1.x >= 0 && p1.x < IMGWIDTH && p1.y >= 0 && p1.y < IMGHEIGHT
				&& p2.x >= 0 && p2.x < IMGWIDTH && p2.y >= 0 && p2.y < IMGHEIGHT
				&& abs(p1.x - p2.x) >= 1 && abs(p1.y && p2.y) >= 1;
	}
	void reset()
	{
		p1.x = p1.y = p2.x = p2.y = -1;
		hold = false;
	}
	cv::Rect getRect()
	{
		Rect rec;
		if (p1.y <= p2.y && p1.x <= p2.x)
		{
			rec.x = p1.x;
			rec.y = p1.y;
			rec.width = abs(p1.x - p2.x);
			rec.height = abs(p1.y - p2.y);
		}
		else if (p1.y <= p2.y && p1.x > p2.x)
		{
			rec.x = p2.x;
			rec.y = p1.y;
			rec.width = abs(p1.x - p2.x);
			rec.height = abs(p1.y - p2.y);
		}
		else if (p1.y > p2.y && p1.x <= p2.x)
		{
			rec.x = p1.x;
			rec.y = p2.y;
			rec.width = abs(p1.x - p2.x);
			rec.height = abs(p1.y - p2.y);
		}
		else if (p1.y > p2.y && p1.x > p2.x)
		{
			rec.x = p2.x;
			rec.y = p2.y;
			rec.width = abs(p1.x - p2.x);
			rec.height = abs(p1.y - p2.y);
		}
		return rec;
	}
};
/**
* @ingroup VisionModule
*
* @brief A container class for managing gui events
**/
class GuiManager
{
private:
	DistanceSizeS distanceSizeSample;
	Point mouseLastMovePos;
	vector<Point> histogramContour;
	GuiRetS rectSaver;
	ros::Publisher vision_msg_pub;
	CameraProjections *projection;
	CascadeClassifier object_cascade;
public:
	ros::NodeHandle nodeHandle;
	ros::Subscriber gui_events_sub;
	ros::Subscriber rviz_click_sub;

	GuiManager(CameraProjections *projection) :
			mouseLastMovePos(0, 0), projection(projection)
	{
		vision_msg_pub = nodeHandle.advertise<rqt_vision_module::ConsoleMsg>(
				"/rqt_vision_module/console", 10);
		gui_events_sub = nodeHandle.subscribe<rqt_vision_module::GuiEvent>(
				"/rqt_vision_module/gui_events", 30, &GuiManager::gui_events_callback,
				this);
		rviz_click_sub = nodeHandle.subscribe<geometry_msgs::PointStamped>(
				"/clicked_point", 1, &GuiManager::rviz_click_callback, this);
	}

	void writeGuiConsole(Scalar color, const char * format, ...)
	{
		char buffer[256];
		va_list args;
		va_start(args, format);
		vsnprintf(buffer, 255, format, args);
		ROS_INFO(" %s", buffer);
		rqt_vision_module::ConsoleMsg msg;
		msg.message = buffer;
		msg.b = color.val[0];
		msg.g = color.val[1];
		msg.r = color.val[2];
		msg.font_size=12;
		vision_msg_pub.publish(msg);
		va_end(args);
	}

	void writeGuiConsoleFormat(Scalar color, const char * format, ...)
	{
		time_t t = time(0);
		char bufferTime[9] =
		{ 0 };
		strftime(bufferTime, 12, "%H:%M:%S | ", localtime(&t));

		char buffer[256];
		va_list args;
		va_start(args, format);
		vsnprintf(buffer, 255, format, args);

		string txt=bufferTime;
		txt+=buffer;
		if (color == redColor())
		{
			ROS_ERROR(" %s", txt.c_str());
		}
		if (color == yellowColor())
		{
			ROS_WARN(" %s",  txt.c_str());
		}
		else
		{
			ROS_INFO(" %s",  txt.c_str());
		}
		rqt_vision_module::ConsoleMsg msg;
		msg.message = txt;
		msg.message += "\r\n";
		msg.b = color.val[0];
		msg.g = color.val[1];
		msg.r = color.val[2];
		msg.font_size=12;
		vision_msg_pub.publish(msg);
		va_end(args);
	}

	inline bool Init()
	{
		return LoadCascade();
	}
	inline bool LoadCascade()
	{
		return object_cascade.load(params.configPath + "cascadeBall.xml");
	}
	void Publish(const Mat &gray, Mat &guiRawImg, bool shouldPublish);
	void Update(const Mat &rawHSV, const Mat & img);
	void rviz_click_callback(const geometry_msgs::PointStampedConstPtr& msg);
	void gui_events_callback(const rqt_vision_module::GuiEvent::ConstPtr & msg);
	inline ~GuiManager()
	{
	}

};

