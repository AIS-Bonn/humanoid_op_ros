// Image display
// Author: Max Schwarz <Max@x-quadraht.de>

#ifndef CAMERA_V4L2_H
#define CAMERA_V4L2_H

#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <qtguithread/qtguithread.h>

#include <vector>
#include <stdint.h>

class ImageWidget;
class QWidget;

class DisplayImage : public nodelet::Nodelet, public WidgetProvider
{
public:
	DisplayImage();
	virtual ~DisplayImage();

	virtual void onInit();
	QWidget* createWidget();

	void processImage(const sensor_msgs::Image::Ptr& img);
private:
    ros::Subscriber m_sub_input;
	ImageWidget* m_gui;
};

#endif
