// Image display
// Author: Max Schwarz <Max@x-quadraht.de>

#include "display_image.h"

#include <QtGui/QImage>
#include <QtGui/QLabel>
#include <QtGui/QPainter>

#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/image_encodings.h>

#include "imagewidget.h"

PLUGINLIB_DECLARE_CLASS(openplatform, display_image, DisplayImage, nodelet::Nodelet)

DisplayImage::DisplayImage()
{
}

DisplayImage::~DisplayImage()
{
}

void DisplayImage::onInit()
{
	m_sub_input = getNodeHandle().subscribe("image", 1, &DisplayImage::processImage, this);

	GUIThread::getInstance()->addWidgetProvider(this);
}

QWidget* DisplayImage::createWidget()
{
	ImageWidget* gui = new ImageWidget();
	gui->setWindowTitle("DisplayImage");
	gui->show();
	m_gui = gui;

	return m_gui;
}

void DisplayImage::processImage(const sensor_msgs::Image::Ptr& img)
{
	if(m_gui)
	{
		m_gui->updateWith(img);
		GUIThread::getInstance()->updateWidget(m_gui);
	}
}
