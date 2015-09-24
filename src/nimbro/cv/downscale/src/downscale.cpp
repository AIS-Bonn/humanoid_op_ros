// Image downscaler
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "downscale.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pluginlib/class_list_macros.h>

namespace downscale
{

Downscale::Downscale()
 : m_subscribed(false)
{
}

Downscale::~Downscale()
{
}

void Downscale::onInit()
{
	ros::NodeHandle nh = getPrivateNodeHandle();

	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub_image = m_it->advertise("out", 1, boost::bind(&Downscale::handleConnection, this));

	double freq;
	nh.param("frequency", freq, 0.5);

	m_period = ros::Duration(1.0 / freq);
}

void Downscale::handleConnection()
{
	if(!m_subscribed)
	{
		ros::NodeHandle nh = getPrivateNodeHandle();
		m_sub_image = nh.subscribe("in", 1, &Downscale::processImage, this);
		m_subscribed = true;
	}
}

void Downscale::processImage(const sensor_msgs::Image::Ptr& img)
{
	if(m_pub_image.getNumSubscribers() == 0)
	{
		if(m_subscribed)
		{
			m_sub_image.shutdown();
			m_subscribed = false;
		}
		return;
	}

	ros::Time now = ros::Time::now();
	if((now - m_lastTime) < m_period)
		return;

	cv_bridge::CvImageConstPtr cvImg;

	if(img->encoding == "YUYV")
	{
		cv_bridge::CvImagePtr mImg = boost::make_shared<cv_bridge::CvImage>();
		mImg->header = img->header;
		mImg->encoding = "bgr8";

		cv::Mat source((int)img->height, (int)img->width, CV_8UC2, &img->data[0], img->step);

		cv::cvtColor(source, mImg->image, CV_YUV2BGR_YUYV);
		cvImg = mImg;
	}
	else
		cvImg = cv_bridge::toCvShare(img, "bgr8");

	cv_bridge::CvImagePtr output = boost::make_shared<cv_bridge::CvImage>();
	output->header = cvImg->header;
	output->encoding = cvImg->encoding;
	cv::pyrDown(cvImg->image, output->image);

	m_pub_image.publish(output->toImageMsg());
	m_lastTime = now;
}

}


PLUGINLIB_EXPORT_CLASS(downscale::Downscale, nodelet::Nodelet)
