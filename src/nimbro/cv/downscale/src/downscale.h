// Image downscaler
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DOWNSCALE_H
#define DOWNSCALE_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include <vector>
#include <stdint.h>

namespace downscale
{

/**
 * @brief Downscale & throttle an image
 * @ingroup nodes
 *
 * Useful for viewing image topics over WiFi. At the moment, the downsampling
 * factor is fixed to 2.0. The throttling frequency can be changed using the
 * `frequency` rosparam.
 **/
class Downscale : public nodelet::Nodelet
{
public:
	Downscale();
	virtual ~Downscale();

	virtual void onInit();

	void processImage(const sensor_msgs::Image::Ptr& img);
	void handleConnection();
private:
	ros::Subscriber m_sub_image;
	bool m_subscribed;

	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Publisher m_pub_image;

	ros::Time m_lastTime;
	ros::Duration m_period;
};

}

#endif
