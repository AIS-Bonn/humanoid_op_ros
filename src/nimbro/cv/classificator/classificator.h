// Color classificator
// Author: Max Schwarz <Max@x-quadraht.de>

#ifndef CAMERA_V4L2_H
#define CAMERA_V4L2_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <calibration/CalibrationLUT.h>
#include <calibration/ClassifyImage.h>
#include <calibration/UpdateLUT.h>

#include <vector>
#include <stdint.h>

#include "colorlut.h"

class CalibWidget;

class Classificator : public nodelet::Nodelet
{
public:
	Classificator();
	virtual ~Classificator();

	virtual void onInit();

	void processImage(const sensor_msgs::Image::Ptr& img);

	bool srvProcessImage(
		calibration::ClassifyImageRequest& req,
		calibration::ClassifyImageResponse& response
	);
	bool srvUpdateLUT(
		calibration::UpdateLUTRequest& req,
		calibration::UpdateLUTResponse& response
	);
private:
	ros::Subscriber m_sub_input;
	ros::Publisher m_pub_output;

	ros::ServiceServer m_srv_process;
	ros::ServiceServer m_srv_updateLUT;

	calibration::CalibrationLUT::Ptr m_lut;

	uint8_t getClass(uint8_t y, uint8_t u, uint8_t v);
	void doProcessImage(const sensor_msgs::Image& img, sensor_msgs::Image* dest);
};

#endif
