// Color classificator
// Author: Max Schwarz <Max@x-quadraht.de>

#include "classificator.h"

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/make_shared.hpp>

PLUGINLIB_DECLARE_CLASS(openplatform, classificator, Classificator, nodelet::Nodelet)

Classificator::Classificator()
{
}

Classificator::~Classificator()
{
}

void Classificator::onInit()
{
	ros::NodeHandle& nh = getNodeHandle();

	m_sub_input = nh.subscribe("image", 1, &Classificator::processImage, this);
	m_pub_output = nh.advertise<sensor_msgs::Image>("classes", 10);

	m_srv_process = getPrivateNodeHandle().advertiseService("processImage", &Classificator::srvProcessImage, this);
	m_srv_updateLUT = getPrivateNodeHandle().advertiseService("updateLUT", &Classificator::srvUpdateLUT, this);
}

uint8_t Classificator::getClass(uint8_t y, uint8_t u, uint8_t v)
{
	if(!m_lut)
		return 255;

	int c = m_lut->lut[256*u + v];

	if(c == 255)
		return c;

	const calibration::ColorSpec& spec = m_lut->colors[c];
	if(y < spec.minY || y > spec.maxY)
		return 255;

	return c;
}

void Classificator::processImage(const sensor_msgs::Image::Ptr& img)
{
	sensor_msgs::Image::Ptr output = boost::make_shared<sensor_msgs::Image>();
	doProcessImage(*img, &*output);

	m_pub_output.publish(output);
}

void Classificator::doProcessImage(const sensor_msgs::Image& img, sensor_msgs::Image* output)
{
	output->encoding = "classes";
	output->data.resize(img.width*img.height);
	output->width = img.width;
	output->height = img.height;
	output->step = img.width;

	const uint8_t* line = &img.data[0];
	uint8_t* dline = &output->data[0];
	for(size_t y = 0; y < img.height; ++y)
	{
		const uint8_t* base = line;
		uint8_t* dbase = dline;
		for(size_t x = 0; x < img.width-1; x += 2)
		{
			uint8_t y1 = base[0];
			uint8_t u = base[1];
			uint8_t y2 = base[2];
			uint8_t v = base[3];

			dbase[0] = getClass(y1, u, v);
			dbase[1] = getClass(y2, u, v);

			base += 4;
			dbase += 2;
		}
		line += img.step;
		dline += output->step;
	}

	output->header.frame_id="base_link";
	output->header.stamp = img.header.stamp;
}

bool Classificator::srvUpdateLUT(
	calibration::UpdateLUTRequest& req,
	calibration::UpdateLUTResponse& response)
{
	NODELET_INFO("Got new color LUT");
	m_lut = boost::make_shared<calibration::CalibrationLUT>(req.lut);
	return true;
}


bool Classificator::srvProcessImage(
	calibration::ClassifyImageRequest& req,
	calibration::ClassifyImageResponse& response)
{
	doProcessImage(req.input, &response.output);
	return true;
}

