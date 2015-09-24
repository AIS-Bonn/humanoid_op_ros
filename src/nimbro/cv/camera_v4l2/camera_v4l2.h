// V4L2 camera driver
// Author: Max Schwarz <Max@x-quadraht.de>

#ifndef CAMERA_V4L2_H
#define CAMERA_V4L2_H

#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

#include <vector>
#include <stdint.h>

#include <linux/videodev2.h>

#include <camera_v4l2/EnumerateCameraParams.h>
#include <camera_v4l2/SetCameraParam.h>
#include <ros/service_server.h>
#include <ros/node_handle.h>
#include <boost/thread.hpp>

#include "configmanager.h"

struct Buffer;

class CameraV4L2 : public nodelet::Nodelet
{
public:
    CameraV4L2();
    virtual ~CameraV4L2();

	virtual void onInit();
	virtual void update();
private:
	bool srvEnumerateParams(
		camera_v4l2::EnumerateCameraParamsRequest& req,
		camera_v4l2::EnumerateCameraParamsResponse& resp
	);
	bool srvSetParam(
		camera_v4l2::SetCameraParamRequest& req,
		camera_v4l2::SetCameraParamResponse& resp
	);

	void initCamera();

	ros::Publisher m_pub_image;
	int m_fd;

	std::vector<Buffer*> m_buffers;
	v4l2_format m_format;

	ros::Timer m_timer;

	std::string m_device;

	ros::ServiceServer m_srv_enumerate;
	ros::ServiceServer m_srv_setParam;

	boost::mutex m_mutex;

	ConfigManager m_config;
	ros::Time m_lastWrite;
	std::string m_configFile;

	ros::Timer m_restartTimer;
};

#endif
