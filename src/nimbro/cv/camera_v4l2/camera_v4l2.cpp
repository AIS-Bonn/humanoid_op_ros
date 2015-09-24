// V4L2 camera driver
// Author: Max Schwarz <Max@x-quadraht.de>

#include "camera_v4l2.h"

#include <pluginlib/class_list_macros.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <yaml-cpp/yaml.h>
#include <XmlRpcValue.h>

#include <sensor_msgs/image_encodings.h>

#include <stdio.h>
#include <string.h>
#include <fstream>
#include <boost/make_shared.hpp>
#include <ros/node_handle.h>

PLUGINLIB_DECLARE_CLASS(openplatform, camera_v4l2, CameraV4L2, nodelet::Nodelet)

struct Buffer
{
	uint8_t* data_ptr;
	size_t length;
	sensor_msgs::Image::Ptr msg;
	bool queued;
};

CameraV4L2::CameraV4L2()
 : m_fd(-1)
{
}

CameraV4L2::~CameraV4L2()
{
	ROS_WARN("CameraV4L2: shutting down...");

	if(m_config.writePending() && !m_configFile.empty())
	{
		std::ofstream out(m_configFile.c_str());
		m_config.writeConfigFile(m_fd, &out);
	}

	close(m_fd);
}

void CameraV4L2::onInit()
{
	boost::mutex::scoped_lock lock(m_mutex);

	ros::NodeHandle nh = getPrivateNodeHandle();

//	m_pub_image = getNodeHandle().advertise<sensor_msgs::Image>("image_raw", 2);
	m_pub_image = getNodeHandle().advertise<sensor_msgs::Image>("image", 2);

	m_restartTimer = nh.createTimer(ros::Duration(1.0), boost::bind(&CameraV4L2::initCamera, this), false, false);

	m_srv_enumerate = nh.advertiseService(
		"enumerateParams", &CameraV4L2::srvEnumerateParams, this);
	m_srv_setParam = nh.advertiseService(
		"setParam", &CameraV4L2::srvSetParam, this);

	initCamera();
}

void CameraV4L2::initCamera()
{
	ros::NodeHandle nh = getPrivateNodeHandle();
 	nh.param("device", m_device, std::string("/dev/video0"));
//	nh.param("device", m_device, std::string("v4l/by-id/usb-046d_080a_82749167-video-index0"));

	if(m_fd >= 0)
		close(m_fd);

	m_fd = open(m_device.c_str(), O_RDWR);
	if (m_fd >= 0)
		NODELET_INFO("Using device: %s", m_device.c_str());

	if(m_fd < 0)
	{
		NODELET_ERROR("Could not open device : %s. Retrying in 5 seconds", m_device.c_str());
		m_restartTimer.setPeriod(ros::Duration(5.0));
		m_restartTimer.start();
		return;
	}

	v4l2_capability caps;
	if(ioctl(m_fd, VIDIOC_QUERYCAP, &caps) != 0)
	{
		perror("ioctl()");
		NODELET_FATAL("Could not open video device");
		return;
	}

	NODELET_INFO("V4L2 device: %s", caps.card);
	NODELET_INFO("V4L2 driver: %s", caps.driver);

	if(!(caps.capabilities & V4L2_CAP_STREAMING))
	{
		NODELET_FATAL("V4L2 device does not support streaming API");
		return;
	}

	v4l2_fmtdesc fmt;
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.index = 0;

	NODELET_INFO("Supported formats:");
	while(1)
	{
		int ret = ioctl(m_fd, VIDIOC_ENUM_FMT, &fmt);
		if(ret != 0)
		{
			if(errno == EINVAL)
				break;

			NODELET_FATAL("Could not get formats: %s", strerror(errno));
			return;
		}

		NODELET_INFO(" - %s", fmt.description);

		NODELET_INFO("   ^ Supported resolutions:");
		v4l2_frmsizeenum framesize;
		framesize.index = 0;
		framesize.pixel_format = fmt.pixelformat;
		while(1)
		{
			ret = ioctl(m_fd, VIDIOC_ENUM_FRAMESIZES, &framesize);
			if(ret != 0)
			{
				if(errno == EINVAL)
					break;

				NODELET_FATAL("Could not get frame size: %s", strerror(errno));
				return;
			}

			NODELET_INFO("      - %dx%d", framesize.discrete.width, framesize.discrete.height);
			framesize.index++;
		}

		fmt.index++;
	}

	NODELET_INFO("Using format:");
	memset(&m_format, 0, sizeof(m_format));
	m_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if(ioctl(m_fd, VIDIOC_G_FMT, &m_format) != 0)
	{
		NODELET_FATAL("Could not get default format: %s", strerror(errno));
		return;
	}

	int w,h;
	nh.param("width", w, 800);
	nh.param("height", h, 600);

	m_format.fmt.pix.width = w;
	m_format.fmt.pix.height = h;

	if(ioctl(m_fd, VIDIOC_S_FMT, &m_format) != 0)
	{
		NODELET_FATAL("Could not set format: %s", strerror(errno));
		return;
	}
	NODELET_INFO(" - resolution: %dx%d", m_format.fmt.pix.width, m_format.fmt.pix.height);

	v4l2_streamparm parm;
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(m_fd, VIDIOC_G_PARM, &parm) != 0)
	{
		NODELET_FATAL("Could not get streaming parameters: %s", strerror(errno));
		return;
	}

	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = 30;
	if(ioctl(m_fd, VIDIOC_S_PARM, &parm) != 0)
	{
		NODELET_FATAL("Could not set streaming parameters: %s", strerror(errno));
		return;
	}

	NODELET_INFO(" - timeperframe: %d/%d",
				 parm.parm.capture.timeperframe.numerator,
			  parm.parm.capture.timeperframe.denominator
				);

	NODELET_INFO("Requesting buffers...");
	v4l2_requestbuffers reqbuf;
	memset(&reqbuf, 0, sizeof(reqbuf));
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	reqbuf.count = 8;

	if(ioctl(m_fd, VIDIOC_REQBUFS, &reqbuf) != 0)
	{
		NODELET_FATAL("Could not request mmap buffers: %s", strerror(errno));
		return;
	}

	NODELET_INFO("Got %d buffers", reqbuf.count);

	if(reqbuf.count < 1)
	{
		NODELET_FATAL("Got too few buffers to operate well");
		return;
	}

	m_buffers.clear();
	for(size_t i = 0; i < reqbuf.count; ++i)
	{
		Buffer* buf = new Buffer;
		v4l2_buffer req;
		memset(&req, 0, sizeof(req));
		req.type = reqbuf.type;
		req.memory = V4L2_MEMORY_MMAP;
		req.index = i;

		if(ioctl(m_fd, VIDIOC_QUERYBUF, &req) != 0)
		{
			NODELET_FATAL("Could not get buffer information: %s",
				strerror(errno)
			);
			return;
		}

		NODELET_DEBUG_STREAM("mmap(" << i << "): " << m_fd << ", " << req.m.offset << ", " << req.length);
		buf->data_ptr = (uint8_t*)mmap(NULL, req.length,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			m_fd, req.m.offset
		);

		if((void*)buf->data_ptr == MAP_FAILED)
		{
			NODELET_FATAL("Could not mmap(): %s", strerror(errno));
			return;
		}

		if(ioctl(m_fd, VIDIOC_QBUF, &req) != 0)
		{
			NODELET_FATAL("Could not queue buffer (init)");
			return;
		}
		buf->queued = true;

		buf->length = req.length;
		buf->msg = sensor_msgs::Image::Ptr(new sensor_msgs::Image());
		buf->msg->width = m_format.fmt.pix.width;
		buf->msg->height = m_format.fmt.pix.height;
		buf->msg->encoding = std::string((const char*)&m_format.fmt.pix.pixelformat, 4);
		buf->msg->step = m_format.fmt.pix.bytesperline;
		buf->msg->is_bigendian = 0;
		buf->msg->data.resize(buf->length);

		m_buffers.push_back(buf);
	}

	XmlRpc::XmlRpcValue parameter_param;
	std::vector<std::string> parameters;
	nh.getParam("parameters", parameter_param);
	ROS_ASSERT(parameter_param.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i = 0; i < parameter_param.size(); ++i)
		parameters.push_back(parameter_param[i]);

	if(m_config.init(m_fd, parameters) != 0)
	{
		NODELET_FATAL("Could not initialize config subsystem");
		return;
	}
	if(nh.hasParam("configFile"))
	{
		nh.param("configFile", m_configFile, std::string());

		std::ifstream file(m_configFile.c_str());
		m_config.readConfigFile(m_fd, &file);
	}

	NODELET_INFO("Starting streaming");
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(m_fd, VIDIOC_STREAMON, &type) != 0)
	{
		NODELET_FATAL("Could not start streaming: %s", strerror(errno));
		return;
	}

	m_timer = getMTNodeHandle().createTimer(
		ros::Duration(1.0 / 35.0),
		boost::bind(&CameraV4L2::update, this)
	);

	m_restartTimer.stop();
}

void CameraV4L2::update()
{
	boost::mutex::scoped_lock lock(m_mutex);
	v4l2_buffer req;
	memset(&req, 0, sizeof(req));
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if(ioctl(m_fd, VIDIOC_DQBUF, &req) != 0)
	{
		NODELET_ERROR("Could not get filled buffer: %s", strerror(errno));
		perror("Could not get filled buffer");
		NODELET_ERROR("Trying to restart in one second...");
		close(m_fd);
		m_fd = -1;
		m_timer.stop();
		if(!m_restartTimer.hasPending())
			m_restartTimer.start();
		return;
	}

	Buffer* buf = m_buffers[req.index];
	buf->queued = false;

	sensor_msgs::Image::Ptr out = boost::make_shared<sensor_msgs::Image>();
	out->width = m_format.fmt.pix.width;
	out->height = m_format.fmt.pix.height;
	out->step = m_format.fmt.pix.bytesperline;
	out->encoding = std::string((const char*)&m_format.fmt.pix.pixelformat, 4);
	out->data.resize(buf->length);
	out->header.stamp = ros::Time::now();
	out->header.frame_id = "/camera_optical";


	memcpy(&out->data[0], buf->data_ptr, buf->length);
	m_pub_image.publish(out);

	if(ioctl(m_fd, VIDIOC_QBUF, &req) != 0)
	{
		NODELET_ERROR("Could not queue buffer: %s", strerror(errno));
		perror("Could not queue buffer");
		return;
	}

	if(m_config.writePending())
	{
		ros::Time now = ros::Time::now();
		if(now - m_lastWrite > ros::Duration(3.0))
		{
			std::ofstream out(m_configFile.c_str());
			m_config.writeConfigFile(m_fd, &out);
			m_lastWrite = now;
			NODELET_INFO("Updated config file");
		}
	}
}

bool CameraV4L2::srvEnumerateParams(camera_v4l2::EnumerateCameraParamsRequest& req, camera_v4l2::EnumerateCameraParamsResponse& resp)
{
	resp = m_config.enumerateParams();

	return true;
}

bool CameraV4L2::srvSetParam(camera_v4l2::SetCameraParamRequest& req, camera_v4l2::SetCameraParamResponse& resp)
{
	return (m_config.setParamValue(m_fd, req.param.id, req.param.value) == 0);
}
