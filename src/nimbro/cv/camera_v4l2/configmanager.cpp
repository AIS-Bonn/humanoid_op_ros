// Configuration (e.g. camera parameters) manager
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "configmanager.h"

#include <errno.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <ros/console.h>
#include <yaml-cpp/yaml.h>

ConfigManager::ConfigManager()
 : m_writePending(false)
{
}

ConfigManager::~ConfigManager()
{
}

static int enumerateMenu(int fd, camera_v4l2::CameraParam* param, const v4l2_queryctrl& query)
{
	v4l2_querymenu qm;

	memset(&qm, 0, sizeof(qm));
	qm.id = query.id;

	for(qm.index = query.minimum; qm.index <= (uint)query.maximum; qm.index++)
	{
		if(ioctl(fd, VIDIOC_QUERYMENU, &qm) != 0)
		{
			if(errno == EINVAL)
				continue;

			ROS_FATAL("Could not query menu for id 0x%X: %s", query.id, strerror(errno));
			return -1;
		}

		ROS_INFO(" - % 2d: %s", qm.index, qm.name);

		camera_v4l2::CameraParamChoice choice;
		choice.label = (const char*)qm.name;
		choice.value = qm.index;
		param->choices.push_back(choice);
	}

	return 0;
}

int ConfigManager::init(int fd, const std::vector<std::string>& controls)
{
	v4l2_queryctrl q;
	memset (&q, 0, sizeof (q));

	m_controls.params.clear();
	m_controls.params.resize(controls.size());

	q.id = V4L2_CTRL_FLAG_NEXT_CTRL;
	while(1)
	{
		if(ioctl(fd, VIDIOC_QUERYCTRL, &q) != 0)
		{
			if(errno == EINVAL)
				break;

			ROS_FATAL("Could not query control: %d: %s", errno, strerror(errno));
			return -1;
		}

		if (q.flags & V4L2_CTRL_FLAG_DISABLED)
		{
			ROS_INFO("skipping disabled control '%s'", (const char*)q.name);
			continue;
		}

		v4l2_control control;
		memset(&control, 0, sizeof(control));
		control.id = q.id;

		if(ioctl(fd, VIDIOC_G_CTRL, &control) != 0)
		{
			ROS_FATAL("Could not get control value");
			return -1;
		}

//		ROS_INFO("(id=0x%X) Control '%s' (type %s): %d", q.id, q.name, v4l2_type_to_str(q.type), control.value);

		camera_v4l2::CameraParam param;
		param.id = q.id;
		param.label = (const char*)q.name;
		param.maximum = q.maximum;
		param.minimum = q.minimum;
		param.value = control.value;

		if(q.type == V4L2_CTRL_TYPE_MENU)
		{
			if(enumerateMenu(fd, &param, q) != 0)
			{
				ROS_FATAL("Could not query menu");
				return -1;
			}
		}

		std::vector<std::string>::const_iterator it = std::find(
			controls.begin(), controls.end(), param.label
		);

		if(it != controls.end())
		{
			int idx = it - controls.begin();
			m_controls.params[idx] = param;
		}
		else
			ROS_INFO("Ignoring unknown parameter '%s'", q.name);

		q.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
	}

	bool error = false;
	for(size_t i = 0; i < m_controls.params.size(); ++i)
	{
		if(m_controls.params[i].label == "")
		{
			ROS_ERROR("Could not find whitelisted camera parameter '%s'", controls[i].c_str());
			error = true;
		}
	}

	if(error)
		return -1;

	return 0;
}

camera_v4l2::CameraParam* ConfigManager::getParamInfo(const std::string& name)
{
	for(uint i = 0; i < m_controls.params.size(); ++i)
	{
		if(m_controls.params[i].label == name)
			return &m_controls.params[i];
	}

	// not found
	return 0;
}

camera_v4l2::CameraParam* ConfigManager::getParamInfo(int id)
{
	for(uint i = 0; i < m_controls.params.size(); ++i)
	{
		if(m_controls.params[i].id == id)
			return &m_controls.params[i];
	}

	return 0;
}


int ConfigManager::getParamValue(const std::string& name, int* value)
{
	camera_v4l2::CameraParam* info = getParamInfo(name);

	if(!info)
	{
		ROS_FATAL("Could not find param '%s'", name.c_str());
		return -1;
	}

	*value = info->value;
	return 0;
}

int ConfigManager::getParamValue(int id, int* value)
{
	camera_v4l2::CameraParam* info = getParamInfo(id);

	if(!info)
	{
		ROS_FATAL("Could not find param with ID %d", id);
		return -1;
	}

	*value = info->value;
	return 0;
}

int ConfigManager::readFromCamera(int fd)
{
	int ret = 0;

	for(uint i = 0; i < m_controls.params.size(); ++i)
	{
		camera_v4l2::CameraParam* param = &m_controls.params[i];

		v4l2_control ctrl;
		memset(&ctrl, 0, sizeof(ctrl));
		ctrl.id = param->id;

		if(ioctl(fd, VIDIOC_G_CTRL, &ctrl) != 0)
		{
			ROS_FATAL("Could not query control '%s' (id 0x%X): %s",
				param->label.c_str(), param->id, strerror(errno)
			);

			// Try to read remaining controls
			ret = -1;
			continue;
		}

		param->value = ctrl.value;
	}

	return ret;
}

int ConfigManager::setParamValue(int fd, const std::string& name, int value)
{
	camera_v4l2::CameraParam* info = getParamInfo(name);

	if(!info)
	{
		ROS_FATAL("Could not find param '%s'", name.c_str());
		return -1;
	}

	v4l2_control ctrl;
	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = info->id;
	ctrl.value = value;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl) != 0)
	{
		ROS_FATAL("Could not set control '%s' (id 0x%X): %s",
			info->label.c_str(), info->id, strerror(errno)
		);
		return -1;
	}

	info->value = value;
	m_writePending = true;

	return 0;
}

int ConfigManager::setParamValue(int fd, int id, int value)
{
	camera_v4l2::CameraParam* info = getParamInfo(id);

	if(!info)
	{
		ROS_FATAL("Could not find param with ID %d", id);
		return -1;
	}

	v4l2_control ctrl;
	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = info->id;
	ctrl.value = value;

	if(ioctl(fd, VIDIOC_S_CTRL, &ctrl) != 0)
	{
		ROS_FATAL("Could not set control '%s' (id 0x%X): %s",
			info->label.c_str(), info->id, strerror(errno)
		);
		return -1;
	}

	info->value = value;
	m_writePending = true;

	return 0;
}


int ConfigManager::readConfigFile(int fd, std::istream* stream)
{
	YAML::Node doc;
	int ret = 0;
	std::vector<v4l2_ext_control> controls;

	try
	{
		doc = YAML::Load((*stream));
	}

	catch (YAML::Exception& e)
	{
		ROS_FATAL("Could not parse config file: %s", e.what());
		return -1;
	}

	for(YAML::const_iterator it = doc.begin(); it != doc.end(); ++it)
	{
		std::string key, value;

		try
		{
			key = it->begin()->first.as<std::string>();
			value = it->begin()->first.as<std::string>();
		}
		catch(YAML::Exception& e)
		{
			ROS_FATAL("Invalid parameter specification: %s", e.what());
			return -1;
		}

		int ivalue = -1;
		camera_v4l2::CameraParam* info = getParamInfo(key);
		if(!info)
		{
			ROS_FATAL("Unknown parameter '%s' in config file, ignoring...", key.c_str());
			continue;
		}

		ivalue = atoi(value.c_str());

		info->value = ivalue;
		v4l2_ext_control control;
		memset(&control, 0, sizeof(control));
		control.id = info->id;
		control.value = ivalue;

		controls.push_back(control);
	}

	v4l2_ext_controls request;
	memset(&request, 0, sizeof(request));
	request.controls = &controls[0];
	request.count = controls.size();
	request.ctrl_class = V4L2_CTRL_CLASS_USER;

	if(ioctl(fd, VIDIOC_S_EXT_CTRLS, &request) != 0)
	{
		ROS_FATAL("Control setting failed. This may have happened at control 0x%X with value %d",
			controls[request.error_idx].id,
			controls[request.error_idx].value
		);
		return -1;
	}
	ROS_INFO("Controls set successfully from config file");

	return ret;
}

void ConfigManager::writeConfigFile(int fd, std::ostream* ostream)
{
	YAML::Emitter em;

	em << YAML::BeginSeq;
	for(uint i = 0; i < m_controls.params.size(); ++i)
	{
		const camera_v4l2::CameraParam& param = m_controls.params[i];

		em << YAML::BeginMap
		   << YAML::Key << param.label << YAML::Value << param.value
		   << YAML::EndMap;
	}
	em << YAML::EndSeq;

	(*ostream) << em.c_str() << '\n';

	m_writePending = false;
}
