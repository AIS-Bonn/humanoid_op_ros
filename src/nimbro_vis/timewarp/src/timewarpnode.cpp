// Provides a time-warped view on selected topics
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <timewarp/timewarpnode.h>
#include <timewarp/topichandler.h>
#include <timewarp/topicthread.h>
#include <timewarp/timewarpio.h>

#include <rosgraph_msgs/Clock.h>
#include <plot_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>

namespace timewarp
{

TimeWarpNode::TimeWarpNode()
 : m_nh("~")
 , m_confTfCacheTime("tfCacheTime", 10.0, 10.0, 600.0, TF_CACHE_TIME)
 , m_tfCacheUpdate(0)
 , m_confEnableImageTopics("enableImageWarping", false)
 , m_live(true)
 , m_time(0, 0)
 , m_lastLiveTime(0, 0)
 , m_tfHandler(this)
{
	m_lastControl.live = m_live;
	m_lastControl.time = m_time;

	m_confTfCacheTime.setCallback(boost::bind(&TimeWarpNode::handleTfCacheTime, this), true);
	m_confEnableImageTopics.setCallback(boost::bind(&TimeWarpNode::handleEnableImageTopics, this), true);

	m_srv_clear = m_nh.advertiseService("clear", &TimeWarpNode::handleTimeWarpClear, this);
	m_srv_load = m_nh.advertiseService("load", &TimeWarpNode::handleTimeWarpLoad, this);
	m_srv_save = m_nh.advertiseService("save", &TimeWarpNode::handleTimeWarpSave, this);

	m_sub_control = m_nh.subscribe("control", 1, &TimeWarpNode::handleControl, this);
	m_pub_controlStatus = m_nh.advertise<timewarp::TimeWarpControl>("control_status", 1);

	m_pub_clock = m_nh.advertise<rosgraph_msgs::Clock>(WARP_PREFIX "/clock", 1);
	ROS_INFO("Publishing warped time on topic '%s'", m_pub_clock.getTopic().c_str());

	m_timer = m_nh.createWallTimer(ros::WallDuration(0.1), boost::bind(&TimeWarpNode::update, this));
	m_timer.start();

	m_extraTopics.clear();
	XmlRpc::XmlRpcValue list;
	if(m_nh.getParam("extra_topics", list))
	{
		const std::string prefix(WARP_PREFIX "/");

		ROS_INFO("Using extra_topics parameter for timewarp:");
		ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for(int i = 0; i < list.size(); ++i)
		{
			std::string name = static_cast<std::string>(list[i]);
			if(name.compare(0, prefix.length(), prefix) == 0)
			{
				ROS_WARN("NOT creating timewarp handler for reserved extra topic '%s'", name.c_str());
				continue;
			}
			TopicHandler* handler = new TopicHandler(&m_nh, name);
			m_handlers.push_back(handler);
			m_extraTopics.push_back(handler->name());
			ROS_INFO("Created timewarp handler for extra topic '%s'", handler->name().c_str());
		}
	}

	ROS_INFO("Created timewarp handler for topic '" TF_TOPIC "'");
	m_handlers.push_back(new SmartTopicHandler<sensor_msgs::JointState>(&m_nh, JS_TOPIC));
	ROS_INFO("Created timewarp handler for topic '" JS_TOPIC "'");
	m_handlers.push_back(new SmartTopicHandler<plot_msgs::JointCommand>(&m_nh, JC_TOPIC));
	ROS_INFO("Created timewarp handler for topic '" JC_TOPIC "'");

	m_topicThread = TopicThread::launch(m_nh, boost::bind(&TimeWarpNode::updateTopics, this, _1));
}

TimeWarpNode::~TimeWarpNode()
{
	m_timer.stop();

	for(std::size_t i = 0; i < m_handlers.size(); ++i)
		delete m_handlers[i];
}

bool TimeWarpNode::handleTimeWarpClear(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	clear();
	return true;
}

bool TimeWarpNode::handleTimeWarpLoad(TimeWarpLoadRequest& req, TimeWarpLoadResponse& resp)
{
	return TimeWarpIO::load(this, req.bagPath);
}

bool TimeWarpNode::handleTimeWarpSave(TimeWarpSaveRequest& req, TimeWarpSaveResponse& resp)
{
	return TimeWarpIO::save(this, req.bagPath, (req.append != 0), req.startTime, req.stopTime);
}

void TimeWarpNode::handleControl(const timewarp::TimeWarpControlConstPtr& msg)
{
	m_live = msg->live;
	m_time = msg->time;

	m_tfHandler.setTime(msg->time);

	for(std::size_t i = 0; i < m_handlers.size(); ++i)
	{
		m_handlers[i]->setLive(msg->live);
		m_handlers[i]->setTime(msg->time);
	}

	m_lastControl = *msg;
}

void TimeWarpNode::handleTfCacheTime()
{
	m_tfCacheUpdate = 10;
}

void TimeWarpNode::handleEnableImageTopics()
{
	TopicHandler::enableImageTopics = m_confEnableImageTopics();
	for(std::size_t i = 0; i < m_handlers.size(); ++i)
		m_handlers[i]->updateImageEnable();
}

void TimeWarpNode::updateControlStatus()
{
	m_pub_controlStatus.publish(m_lastControl);
}

void TimeWarpNode::clear()
{
	m_tfHandler.reset();
	for(std::vector<timewarp::TopicHandler*>::iterator it = m_handlers.begin(); it != m_handlers.end(); ++it)
		(*it)->resetBuf();
	ROS_INFO("Cleared all cached timewarp data");
}

void TimeWarpNode::update()
{
	rosgraph_msgs::Clock clock;
	ros::Time now = ros::Time::now();

	if(m_live)
	{
		if(now < m_lastLiveTime)
		{
			ROS_WARN("Timewarp detected a backward time change!");
			clear();
		}

		clock.clock = now;
		m_lastLiveTime = now;
	}
	else
		clock.clock = m_time;

	m_pub_clock.publish(clock);
	updateControlStatus();

	if(!m_live)
	{
		m_tfHandler.publish(now);
		for(std::size_t i = 0; i < m_handlers.size(); ++i)
			m_handlers[i]->publish(now);
	}

	if(m_tfCacheUpdate == 1)
	{
		double cacheTime = m_confTfCacheTime();
		if(m_tfHandler.getCacheTime() != cacheTime)
		{
			m_tfHandler.setCacheTime(cacheTime);
			ROS_INFO("Timewarp TF cache time reset to %.0fs", cacheTime);
		}
	}
	if(m_tfCacheUpdate >= 1)
		m_tfCacheUpdate--;
	else
		m_tfCacheUpdate = 0;
}

int TimeWarpNode::findHandler(const std::string& name)
{
	for(std::size_t i = 0; i < m_handlers.size(); ++i)
	{
		TopicHandler* handler = m_handlers[i];

		if(handler->name() == name)
			return i;
	}

	return -1;
}

void TimeWarpNode::updateTopics(const ros::master::V_TopicInfo& topics)
{
	const std::string prefix(WARP_PREFIX "/");

	const char* dataTypeMarker = ros::message_traits::DataType<visualization_msgs::Marker>::value();
	const char* dataTypeMarkerArray = ros::message_traits::DataType<visualization_msgs::MarkerArray>::value();

	for(ros::master::V_TopicInfo::const_iterator it = topics.begin(); it != topics.end(); ++it)
	{
		const ros::master::TopicInfo& info = *it;

		bool isMarker = (info.datatype.compare(dataTypeMarker) == 0);
		bool isMarkerArray = (info.datatype.compare(dataTypeMarkerArray) == 0);

		if(isMarker || isMarkerArray)
		{
			// Check that it's not one of our remapped topics
			if(info.name.compare(0, prefix.length(), prefix) == 0)
				continue;

			// Check whether we already have the topic in our list
			int idx = findHandler(info.name);
			if(idx != -1)
				continue;

			// Add a topic handler for this topic to our list
			if(isMarker)
				m_handlers.push_back(new SmartTopicHandler<visualization_msgs::Marker>(&m_nh, info.name));
			else if(isMarkerArray)
				m_handlers.push_back(new SmartTopicHandler<visualization_msgs::MarkerArray>(&m_nh, info.name));
			ROS_INFO("Created timewarp handler for topic '%s'", info.name.c_str());
		}
	}
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "timewarp");

	timewarp::TimeWarpNode n;

	ros::spin();
	return 0;
}
