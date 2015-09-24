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
 , m_live(true)
 , m_tfHandler(this)
{
	m_srv_control = m_nh.advertiseService("control", &TimeWarpNode::handleTimeWarpControl, this);
	m_srv_load = m_nh.advertiseService("load", &TimeWarpNode::handleTimeWarpLoad, this);
	m_srv_save = m_nh.advertiseService("save", &TimeWarpNode::handleTimeWarpSave, this);

	m_pub_clock = m_nh.advertise<rosgraph_msgs::Clock>(WARP_PREFIX "/clock", 1);
	ROS_INFO("Publishing warped time on topic '%s'", m_pub_clock.getTopic().c_str());

	m_timer = m_nh.createWallTimer(ros::WallDuration(0.1), boost::bind(&TimeWarpNode::update, this));
	m_timer.start();

	XmlRpc::XmlRpcValue list;
	if(m_nh.getParam("extra_topics", list))
	{
		ROS_INFO("Using extra_topics parameter for timewarp:");
		ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for(int i = 0; i < list.size(); ++i)
		{
			std::string name = static_cast<std::string>(list[i]);
			m_handlers.push_back(new TopicHandler(&m_nh, name));
			ROS_INFO("Created timewarp handler for extra topic '%s'", name.c_str());
		}
	}
	
	ROS_INFO("Created timewarp handler for topic '/tf'");

	m_handlers.push_back(new SmartTopicHandler<sensor_msgs::JointState>(&m_nh, "/joint_states"));
	ROS_INFO("Created timewarp handler for topic '/joint_states'");
	m_handlers.push_back(new SmartTopicHandler<plot_msgs::JointCommand>(&m_nh, "/joint_commands"));
	ROS_INFO("Created timewarp handler for topic '/joint_commands'");

	m_topicThread = TopicThread::launch(m_nh, boost::bind(&TimeWarpNode::updateTopics, this, _1));
}

TimeWarpNode::~TimeWarpNode()
{
	for(size_t i = 0; i < m_handlers.size(); ++i)
		delete m_handlers[i];
}

bool TimeWarpNode::handleTimeWarpControl(TimeWarpControlRequest& req, TimeWarpControlResponse& resp)
{
	m_live = req.live;
	m_time = req.time;

	m_tfHandler.setTime(req.time);

	for(size_t i = 0; i < m_handlers.size(); ++i)
	{
		m_handlers[i]->setLive(req.live);
		m_handlers[i]->setTime(req.time);
	}

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

void TimeWarpNode::update()
{
	rosgraph_msgs::Clock clock;
	ros::Time now = ros::Time::now();

	if(m_live)
	{
		if(now < m_lastLiveTime)
		{
			ROS_WARN("Timewarp detected a backward time change. Cleared all cached timewarp data.");
			m_tfHandler.reset();
			for(size_t i = 0; i < m_handlers.size(); ++i)
				m_handlers[i]->reset();
		}

		clock.clock = now;
		m_lastLiveTime = now;
	}
	else
		clock.clock = m_time;

	m_pub_clock.publish(clock);

	if(!m_live)
	{
		m_tfHandler.publish(now);

		for(size_t i = 0; i < m_handlers.size(); ++i)
		{
			m_handlers[i]->publish(now);
		}
	}
}

int TimeWarpNode::findHandler(const std::string& name)
{
	for(size_t i = 0; i < m_handlers.size(); ++i)
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
	
	const char* vis_typea = ros::message_traits::DataType<visualization_msgs::MarkerArray>::value();
	const char* vis_typeb = ros::message_traits::DataType<visualization_msgs::Marker>::value();
	
	for(ros::master::V_TopicInfo::const_iterator it = topics.begin(); it != topics.end(); ++it)
	{
		const ros::master::TopicInfo& info = *it;

		if(info.datatype == vis_typea || info.datatype == vis_typeb)
		{
			// Check that it's not one of our remapped topics
			if(info.name.compare(0, prefix.length(), prefix) == 0)
				continue;

			// Check whether we already have the topic in our list
			int idx = findHandler(info.name);
			if(idx != -1)
				continue;

			// Add a topic handler for this topic to our list
			if(info.datatype == vis_typea)
				m_handlers.push_back(new SmartTopicHandler<visualization_msgs::MarkerArray>(&m_nh, info.name));
			else if(info.datatype == vis_typeb)
				m_handlers.push_back(new SmartTopicHandler<visualization_msgs::Marker>(&m_nh, info.name));
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
