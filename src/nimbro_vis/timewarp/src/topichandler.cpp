// Handles one ROS topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <timewarp/timewarpnode.h>
#include <timewarp/topichandler.h>

namespace timewarp
{

TopicHandler::TopicHandler(ros::NodeHandle* nh, const std::string& name)
 : m_buf(16384)
 , m_live(true)
 , m_nh(nh)
 , m_pubInit(false)
{
	reset();
	m_sub = nh->subscribe(name, 50, &TopicHandler::handleData, this);
}

TopicHandler::~TopicHandler()
{
}

void TopicHandler::initPublisher()
{
	if(!m_pubInit && !m_buf.empty())
	{
		m_pub = m_buf.front().data->advertise(*m_nh, WARP_PREFIX "/" + m_sub.getTopic(), 50);
		m_pubInit = true;
	}
}

void TopicHandler::setLive(bool live)
{
	m_live = live;
}

void TopicHandler::handleData(const boost::shared_ptr<topic_tools::ShapeShifter>& data)
{
	if(!m_live)
		return;

	Entry entry;
	entry.captureTime = ros::Time::now();
	entry.data = data;
	m_buf.push_back(entry);
	
	if(!m_pubInit) initPublisher();
	m_pub.publish(data);
}

struct EntryComp
{
	bool operator()(const TopicHandler::Entry& a, const TopicHandler::Entry& b)
	{
		return a.captureTime < b.captureTime;
	}
};

boost::circular_buffer< TopicHandler::Entry >::iterator TopicHandler::findPacketForTime(const ros::Time& time)
{
	Entry cmp;
	cmp.captureTime = time;

	return std::lower_bound(m_buf.begin(), m_buf.end(), cmp, EntryComp());
}

bool TopicHandler::isSameTime(const CircBuf::iterator& a, const CircBuf::iterator& b)
{
	return (*a).captureTime == (*b).captureTime;
}

void TopicHandler::setTime(const ros::Time& time)
{
	m_msgIt = findPacketForTime(time);
}

void TopicHandler::publish(const ros::Time& now)
{
	if(!m_pubInit)
		return;

	if(m_msgIt != m_buf.end())
	{
		CircBuf::iterator it = m_msgIt;

		for(; it != m_buf.end() && isSameTime(it, m_msgIt); ++it)
			m_pub.publish((*it).data);
	}
}

void TopicHandler::reset()
{
	m_buf.clear();
	m_msgIt = m_buf.end();
}


}
