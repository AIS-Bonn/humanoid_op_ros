// Handles one ROS topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <timewarp/timewarpnode.h>
#include <timewarp/topichandler.h>
#include <sensor_msgs/Image.h>

namespace timewarp
{

const std::size_t TopicHandler::BUF_SIZE = 16384;
const std::size_t TopicHandler::BUF_SIZE_IMG = 256;

bool TopicHandler::enableImageTopics = false;

TopicHandler::TopicHandler(ros::NodeHandle* nh, const std::string& name)
 : m_buf(BUF_SIZE)
 , m_live(true)
 , m_nh(nh)
 , m_name(name)
 , m_active(false)
 , m_isImage(false)
 , m_pubInit(false)
{
	resetBuf();
	setActive(true);
}

TopicHandler::~TopicHandler()
{
	setActive(false);
}

void TopicHandler::setActive(bool active)
{
	if(active == m_active) return;
	m_active = active;

	resetBuf();

	if(active)
		m_sub = m_nh->subscribe(m_name, 50, &TopicHandler::handleData, this);
	else
	{
		m_sub.shutdown();
		deinitPublisher();
	}
}

void TopicHandler::initPublisher()
{
	if(!m_pubInit && !m_buf.empty())
	{
		m_pub = m_buf.front().data->advertise(*m_nh, WARP_PREFIX "/" + m_sub.getTopic(), (m_isImage ? 5 : 50));
		m_pubInit = true;
	}
}

void TopicHandler::deinitPublisher()
{
	m_pub.shutdown();
	m_pubInit = false;
}

void TopicHandler::setLive(bool live)
{
	m_live = live;
}

void TopicHandler::handleData(const boost::shared_ptr<topic_tools::ShapeShifter>& data)
{
	if(!m_live)
		return;

	if(!m_isImage && data->getDataType().compare(ros::message_traits::DataType<sensor_msgs::Image>::value()) == 0)
	{
		m_buf.rset_capacity(BUF_SIZE_IMG);
		m_isImage = true;
		updateImageEnable();
		if(!m_active) return;
	}

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

TopicHandler::CircBuf::iterator TopicHandler::findPacketForTime(const ros::Time& time)
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
		if(m_isImage)
		{
			for(; it != m_buf.end() && isSameTime(it, m_msgIt); ++it)
				m_msgIt = it;
			m_pub.publish((*m_msgIt).data);
		}
		else
		{
			for(; it != m_buf.end() && isSameTime(it, m_msgIt); ++it)
				m_pub.publish((*it).data);
		}
	}
}

void TopicHandler::resetBuf()
{
	m_buf.clear();
	m_msgIt = m_buf.end();
}

}
