// Handles one ROS topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TOPICHANDLER_H
#define TOPICHANDLER_H

#include <ros/node_handle.h>
#include <string>

#include <boost/circular_buffer.hpp>

#include <topic_tools/shape_shifter.h>
#include <timewarp/timeextractor.h>

namespace timewarp
{

class TopicHandler
{
public:
	TopicHandler(ros::NodeHandle* nh, const std::string& name);
	virtual ~TopicHandler();

	void initPublisher();

	void setLive(bool live);
	void setTime(const ros::Time& time);
	void publish(const ros::Time& now);
	void reset();
	
	struct Entry
	{
		ros::Time captureTime;
		boost::shared_ptr<topic_tools::ShapeShifter> data;
	};
	typedef boost::circular_buffer<Entry> CircBuf;

	inline std::string name() const
	{ return m_sub.getTopic(); }
	
protected:
	CircBuf m_buf;

	bool m_live;

	virtual CircBuf::iterator findPacketForTime(const ros::Time& time);
	virtual bool isSameTime(const CircBuf::iterator& a, const CircBuf::iterator& b);

private:
	void handleData(const boost::shared_ptr<topic_tools::ShapeShifter>& data);

	ros::NodeHandle* m_nh;

	bool m_pubInit;
	ros::Subscriber m_sub;
	ros::Publisher m_pub;

	CircBuf::iterator m_msgIt;

	friend class EntryComp;
	friend class TimeWarpIO;
};

template<class MsgType>
class SmartTopicHandler : public TopicHandler
{
public:
	SmartTopicHandler(ros::NodeHandle* nh, const std::string& name)
	 : TopicHandler(nh, name)
	{}

	virtual ~SmartTopicHandler()
	{}
protected:
	virtual CircBuf::iterator findPacketForTime(const ros::Time& time);
	virtual bool isSameTime(const CircBuf::iterator& a, const CircBuf::iterator& b);
};

template<class MsgType>
boost::circular_buffer< TopicHandler::Entry >::iterator SmartTopicHandler<MsgType>::findPacketForTime(const ros::Time& time)
{
	CircBuf::iterator it, first, last;

	first = m_buf.begin();
	last = m_buf.end();

	std::iterator_traits<CircBuf::iterator>::difference_type count, step;
	count = std::distance(first, last);

	while(count > 0)
	{
		it = first;
		step = count/2;
		std::advance(it, step);

		ros::Time msgTime = extractTime<MsgType>((*it).data);

		if(msgTime < time)
		{
			first = ++it;
			count -= step+1;
		}
		else
			count = step;
	}

	return first;
}

template<class MsgType>
bool SmartTopicHandler<MsgType>::isSameTime(const CircBuf::iterator& a, const CircBuf::iterator& b)
{
	return extractTime<MsgType>((*a).data) == extractTime<MsgType>((*b).data);
}


}

#endif
