// Thread for asynchronously getting topic information
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TIMEWARP_TOPICTHREAD_H
#define TIMEWARP_TOPICTHREAD_H

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <ros/node_handle.h>
#include <ros/master.h>

namespace timewarp
{

class TopicThread
{
public:
	typedef boost::function<void(ros::master::V_TopicInfo)> CallbackType;

	explicit TopicThread(const ros::NodeHandle& nh, const CallbackType& cb);
	virtual ~TopicThread();

	virtual void operator()();

	static boost::thread launch(const ros::NodeHandle& nh, const CallbackType& cb);
private:
	ros::NodeHandle m_nh;
	CallbackType m_cb;
};

}

#endif
