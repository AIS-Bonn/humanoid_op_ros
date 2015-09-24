// Thread for asynchronously getting topic information
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <timewarp/topicthread.h>
#include <timewarp/genericcallback.h>

#include <ros/master.h>
#include <ros/rate.h>
#include <ros/callback_queue_interface.h>

namespace timewarp
{

TopicThread::TopicThread(const ros::NodeHandle& nh, const CallbackType& cb)
 : m_nh(nh)
 , m_cb(cb)
{
}

TopicThread::~TopicThread()
{
}

void TopicThread::operator()()
{
	ros::WallRate rate(0.2);

	while(1)
	{
		ros::master::V_TopicInfo topics;
		if(!ros::master::getTopics(topics))
		{
			ROS_WARN("Could not get list of topics from master");
			return;
		}

		m_nh.getCallbackQueue()->addCallback(
			ros::CallbackInterfacePtr(
				new GenericCallback(boost::bind(m_cb, topics))
			)
		);

		rate.sleep();
	}
}

boost::thread TopicThread::launch(const ros::NodeHandle& nh, const TopicThread::CallbackType& cb)
{
	return boost::thread(TopicThread(nh, cb));
}

}
