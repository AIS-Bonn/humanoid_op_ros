// Handles tf data
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "timewarp/tfhandler.h"
#include "timewarp/timewarpnode.h"

#include <tf/tfMessage.h>

#include <boost/algorithm/string/predicate.hpp>

#include <ros/names.h>

namespace timewarp
{

const ros::Duration TFHandler::CacheTime = ros::Duration(TF_CACHE_TIME);

TFHandler::TFHandler(timewarp::TimeWarpNode* node)
 : m_node(node)
 , m_tf(true, CacheTime)
{
	reset();
	
	m_sub_tf = node->nodeHandle()->subscribe("/tf", 100, &TFHandler::handleMsg, this);
	m_pub_tf = node->nodeHandle()->advertise<tf::tfMessage>(WARP_PREFIX "/tf", 100);
}

TFHandler::~TFHandler()
{
}

void TFHandler::reset()
{
	m_tf.clear();
	m_rawTFLog.clear();
	m_rawTFLogLatest.fromNSec(0);
	m_msg.transforms.clear();
}

void TFHandler::processMsg(const tf::tfMessage& msg)
{
	for(size_t i = 0; i < msg.transforms.size(); ++i)
	{
		tf::StampedTransform trans;
		const geometry_msgs::TransformStamped& tform = msg.transforms[i];
		tf::transformStampedMsgToTF(tform, trans);
		
		if(tform.header.stamp > m_rawTFLogLatest)
			m_rawTFLogLatest = tform.header.stamp;

		try
		{
			m_tf.setTransform(trans);
		}
		catch(tf::TransformException&)
		{
			continue;
		}
	}
}

void TFHandler::handleMsg(const tf::tfMessage& msg)
{
	if(!m_node->live())
		return;

	processMsg(msg);

	m_rawTFLog.push_back(msg);
	trimRawTFLog();

	m_pub_tf.publish(msg);
}

void TFHandler::trimRawTFLog()
{
	ros::Time cutoff = m_rawTFLogLatest - CacheTime;
	for(std::list<tf::tfMessage>::iterator it = m_rawTFLog.begin(); it != m_rawTFLog.end(); it++)
	{
		bool keep = false;
		for(tf::tfMessage::_transforms_type::iterator itt = it->transforms.begin(); itt != it->transforms.end(); itt++)
		{
			if(itt->header.stamp > cutoff)
			{
				keep = true;
				break;
			}
		}
		
		if(keep)
		{
			m_rawTFLog.erase(m_rawTFLog.begin(), it); // Erase everything before the first one we want to keep
			break;
		}
	}
}

void TFHandler::setTime(const ros::Time& now)
{
	std::vector<std::string> frames;
	m_tf.getFrameStrings(frames);

	m_msg.transforms.clear();

	for(size_t i = 0; i < frames.size(); ++i)
	{
		const std::string& frame = frames[i];
		std::string parentFrame;

		if(!m_tf.getParent(frame, now, parentFrame))
			continue;

		tf::StampedTransform transform;
		try
		{
			m_tf.lookupTransform(parentFrame, frame, now, transform);
		}
		catch(tf::TransformException&)
		{
			continue;
		}

		geometry_msgs::TransformStamped m;
		tf::transformStampedTFToMsg(transform, m);

		m_msg.transforms.push_back(m);
	}
}

void TFHandler::publish(const ros::Time& now)
{
	m_pub_tf.publish(m_msg);
}


}
