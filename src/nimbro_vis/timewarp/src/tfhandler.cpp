// Handles tf data
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "timewarp/tfhandler.h"
#include "timewarp/timewarpnode.h"
#include <boost/algorithm/string/predicate.hpp>
#include <ros/names.h>

namespace timewarp
{

TFHandler::TFHandler(timewarp::TimeWarpNode* node)
 : m_node(node)
 , m_tf(NULL)
{
	newTfTransformer(TF_CACHE_TIME);

	m_sub_tf = node->nodeHandle()->subscribe(TF_TOPIC, 100, &TFHandler::handleMsg, this);
	m_pub_tf = node->nodeHandle()->advertise<tf2_msgs::TFMessage>(WARP_TF_TOPIC, 100);
}

TFHandler::~TFHandler()
{
	m_sub_tf.shutdown();
	m_pub_tf.shutdown();
	deleteTfTransformer();
}

void TFHandler::reset()
{
	m_tf->clear();
	m_rawTFLog.clear();
	m_rawTFLogLatest.fromNSec(0);
	m_msg.transforms.clear();
}

void TFHandler::newTfTransformer(double cacheTime)
{
	deleteTfTransformer();
	if(cacheTime < 1.0)
		cacheTime = 1.0;
	m_cacheTime = ros::Duration(cacheTime);
	m_tf = new tf::Transformer(true, m_cacheTime);
	reset();
}

void TFHandler::deleteTfTransformer()
{
	if(m_tf)
		delete m_tf;
}

void TFHandler::processMsg(const tf2_msgs::TFMessage& msg)
{
	for(std::size_t i = 0; i < msg.transforms.size(); ++i)
	{
		tf::StampedTransform trans;
		const geometry_msgs::TransformStamped& tform = msg.transforms[i];
		tf::transformStampedMsgToTF(tform, trans);

		if(tform.header.stamp > m_rawTFLogLatest)
			m_rawTFLogLatest = tform.header.stamp;

		try { m_tf->setTransform(trans); }
		catch(tf::TransformException&) { continue; }
	}
}

void TFHandler::handleMsg(const tf2_msgs::TFMessage& msg)
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
	ros::Time cutoff;
	if(m_rawTFLogLatest.toNSec() >= (uint64_t) m_cacheTime.toNSec()) // Otherwise std::runtime_error error is thrown because we get a negative time! (Note: m_cacheTime is always positive, so the cast is ok)
		cutoff = m_rawTFLogLatest - m_cacheTime;
	for(std::list<tf2_msgs::TFMessage>::iterator it = m_rawTFLog.begin(); it != m_rawTFLog.end(); it++)
	{
		bool keep = false;
		for(tf2_msgs::TFMessage::_transforms_type::iterator itt = it->transforms.begin(); itt != it->transforms.end(); itt++)
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
	m_tf->getFrameStrings(frames);

	m_msg.transforms.clear();

	for(std::size_t i = 0; i < frames.size(); ++i)
	{
		const std::string& frame = frames[i];
		std::string parentFrame;

		if(!m_tf->getParent(frame, now, parentFrame))
			continue;

		tf::StampedTransform transform;
		try { m_tf->lookupTransform(parentFrame, frame, now, transform); }
		catch(tf::TransformException&) { continue; }

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
