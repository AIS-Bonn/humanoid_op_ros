// Handles tf data
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TFHANDLER_H
#define TFHANDLER_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <list>

#define TF_CACHE_TIME 180.0

namespace timewarp
{
class TimeWarpNode;

class TFHandler
{
public:
	static const ros::Duration CacheTime;

	explicit TFHandler(TimeWarpNode* node);
	virtual ~TFHandler();

	void reset();

	void setTime(const ros::Time& now);
	void publish(const ros::Time& now);

private:
	void processMsg(const tf::tfMessage& msg);
	void handleMsg(const tf::tfMessage& msg);
	void trimRawTFLog();

	TimeWarpNode* m_node;
	tf::Transformer m_tf;
	ros::Subscriber m_sub_tf;
	ros::Publisher m_pub_tf;

	tf::tfMessage m_msg;
	
	std::list<tf::tfMessage> m_rawTFLog;
	ros::Time m_rawTFLogLatest;

	friend class TimeWarpIO;
};

}

#endif
