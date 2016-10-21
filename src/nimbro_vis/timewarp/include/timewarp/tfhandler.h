// Handles tf data
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TFHANDLER_H
#define TFHANDLER_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <list>

#define TF_CACHE_TIME  180.0

namespace timewarp
{
class TimeWarpNode;

class TFHandler
{
public:
	explicit TFHandler(TimeWarpNode* node);
	virtual ~TFHandler();

	void reset();

	void setCacheTime(double cacheTime) { newTfTransformer(cacheTime); } // Note: This implicitly resets the TF handler too
	double getCacheTime() const { return m_cacheTime.toSec(); }

	void setTime(const ros::Time& now);
	void publish(const ros::Time& now);

private:
	void newTfTransformer(double cacheTime);
	void deleteTfTransformer();
	ros::Duration m_cacheTime;

	void processMsg(const tf2_msgs::TFMessage& msg);
	void handleMsg(const tf2_msgs::TFMessage& msg);
	void trimRawTFLog();

	TimeWarpNode* m_node;
	tf::Transformer* m_tf;
	ros::Subscriber m_sub_tf;
	ros::Publisher m_pub_tf;

	tf2_msgs::TFMessage m_msg;
	
	std::list<tf2_msgs::TFMessage> m_rawTFLog;
	ros::Time m_rawTFLogLatest;

	friend class TimeWarpIO;
};

}

#endif
