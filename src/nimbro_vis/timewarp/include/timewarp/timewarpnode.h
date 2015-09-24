// Provides a time-warped view on selected topics
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

// Ensure header is only included once
#ifndef TIMEWARPNODE_H
#define TIMEWARPNODE_H

// Includes
#include <ros/node_handle.h>
#include <timewarp/tfhandler.h>
#include <timewarp/topicthread.h>
#include <timewarp/TimeWarpControl.h>
#include <timewarp/TimeWarpLoad.h>
#include <timewarp/TimeWarpSave.h>

// Defines
#define WARP_PREFIX "/vis"

namespace timewarp
{

class TopicHandler;

class TimeWarpNode
{
public:
	TimeWarpNode();
	virtual ~TimeWarpNode();

	inline ros::NodeHandle* nodeHandle()
	{ return &m_nh; }

	inline bool live() const
	{ return m_live; }

	inline ros::Time time() const
	{ return m_time; }

private:
	bool handleTimeWarpControl(TimeWarpControlRequest& req, TimeWarpControlResponse& resp);
	bool handleTimeWarpLoad(TimeWarpLoadRequest& req, TimeWarpLoadResponse& resp);
	bool handleTimeWarpSave(TimeWarpSaveRequest& req, TimeWarpSaveResponse& resp);
	
	void update();
	void updateTopics(const ros::master::V_TopicInfo& list);

	int findHandler(const std::string& name);

	ros::NodeHandle m_nh;
	
	ros::WallTimer m_timer;

	bool m_live;
	ros::Time m_time;
	ros::Time m_lastLiveTime;

	TFHandler m_tfHandler;
	
	ros::Publisher m_pub_clock;

	ros::ServiceServer m_srv_control;
	ros::ServiceServer m_srv_load;
	ros::ServiceServer m_srv_save;

	std::vector<TopicHandler*> m_handlers;

	boost::thread m_topicThread;
	
	friend class TimeWarpIO;
};

}

#endif
