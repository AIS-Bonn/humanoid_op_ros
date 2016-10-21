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
#include <config_server/parameter.h>
#include <std_srvs/Empty.h>

// Defines
#define TF_TOPIC      "/tf"
#define JS_TOPIC      "/joint_states"
#define JC_TOPIC      "/joint_commands"
#define WARP_PREFIX   "/vis"
#define WARP_TF_TOPIC (WARP_PREFIX TF_TOPIC)

namespace timewarp
{

class TopicHandler;

class TimeWarpNode
{
public:
	TimeWarpNode();
	virtual ~TimeWarpNode();

	inline ros::NodeHandle* nodeHandle() { return &m_nh; }
	inline bool live() const { return m_live; }
	inline ros::Time time() const { return m_time; }

private:
	bool handleTimeWarpClear(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
	bool handleTimeWarpLoad(TimeWarpLoadRequest& req, TimeWarpLoadResponse& resp);
	bool handleTimeWarpSave(TimeWarpSaveRequest& req, TimeWarpSaveResponse& resp);

	void handleControl(const timewarp::TimeWarpControlConstPtr& msg);
	void updateControlStatus();

	void clear();

	void update();
	void updateTopics(const ros::master::V_TopicInfo& list);

	int findHandler(const std::string& name);

	ros::NodeHandle m_nh;

	config_server::Parameter<float> m_confTfCacheTime; // Note: Every time this config parameter is changed during runtime, the TF cache is cleared
	void handleTfCacheTime();
	int m_tfCacheUpdate;

	config_server::Parameter<bool> m_confEnableImageTopics;
	void handleEnableImageTopics();

	ros::WallTimer m_timer;

	bool m_live;
	ros::Time m_time;
	ros::Time m_lastLiveTime;

	TFHandler m_tfHandler;

	ros::Publisher m_pub_clock;

	ros::ServiceServer m_srv_clear;
	ros::ServiceServer m_srv_load;
	ros::ServiceServer m_srv_save;

	ros::Subscriber m_sub_control;
	ros::Publisher m_pub_controlStatus;
	TimeWarpControl m_lastControl;

	std::vector<std::string> m_extraTopics;
	std::vector<TopicHandler*> m_handlers;

	boost::thread m_topicThread;

	friend class TimeWarpIO;
};

}

#endif
