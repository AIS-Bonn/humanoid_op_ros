// Config server client singleton
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CONFIGSERVER_PARAMETERCLIENT_H
#define CONFIGSERVER_PARAMETERCLIENT_H

#include <config_server/SetParameter.h>
#include <config_server/ParameterDescription.h>
#include <config_server/SubscribeList.h>
#include <std_msgs/Time.h>

#include <map>
#include <string>

#include <ros/timer.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <boost/thread/recursive_mutex.hpp>

namespace config_server
{

class ParameterBase;

class ParameterClient
{
public:
	static void initialize(ros::NodeHandle& nhs);
	static ParameterClient* instance();

	void registerParameter(ParameterBase* param, const ParameterDescription& desc);
	void unregisterParameter(ParameterBase* param);
	void registerAllParametersAgain();

	void notify(ParameterBase* param, const std::string& value);

	void cork();
	void uncork();

	void sync();

	ros::Time serverUid() const { return m_serverUid; }

private:
	ParameterClient();
	ParameterClient(ros::NodeHandle& nhs);
	virtual ~ParameterClient();

	void init(ros::NodeHandle& nhs);

	bool handleSet(SetParameterRequest& req, SetParameterResponse& resp);

	static ParameterClient* g_instance;

	std::string m_serverNamespace;
	std::string m_serverName;

	typedef std::multimap<std::string, ParameterBase*> ParameterMap;
	ParameterMap m_parameters;

	ros::ServiceServer m_srv;
	std::string m_srv_name;

	ros::Time m_serverUid;
	ros::Subscriber m_sub_serverUid;
	void handleUid(const std_msgs::TimeConstPtr& msg);
	ros::Time m_registeredUid;
	bool m_haveUidDiscrepancy;

	int m_corked;
	config_server::SubscribeList m_subscribe;

	boost::recursive_mutex m_mutex;

	bool m_serverEnabled;
	ros::WallDuration m_waitDuration;
};

}

#endif
