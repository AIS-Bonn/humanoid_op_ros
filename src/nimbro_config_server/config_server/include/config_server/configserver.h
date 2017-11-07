// Configuration server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CONFIGSERVER_H
#define CONFIGSERVER_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>

#include <config_server/ParameterDescription.h>
#include <config_server/SetParameter.h>
#include <config_server/GetParameter.h>
#include <config_server/Subscribe.h>
#include <config_server/SubscribeList.h>
#include <config_server/Save.h>
#include <config_server/Load.h>
#include <config_server/ShowDeadVars.h>
#include <std_msgs/Time.h>

namespace YAML { class Node; }

namespace config_server
{

struct Parameter
{
	ParameterDescription desc;
	std::string value;

	// Do we already have a value for this parameter?
	// This may be false if we just have subscribers who
	// are interested in updates.
	bool hasValue;

	pthread_mutex_t mutex;
	std::vector<std::string> subscribers;

	void notify(const std::string& exclude = "");
};

class ConfigServer
{
public:
	ConfigServer();
	virtual ~ConfigServer();

	void finalise();

private:
	static const std::string fileExtension;
	static const std::string defaultBackupDir;

	typedef std::map<std::string, Parameter> ParameterMap;
	ParameterMap m_params;

	ros::NodeHandle m_nh;
	ros::ServiceServer m_srv_setParameter;
	ros::ServiceServer m_srv_getParameter;
	ros::ServiceServer m_srv_subscribe;
	ros::ServiceServer m_srv_subscribeList;
	ros::ServiceServer m_srv_showDeadVars;
	ros::Publisher m_pub_paramList;
	ros::Publisher m_pub_currentValues;
	ros::Publisher m_pub_uid;
	ros::Publisher m_pub_loadTime;

	ros::ServiceServer m_srv_save;
	ros::ServiceServer m_srv_load;

	bool load(const std::string& filename = std::string());

	bool handleLoad(LoadRequest& req, LoadResponse& resp);
	bool handleSave(SaveRequest& req, SaveResponse& resp, bool show);

	bool handleSetParameter(SetParameterRequest& req, SetParameterResponse& resp);
	bool handleGetParameter(GetParameterRequest& req, GetParameterResponse& resp);

	bool doSubscribe(const std::string& callback, const ParameterDescription& desc, std::string* value, bool *changed = 0);
	bool handleSubscribe(SubscribeRequest& req, SubscribeResponse& resp);
	bool handleSubscribeList(SubscribeListRequest& req, SubscribeListResponse& resp);
	
	bool handleShowDeadVars(ShowDeadVarsRequest& req, ShowDeadVarsResponse& resp);

	void planUpdate();
	void planValueUpdate();
	void updateParameterList();
	void updateParameterValueList();
	void updateUid();

	void insertFromYAML(const YAML::Node& n, const std::string& path);

	std::string defaultConfigName();

	std::string m_configPath;
	std::string m_robotName;

	ros::WallTimer m_publishParamsTimer;
	int m_publishParamsCounter;

	ros::WallTimer m_publishValuesTimer;
	int m_publishValuesCounter;

	std_msgs::Time m_uid;

	void initBackups();
	bool ensureBackupDir();
	void handleBackup();
	std::string m_backupDir;
	bool m_saveBackups;
	ros::WallTimer m_backupTimer;
	bool m_changed;
};

}

#endif
