//Client that reads from Configserver
//Author: Sebastian Schüller

#ifndef CONFIGCLIENT_H
#define CONFIGCLIENT_H

#include <QString>

#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <config_server/parameter.h>
#include <config_server/ParameterList.h>

#include <Config.h>
#include <State.h>

#include "configitem.h"


namespace config_client
{

class ConfigClient
{
public:
	ConfigClient();
	~ConfigClient();

	void init();

	void setCallback(const boost::function<void()>& cb);

private:
	std::vector<boost::shared_ptr<GaitConfigItem> > m_itemList;

	ros::Subscriber m_sub_paramList;

	void createParameter(const config_server::ParameterDescription& description, QString configName);

};

}

#endif