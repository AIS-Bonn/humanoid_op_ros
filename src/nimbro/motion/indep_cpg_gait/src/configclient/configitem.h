//client object for a single Parameter
//Author: Sebastian Schüller

#ifndef CONFIGITEM_H
#define CONFIGITEM_H

#include <QString>

#include <Config.h>

#include <config_server/parameter.h>

namespace config_client
{

class GaitConfigItem
{
public:
	GaitConfigItem(const config_server::ParameterDescription& description, QString configName);

	void setCallback(const boost::function<void()>& cb);
private:
	boost::function<void()> m_cb;
	config_server::Parameter<float> m_parameter;
	double* m_value;

	void called(float value);
};

}

#endif