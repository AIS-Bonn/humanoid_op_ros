//client object for a single Parameter
//Author: Sebastian Schüller

#include "configitem.h"

namespace config_client
{

GaitConfigItem::GaitConfigItem(const config_server::ParameterDescription& description, QString configName)
 : m_parameter(description, 0, true)
 , m_value(&indep_cpg_gait::config(configName))
{
	m_parameter.setCallback(boost::bind(&GaitConfigItem::called, this, _1));
	assert(m_value);

	*m_value = m_parameter();
}

void GaitConfigItem::setCallback(const boost::function<void()>& cb)
{
	m_cb = cb;
}

void GaitConfigItem::called(float value)
{
	*m_value = (double)value;
	if(m_cb)
		m_cb();
}

}
