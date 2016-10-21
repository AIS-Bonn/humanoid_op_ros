// Client object
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <config_server/parameter.h>
#include <config_server/parameterclient.h>
#include <config_server/Subscribe.h>
#include <ros/service.h>
#include <ros/this_node.h>
#include <cfloat>

namespace config_server
{

const float ParameterBase::MinMaxEpsilon = FLT_EPSILON * 32;

ParameterBase::~ParameterBase()
{
	if(!m_name.empty())
		ParameterClient::instance()->unregisterParameter(this);
	m_haveDesc = false;
}

void ParameterBase::init(const ParameterDescription& desc, ros::NodeHandle* nh, bool create)
{
	if(desc.name.empty()) return;
	m_name = desc.name;
	if(m_name[0] != '/')
		m_name = ros::this_node::getName() + "/" + desc.name;

	ParameterDescription createDesc;
	if(create)
		createDesc = desc;
	else
	{
		createDesc.min = -INFINITY;
		createDesc.max = INFINITY;
	}

	createDesc.name = m_name;

	m_desc = createDesc;
	m_haveDesc = true;

	ParameterClient::instance()->registerParameter(this, createDesc);
}

void ParameterBase::reinit()
{
	if(!m_haveDesc) return;
	config_server::ParameterClient* client = ParameterClient::instance();
	client->unregisterParameter(this);
	client->registerParameter(this, m_desc);
}

bool ParameterBase::handleSet(const std::string& value)
{
	if(!deserialize(value))
	{
		ROS_WARN("Attempted to set config parameter '%s' to bad value '%s', using '%s' instead!", m_name.c_str(), value.c_str(), serialize().c_str());
		return false;
	}
	notifyClient();
	return true;
}

void ParameterBase::notifyServer()
{
	ParameterClient::instance()->notify(this, serialize());
}

void ParameterBase::notifyClient()
{
}

}
// EOF