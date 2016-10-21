// Config server client singleton
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <config_server/parameterclient.h>
#include <config_server/parameter.h>

#include <ros/service.h>
#include <inttypes.h>

namespace config_server
{

ParameterClient* ParameterClient::g_instance = 0;
static bool g_waited = false;

ParameterClient* ParameterClient::instance()
{
	// FIXME: This is not thread-safe!
	if(!g_instance)
		g_instance = new ParameterClient();

	return g_instance;
}

void ParameterClient::initialize(ros::NodeHandle& nh)
{
	if(g_instance)
		return;

	g_instance = new ParameterClient(nh);
}

ParameterClient::ParameterClient(ros::NodeHandle& nh)
{
	init(nh);
}

ParameterClient::ParameterClient()
{
	ros::NodeHandle nh("~");
	init(nh);
}

void ParameterClient::init(ros::NodeHandle& nh)
{
	m_serverUid.fromNSec(0);
	m_registeredUid.fromNSec(0);
	m_haveUidDiscrepancy = false;
	m_corked = 0;
	m_serverEnabled = true;

	m_srv = nh.advertiseService("configure", &ParameterClient::handleSet, this);
	m_srv_name = m_srv.getService();

	m_sub_serverUid = nh.subscribe("/config_server/uid", 1, &ParameterClient::handleUid, this);

	m_subscribe.request.callback = m_srv_name;

	double secs;
	nh.param("/config_server/wait_duration", secs, 5.0);
	nh.getParam("config_server/wait_duration", secs);
	m_waitDuration = ros::WallDuration(secs);

	nh.param("/config_server/enabled", m_serverEnabled, true);
}

ParameterClient::~ParameterClient()
{
}

void ParameterClient::registerParameter(ParameterBase* param, const ParameterDescription& desc)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	m_parameters.insert(std::pair<std::string, ParameterBase*>(param->name(), param));

	if(!m_serverEnabled)
	{
		// Quietly set to default...
		param->handleSet(desc.default_value);
		m_haveUidDiscrepancy = true;
		m_registeredUid.fromNSec(0);
		return;
	}

	if(m_corked)
	{
		for(std::size_t i = 0; i < m_subscribe.request.parameters.size(); ++i)
		{
			if(m_subscribe.request.parameters[i].name == desc.name)
				return;
		}

		m_subscribe.request.parameters.push_back(desc);
	}
	else
	{
		config_server::Subscribe srv;
		srv.request.callback = m_srv_name;
		srv.request.prop = param->name();
		srv.request.desc = desc;
		srv.request.desc.name = param->name();

		if(!g_waited)
		{
			// Note: ros::service::waitForService() does a ros time sleep, which hangs if time is simulated & stopped...
			bool displayed = false;
			ros::WallRate rate(1.0);
			ros::WallTime start = ros::WallTime::now();
			while(!ros::service::exists("/config_server/subscribe", false) && (m_waitDuration.toSec() < 0.0 || ros::WallTime::now() - start < m_waitDuration))
			{
				if(!displayed)
				{
					if(m_waitDuration.toSec() < 0.0)
						ROS_INFO("Waiting forever for config_server...");
					else
						ROS_INFO("Waiting %.1fs for config_server...", m_waitDuration.toSec());
					displayed = true;
				}
				rate.sleep();
				if (!ros::ok())
				{
					exit(1);
				}
			}

			g_waited = true;
		}

		if(!ros::service::call("/config_server/subscribe", srv))
		{
			ROS_ERROR("Could not call /config_server/subscribe for parameter '%s'", param->name().c_str());
			param->handleSet(srv.request.desc.default_value);
			m_haveUidDiscrepancy = true;
			m_registeredUid.fromNSec(0);
		}
		else
		{
			param->handleSet(srv.response.value);
			m_haveUidDiscrepancy |= (srv.response.uid != m_registeredUid && !m_registeredUid.isZero());
			m_registeredUid = srv.response.uid;
		}
	}
}

void ParameterClient::cork()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_corked++;
}

void ParameterClient::uncork()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if(m_corked <= 0)
	{
		ROS_ERROR("ParameterClient::uncork() called while not corked!");
		m_corked = 0;
		return;
	}

	if(--m_corked != 0)
		return;

	sync();
}

void ParameterClient::handleUid(const std_msgs::TimeConstPtr& msg)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	bool discrepancy = (msg->data != m_registeredUid || m_haveUidDiscrepancy);
	if(m_serverEnabled && (msg->data != m_serverUid || discrepancy))
	{
		bool oldServerUidZero = m_serverUid.isZero();
		m_serverUid = msg->data;
		m_registeredUid = msg->data;
		ROS_INFO("Using config server time UID: %" PRIu32 ".%09" PRIu32, msg->data.sec, msg->data.nsec);
		if(!oldServerUidZero || discrepancy)
		{
			m_haveUidDiscrepancy = false;
			registerAllParametersAgain();
		}
	}
}

void ParameterClient::registerAllParametersAgain()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	g_waited = false;

	cork();

	ParameterMap copy(m_parameters);
	for(ParameterMap::iterator it = copy.begin(); it != copy.end(); ++it)
		it->second->reinit();

	uncork();
}

void ParameterClient::sync()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if(!ros::service::call("/config_server/subscribe_list", m_subscribe))
	{
		ROS_ERROR("Could not call /config_server/subscribe_list");
		m_haveUidDiscrepancy = true;
		m_registeredUid.fromNSec(0);
		return;
	}

	if(m_subscribe.response.values.size() != m_subscribe.request.parameters.size())
	{
		ROS_ERROR("Invalid response size from service call /config_server/subscribe_list => Initial parameter values will be wrong");
		m_haveUidDiscrepancy = true;
		m_registeredUid.fromNSec(0);
		return;
	}

	m_haveUidDiscrepancy |= (m_subscribe.response.uid != m_registeredUid && !m_registeredUid.isZero());
	m_registeredUid = m_subscribe.response.uid;

	for(std::size_t i = 0; i < m_subscribe.request.parameters.size(); ++i)
	{
		std::pair<ParameterMap::iterator, ParameterMap::iterator> eq;
		eq = m_parameters.equal_range(m_subscribe.request.parameters[i].name);

		for(ParameterMap::iterator it = eq.first; it != eq.second; ++it)
			it->second->handleSet(m_subscribe.response.values[i]);
	}

	m_subscribe.request.parameters.clear();
}

void ParameterClient::unregisterParameter(ParameterBase* param)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	for(ParameterMap::iterator it = m_parameters.begin(); it != m_parameters.end(); ++it)
	{
		if(it->second == param)
		{
			m_parameters.erase(it);
			return;
		}
	}

	ROS_ERROR("config_server::ParameterClient: Tried to unregister unknown parameter '%s'", param->name().c_str());
}

bool ParameterClient::handleSet(SetParameterRequest& req, SetParameterResponse& resp)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	std::pair<ParameterMap::iterator, ParameterMap::iterator> eq;
	eq = m_parameters.equal_range(req.name);

	resp.badValue = false;
	for(ParameterMap::iterator it = eq.first; it != eq.second; ++it)
	{
		if(!it->second->handleSet(req.value))
			resp.badValue = true;
	}

	return true;
}

void ParameterClient::notify(ParameterBase* param, const std::string& value)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	// Handle local notification
	std::pair<ParameterMap::iterator, ParameterMap::iterator> eq;
	eq = m_parameters.equal_range(param->name());

	for(ParameterMap::iterator it = eq.first; it != eq.second; ++it)
	{
		if(it->second == param)
			continue;

		it->second->handleSet(value);
	}

	// Notify the server
	config_server::SetParameter srv;
	srv.request.name = param->name();
	srv.request.value = value;
	srv.request.no_notify = m_srv_name;

	if(!ros::service::call("/config_server/set_parameter", srv))
		ROS_ERROR("Could not set parameter '%s' on the parameter server", param->name().c_str());
}

}
