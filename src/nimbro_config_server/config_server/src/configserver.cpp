// Configuration server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <config_server/configserver.h>
#include <config_server/ParameterList.h>
#include <config_server/ParameterLoadTime.h>
#include <config_server/ParameterValueList.h>

#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/package.h>
#include <ros/this_node.h>

#include <yaml-cpp/yaml.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/concept_check.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include <std_msgs/Empty.h>

#include <inttypes.h>
#include <iomanip>
#include <fstream>
#include <limits>
#include <time.h>
#include <deque>

namespace fs = boost::filesystem;

namespace config_server
{

class NotifyThread
{
public:
	void handleRequest()
	{
		pthread_mutex_lock(&m_mutex);
		while(m_jobs.size() == 0)
			pthread_cond_wait(&m_cond, &m_mutex);

		Job* job = &m_jobs[0];
		std::string subscriber = job->subscribers[job->subscribers.size()-1];
		job->subscribers.pop_back();
		std::string value = job->value;

		Parameter* param = job->param;

		if(job->subscribers.size() == 0)
			m_jobs.pop_front();
		pthread_mutex_unlock(&m_mutex);

		config_server::SetParameter srv;
		srv.request.name = param->desc.name;
		srv.request.value = value;
		srv.response.badValue = false;
		
		if(ros::service::call(subscriber, srv))
		{
			if(srv.response.badValue)
				ROS_WARN("Subscriber '%s' of config '%s' rejected bad value '%s'", subscriber.c_str(), srv.request.name.c_str(), srv.request.value.c_str());
		}
		else
		{
			ROS_WARN("Subscriber '%s' of config '%s' is unavailable => Removing from list", subscriber.c_str(), srv.request.name.c_str());
			pthread_mutex_lock(&param->mutex);
			param->subscribers.erase(std::remove(param->subscribers.begin(), param->subscribers.end(), subscriber), param->subscribers.end());
			pthread_mutex_unlock(&param->mutex);
		}
	}

	void enqueue(Parameter* param, const std::string& value, const std::vector<std::string>& subscribers)
	{
		pthread_mutex_lock(&m_mutex);

		for(size_t i = 0; i < m_jobs.size(); ++i)
		{
			if(m_jobs[i].param == param)
			{
				m_jobs[i].subscribers = subscribers;
				m_jobs[i].value = value;
				pthread_cond_signal(&m_cond);
				pthread_mutex_unlock(&m_mutex);
				return;
			}
		}

		Job job;
		job.subscribers = subscribers;
		job.value = value;
		job.param = param;
		m_jobs.push_back(job);

		pthread_cond_signal(&m_cond);
		pthread_mutex_unlock(&m_mutex);
	}

	static NotifyThread* instance()
	{
		if(!m_instance)
			m_instance = new NotifyThread();

		return m_instance;
	}
private:
	struct Job
	{
		std::vector<std::string> subscribers;
		std::string value;
		Parameter* param;
	};

	std::deque<Job> m_jobs;
	pthread_cond_t m_cond;
	pthread_mutex_t m_mutex;

	static NotifyThread* m_instance;
	NotifyThread()
	{
		pthread_mutex_init(&m_mutex, 0);
		pthread_cond_init(&m_cond, 0);
	}
};

NotifyThread* NotifyThread::m_instance;

void Parameter::notify(const std::string& exclude)
{
	if(subscribers.size() == 0)
		return;

	std::vector<std::string> filtered;
	filtered.reserve(subscribers.size());
	for(size_t i = 0; i < subscribers.size(); ++i)
	{
		if(subscribers[i] != exclude)
			filtered.push_back(subscribers[i]);
	}

	if(filtered.size() != 0)
		NotifyThread::instance()->enqueue(this, value, filtered);
}

const std::string ConfigServer::fileExtension = ".yaml";
const std::string ConfigServer::defaultBackupDir = "/tmp/config_server/";

ConfigServer::ConfigServer()
 : m_nh("~")
 , m_publishParamsCounter(0)
{
	m_uid.data.fromNSec(ros::WallTime::now().toNSec());
	ROS_INFO("Config server launched with time UID: %" PRIu32 ".%09" PRIu32, m_uid.data.sec, m_uid.data.nsec);

	m_srv_setParameter = m_nh.advertiseService("set_parameter", &ConfigServer::handleSetParameter, this);
	m_srv_getParameter = m_nh.advertiseService("get_parameter", &ConfigServer::handleGetParameter, this);
	m_srv_subscribe = m_nh.advertiseService("subscribe", &ConfigServer::handleSubscribe, this);
	m_srv_subscribeList = m_nh.advertiseService("subscribe_list", &ConfigServer::handleSubscribeList, this);
	m_srv_showDeadVars = m_nh.advertiseService("show_dead_vars", &ConfigServer::handleShowDeadVars, this);

	m_pub_paramList = m_nh.advertise<ParameterList>("parameter_list", 1, true);
	m_pub_currentValues = m_nh.advertise<ParameterValueList>("parameter_values", 1, true);
	m_pub_uid = m_nh.advertise<std_msgs::Time>("uid", 1, true);
	m_pub_loadTime = m_nh.advertise<ParameterLoadTime>("load_time", 1, true);

	m_srv_save = m_nh.advertiseService<config_server::SaveRequest, config_server::SaveResponse>("save", boost::bind(&ConfigServer::handleSave, this, _1, _2, true));
	m_srv_load = m_nh.advertiseService("load", &ConfigServer::handleLoad, this);

	m_configPath = m_nh.param<std::string>("config_path", ros::package::getPath("config_server")) + "/";
	m_robotName = m_nh.param<std::string>("robot_name", std::string());

	m_publishParamsTimer = m_nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&ConfigServer::updateParameterList, this), true, false); // Create a one-shot timer for coalescing parameter list updates
	m_publishValuesTimer = m_nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&ConfigServer::updateParameterValueList, this), true, false); // The same for the current values publisher

	if(!load())
		ROS_ERROR("Could not load initial configuration => Starting with default values!");

	updateParameterList();
	updateParameterValueList();
	updateUid();

	initBackups();
}

ConfigServer::~ConfigServer()
{
	m_publishParamsTimer.stop();
	m_publishValuesTimer.stop();
	m_backupTimer.stop();
}

void ConfigServer::initBackups()
{
	m_saveBackups = m_nh.param<bool>("save_backups", false);

	if(!m_saveBackups)
		return;

	std::string backupDir = m_nh.param<std::string>("backup_path", defaultBackupDir);
	int maxBackupUidsInt = m_nh.param<int>("max_backup_uids", 3);
	double backupInterval = m_nh.param<double>("backup_interval", 60.0);

	std::size_t maxBackupUids = 1;
	if(maxBackupUidsInt >= 1)
		maxBackupUids = (std::size_t) maxBackupUidsInt;

	std::string nodeNs = ros::names::clean(ros::this_node::getNamespace()); // Note: Can only contain [0-9a-zA-Z_/]
	std::size_t startIndex = nodeNs.find_first_not_of('/');
	std::size_t endIndex = nodeNs.find_last_not_of('/');
	std::string dirSuffix;
	if(startIndex != std::string::npos && endIndex != std::string::npos)
	{
		dirSuffix = "_" + nodeNs.substr(startIndex, endIndex - startIndex + 1);
		std::replace(dirSuffix.begin(), dirSuffix.end(), '/', '_');
	}

	std::ostringstream ss;
	if(backupDir.empty())
		backupDir = defaultBackupDir;
	if(*backupDir.rbegin() != '/')
		backupDir += '/';
	ss << backupDir << "UID_" << std::setfill('0') << std::setw(10) << m_uid.data.sec << "." << std::setw(9) << m_uid.data.nsec << dirSuffix << "/";
	m_backupDir = ss.str();

	if(!ensureBackupDir())
	{
		m_saveBackups = false;
		return;
	}

	ROS_INFO("Performing config backups every %.1fs when changes are detected, into directory: %s", backupInterval, m_backupDir.c_str());

	try
	{
		fs::path dir(backupDir);
		if(!fs::exists(dir) || !fs::is_directory(dir))
			throw std::runtime_error("The specified backup path '" + dir.string() + "' does not exist or is not a directory!");

		const fs::directory_iterator end;
		fs::directory_iterator file(dir);

		const boost::regex dirNameRegex("^UID_[0-9]{10,}\\.[0-9]{9}" + dirSuffix + "$");
		std::vector<fs::path> uidPathList;
		for(; file != end; ++file)
		{
			if(!fs::is_directory(file->status())) continue;
			fs::path dirPath = file->path();
			if(!dirPath.has_filename()) continue;
			std::string dirName = dirPath.filename().string();

			if(!boost::regex_search(dirName, dirNameRegex)) continue;

			std::size_t i = 0;
			for(; i < uidPathList.size(); i++)
				if(dirPath.string().compare(uidPathList[i].string()) < 0) break;
			uidPathList.insert(uidPathList.begin() + i, dirPath);
		}

		std::size_t dirsToDelete = 0;
		if(uidPathList.size() > maxBackupUids)
			dirsToDelete = uidPathList.size() - maxBackupUids;
		if(dirsToDelete >= 1 && uidPathList.size() >= 1)
		{
			if(dirsToDelete >= uidPathList.size())
				dirsToDelete = uidPathList.size() - 1;
			int count = 0;
			for(std::size_t i = 0; i < dirsToDelete; i++)
			{
				const fs::path& oldDir = uidPathList[i];
				try { fs::remove_all(oldDir); count++; }
				catch(std::runtime_error& e) { ROS_WARN("Failed to delete config backup UID folder: %s", oldDir.string().c_str()); }
			}
			ROS_INFO("Deleted %d old config backup UID folder%c", count, (count > 1 ? 's' : '\0'));
		}
	}
	catch(std::runtime_error& e)
	{
		ROS_WARN("Failed to delete old config backups from directory '%s': %s", backupDir.c_str(), e.what());
	}

	m_changed = true;
	handleBackup();

	m_backupTimer = m_nh.createWallTimer(ros::WallDuration(backupInterval), boost::bind(&ConfigServer::handleBackup, this), false, true);
}

bool ConfigServer::ensureBackupDir()
{
	if(!m_saveBackups)
		return false;

	try
	{
		fs::path backup(m_backupDir);
		boost::system::error_code returnedError;
		fs::create_directories(backup, returnedError);
		if(returnedError)
			throw std::runtime_error(returnedError.message());
	}
	catch(std::runtime_error& e)
	{
		ROS_ERROR("Failed to create backup directory '%s': %s", m_backupDir.c_str(), e.what());
		return false;
	}

	return true;
}

bool ConfigServer::handleSetParameter(SetParameterRequest& req, SetParameterResponse& resp)
{
	ParameterMap::iterator it = m_params.find(req.name);
	if(it == m_params.end())
	{
		ROS_ERROR("Request to set unknown parameter '%s'", req.name.c_str());
		return false;
	}

	it->second.value = req.value;
	it->second.hasValue = true;
	it->second.notify(req.no_notify);

	if(m_pub_currentValues.getNumSubscribers() != 0)
		planValueUpdate();

	m_changed = true;

	return true;
}

bool ConfigServer::handleGetParameter(GetParameterRequest& req, GetParameterResponse& resp)
{
	ParameterMap::iterator it = m_params.find(req.name);
	if(it == m_params.end())
	{
		ROS_ERROR("Request to get unknown parameter '%s'", req.name.c_str());
		return false;
	}

	resp.value = it->second.value;
	return true;
}

bool almostEqual(float x, float y)
{
	float diff = std::abs(x - y);
	return ((diff < std::numeric_limits<float>::epsilon() * std::abs(x + y) * 4) || (diff < std::numeric_limits<float>::min()));
}

bool ConfigServer::doSubscribe(const std::string& callback, const ParameterDescription& desc, std::string* value, bool* changed)
{
	ParameterMap::iterator it = m_params.find(desc.name);
	if(it == m_params.end())
	{
		ROS_WARN("New parameter '%s', type '%s', default: '%s'", desc.name.c_str(), desc.type.c_str(), desc.default_value.c_str());
		std::pair<std::string, Parameter> value;
		value.first = desc.name;
		value.second.desc = desc;

		if(desc.type.empty())
			value.second.hasValue = false;
		else
		{
			value.second.hasValue = true;
			value.second.value = desc.default_value;
		}

		it = m_params.insert(m_params.end(), value);

		if(changed)
			*changed = true;
	}

	if(std::find(it->second.subscribers.begin(), it->second.subscribers.end(), callback) == it->second.subscribers.end())
		it->second.subscribers.push_back(callback); // Add the registering subscriber to our list of subscribers for the parameter

	if(!desc.type.empty())
	{
		if(!it->second.desc.type.empty() && (!almostEqual(desc.min, it->second.desc.min) || !almostEqual(desc.max, it->second.desc.max)))
			ROS_WARN("Subscriber '%s' has registered the parameter '%s' with min/max [%g,%g], which conflicts with the existing min/max [%g,%g] => Updating min/max anyway...", callback.c_str(), desc.name.c_str(), desc.min, desc.max, it->second.desc.min, it->second.desc.max);

		it->second.desc = desc; // If the currently registering parameter has a valid type then its parameter description overrides the previously stored one

		if(!it->second.hasValue) // If the parameter does not have a value yet (e.g. if only registered previously with an invalid type), then set the value to the default value of the newly registering parameter
		{
			it->second.value = desc.default_value;
			it->second.hasValue = true;
		}

		if(changed)
			*changed = true;
	}

	if(value)
		*value = it->second.value;

	return true;
}

void ConfigServer::planUpdate()
{
	planValueUpdate();

	if(m_publishParamsCounter == 0)
	{
		m_publishParamsTimer.start();
		m_publishParamsCounter++;
		return;
	}
	else if(m_publishParamsCounter < 5)
	{
		m_publishParamsCounter++;
		return;
	}

	updateParameterList();
}

void ConfigServer::planValueUpdate()
{
	m_changed = true;

	if(m_publishValuesCounter == 0)
	{
		m_publishValuesTimer.start();
		m_publishValuesCounter++;
		return;
	}
	else if(m_publishValuesCounter < 5)
	{
		m_publishValuesCounter++;
		return;
	}

	updateParameterValueList();
}

bool ConfigServer::handleSubscribe(SubscribeRequest& req, SubscribeResponse& resp)
{
	resp.uid = m_uid.data;

	bool changed = false;
	if(!doSubscribe(req.callback, req.desc, &resp.value, &changed))
		return false;

	if(changed)
		planUpdate();

	return true;
}

bool ConfigServer::handleSubscribeList(SubscribeListRequest& req, SubscribeListResponse& resp)
{
	resp.uid = m_uid.data;

	bool changed = false;

	resp.values.resize(req.parameters.size());
	for(size_t i = 0; i < req.parameters.size(); ++i)
	{
		if(!doSubscribe(req.callback, req.parameters[i], &resp.values[i], &changed))
			return false;
	}

	if(changed)
		planUpdate();

	return true;
}

bool ConfigServer::handleShowDeadVars(ShowDeadVarsRequest& req, ShowDeadVarsResponse& resp)
{
	std::string scope = ((req.configPath.empty() || req.configPath.at(0) != '/') ? '/' + req.configPath : req.configPath);
	if(scope.empty() || scope == "/")
		ROS_WARN("Showing all dead config parameters:");
	else
		ROS_WARN("Showing all dead config parameters in '%s':", scope.c_str());
	
	int count = 0;
	for(ParameterMap::iterator it = m_params.begin(); it != m_params.end(); ++it)
	{
		if(it->first.compare(0, scope.size(), scope) != 0) continue;
		if(it->second.desc.type.empty() || it->second.subscribers.empty())
		{
			ROS_INFO("%s => '%s'", it->first.c_str(), it->second.value.c_str());
			count++;
		}
	}
	
	ROS_WARN("End of list (%d items)", count);
	return true;
}

void ConfigServer::updateParameterList()
{
	// Stop the timer. This is actually needed to reset the "one-shot" state
	// of the timer, even if we got called from the timer!
	m_publishParamsTimer.stop();

	ParameterListPtr list = boost::make_shared<ParameterList>();

	list->parameters.reserve(m_params.size());

	for(ParameterMap::iterator it = m_params.begin(); it != m_params.end(); ++it)
		list->parameters.push_back(it->second.desc);

	m_pub_paramList.publish(list);

	m_publishParamsCounter = 0;

	updateUid();
}

void ConfigServer::updateParameterValueList()
{
	// Stop the timer. This is actually needed to reset the "one-shot" state
	// of the timer, even if we got called from the timer!
	m_publishValuesTimer.stop();

	ParameterValueListPtr list = boost::make_shared<ParameterValueList>();

	list->parameters.reserve(m_params.size());

	for(ParameterMap::iterator it = m_params.begin(); it != m_params.end(); ++it)
	{
		ParameterValue val;
		val.name = it->first;
		val.value = it->second.value;
		val.type = it->second.desc.type;
		val.min = it->second.desc.min;
		val.max = it->second.desc.max;
		val.step = it->second.desc.step;
		list->parameters.push_back(val);
	}

	m_pub_currentValues.publish(list);

	m_publishValuesCounter = 0;
}

void ConfigServer::handleBackup()
{
	if(!m_saveBackups) return;

	if(!m_changed) return;
	m_changed = false;

	if(!ensureBackupDir())
	{
		m_saveBackups = false;
		m_backupTimer.stop();
		return;
	}

	char buf[32];
	std::time_t newTime;
	std::time(&newTime);
	if(std::strftime(buf, 32, "%Y%m%d_%H%M%S", std::localtime(&newTime)) != 15)
	{
		ROS_ERROR("Unexpected error with std::strftime, this should never happen!");
		return;
	}

	std::ostringstream ss;
	ss << m_backupDir << "config_" << m_robotName << "_" << buf << fileExtension;

	Save srv;
	srv.request.filename = ss.str();
	if(!handleSave(srv.request, srv.response, false))
		ROS_ERROR("Backup of config server to '%s' failed!", srv.request.filename.c_str());
}

void ConfigServer::updateUid()
{
	m_pub_uid.publish(m_uid);
}

std::string ConfigServer::defaultConfigName()
{
	if(m_robotName.length() != 0)
		return "config_" + m_robotName + fileExtension;
	else
		return "config" + fileExtension;
}

typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;

bool ConfigServer::handleSave(SaveRequest& req, SaveResponse& resp, bool show)
{
	std::vector<std::string> current_path;
	YAML::Emitter em;

	boost::char_separator<char> sep("/");

	em << YAML::BeginMap;
	for(ParameterMap::iterator it = m_params.begin(); it != m_params.end(); ++it)
	{
		Tokenizer tokenizer(it->first, sep);
		std::vector<std::string> tokens;
		BOOST_FOREACH(const std::string& t, tokenizer)
			tokens.push_back(t);

		std::string prop_name = tokens[tokens.size()-1];
		tokens.pop_back();

		int idx = 0;
		size_t tok_idx;

		for(tok_idx = 0; tok_idx < tokens.size(); ++tok_idx)
		{
			if(tok_idx >= current_path.size() || tokens[tok_idx] != current_path[idx])
				break;

			++idx;
		}

		// Pop everything till idx from stack
		for(int i = current_path.size()-1; i >= idx; --i)
		{
			em << YAML::EndMap;
			current_path.pop_back();
		}

		// Push new paths
		for(; tok_idx < tokens.size(); ++tok_idx)
		{
			em << YAML::Key << tokens[tok_idx];
			em << YAML::Value << YAML::BeginMap;

			current_path.push_back(tokens[tok_idx]);
		}

		em << YAML::Key << prop_name;
		em << YAML::Value << it->second.value;
	}
	em << YAML::EndMap;

	std::string fname;
	if(req.filename.empty())
		fname = m_configPath + defaultConfigName();
	else if(req.filename[0] == '/')
		fname = req.filename;
	else
		fname = m_configPath + req.filename;

	bool endsInYaml = false;
	if(fname.length() >= fileExtension.length())
		endsInYaml = (fname.compare(fname.length() - fileExtension.length(), fileExtension.length(), fileExtension) == 0);
	if(!endsInYaml)
		fname += fileExtension;

	std::ofstream out;
	out.open(fname.c_str());
	if(out.is_open())
	{
		out << em.c_str() << "\n";
		out.close();
		if(show)
			ROS_INFO("Saved config parameters to '%s'!", fname.c_str());
	}
	else
	{
		ROS_ERROR("Failed to save config parameters to '%s'!", fname.c_str());
		return false;
	}

	return true;
}

bool ConfigServer::load(const std::string& filename)
{
	std::string fname;
	if(filename.empty())
		fname = m_configPath + defaultConfigName();
	else if(filename[0] == '/')
		fname = filename;
	else
		fname = m_configPath + filename;

	bool endsInYaml = false;
	if(fname.length() >= fileExtension.length())
		endsInYaml = (fname.compare(fname.length() - fileExtension.length(), fileExtension.length(), fileExtension) == 0);
	if(!endsInYaml)
		fname += fileExtension;

	YAML::Node n;
	try
	{
		n = YAML::LoadFile(fname);
	}
	catch (YAML::Exception& e)
	{
		ROS_ERROR("Could not parse config file %s: %s", fname.c_str(), e.what());
		return false;
	}

	insertFromYAML(n, "");

	planUpdate();

	ParameterLoadTime msg;
	msg.stamp.fromNSec(ros::WallTime::now().toNSec());
	msg.filename = fname;
	m_pub_loadTime.publish(msg);

	ROS_INFO("Loaded config file: %s", fname.c_str());

	return true;
}

void ConfigServer::insertFromYAML(const YAML::Node& n, const std::string& path)
{
	if(n.Type() == YAML::NodeType::Map)
	{
		for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
		{
			insertFromYAML(it->second, path + "/"+it->first.as<std::string>());
		}
	}

	if(n.Type() == YAML::NodeType::Scalar)
	{
		bool needNotify = false;
		if(!m_params.count(path))
		{
			m_params[path] = Parameter();
			needNotify = true;
		}

		Parameter& p = m_params[path];
		std::string newValue = n.as<std::string>();
		needNotify |= (p.desc.name != path || p.value != newValue);
		p.desc.name = path;
		p.value = newValue;
		p.hasValue = true;

		if(needNotify)
			p.notify();
	}
}

bool ConfigServer::handleLoad(LoadRequest& req, LoadResponse& resp)
{
	return load(req.filename);
}

void ConfigServer::finalise()
{
	if(m_saveBackups)
	{
		m_changed = true;
		handleBackup();
	}
}

static void* notify_thread(void*)
{
	NotifyThread* thread = NotifyThread::instance();

	while(1)
		thread->handleRequest();

	return 0;
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "config_server");

	ros::NodeHandle nh("~");

	config_server::NotifyThread::instance();

	pthread_t notifyThread;
	pthread_create(&notifyThread, NULL, config_server::notify_thread, NULL);

	config_server::ConfigServer server;

	ros::spin();

	server.finalise();

	return 0;
}

