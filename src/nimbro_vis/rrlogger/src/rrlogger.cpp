// Maintain round robin rosbags
// author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include "rrlogger/rrlogger.h"

#include <ros/init.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/bind.hpp>
#include <time.h>
#include <ctype.h>

namespace fs = boost::filesystem;

namespace logger
{
RRLogger::RRLogger()
 : m_nh("~")
 , m_deleteLog(true)
 , m_numOfLogs(3)
 , QUEUE_SIZE(50)
 , LOG_PATH("/var/log/nimbro")
{
	openBag();

	XmlRpc::XmlRpcValue list;
	if (m_nh.getParam("topics", list))
	{
		ROS_INFO("Reading topic list");
		ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		int i;
		for (i = 0; i < list.size(); i++)
		{
			std::string name = static_cast<std::string>(list[i]);

			boost::function<void(const boost::shared_ptr<topic_tools::ShapeShifter>&)> func =
				boost::bind(&RRLogger::writeCb, this, _1, name);

			ros::Subscriber sub = m_nh.subscribe<topic_tools::ShapeShifter>(name, QUEUE_SIZE, func);

			ROS_INFO("Subscribed to topic: %s", sub.getTopic().c_str());
			m_subscribers.push_back(sub);
		}

	}

}

RRLogger::~RRLogger()
{
	try
	{
		m_bag.close();
	}
	catch (rosbag::BagIOException& e)
	{}
}

void RRLogger::writeCb(const boost::shared_ptr< topic_tools::ShapeShifter >& data, const std::string& topic)
{
	m_bag.write(topic, ros::Time::now(), data);
}

void RRLogger::openBag()
{
	fs::path p(LOG_PATH);
	
	if (!fs::exists(p) || !fs::is_directory(p))
	{
		ROS_ERROR("'%s' doesn't exist or isn't a directory", p.string().c_str());
		throw std::runtime_error("'"+p.string()+"' doesn't exist or isn't a directory");
	}
	
	const std::string filetype = ".bag";
	const boost::regex filetypeRegEx("\\.bag$");
	
	const fs::directory_iterator end;
	fs::directory_iterator file(p);
	
	std::string filename;
	std::string reference;
	fs::path oldLogPath;
	
	int logCnt = 0;
	for (; file != end; file++)
	{
		if (fs::is_directory(*file))
			continue;
		
		filename = file->path().leaf().string();
		
		if (!boost::regex_search(filename, filetypeRegEx))
			continue;

		logCnt++;


		//strip out the filetype
		filename = filename.substr(0, filename.size() - filetype.size());

		if (logCnt == 1)
		{
			reference = filename;
			oldLogPath = file->path();
		}

		try
		{
			if (compareDate(filename, reference) < 0)
			{
				oldLogPath = file->path();
				reference = filename;
			}
		}
		catch (std::invalid_argument& e)
		{
			ROS_ERROR("The filenames '%s' and '%s' could not be compared.", filename.c_str(), reference.c_str());
			ROS_ERROR("No logfiles will be deleted!");
			ROS_ERROR("Remove all but the generated files to enable automated control.");
			m_deleteLog = false;
			break;
		}
	}

	std::string nLogfileName = nameFromDate();

	if ((logCnt >= m_numOfLogs) && m_deleteLog)
	{
		ROS_INFO("Deleting bag %s", oldLogPath.string().c_str());
		if (!fs::remove(oldLogPath))
			ROS_WARN("Old logfile '%s' could not be removed.", oldLogPath.string().c_str());
	}

	try
	{
		std::string absFileName = LOG_PATH + "/" + nLogfileName + filetype;
		m_bag.open(absFileName, rosbag::bagmode::Write);
		ROS_INFO("Creating bag %s", absFileName.c_str());
	}
	catch (rosbag::BagException& e)
	{
		ROS_ERROR("%s", e.what());
		ROS_ERROR("Please ensure that write permissions are set for %s", LOG_PATH.c_str());
		throw;
	}
}

int RRLogger::compareDate(std::string a, std::string b)
{
	if (a.length() != b.length())
		throw std::invalid_argument("Input strings not equal in length");

	unsigned int i;
	char aC, bC;
	for (i = 0; i < a.length(); i++)
	{
		aC = a.at(i);
		bC = b.at(i);
		if ((isdigit(aC) == 0) || (isdigit(bC) == 0))
			throw std::invalid_argument("Malformed input strings.");

		if (aC > bC)
			return 1;
		else if (aC < bC)
			return -1;
	}
	return 0;
}

std::string RRLogger::nameFromDate()
{
	char buf[128];

	std::time_t t;
	std::time(&t);
	std::tm* date = std::localtime(&t);
	std::strftime(buf, 128, "%y%m%d%H%M%S", date);

	return buf;
}





}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rrlogger");

	try
	{
		logger::RRLogger l;

		ros::spin();
		return 0;
	}
	catch(rosbag::BagException& e)
	{
		return -1;
	}
	catch(std::runtime_error& e)
	{
		return -1;
	}
}
