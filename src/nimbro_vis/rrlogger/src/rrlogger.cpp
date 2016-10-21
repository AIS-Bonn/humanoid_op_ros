// Maintain a round robin of logging ROS bags
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Includes
#include <rrlogger/rrlogger.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/bind.hpp>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <signal.h>
#include <sstream>
#include <time.h>

// Namespaces
using namespace rrlogger;

// Namespace aliases
namespace fs = boost::filesystem;

// Constants
const std::string RRLogger::PLOT_TOPIC = "/plot";
const std::string RRLogger::TF_TOPIC = "/tf";
const std::string RRLogger::LOG_PATH = "/var/log/nimbro";
const int RRLogger::TOPIC_QUEUE_SIZE = 20;
const std::size_t RRLogger::MAX_MAX_NUM_BAGS = 100;
const std::size_t RRLogger::MAX_MAX_TOTAL_MB = (1 << 20);
const std::size_t RRLogger::WARN_BAGS_NUM = 5;
const double RRLogger::WARN_BAGS_SEC = 15.0;
const std::size_t RRLogger::MAX_HEARTBEAT_SOURCES = 10;

//
// Constructor/destructor
//

// Constructor
RRLogger::RRLogger()
 : m_nh("~")
 , m_confEnable("enable", true)
 , m_confPause("pause", false)
 , m_confStartNewBag("startNewBag", false)
 , m_confForceHeartbeat("forceHeartbeat", false)
 , m_confKeepAllBags("keepAllBags", false)
 , m_confKeepNoBags("keepNoBags", false)
 , m_inited(false)
 , m_enabled(false)
 , m_started(false)
 , m_bagOpen(false)
 , m_paused(false)
 , m_tfLogAll(false)
 , m_showEnabledState(false)
{
	// Reset the logger configuration
	resetConfig();

	// Reset the class
	reset();

	// Config parameter callbacks
	m_confStartNewBag.set(false);
	m_confEnable.setCallback(boost::bind(&RRLogger::handleConfEnable, this), true);
	m_confPause.setCallback(boost::bind(&RRLogger::handleConfPause, this), true);
	m_confStartNewBag.setCallback(boost::bind(&RRLogger::handleConfStartNewBag, this), false);
	m_confForceHeartbeat.setCallback(boost::bind(&RRLogger::handleConfForceHeartbeat, this), true);

	// Subscribe to ROS topics
	m_sub_heartbeat = m_nh.subscribe<rrlogger::LoggerHeartbeat>("heartbeat", 10, &RRLogger::handleHeartbeat, this);

	// Create ROS timers
	m_heartbeatTimer = m_nh.createTimer(ros::Duration(1.0), boost::bind(&RRLogger::handleHeartbeatTimer, this, _1), true, false); // Note: One-shot timer without autostart
}

// Destructor
RRLogger::~RRLogger()
{
	// Stop the heartbeat timer
	m_heartbeatTimer.stop();

	// Stop logging
	stopLogging();
}

//
// Reset functions
//

// Reset function
void RRLogger::reset()
{
	// Deinitialise the logger
	init(false);

	// Clear all topics from our list of subscribers
	clearTopics();

	// Clear all plot topics from our list
	clearPlotTopics();

	// Clear all tf frames from our list
	clearTfFrames();

	// Reset the logger configuration
	resetConfig();
}

// Reset logger configuration function
void RRLogger::resetConfig()
{
	// Reset logger configuration
	m_maxNumBags = 3;
	m_maxTotalMB = 4096;
	m_prefix = "rrlogger";
	setHeartbeatParam(false, 1.0, false);
}

//
// Initialisation function
//

// Initialisation function
void RRLogger::init(bool init)
{
	// Set the initialisation state of the logger
	m_inited = init;

	// Reset the bag timestamp buffer
	m_bagStamps.clear();
	m_bagStamps.rset_capacity(WARN_BAGS_NUM);

	// Show the enabled state information to the user if the logger is being initialised
	m_showEnabledState = init;

	// Update the enabled state of the logger
	updateEnabled();
}

//
// Topic management functions
//

// Add a topic to be logged (throws std::runtime_error on failure)
void RRLogger::addTopic(const std::string& name, std::string* resolvedName)
{
	// Clear the output resolved name
	if(resolvedName)
		resolvedName->clear();

	// Do nothing if the name is empty
	if(name.empty())
		throw std::runtime_error("Tried to add empty topic '' to the topic list!");

	// Add a '/' to the beginning of the topic if there is none, as a topic will practically never be intended to be resolved relative to this logger node
	std::string topic = name;
	if(topic[0] != '/')
		topic.insert(topic.begin(), '/');

	// Try to resolve the topic name
	std::string resolvedTopic;
	try { resolvedTopic = m_nh.resolveName(topic); }
	catch(std::exception& e)
	{
		std::ostringstream ss;
		ss << "Could not resolve topic '" << topic << "': " << e.what();
		throw std::runtime_error(ss.str());
	}

	// Output the resolved name
	if(resolvedName)
		*resolvedName = resolvedTopic;

	// Ensure we don't duplicate topics
	if(haveTopic(resolvedTopic))
	{
		std::ostringstream ss;
		ss << "Not adding topic '" << resolvedTopic << "' resolved from '" << topic << "' because it already exists in the topic list!";
		throw std::runtime_error(ss.str());
	}

	// Attempt to subscribe to the required topic
	ros::Subscriber sub;
	try
	{
		if(resolvedTopic == PLOT_TOPIC)
			sub = m_nh.subscribe(resolvedTopic, TOPIC_QUEUE_SIZE, &RRLogger::writePlotCb, this);
		else if(resolvedTopic == TF_TOPIC)
			sub = m_nh.subscribe(resolvedTopic, TOPIC_QUEUE_SIZE, &RRLogger::writeTfCb, this);
		else
		{
			boost::function<void(const boost::shared_ptr<topic_tools::ShapeShifter>&)> callback = boost::bind(&RRLogger::writeCb, this, _1, resolvedTopic);
			sub = m_nh.subscribe<topic_tools::ShapeShifter>(resolvedTopic, TOPIC_QUEUE_SIZE, callback);
		}
	}
	catch(std::exception& e)
	{
		std::ostringstream ss;
		ss << "Failed to subscribe to topic '" << resolvedTopic << "' resolved from '" << topic << "': " << e.what();
		throw std::runtime_error(ss.str());
	}
	if(!sub)
	{
		std::ostringstream ss;
		ss << "Failed to subscribe to topic '" << resolvedTopic << "' resolved from '" << topic << "' for an unknown reason!";
		throw std::runtime_error(ss.str());
	}

	// Sanity check that the topic was not modified by the call to subscribe
	std::string subscriberTopic = sub.getTopic();
	if(subscriberTopic != resolvedTopic)
		ROS_WARN("[%s] Topic mismatch inside the ROS subscriber ('%s' != '%s'), this should never happen!", m_prefix.c_str(), subscriberTopic.c_str(), resolvedTopic.c_str());

	// Add the new subscriber to the topic list
	m_topics.push_back(sub);
}

// Remove a topic from the list of topics to be logged
bool RRLogger::removeTopic(const std::string& resolvedName)
{
	// Remove all instances of the topic from our list
	bool foundOne = false;
	for(std::size_t i = m_topics.size(); i-- > 0; )
	{
		ros::Subscriber& sub = m_topics[i];
		if(sub.getTopic() == resolvedName)
		{
			sub.shutdown();
			m_topics.erase(m_topics.begin() + i);
			foundOne = true;
		}
	}

	// Return whether the topic list was modified
	return foundOne;
}

// Check whether we currently have a particular topic in our list
bool RRLogger::haveTopic(const std::string& resolvedName) const
{
	// Go through each topic in our list and see whether it is the one we're looking for
	for(std::vector<ros::Subscriber>::const_iterator it = m_topics.begin(); it != m_topics.end(); ++it)
	{
		const ros::Subscriber& sub = *it;
		if(sub.getTopic() == resolvedName)
			return true;
	}
	return false;
}

// Clear all topics in our list
void RRLogger::clearTopics()
{
	// Remove all topics from our list
	for(std::vector<ros::Subscriber>::iterator it = m_topics.begin(); it != m_topics.end(); ++it)
		(*it).shutdown();
	m_topics.clear();
}

//
// Plot topic management functions
//

// Add a plot topic to be logged (returns false on failure)
bool RRLogger::addPlotTopic(const std::string& name, std::string* resolvedName)
{
	// Clear the output resolved name
	if(resolvedName)
		resolvedName->clear();

	// Do nothing if the name is empty
	if(name.empty())
		return false;

	// Add a '/' to the beginning of the plot topic if there is none
	std::string plotTopic = name;
	if(plotTopic[0] != '/')
		plotTopic.insert(plotTopic.begin(), '/');

	// Output the resolved name
	if(resolvedName)
		*resolvedName = plotTopic;

	// Ensure we don't duplicate plot topics
	if(havePlotTopic(plotTopic))
		return false;

	// Add the plot topic to the plot topic list
	m_plotTopics.push_back(plotTopic);

	// Return that the plot topic was successfully added
	return true;
}

// Remove a plot topic from the list of plot topics to be logged
bool RRLogger::removePlotTopic(const std::string& resolvedName)
{
	// Remove all occurrences (should only ever be one) of the given plot topic
	size_t initSize = m_plotTopics.size();
	m_plotTopics.erase(std::remove(m_plotTopics.begin(), m_plotTopics.end(), resolvedName), m_plotTopics.end());
	return (m_plotTopics.size() != initSize);
}

//
// Tf frame management functions
//

// Add a tf frame to be logged (returns false on failure)
bool RRLogger::addTfFrame(const std::string& name, std::string* resolvedName)
{
	// Clear the output resolved name
	if(resolvedName)
		resolvedName->clear();

	// Remove all leading '/' characters from the tf frame
	std::string tfFrame = name;
	std::size_t startIndex = tfFrame.find_first_not_of('/');
	if(startIndex == std::string::npos)
		tfFrame.clear();
	else if(startIndex > 0)
		tfFrame = tfFrame.substr(startIndex);

	// Output the resolved name
	if(resolvedName)
		*resolvedName = tfFrame;

	// Ensure we don't duplicate tf frames
	if(haveTfFrame(tfFrame))
		return false;

	// Add the tf frame to the tf frame list
	m_tfFrames.push_back(tfFrame);
	if(tfFrame.empty())
		m_tfLogAll = true;

	// Return that the tf frame was successfully added
	return true;
}

// Remove a tf frame from the list of tf frames to be logged
bool RRLogger::removeTfFrame(const std::string& resolvedName)
{
	// Remove all occurrences (should only ever be one) of the given tf frame
	std::size_t initSize = m_tfFrames.size();
	m_tfFrames.erase(std::remove(m_tfFrames.begin(), m_tfFrames.end(), resolvedName), m_tfFrames.end());
	if(resolvedName.empty())
		m_tfLogAll = false;
	return (m_tfFrames.size() != initSize);
}

//
// Config server parameter callbacks
//

// Callback for the start new bag config parameter
void RRLogger::handleConfStartNewBag()
{
	// Start a new bag if required
	if(m_confStartNewBag())
	{
		m_confStartNewBag.set(false);
		ROS_INFO("[%s] Manual trigger to start a new bag has been received%s!", m_prefix.c_str(), (m_bagOpen ? "" : ", although no logging is currently in progress"));
		stopLogging();
		updateLoggingState();
	}
}

//
// Update functions
//

// Update the enabled state of the logger
void RRLogger::updateEnabled()
{
	// Work out whether the logger should be enabled
	bool shouldBeEnabled = (m_inited && m_confEnable());

	// Enable the logger if required
	if(shouldBeEnabled && (!m_enabled || m_showEnabledState))
	{
		ROS_INFO("[%s] Logger has been enabled...", m_prefix.c_str());
		m_enabled = true;
		m_showEnabledState = false;
		resetHeartbeat(); // Note: This calls updateLoggingState() internally
	}

	// Disable the logger if required
	if(!shouldBeEnabled && (m_enabled || m_showEnabledState))
	{
		m_enabled = false;
		m_showEnabledState = false;
		resetHeartbeat(); // Note: This calls updateLoggingState() internally
		ROS_INFO("[%s] Logger has been disabled...", m_prefix.c_str());
	}
}

// Update the pause state of the logger
void RRLogger::updatePaused(bool showInfo)
{
	// Work out whether the logger should be paused
	bool shouldBePaused = m_confPause();

	// Indicate to the user that the logger is paused if required
	if(shouldBePaused && m_paused && showInfo && m_started)
		ROS_INFO("[%s] Logging is paused...", m_prefix.c_str());

	// Pause logging if required
	if(shouldBePaused && !m_paused)
	{
		if(m_started)
			ROS_INFO("[%s] Logging has been paused...", m_prefix.c_str());
		m_paused = true;
	}

	// Resume logging if required
	if(!shouldBePaused && m_paused)
	{
		if(m_started)
			ROS_INFO("[%s] Logging has been resumed...", m_prefix.c_str());
		m_paused = false;
	}
}

// Update the logging state of the logger
void RRLogger::updateLoggingState()
{
	// By default we don't want to be logging
	bool shouldLog = false;

	// Determine whether we should be logging
	std::vector<bool> sourceStates;
	double maxTimeToFalse = 0.0;
	if(m_enabled)
	{
		if(m_heartbeatEnabled)
		{
			if(m_confForceHeartbeat())
				shouldLog = true;
			else
			{
				if(m_heartbeatMap.empty())
					shouldLog = m_heartbeatOptional;
				else
				{
					shouldLog = false;
					ros::Time now = ros::Time::now();
					for(HMapType::const_iterator it = m_heartbeatMap.begin(); it != m_heartbeatMap.end(); ++it)
					{
						bool enableLogging = it->second.second;
						if(!m_heartbeatOptional)
						{
							const ros::Time& stamp = it->second.first;
							double timeToFalse = m_heartbeatTimeout - (now - stamp).toSec();
							if(timeToFalse > m_heartbeatTimeout)
								timeToFalse = m_heartbeatTimeout;
							if(timeToFalse <= 0.0)
								enableLogging = false;
							else if(enableLogging && timeToFalse > maxTimeToFalse)
								maxTimeToFalse = timeToFalse;
						}
						sourceStates.push_back(enableLogging);
						shouldLog |= enableLogging;
					}
				}
			}
		}
		else
		{
			// If heartbeating is disabled then logging should be active by default
			shouldLog = true;
		}
	}

	// Start or stop logging if required
	bool haveSourceStates = (!sourceStates.empty() && sourceStates.size() == m_heartbeatMap.size());
	if(shouldLog && !m_started)
	{
		if(haveSourceStates)
		{
			std::ostringstream ss;
			int numSources = 0;
			HMapType::const_iterator it = m_heartbeatMap.begin();
			for(std::size_t i = 0; i < sourceStates.size(); ++i, ++it)
			{
				if(sourceStates[i])
				{
					if(numSources++ > 0)
						ss << ", ";
					ss << it->first;
				}
			}
			ROS_INFO("[%s] Active heartbeat sources: %s", m_prefix.c_str(), ss.str().c_str());
		}
		startLogging();
	}
	if(!shouldLog && m_started)
	{
		if(haveSourceStates)
		{
			if(m_heartbeatOptional)
				ROS_INFO("[%s] All heartbeat sources say to stop logging!", m_prefix.c_str());
			else
				ROS_INFO("[%s] All heartbeat sources either say to stop logging or have timed out!", m_prefix.c_str());
		}
		stopLogging();
	}

	// Manage the heartbeat timeout timer to call this function again at the earliest point that the simple passing of time could change its outcome
	m_heartbeatTimer.stop();
	if(maxTimeToFalse > 0.0)
	{
		m_heartbeatTimer.setPeriod(ros::Duration(maxTimeToFalse + 0.05)); // Note: The little bit of extra time added on is to make sure that the heartbeat being waited out has a large likelihood of actually having timed out by the time this function is called again
		m_heartbeatTimer.start();
	}
}

//
// Logging control
//

// Start logging
bool RRLogger::startLogging()
{
	// Stop logging if we currently already have a bag open
	if(m_started)
		stopLogging();

	// Set the started flag
	m_started = true;

	// Indicate to the user that logging has started
	ROS_INFO("[%s] Logging has started...", m_prefix.c_str());

	// Update the logging paused state
	updatePaused(true);

	// Open up a new bag for logging
	if(openBag())
		return true;
	else
	{
		ROS_ERROR("[%s] Failed to create a new bag in directory '%s'!", m_prefix.c_str(), LOG_PATH.c_str());
		return false;
	}
}

// Stop logging
void RRLogger::stopLogging()
{
	// Indicate to the user that logging has stopped
	if(m_started)
		ROS_INFO("[%s] Logging has stopped...", m_prefix.c_str());

	// Close any bag we might have open
	closeBag();

	// Clear the started flag
	m_started = false;
}

//
// Bag I/O functions
//

// Bag entry struct
struct BagEntry
{
	BagEntry() : path(), date(), time(), bytes(0) {}
	BagEntry(const fs::path& path, const std::string& date, const std::string& time, boost::uintmax_t bytes) : path(path), date(date), time(time), bytes(bytes) {}
	fs::path path;
	std::string date;
	std::string time;
	boost::uintmax_t bytes;
};

// Open up a new bag to be ready for immediate logging (includes the management of the round robin bag scheme, assumes no bag is currently open)
bool RRLogger::openBag()
{
	// Generate the required name of the new bag
	char buf[32];
	std::time_t newTime;
	std::time(&newTime);
	if(std::strftime(buf, 32, "_%Y%m%d_%H%M%S.bag", std::localtime(&newTime)) != 20)
	{
		ROS_ERROR("[%s] Unexpected error with std::strftime, this should never happen!", m_prefix.c_str());
		return false;
	}
	std::string newBagFileName = m_prefix + std::string(buf);
	std::string newBagFilePath = LOG_PATH + "/" + newBagFileName;

	// Issue a warning if we are opening a lot of bags in a short time interval
	ros::Time now = ros::Time::now();
	if(!m_bagStamps.empty() && m_bagStamps.full() && (now - m_bagStamps.front()).toSec() < WARN_BAGS_SEC)
		ROS_WARN("[%s] Warning: Creating more than %d bags in %.1f seconds!", m_prefix.c_str(), (int)m_bagStamps.capacity(), WARN_BAGS_SEC);
	m_bagStamps.push_back(now);

	// Be safe about catching any exceptions
	try
	{
		// Set the logging directory and verify that it is valid
		fs::path dir(LOG_PATH);
		if(!fs::exists(dir) || !fs::is_directory(dir))
		{
			ROS_ERROR("[%s] The logging directory '%s' does not exist or is not a directory!", m_prefix.c_str(), dir.string().c_str());
			return false;
		}

		// Look what other bags of ours are already in the logging directory
		if(m_confKeepAllBags())
			ROS_WARN("[%s] Not deleting any old bags because the config server says to keep all of them!", m_prefix.c_str());
		else
		{
			// Set up the regex to filter the logging directory contents by
			const boost::regex fileNameRegex("^.*_[0-9]{8}_[0-9]{6}\\.bag$");

			// Get an iterator to the contents of the logging directory
			const fs::directory_iterator end;
			fs::directory_iterator file(dir);

			// Loop through each item in the logging directory and create a chronological list of the bags that are ours
			std::vector<BagEntry> bagList;
			boost::uintmax_t totalBytes = 0;
			for(; file != end; ++file)
			{
				// Ignore items that aren't regular files
				if(!fs::is_regular_file(file->status())) continue;

				// Retrieve the file path and file name
				fs::path filePath = file->path();
				if(!filePath.has_leaf()) continue;
				std::string fileName = filePath.leaf().string();

				// Ignore files that don't match the regex of the bags we are creating
				if(!boost::regex_search(fileName, fileNameRegex)) continue;

				// Extract the components of the file name
				std::size_t len = fileName.size();
				if(len < 20)
				{
					ROS_ERROR("[%s] Logical problem with the bag filename regex, this should never happen!", m_prefix.c_str());
					continue;
				}
				std::string prefix = fileName.substr(0, len - 20);
				std::string date = fileName.substr(len - 19, 8);
				std::string time = fileName.substr(len - 10, 6);

				// Ignore bags that don't have our prefix
				if(prefix != m_prefix) continue;

				// Retrieve the size of the bag in bytes
				boost::uintmax_t bagBytes = fs::file_size(filePath);
				if(bagBytes == static_cast<boost::uintmax_t>(-1))
				{
					ROS_WARN("[%s] Failed to retrieve the size of the existing bag '%s'!", m_prefix.c_str(), fileName.c_str());
					bagBytes = 0;
				}
				totalBytes += bagBytes;

				// Insert the bag into our list at the appropriate chronological location
				std::size_t i = 0;
				for(; i < bagList.size(); i++)
				{
					int dateCompare = date.compare(bagList[i].date);
					int timeCompare = time.compare(bagList[i].time);
					if(dateCompare < 0 || (dateCompare == 0 && timeCompare < 0)) break;
				}
				bagList.insert(bagList.begin() + i, BagEntry(filePath, date, time, bagBytes));
			}

			// Calculate the bag quota limit in bytes
			boost::uintmax_t maxTotalBytes = m_maxTotalMB;
			maxTotalBytes <<= 20;

			// Delete as many of the oldest existing bags as required to get back to the maximum allowed number and disk quota
			std::size_t bagsToDelete = 0;
			if(bagList.size() >= m_maxNumBags && bagList.size() >= 1)
				bagsToDelete = std::max(bagsToDelete, bagList.size() - m_maxNumBags + 1);
			if(totalBytes > maxTotalBytes)
			{
				std::size_t i = 0;
				boost::uintmax_t byteCount = 0;
				for(; i < bagList.size() && maxTotalBytes + byteCount < totalBytes; i++)
					byteCount += bagList[i].bytes;
				bagsToDelete = std::max(bagsToDelete, i);
			}
			if(m_confKeepNoBags())
			{
				ROS_WARN("[%s] Deleting all old bags because the config server says to keep none!", m_prefix.c_str());
				bagsToDelete = bagList.size();
			}
			if(bagsToDelete > 0)
			{
				if(bagsToDelete > bagList.size())
					bagsToDelete = bagList.size();
				for(std::size_t i = 0; i < bagsToDelete; i++)
				{
					const fs::path& oldBag = bagList[i].path;
					ROS_INFO("[%s] Deleting bag %s", m_prefix.c_str(), oldBag.c_str());
					try
					{
						if(!fs::remove(oldBag))
							ROS_WARN("[%s] The bag to delete was not found!", m_prefix.c_str());
					}
					catch(std::exception& e)
					{
						ROS_WARN("[%s] Bag deletion failed: %s", m_prefix.c_str(), e.what());
					}
				}
			}
		}

		// Indicate that a bag is being opened
		ROS_INFO("[%s] Creating bag %s", m_prefix.c_str(), newBagFilePath.c_str());

		// Open up the required new bag
		m_bag.open(newBagFilePath, rosbag::bagmode::Write);
	}
	catch(std::exception& e)
	{
		ROS_ERROR("[%s] Failed to open bag: %s", m_prefix.c_str(), e.what());
		closeBag(); // In case we managed to open the bag but then errored out...
		return false;
	}

	// Set that we have a bag open now
	m_bagOpen = true;

	// Return that a bag was successfully opened
	return true;
}

// Close a bag if we have one open
void RRLogger::closeBag()
{
	// Indicate that the bag was closed
	std::string fileName = m_bag.getFileName();
	if(!fileName.empty())
		ROS_INFO("[%s] Closed bag %s", m_prefix.c_str(), fileName.c_str());

	// Try to close the bag (does nothing if we don't have one open anyway)
	try { m_bag.close(); }
	catch(rosbag::BagIOException&) {}

	// Set that we don't have a bag open anymore
	m_bagOpen = false;
}

// Callback to write data to our bag
void RRLogger::writeCb(const boost::shared_ptr<topic_tools::ShapeShifter>& data, const std::string& resolvedName)
{
	// Don't do anything if no bag is open or the logger is paused
	if(!m_bagOpen || m_paused) return;

	// Write the required data with the current timestamp
	m_bag.write(resolvedName, ros::Time::now(), data);
}

// Callback to write plot data to our bag
void RRLogger::writePlotCb(const plot_msgs::PlotConstPtr& data)
{
	// Don't do anything if no bag is open or the logger is paused
	if(!m_bagOpen || m_paused) return;

	// Retrieve the plot topic basename that this plot message belongs to
	std::string dataTopic;
	if(!data->header.frame_id.empty()) // Ideally this should always be the case...
		dataTopic = data->header.frame_id;
	else if(!data->points.empty())
		dataTopic = data->points[0].name;
	else if(!data->events.empty())
		dataTopic = data->events[0];
	if(dataTopic.empty()) return;
	if(dataTopic[0] != '/')
		dataTopic.insert(dataTopic.begin(), '/');

	// Check whether the data topic starts with one of the plot topics we should be logging
	bool shouldLog = false;
	for(std::vector<std::string>::iterator it = m_plotTopics.begin(); it != m_plotTopics.end(); ++it)
	{
		if(dataTopic.compare(0, it->size(), *it) == 0)
		{
			shouldLog = true;
			break;
		}
	}
	if(!shouldLog) return;

	// Write the required data with the current timestamp
	m_bag.write(PLOT_TOPIC, ros::Time::now(), data);
}

// Callback to write tf data to our bag
void RRLogger::writeTfCb(const tf2_msgs::TFMessageConstPtr& data)
{
	// Don't do anything if no bag is open or the logger is paused
	if(!m_bagOpen || m_paused) return;

	// Don't do anything if there are no transforms in the data
	if(data->transforms.empty()) return;

	// Decide whether this transform concerns us
	if(m_tfLogAll)
		m_bag.write(TF_TOPIC, ros::Time::now(), data);
	else
	{
		m_tfMsg.transforms.clear(); // The capacity will normally not decrease here, so memory allocations isn't a problem
		for(std::size_t i = 0; i < data->transforms.size(); i++)
		{
			if(tfIsRelevant(data->transforms[i]))
				m_tfMsg.transforms.push_back(data->transforms[i]);
		}
		m_bag.write(TF_TOPIC, ros::Time::now(), m_tfMsg);
	}
}

// Return whether a tf is relevant given the frames we wish to filter by
bool RRLogger::tfIsRelevant(const geometry_msgs::TransformStamped& tform) const
{
	// Return whether the tf is relevant
	const std::string& childFrame = tform.child_frame_id;
	const std::string& parentFrame = tform.header.frame_id;
	if(childFrame.empty()) return false;
	if(parentFrame.empty()) return false;
	if(std::find(m_tfFrames.begin(), m_tfFrames.end(), (childFrame[0] == '/' ? childFrame.substr(1) : childFrame)) == m_tfFrames.end()) return false;
	if(std::find(m_tfFrames.begin(), m_tfFrames.end(), (parentFrame[0] == '/' ? parentFrame.substr(1) : parentFrame)) == m_tfFrames.end()) return false;
	return true;
}

//
// Heartbeat functions
//

// Reset the heartbeat state
void RRLogger::resetHeartbeat()
{
	// Clear the heartbeat map
	m_heartbeatMap.clear();

	// Update whether we should be logging or not
	updateLoggingState();
}

// Set the heartbeat parameters
void RRLogger::setHeartbeatParam(bool enabled, double timeout, bool optional)
{
	// Set the required heartbeat parameters
	m_heartbeatEnabled = enabled;
	m_heartbeatTimeout = (timeout < 0.1 ? 0.1 : timeout);
	m_heartbeatOptional = optional;

	// Reset the heartbeat state
	resetHeartbeat();
}

// Handle a received heartbeat message
void RRLogger::handleHeartbeat(const LoggerHeartbeatConstPtr& msg)
{
	// Don't do anything if the logger is disabled or heartbeating is disabled
	if(!m_enabled || !m_heartbeatEnabled) return;

	// Reject messages that don't specify a source node
	if(msg->sourceNode.empty())
	{
		ROS_WARN_THROTTLE(1.0, "[%s] Received a heartbeat with an empty sourceNode field => Ignoring...", m_prefix.c_str());
		return;
	}

	// State if this is a new heartbeat source
	HMapType::const_iterator sourceIt = m_heartbeatMap.find(msg->sourceNode);
	if(sourceIt == m_heartbeatMap.end())
		ROS_INFO("[%s] Heartbeat source '%s' says %sto log (NEW)", m_prefix.c_str(), msg->sourceNode.c_str(), (msg->enableLogging ? "" : "NOT "));
	else if(msg->enableLogging != sourceIt->second.second)
		ROS_INFO("[%s] Heartbeat source '%s' says %sto log", m_prefix.c_str(), msg->sourceNode.c_str(), (msg->enableLogging ? "" : "NOT "));

	// Limit the number of heartbeat sources
	if(m_heartbeatMap.size() >= MAX_HEARTBEAT_SOURCES)
	{
		HMapType::iterator end = m_heartbeatMap.end();
		HMapType::iterator oldestTrueIt = end;
		HMapType::iterator oldestFalseIt = end;
		for(HMapType::iterator it = m_heartbeatMap.begin(); it != end; ++it)
		{
			bool enableLogging = it->second.second;
			const ros::Time& stamp = it->second.first;
			if(enableLogging && (oldestTrueIt == end || stamp < oldestTrueIt->second.first)) oldestTrueIt = it;
			if(!enableLogging && (oldestFalseIt == end || stamp < oldestFalseIt->second.first)) oldestFalseIt = it;
		}
		HMapType::iterator discardIt = end;
		if(oldestFalseIt != end)
			discardIt = oldestFalseIt;
		else if(oldestTrueIt != end)
			discardIt = oldestTrueIt;
		if(discardIt != end)
		{
			ROS_WARN("[%s] Reached maximum number %d of maintained heartbeat sources => Discarding old heartbeat source '%s' to make room!", m_prefix.c_str(), (int)MAX_HEARTBEAT_SOURCES, discardIt->first.c_str());
			m_heartbeatMap.erase(discardIt);
		}
	}

	// Enter the received heartbeat into the heartbeat map
	HMapValueType& value = m_heartbeatMap[msg->sourceNode];
	value.first = ros::Time::now();
	value.second = msg->enableLogging;

	// Update whether we should be logging or not
	updateLoggingState();
}

//
// Main function
//

// Custom SIGINT handler
bool g_shutdown = false;
void sigint_handler(int signal)
{
	// Trigger a shutdown
	g_shutdown = true;
}

// Main function
int main(int argc, char** argv)
{
	// Override the default SIGINT handler
	signal(SIGINT, sigint_handler);

	// Initialise ROS
	ros::init(argc, argv, "rrlogger", ros::init_options::NoSigintHandler);

	// Retrieve a ROS node handle
	ros::NodeHandle nh("~");

	// ROS parameter names
	const std::string topicsParam = "topics";
	const std::string plotTopicsParam = "plotTopics";
	const std::string tfFramesParam = "tfFrames";

	// Construct the logger
	RRLogger* rrlogger = new RRLogger();

	// Decide on a logger prefix
	std::string prefix = rrlogger->getPrefix();
	std::string nodeName = ros::this_node::getName();
	std::size_t indexStart = nodeName.find_first_not_of("\\/");
	if(indexStart != std::string::npos)
	{
		std::size_t indexEnd = nodeName.find_first_of("\\/", indexStart);
		if(indexEnd == std::string::npos)
			prefix = nodeName.substr(indexStart);
		else if(indexEnd > indexStart)
			prefix = nodeName.substr(indexStart, indexEnd - indexStart);
	}

	// Retrieve parameters off the parameter server
	int maxNumBagsInt = nh.param<int>("maxNumBags", rrlogger->getMaxNumBags());
	int maxTotalMBInt = nh.param<int>("maxTotalMB", rrlogger->getMaxTotalMB());
	bool heartbeatEnabled = nh.param<bool>("heartbeatEnabled", rrlogger->getHeartbeatEnabled());
	bool heartbeatOptional = nh.param<bool>("heartbeatOptional", rrlogger->getHeartbeatOptional());
	double heartbeatTimeout = nh.param<double>("heartbeatTimeout", rrlogger->getHeartbeatTimeout());

	// Safely convert the required parameters to std::size_t
	std::size_t maxNumBags = (std::size_t) maxNumBagsInt;
	if(maxNumBagsInt < 0)
		maxNumBags = 0;
	std::size_t maxTotalMB = (std::size_t) maxTotalMBInt;
	if(maxTotalMBInt < 0)
		maxTotalMB = 0;

	// Configure the logger
	maxNumBags = rrlogger->configureMaxNumBags(maxNumBags);
	maxTotalMB = rrlogger->configureMaxTotalMB(maxTotalMB);
	prefix = rrlogger->configurePrefix(prefix);
	heartbeatTimeout = rrlogger->configureHeartbeat(heartbeatEnabled, heartbeatTimeout, heartbeatOptional);

	// Inform the user of the configuration of the logger
	if(heartbeatEnabled)
		ROS_INFO("[%s] RRLogger configured with %s heartbeat of timeout %.2fs (max %d bags / %dMB)", prefix.c_str(), (heartbeatOptional ? "an optional" : "a mandatory"), heartbeatTimeout, (int)maxNumBags, (int)maxTotalMB);
	else
		ROS_INFO("[%s] RRLogger configured without a heartbeat (max %d bags / %dMB)", prefix.c_str(), (int)maxNumBags, (int)maxTotalMB);

	// Retrieve the topics and plot topics to log off the parameter server and pass them to the logger
	XmlRpc::XmlRpcValue list;
	if(nh.getParam(topicsParam, list))
	{
		if(list.getType() != XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_ERROR("[%s] The '%s' parameter is invalid (not of type XmlRpcValue::TypeArray) => Exiting...", prefix.c_str(), topicsParam.c_str());
			return -1;
		}
		try
		{
			for(int i = 0; i < list.size(); i++)
			{
				XmlRpc::XmlRpcValue& value = list[i];
				if(value.getType() == XmlRpc::XmlRpcValue::TypeString)
				{
					std::string resolvedTopicName;
					std::string topicName = static_cast<std::string>(value);
					if(topicName.empty())
						ROS_WARN("[%s] Ignored empty topic name!", prefix.c_str());
					else
					{
						try
						{
							// Add the required logger topic
							rrlogger->addTopic(topicName, &resolvedTopicName);
							ROS_INFO("[%s] Added topic: %s", prefix.c_str(), resolvedTopicName.c_str());

							// Configure the required plot topics if this was the plot topic
							if(resolvedTopicName == RRLogger::PLOT_TOPIC)
							{
								bool removePlotSubscriber = false;
								XmlRpc::XmlRpcValue plotList;
								if(nh.getParam(plotTopicsParam, plotList))
								{
									if(plotList.getType() == XmlRpc::XmlRpcValue::TypeArray)
									{
										for(int j = 0; j < plotList.size(); j++)
										{
											XmlRpc::XmlRpcValue& plotValue = plotList[j];
											if(plotValue.getType() == XmlRpc::XmlRpcValue::TypeString)
											{
												std::string resolvedPlotTopicName;
												std::string plotTopicName = static_cast<std::string>(plotValue);
												if(plotTopicName.empty())
													ROS_WARN("[%s]  - <empty> (IGNORED)", prefix.c_str());
												else
												{
													if(rrlogger->addPlotTopic(plotTopicName, &resolvedPlotTopicName))
														ROS_INFO("[%s]  - %s", prefix.c_str(), resolvedPlotTopicName.c_str());
													else
														ROS_WARN("[%s]  - %s (FAILED)", prefix.c_str(), resolvedPlotTopicName.c_str());
												}
											}
											else
											{
												ROS_WARN("[%s] The '%s' parameter contains non-string values!", prefix.c_str(), plotTopicsParam.c_str());
												removePlotSubscriber = true;
												break;
											}
										}
									}
									else
									{
										ROS_WARN("[%s] The '%s' parameter is invalid (not of type XmlRpcValue::TypeArray)!", prefix.c_str(), plotTopicsParam.c_str());
										removePlotSubscriber = true;
									}
								}
								else
								{
									ROS_WARN("[%s] The plot topic '%s' has been specified, but the required '%s' parameter does not exist on the parameter server => Logging ALL plot topics by default...", prefix.c_str(), resolvedTopicName.c_str(), plotTopicsParam.c_str());
									if(rrlogger->addPlotTopic("/"))
										ROS_WARN("[%s]  - /", prefix.c_str());
									else
										ROS_WARN("[%s]  - / (FAILED)", prefix.c_str());
										
								}
								if(removePlotSubscriber)
								{
									rrlogger->removeTopic(resolvedTopicName);
									ROS_WARN("[%s] Removed topic: %s", prefix.c_str(), resolvedTopicName.c_str());
								}
							}

							// Configure the required tf frames if this was the tf topic
							if(resolvedTopicName == RRLogger::TF_TOPIC)
							{
								bool removeTfSubscriber = false;
								XmlRpc::XmlRpcValue tfList;
								if(nh.getParam(tfFramesParam, tfList))
								{
									if(tfList.getType() == XmlRpc::XmlRpcValue::TypeArray)
									{
										for(int j = 0; j < tfList.size(); j++)
										{
											XmlRpc::XmlRpcValue& tfFrameValue = tfList[j];
											if(tfFrameValue.getType() == XmlRpc::XmlRpcValue::TypeString)
											{
												std::string resolvedTfFrame;
												std::string tfFrame = static_cast<std::string>(tfFrameValue);
												if(rrlogger->addTfFrame(tfFrame, &resolvedTfFrame))
													ROS_INFO("[%s]  - %s", prefix.c_str(), (resolvedTfFrame.empty() ? "<all>" : resolvedTfFrame.c_str()));
												else
													ROS_WARN("[%s]  - %s (FAILED)", prefix.c_str(), (resolvedTfFrame.empty() ? "<all>" : resolvedTfFrame.c_str()));
											}
											else
											{
												ROS_WARN("[%s] The '%s' parameter contains non-string values!", prefix.c_str(), tfFramesParam.c_str());
												removeTfSubscriber = true;
												break;
											}
										}
									}
									else
									{
										ROS_WARN("[%s] The '%s' parameter is invalid (not of type XmlRpcValue::TypeArray)!", prefix.c_str(), tfFramesParam.c_str());
										removeTfSubscriber = true;
									}
								}
								else
								{
									ROS_WARN("[%s] The tf topic '%s' has been specified, but the required '%s' parameter does not exist on the parameter server => Logging ALL tf frames by default...", prefix.c_str(), resolvedTopicName.c_str(), tfFramesParam.c_str());
									if(rrlogger->addTfFrame(""))
										ROS_WARN("[%s]  - <all>", prefix.c_str());
									else
										ROS_WARN("[%s]  - <all> (FAILED)", prefix.c_str());
								}
								if(removeTfSubscriber)
								{
									rrlogger->removeTopic(resolvedTopicName);
									ROS_WARN("[%s] Removed topic: %s", prefix.c_str(), resolvedTopicName.c_str());
								}
							}
						}
						catch(std::exception& e)
						{
							ROS_WARN("[%s] %s", prefix.c_str(), e.what());
							ROS_WARN("[%s] Failed to add topic '%s'!", prefix.c_str(), topicName.c_str());
						}
					}
				}
				else
					throw std::runtime_error("The '" + topicsParam + "' parameter contains non-string values!");
			}
		}
		catch(std::exception& e)
		{
			ROS_ERROR("[%s] %s", prefix.c_str(), e.what());
			ROS_ERROR("[%s] Failed to compile the list of topics to log => Exiting...", prefix.c_str());
			return -1;
		}
	}
	else
		ROS_ERROR("[%s] The required '%s' parameter does not exist on the parameter server!", prefix.c_str(), topicsParam.c_str());

	// Error out if there are no topics to log
	if(rrlogger->numTopics() <= 0)
	{
		ROS_ERROR("[%s] No topics have been specified to log => Exiting...", prefix.c_str());
		return -1;
	}

	// Indicate that we have finished the initialisation
	ROS_INFO("[%s] RRLogger initialisation complete", prefix.c_str());

	// Initialise the logger
	rrlogger->init();

	// Run the main ROS loop
	try
	{
		while(ros::ok() && !g_shutdown)
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	}
	catch(rosbag::BagException& e)
	{
		ROS_ERROR("[%s] Caught ROS bag exception: %s => Exiting...", prefix.c_str(), e.what());
		return -1;
	}
	catch(std::exception& e)
	{
		ROS_ERROR("[%s] Caught generic std exception: %s => Exiting...", prefix.c_str(), e.what());
		return -1;
	}

	// Deinitialise the logger
	rrlogger->init(false);

	// Delete the logger
	delete rrlogger;

	// Notification that the logger is shutting down
	ROS_INFO("[%s] RRLogger is exiting cleanly...", prefix.c_str());

	// Shut down ROS explicitly
	ros::shutdown();

	// Return success
	return 0;
}
// EOF