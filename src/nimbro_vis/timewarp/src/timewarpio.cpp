// Provides functionality for loading and saving timewarp data to a bag file
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <timewarp/timewarpio.h>
#include <timewarp/timewarpnode.h>
#include <timewarp/topichandler.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//
// TimeWarpIO class
//

// Function to load timewarp data from a bag
bool timewarp::TimeWarpIO::load(timewarp::TimeWarpNode* tw, const std::string& path)
{
	// Warp prefix as a string
	const std::string prefix(WARP_PREFIX "/");

	// Try to open the bag for reading
	rosbag::Bag bag;
	try { bag.open(path, rosbag::bagmode::Read); }
	catch(rosbag::BagException& e)
	{
		ROS_ERROR("Failed to open bag file: %s", e.what());
		return false;
	}

	// Clear all current timewarp data
	tw->clear();

	// Message data type strings
	const char* dataTypeMarker = ros::message_traits::DataType<visualization_msgs::Marker>::value();
	const char* dataTypeMarkerArray = ros::message_traits::DataType<visualization_msgs::MarkerArray>::value();

	// Retrieve a list of all connections in the bag
	rosbag::View viewConn(bag);
	std::vector<const rosbag::ConnectionInfo*> connections = viewConn.getConnections();

	// Construct a list of topics in the bag that should be loaded into timewarp
	bool haveTfTopic = false;
	std::vector<std::string> handlerTopics;
	for(std::vector<const rosbag::ConnectionInfo*>::iterator it = connections.begin(); it != connections.end(); it++)
	{
		// Retrieve the connection topic name
		const std::string& topicName = (*it)->topic;

		// Ignore tf topics as they are handled separately
		if(topicName == WARP_TF_TOPIC) continue;

		// Handle topics that have the warp prefix
		if(topicName.compare(0, prefix.length(), prefix) == 0) // If the topic name starts with the warp prefix...
		{
			std::string topicNameNoPrefix = topicName.substr(prefix.length() - 1);
			if(topicNameNoPrefix.compare(0, prefix.length(), prefix) == 0) continue; // Ignore topics with multiple warp prefixes as they can cause trouble
			if(std::find(handlerTopics.begin(), handlerTopics.end(), topicName) == handlerTopics.end())
				handlerTopics.push_back(topicName);
		}
	}
	for(std::vector<const rosbag::ConnectionInfo*>::iterator it = connections.begin(); it != connections.end(); it++)
	{
		// Retrieve the connection topic name
		const std::string& topicName = (*it)->topic;

		// Ignore tf topics as they are handled separately
		if(topicName == TF_TOPIC)
		{
			haveTfTopic = true;
			continue; 
		}

		// Handle topics that do not have the warp prefix
		if(topicName.compare(0, prefix.length(), prefix) != 0) // If the topic name does not start with the warp prefix...
		{
			const std::string& dataType = (*it)->datatype; // Retrieve the connection data type
			bool isGoodType = (dataType.compare(dataTypeMarker) == 0 || dataType.compare(dataTypeMarkerArray) == 0); // Topics of this type are automatically loaded into timewarp
			std::vector<std::string>::iterator equivWarpTopic = std::find(handlerTopics.begin(), handlerTopics.end(), WARP_PREFIX + topicName);
			bool hasEquivWarp = (equivWarpTopic != handlerTopics.end()); // Topics for which a version with a warp prefix exists are taken and used instead of the warp prefix version
			bool isSpecial = (topicName == JS_TOPIC || topicName == JC_TOPIC);
			for(std::size_t i = 0; i < tw->m_extraTopics.size(); ++i)
			{
				if(topicName == tw->m_extraTopics[i])
				{
					isSpecial = true;
					break;
				}
			}
			if(isGoodType || hasEquivWarp || isSpecial)
			{
				if(hasEquivWarp)
					handlerTopics.erase(equivWarpTopic);
				if(std::find(handlerTopics.begin(), handlerTopics.end(), topicName) == handlerTopics.end())
					handlerTopics.push_back(topicName);
			}
		}
	}

	// Read tf messages from the bag
	rosbag::View viewTF(bag, rosbag::TopicQuery(haveTfTopic ? TF_TOPIC : WARP_TF_TOPIC));
	for(rosbag::View::iterator it = viewTF.begin(); it != viewTF.end(); it++)
	{
		tf2_msgs::TFMessage::ConstPtr msg = it->instantiate<tf2_msgs::TFMessage>();
		if(!msg) continue;
		tw->m_tfHandler.processMsg(*msg);
	}

	// Initialise variables for the loop
	ros::Time earliest(0, 0);
	ros::Time latest(0, 0);
	rosbag::Buffer buf;

	// Read topic handler messages from the bag
	rosbag::View viewTopic(bag, rosbag::TopicQuery(handlerTopics));
	for(rosbag::View::iterator it = viewTopic.begin(); it != viewTopic.end(); it++)
	{
		// Retrieve the serialised data from the MessageInstance pointed to by the iterator
		uint32_t serialSize = it->size();
		buf.setSize(serialSize);
		ros::serialization::OStream ostream(buf.getData(), buf.getSize());
		it->write(ostream);

		// Populate a topic handler entry with the message data
		TopicHandler::Entry entry;
		entry.captureTime = it->getTime();
		entry.data = boost::make_shared<topic_tools::ShapeShifter>();
		ros::serialization::IStream istream(buf.getData(), buf.getSize());
		entry.data->read(istream);
		entry.data->morph(it->getMD5Sum(), it->getDataType(), it->getMessageDefinition(), (it->isLatching() ? "1" : "0"));

		// Find the topic handler corresponding to the topic that the message belongs to
		std::string topicName = it->getTopic();
		if(topicName.compare(0, prefix.length(), prefix) == 0)
			topicName = topicName.substr(prefix.length() - 1);
		int index = tw->findHandler(topicName);
		if(index < 0)
		{
			index = (int) tw->m_handlers.size();
			std::string typeStr = it->getDataType();
			if(typeStr.compare(dataTypeMarker) == 0)
				tw->m_handlers.push_back(new SmartTopicHandler<visualization_msgs::Marker>(&tw->m_nh, topicName));
			else if(typeStr.compare(dataTypeMarkerArray) == 0)
				tw->m_handlers.push_back(new SmartTopicHandler<visualization_msgs::MarkerArray>(&tw->m_nh, topicName));
			else
				tw->m_handlers.push_back(new TopicHandler(&tw->m_nh, topicName)); // The downside of a normal topic handler is that header time stamps (more accurate) can't be inferred, and only the one for when the message was published is used.
			ROS_INFO("Created timewarp handler for topic '%s'", topicName.c_str());
		}

		// Add the message to the circular buffer of the required topic handler
		tw->m_handlers[index]->m_buf.push_back(entry);

		// Make a note of the ROS time bounds
		if(earliest.isZero() || entry.captureTime < earliest)
			earliest = entry.captureTime;
		if(latest.isZero() || entry.captureTime > latest)
			latest = entry.captureTime;
	}

	// Ensure all the topic handler publishers are initialised
	for(std::vector<timewarp::TopicHandler*>::iterator it = tw->m_handlers.begin(); it != tw->m_handlers.end(); it++)
		(*it)->initPublisher();

	// Close the bag
	bag.close();

	// Message about what we did
	ROS_INFO("Loaded timewarp data of time [%.3f,%.3f] of duration %.3fs to '%s'", (earliest.isZero() ? -INFINITY : earliest.toSec()), (latest.isZero() ? INFINITY : latest.toSec()), (latest - earliest).toSec(), path.c_str());

	// Return success
	return true;
}

// Function to save timewarp to a bag
bool timewarp::TimeWarpIO::save(const timewarp::TimeWarpNode* tw, const std::string& path, bool append, ros::Time startTime, ros::Time stopTime)
{
	// Handle zero start/stop times
	bool zeroStart = startTime.isZero();
	bool zeroStop = stopTime.isZero();

	// Try to open the bag for writing/appending as appropriate
	rosbag::Bag bag;
	try { bag.open(path, (append ? rosbag::bagmode::Append : rosbag::bagmode::Write)); }
	catch(rosbag::BagException& e)
	{
		ROS_ERROR("Failed to open bag file: %s", e.what());
		return false;
	}

	// Initialise variables
	ros::Time earliest(0, 0);
	ros::Time latest(0, 0);

	// Write the tf messages into the bag
	for(std::list<tf2_msgs::TFMessage>::const_iterator it = tw->m_tfHandler.m_rawTFLog.begin(); it != tw->m_tfHandler.m_rawTFLog.end(); it++)
	{
		bool inRange = false;
		ros::Time stamp;
		for(tf2_msgs::TFMessage::_transforms_type::const_iterator itt = it->transforms.begin(); itt != it->transforms.end(); itt++)
		{
			if((zeroStart || itt->header.stamp >= startTime) && (zeroStop || itt->header.stamp <= stopTime))
			{
				stamp = itt->header.stamp;
				inRange = true;
				break;
			}
		}
		if(inRange)
		{
			bag.write(WARP_TF_TOPIC, stamp, *it);
			if(earliest.isZero() || stamp < earliest)
				earliest = stamp;
			if(latest.isZero() || stamp > latest)
				latest = stamp;
		}
	}

	// Write the topic handler messages to the bag
	for(std::vector<TopicHandler*>::const_iterator it = tw->m_handlers.begin(); it != tw->m_handlers.end(); it++)
	{
		TopicHandler* TH = *it;
		if(!TH) continue;

		std::string name = TH->name();
		if(name.empty()) continue;

		TopicHandler::CircBuf::const_iterator first, last, itbuf;
		first = TH->m_buf.begin();
		last = TH->m_buf.end();
		for(itbuf = first; itbuf != last; itbuf++)
		{
			const ros::Time& stamp = (*itbuf).captureTime;
			if((zeroStart || stamp >= startTime) && (zeroStop || stamp <= stopTime))
			{
				bag.write(WARP_PREFIX + name, stamp, (*itbuf).data);
				if(earliest.isZero() || stamp < earliest)
					earliest = stamp;
				if(latest.isZero() || stamp > latest)
					latest = stamp;
			}
		}
	}

	// Close the bag
	bag.close();

	// Message about what we did
	ROS_INFO("Saved timewarp data of time [%.3f,%.3f] of duration %.3fs to '%s'", (earliest.isZero() ? -INFINITY : earliest.toSec()), (latest.isZero() ? INFINITY : latest.toSec()), (latest - earliest).toSec(), path.c_str());

	// Return success
	return true;
}
// EOF