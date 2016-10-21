// Maintain a round robin of logging ROS bags
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schüller <schuell1@cs.uni-bonn.de>

// Ensure header is only included once
#ifndef RRLOGGER_H
#define RRLOGGER_H

// Includes
#include <topic_tools/shape_shifter.h>
#include <rrlogger/LoggerHeartbeat.h>
#include <config_server/parameter.h>
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#include <tf2_msgs/TFMessage.h>
#include <plot_msgs/Plot.h>
#include <rosbag/bag.h>
#include <algorithm>

// Round robin logger namespace
namespace rrlogger
{
	// Round robin logger class
	class RRLogger
	{
	public:
		// Constants
		static const std::string PLOT_TOPIC;
		static const std::string TF_TOPIC;

		// Constructor/destructor
		RRLogger();
		virtual ~RRLogger();

		// Reset functions
		void reset();
		void resetConfig();

		// Initialisation function
		void init(bool init = true);

		// Configuration set functions
		std::size_t configureMaxNumBags(std::size_t maxNumBags) { m_maxNumBags = (maxNumBags > MAX_MAX_NUM_BAGS ? MAX_MAX_NUM_BAGS : (maxNumBags < 1 ? MAX_MAX_NUM_BAGS : maxNumBags)); return m_maxNumBags; } // Note: Passing 0 sets the limit to MAX_MAX_NUM_BAGS
		std::size_t configureMaxTotalMB(std::size_t maxTotalMB) { m_maxTotalMB = (maxTotalMB > MAX_MAX_TOTAL_MB ? MAX_MAX_TOTAL_MB : (maxTotalMB < 1 ? MAX_MAX_TOTAL_MB : maxTotalMB)); return m_maxTotalMB; } // Note: Passing 0 sets the limit to MAX_MAX_TOTAL_MB
		std::string configurePrefix(const std::string& prefix) { m_prefix = (prefix.empty() ? "rrlogger" : prefix); return m_prefix; }
		double configureHeartbeat(bool enabled, double timeout, bool optional = false) { setHeartbeatParam(enabled, timeout, optional); return m_heartbeatTimeout; }

		// Configuration get functions
		std::size_t getMaxNumBags() const { return m_maxNumBags; }
		std::size_t getMaxTotalMB() const { return m_maxTotalMB; }
		std::string getPrefix() const { return m_prefix; }
		bool getHeartbeatEnabled() const { return m_heartbeatEnabled; }
		bool getHeartbeatOptional() const { return m_heartbeatOptional; }
		double getHeartbeatTimeout() const { return m_heartbeatTimeout; }

		// Topic management functions
		void addTopic(const std::string& name, std::string* resolvedName = NULL); // Note: Throws std::runtime_error if something goes wrong
		bool removeTopic(const std::string& resolvedName);
		bool haveTopic(const std::string& resolvedName) const;
		void clearTopics();
		std::size_t numTopics() const { return m_topics.size(); }
		std::string getTopic(std::size_t index) const { if(index < m_topics.size()) return m_topics[index].getTopic(); else return std::string(); }

		// Plot topic management functions
		bool addPlotTopic(const std::string& name, std::string* resolvedName = NULL);
		bool removePlotTopic(const std::string& resolvedName);
		bool havePlotTopic(const std::string& resolvedName) const { return (std::find(m_plotTopics.begin(), m_plotTopics.end(), resolvedName) != m_plotTopics.end()); }
		void clearPlotTopics() { m_plotTopics.clear(); }
		std::size_t numPlotTopics() const { return m_plotTopics.size(); }
		std::string getPlotTopic(std::size_t index) const { if(index < m_plotTopics.size()) return m_plotTopics[index]; else return std::string(); }

		// Tf topic management functions
		bool addTfFrame(const std::string& name, std::string* resolvedName = NULL);
		bool removeTfFrame(const std::string& resolvedName);
		bool haveTfFrame(const std::string& resolvedName) const { return (std::find(m_tfFrames.begin(), m_tfFrames.end(), resolvedName) != m_tfFrames.end()); }
		void clearTfFrames() { m_tfFrames.clear(); m_tfLogAll = false; }
		std::size_t numTfFrames() const { return m_tfFrames.size(); }
		std::string getTfFrame(std::size_t index) const { if(index < m_tfFrames.size()) return m_tfFrames[index]; else return std::string(); }

	private:
		// Constants
		static const std::string LOG_PATH;
		static const int TOPIC_QUEUE_SIZE;
		static const std::size_t MAX_MAX_NUM_BAGS;
		static const std::size_t MAX_MAX_TOTAL_MB;
		static const std::size_t WARN_BAGS_NUM;
		static const double WARN_BAGS_SEC;
		static const std::size_t MAX_HEARTBEAT_SOURCES;

		// Node handle
		ros::NodeHandle m_nh;

		// Config server parameters
		config_server::Parameter<bool> m_confEnable;
		config_server::Parameter<bool> m_confPause;
		config_server::Parameter<bool> m_confStartNewBag;
		config_server::Parameter<bool> m_confForceHeartbeat;
		config_server::Parameter<bool> m_confKeepAllBags;
		config_server::Parameter<bool> m_confKeepNoBags;

		// Config server parameter callbacks
		void handleConfEnable() { updateEnabled(); }
		void handleConfPause() { updatePaused(); }
		void handleConfStartNewBag();
		void handleConfForceHeartbeat() { updateLoggingState(); }

		// Logging state
		bool m_inited;  // Set exclusively by init()
		bool m_enabled; // Set exclusively by updateEnabled()
		bool m_started; // Set exclusively by startLogging() and stopLogging()
		bool m_bagOpen; // Set exclusively by openBag() and closeBag()
		bool m_paused;  // Set exclusively by updatePaused()

		// Get functions for logging state
		bool inited() const { return m_inited; }
		bool enabled() const { return m_enabled; }
		bool started() const { return m_bagOpen; }
		bool paused() const { return m_paused; }
		bool running() const { return (m_bagOpen && !m_paused); }

		// Update functions
		void updateEnabled();
		void updatePaused(bool showInfo = false);
		void updateLoggingState();

		// Logging control
		bool startLogging();
		void stopLogging();

		// Bag I/O functions
		bool openBag();
		void closeBag();
		void writeCb(const boost::shared_ptr<topic_tools::ShapeShifter>& data, const std::string& topic);
		void writePlotCb(const plot_msgs::PlotConstPtr& data);
		void writeTfCb(const tf2_msgs::TFMessageConstPtr& data);
		bool tfIsRelevant(const geometry_msgs::TransformStamped& tform) const;

		// Bag timestamp buffer
		typedef boost::circular_buffer<ros::Time> RosTimeBuffer;
		RosTimeBuffer m_bagStamps;

		// Heartbeat functions
		void resetHeartbeat();
		void setHeartbeatParam(bool enabled, double timeout, bool optional);

		// Heartbeat subscriber
		ros::Subscriber m_sub_heartbeat;
		void handleHeartbeat(const LoggerHeartbeatConstPtr& msg);

		// Heartbeat timeouts
		ros::Timer m_heartbeatTimer;
		void handleHeartbeatTimer(const ros::TimerEvent& event) { updateLoggingState(); }

		// Heartbeat map
		typedef std::pair<ros::Time, bool> HMapValueType;
		typedef std::map<std::string, HMapValueType> HMapType;
		HMapType m_heartbeatMap;

		// Logger configuration
		std::size_t m_maxNumBags;
		std::size_t m_maxTotalMB;
		std::string m_prefix;
		bool m_heartbeatEnabled;   // Set exclusively by setHeartbeatParam()
		bool m_heartbeatOptional;  // Set exclusively by setHeartbeatParam()
		double m_heartbeatTimeout; // Set exclusively by setHeartbeatParam()

		// Logger topics
		std::vector<ros::Subscriber> m_topics;

		// Plot topics
		std::vector<std::string> m_plotTopics;

		// Tf frames
		std::vector<std::string> m_tfFrames;
		bool m_tfLogAll;
		tf2_msgs::TFMessage m_tfMsg;

		// ROS bag
		rosbag::Bag m_bag;

		// Miscellaneous
		bool m_showEnabledState;
	};
}

#endif
// EOF