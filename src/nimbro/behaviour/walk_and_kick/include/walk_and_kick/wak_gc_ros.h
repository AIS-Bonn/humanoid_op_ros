// Walk and kick: Game controller ROS interface
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
// Ensure header is only included once
#ifndef WAK_GC_ROS_H
#define WAK_GC_ROS_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <rcup_game_controller/GCData.h>
#include <ros/ros.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class GCRosInterface
	* 
	* @brief An interface class to the ROS world for getting game controller messages.
	**/
	class GCRosInterface
	{
	public:
		// Constructor
		GCRosInterface() : m_nh("~"), m_enabled(true) { resetData(); }
		explicit GCRosInterface(ros::NodeHandle& nh) : m_nh(nh), m_enabled(true) { resetData(); }
		virtual ~GCRosInterface() { stopListening(); }

		// Reset function
		void resetData() { m_data = rcup_game_controller::GCData(); }

		// Start/stop listening functions
		void startListening() { stopListening(); m_subscriber = m_nh.subscribe("/game_controller/data", 1, &GCRosInterface::handleGameControllerData, this); }
		void stopListening() { m_subscriber.shutdown(); }

		// Enable/disable game controller data (without unsubscribing to the topic)
		bool getEnabled() const { return m_enabled; }
		void setEnabled(bool enabled) { m_enabled = enabled; }

		// Get functions
		const rcup_game_controller::GCData& data() const { return m_data; }

	private:
		// ROS node handle
		ros::NodeHandle m_nh;

		// Enabled state of class
		bool m_enabled;

		// Last received game controller data
		rcup_game_controller::GCData m_data;

		// ROS subscriber
		ros::Subscriber m_subscriber;
		void handleGameControllerData(const rcup_game_controller::GCDataConstPtr& msg) { if(m_enabled) m_data = *msg; } // Note: Any game controller packet that is received here definitely concerns us as it has already been preprocessed into 'ownTeam' and 'oppTeam' by the rcup_game_controller node
	};
}

#endif
// EOF