// Walk and kick: Team communications ROS interface
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
// Ensure header is only included once
#ifndef WAK_TC_ROS_H
#define WAK_TC_ROS_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/TeamCommsData.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <vector>

// Walk and kick namespace
namespace walk_and_kick
{
	// Class declarations
	class TCRobotListener;

	// Typedefs
	typedef boost::shared_ptr<const TCRobotListener> TCRobotListenerConstPtr;
	typedef std::vector<TCRobotListenerConstPtr> TCRobotListenerList;

	/**
	* @class TCRosInterface
	* 
	* @brief An interface class to the ROS world for team communications.
	**/
	class TCRosInterface
	{
	public:
		// Constructor
		TCRosInterface() : m_nh("~") {}
		explicit TCRosInterface(ros::NodeHandle& nh) : m_nh(nh) {}

		// Reset function
		void resetData();

		// Start/stop listening functions
		void startListening(bool listenToSelf = false);
		void stopListening();

		// Get functions
		const TCRobotListenerList& teamComms() const { return m_teamComms; }
		unsigned int nextRobotUID() const { return m_teamComms.size(); }

	private:
		// ROS node handle
		ros::NodeHandle m_nh;

		// List of robot team communications listeners
		typedef boost::shared_ptr<TCRobotListener> TCRobotListenerPtr;
		typedef std::vector<TCRobotListenerPtr> TCRobotListenerListInternal;
		TCRobotListenerListInternal m_teamCommsInternal;
		TCRobotListenerList m_teamComms;
	};

	/**
	* @class TCRobotListener
	* 
	* @brief A class that listens to the team communications data of one particular robot.
	**/
	class TCRobotListener
	{
	public:
		// Constructor/destructor
		TCRobotListener(ros::NodeHandle& nh, const std::string& robot, unsigned int robotUID) : robot(robot), robotUID(robotUID), m_lastEmbeddedStamp(0, 0) { m_subscriber = nh.subscribe("/remote/" + robot + "/walk_and_kick/teamPacket", 1, &TCRobotListener::handleTeamPacket, this); }
		virtual ~TCRobotListener() { m_subscriber.shutdown(); }

		// Reset function
		void resetData() { m_data = TeamCommsData(); m_lastEmbeddedStamp = ros::Time(); }

		// Robot name and UID
		const std::string robot;
		const unsigned int robotUID;

		// Get functions
		const TeamCommsData& data() const { return m_data; }

	private:
		// Last received team communications data
		TeamCommsData m_data;

		// ROS subscriber
		ros::Subscriber m_subscriber;
		void handleTeamPacket(const TeamCommsDataConstPtr& msg);

		// Embedded time stamp of the last received data
		ros::Time m_lastEmbeddedStamp;
	};
}

#endif
// EOF