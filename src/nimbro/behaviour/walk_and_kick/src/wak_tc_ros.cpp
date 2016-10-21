// Walk and kick: Team communications ROS interface
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_tc_ros.h>
#include <boost/make_shared.hpp>
#include <XmlRpcValue.h>

// Namespaces
using namespace walk_and_kick;

//
// TCRosInterface class
//

// Reset function
void TCRosInterface::resetData()
{
	// Reset the data of all the robot listeners
	for(std::size_t i = 0; i < m_teamCommsInternal.size(); i++)
		m_teamCommsInternal[i]->resetData();
}

// Start listening to team communications
void TCRosInterface::startListening(bool listenToSelf)
{
	// Clear the current list of robot team communications listeners
	m_teamCommsInternal.clear();
	m_teamComms.clear();

	// Retrieve our robot name
	std::string robotName = m_nh.param<std::string>("/robot_name", std::string());
	if(robotName.empty())
	{
		if(listenToSelf)
			ROS_ERROR("The '/robot_name' ROS parameter is empty => I don't know who I am!");
		else
			ROS_ERROR("The '/robot_name' ROS parameter is empty => I don't know who I am, so I will subscribe to all team communications!");
	}

	// Retrieve the list of robots from the parameter server and create subscribers for their respective team packets
	XmlRpc::XmlRpcValue list;
	bool teamCommsOk = false;
	if(m_nh.getParam("/robots", list))
	{
		if(list.getType() != XmlRpc::XmlRpcValue::TypeArray)
			ROS_ERROR("The '/robots' ROS parameter is not of type XmlRpcValue::TypeArray!");
		else
		{
			try
			{
				for(int i = 0; i < list.size(); i++)
				{
					XmlRpc::XmlRpcValue& value = list[i];
					if(value.getType() == XmlRpc::XmlRpcValue::TypeString)
					{
						std::string name = static_cast<std::string>(value);
						if(name.empty() || (!listenToSelf && name == robotName)) continue;
						ROS_INFO("Subscribing to team communications from: %s", name.c_str());
						TCRobotListenerPtr listener = boost::make_shared<TCRobotListener>(boost::ref(m_nh), name, (unsigned int) m_teamCommsInternal.size());
						m_teamCommsInternal.push_back(listener);
						m_teamComms.push_back(listener);
					}
					else
						ROS_ERROR("The '/robots' ROS parameter array contains non-string values!");
				}
				teamCommsOk = true;
			}
			catch(std::exception& e) { ROS_ERROR("Failed to process the '/robots' ROS parameter: %s", e.what()); }
		}
	}
	else
		ROS_ERROR("Failed to retrieve the list of robots from the '/robots' ROS parameter!");
	if(!teamCommsOk)
		ROS_ERROR("Failed to subscribe to the appropriate team communications topics!");
}

// Stop listening to team communications
void TCRosInterface::stopListening()
{
	// Clear the current list of robot team communications listeners
	m_teamCommsInternal.clear();
	m_teamComms.clear();
}

//
// TCRobotListener class
//

// Handle a new team communications packet
void TCRobotListener::handleTeamPacket(const TeamCommsDataConstPtr& msg)
{
	// Only accept the packet if the source time stamp has increased over what we have last heard, and if the packet ID has changed
	if(msg->timestamp >= m_lastEmbeddedStamp && msg->packetID != m_data.packetID)
	{
		m_lastEmbeddedStamp = msg->timestamp;
		ros::Time now = ros::Time::now();
		double timeDiff = (msg->timestamp - now).toSec();
		if(timeDiff < -10.0 || timeDiff > 5.0)
			ROS_WARN_THROTTLE(10.0, "A team packet received from the %s has a timestamp of %+.1fs relative to the current local time => Bad network or a time sync problem?", robot.c_str(), timeDiff);
		m_data = *msg;
		m_data.timestamp = now; // Override the timestamp with our local ROS time...
	}
}
// EOF