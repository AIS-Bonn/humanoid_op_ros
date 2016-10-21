// Interface to receive Robocup game control data
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schüller <schuell1@cs.uni-bonn.de>
//         Hafez Farazi <farazi@ais.uni-bonn.de>

// Includes
#include <rcup_game_controller/game_controller.h>
#include <rcup_game_controller/packet_parser_v8.h>
#include <boost/make_shared.hpp>
#include <inttypes.h>
#include <ros/ros.h>
#include <cstdlib>

// Defines
#define LOOP_WAIT_TIME_MS           300
#define NETWORK_WARNING_TIME        20.0
#define NETWORK_WARNING_TIME_FIRST  6.0

// Namespaces
using namespace rcup_game_controller;

//
// GameController class
//

// Constructor
GameController::GameController()
{
	// Create a ROS node handle
	ros::NodeHandle nh("~");

	// Activate silent mode (no return packets) if this is not a real robot
	std::string robotName;
	nh.param<std::string>("/robot_name", robotName, std::string());
	m_silentMode = (robotName == "xs0");
	if(m_silentMode)
		ROS_WARN("[GameController] Not a real robot (%s) => Not sending any return packets to the game controller...", robotName.c_str());

	// Load the required packet parsers
	m_parsers.push_back(boost::make_shared<PacketParserV8>(config));

	// Initialise the packet parser response packets
	for(std::size_t i = 0; i < m_parsers.size(); i++)
		m_parsers[i]->initGCResponse();

	// Initialise the published data message
	m_msg = boost::make_shared<GCData>();
	for(std::size_t i = 0; i < m_parsers.size(); i++)
		m_parsers[i]->initGCRosData(m_msg);

	// Advertise the game controller data topic
	m_pub_gc = nh.advertise<rcup_game_controller::GCData>("data", 1);

	// Allocate memory for the received packet buffer
	m_recvBufSize = 0;
	for(std::size_t i = 0; i < m_parsers.size(); i++)
		m_recvBufSize = std::max(m_parsers[i]->recvPacketSize(), m_recvBufSize);
	if(m_recvBufSize > 0)
		m_recvBuf = std::malloc(m_recvBufSize);
	else
		m_recvBuf = NULL;
	m_recvPacketSize = -1;
	m_haveHadPacket = false;

	// Initialise network connection variables
	m_socket = -1;
	memset(&m_ret_addr, 0, sizeof(m_ret_addr));
	m_ret_addr_len = sizeof(m_ret_addr);

	// Initialise the configuration parameters
	m_enabled = true;
	m_lastServerIP.clear();
	config.useLastServerIP.set(false);
	config.useLastServerIP.setCallback(boost::bind(&GameController::handleUseLastServerIP, this));
	config.enableGameController.setCallback(boost::bind(&GameController::handleEnableGameController, this), true);
}

// Initialisation function
bool GameController::init()
{
	// There must be at least one parser
	if(m_parsers.empty())
	{
		ROS_ERROR("[GameController] No packet parsers have been specified or loaded!");
		return false;
	}

	// Do nothing if we have no incoming packet buffer
	if(!m_recvBuf || m_recvBufSize == 0)
	{
		ROS_ERROR("[GameController] Failed to create buffer for incoming packets, or it is of zero size!");
		return false;
	}

	// Initialise the network connection
	if(!initNetwork())
		return false;

	// Initialise the packet received variables
	resetPacketReceived();

	// Return success
	return true;
}

// Initialise the network connection
bool GameController::initNetwork()
{
	// Configure the required internet socket address
	sockaddr_in6 addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin6_family = AF_INET;
	addr.sin6_port = htons(GAMECONTROLLER_PORT);
	addr.sin6_addr = in6addr_any;

	// Create a new socket
	m_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if(m_socket < 0)
	{
		ROS_ERROR("[GameController] Creating socket failed: %s", strerror(errno));
		return false;
	}

	// Give the new socket the required local address
	int bindErr = bind(m_socket, (sockaddr*) &addr, sizeof(sockaddr_in6));
	if(bindErr != 0)
	{
		ROS_ERROR("[GameController] Binding socket failed with %d: %s", bindErr, strerror(errno));
		close(m_socket);
		return false;
	}

	// Indicate that the socket was successfully initialised
	ROS_INFO("[GameController] Initialised UDP socket and listening on port %d", ntohs(addr.sin6_port));

	// Return success
	return true;
}

// Listen for a packet from the game controller
bool GameController::listenForPacket(int msTimeout)
{
	// Get the current wall time
	ros::WallTime now = ros::WallTime::now();

	// Initialise the select timeout and what it should wait for
	timeval timeout;
	int usTimeout = msTimeout * 1000;
	timeout.tv_sec = usTimeout / 1000000;
	timeout.tv_usec = usTimeout % 1000000;
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(m_socket, &rfds);

	// Wait for an arriving packet
	int req = select(m_socket + 1, &rfds, 0, 0, &timeout);
	if(req == 0)
	{
		if(m_enabled)
		{
			double timeSinceWarning = (now - m_lastPacketWarnTime).toSec();
			if(timeSinceWarning >= NETWORK_WARNING_TIME || (!m_haveHadPacket && timeSinceWarning >= NETWORK_WARNING_TIME_FIRST))
			{
				double timeSincePacket = (now - m_lastPacketWallTime).toSec();
				ROS_WARN("[GameController] Haven't received any packet at all for the past %.1fs => Check that I am on the network at all, and that the game controller is even running?", timeSincePacket);
				m_lastPacketWarnTime = now;
			}
		}
		return false;
	}
	else if(req < 0)
	{
		if(errno != EINTR) // To avoid an error message when the game controller is sent a signal to terminate
			ROS_ERROR("[GameController] Function select() failed: %s", strerror(errno));
		return false;
	}
	m_lastPacketWallTime = now;
	m_lastPacketWarnTime = now;
	m_haveHadPacket = true;

	// Retrieve the arrived packet into our local buffer
	m_ret_addr_len = sizeof(m_ret_addr);
	m_recvPacketSize = recvfrom(m_socket, m_recvBuf, m_recvBufSize, 0, (sockaddr*) &m_ret_addr, &m_ret_addr_len);
	if(!m_enabled)
		return false;
	if(m_recvPacketSize < 0)
	{
		ROS_ERROR("[GameController] Receiving packet failed: %s", strerror(errno));
		return false;
	}
	if((std::size_t) m_recvPacketSize > m_recvBufSize)
	{
		ROS_ERROR("[GameController] An oversize packet of size %d bytes was received and discarded!", m_recvPacketSize);
		return false;
	}

	// Check by IP whether the sender of the packet is our known server
	bool senderIsKnown = false;
	char ipAddress[INET6_ADDRSTRLEN] = {0};
	if(m_ret_addr_len == sizeof(sockaddr_in))
	{
		sockaddr_in* in_addr = (sockaddr_in*) &m_ret_addr;
		inet_ntop(AF_INET, &(in_addr->sin_addr), ipAddress, INET6_ADDRSTRLEN);
	}
	else if(m_ret_addr_len == sizeof(sockaddr_in6))
	{
		sockaddr_in6* in_addr6 = (sockaddr_in6*) &m_ret_addr;
		inet_ntop(AF_INET, &(in_addr6->sin6_addr), ipAddress, INET6_ADDRSTRLEN);
	}
	if(strlen(ipAddress) > 0)
	{
		m_lastServerIP = ipAddress;
		if(config.serverIP().compare(ipAddress) == 0)
			senderIsKnown = true;
	}

	// Ignore packets from unknown senders
	if(!senderIsKnown)
	{
		ROS_WARN_THROTTLE(5.0, "[GameController] Ignoring packets from unknown sender '%s'", ipAddress);
		return false;
	}

	// Indicate to the user that packets are arriving
	ROS_INFO_THROTTLE(30.0, "[GameController] Receiving/parsing packets from sender '%s'", ipAddress);

	// Return that a packet was successfully received and is ready for processing
	return true;
}

// Process a received packet and send the appropriate response
bool GameController::processPacket()
{
	// No-one can parse a non-existent packet
	if(m_recvPacketSize <= 0)
	{
		ROS_WARN("[GameController] Rejected an invalid packet of zero size!");
		return false;
	}
	std::size_t recvPacketSize = (std::size_t) m_recvPacketSize;

	// Warn and return false if no parsers are enabled
	bool someParserEnabled = false;
	for(std::size_t i = 0; i < m_parsers.size(); i++)
	{
		if(m_parsers[i]->enabled())
		{
			someParserEnabled = true;
			break;
		}
	}
	if(!someParserEnabled)
	{
		ROS_WARN_THROTTLE(5.0, "[GameController] Receiving packets, but none of the parsers are enabled!");
		return false;
	}

	// Go through the package parsers in turn and see whether any of them can parse the packet
	for(std::size_t i = 0; i < m_parsers.size(); i++)
	{
		const PacketParserPtr& parser = m_parsers[i];
		if(parser->recvPacketSize() != recvPacketSize) continue;
		if(parser->canParsePacket(m_recvBuf, recvPacketSize))
		{
			// We have a parser for this packet, but it is not enabled, so silently ignore the packet
			if(!parser->enabled())
				return false;

			// Parse the packet
			if(parser->parsePacket(m_recvBuf, recvPacketSize, m_msg))
			{
				// Attempt to send a response packet
				if(!m_silentMode)
				{
					std::size_t respPacketSize = 0;
					const void* respPacket = parser->getGCResponse(respPacketSize);
					if(!respPacket || respPacketSize <= 0)
						ROS_ERROR("[GameController] A parser parsed a packet but failed to construct a valid response packet!");
					else
					{
						// Set the correct target port of the gamecontroller (the IP is already set to the one that the packet was received from)
						if(m_ret_addr_len == sizeof(sockaddr_in))
						{
							sockaddr_in* in_addr = (sockaddr_in*) &m_ret_addr;
							in_addr->sin_port = htons(GAMECONTROLLER_PORT);
						}
						else
						{
							sockaddr_in6* in_addr6 = (sockaddr_in6*) &m_ret_addr;
							in_addr6->sin6_port = htons(GAMECONTROLLER_PORT);
						}

						// Send off the required response packet
						if(sendto(m_socket, respPacket, respPacketSize, 0, (sockaddr*) &m_ret_addr, m_ret_addr_len) < 0)
							ROS_ERROR("[GameController] Sending packet failed: %s", strerror(errno));
					}
				}

				// Return that the packet was successfully parsed (even if the response packet may have failed)
				return true;
			}
			else
			{
				// We have a parser for this packet, but the parse failed, so silently ignore the packet (the parser itself should have displayed any appropriate error message)
				return false;
			}
		}
	}

	// Display a warning that this packet has been rejected
	if(recvPacketSize < 8)
		ROS_WARN("[GameController] Received and rejected a bad packet of under 8 bytes!");
	else
	{
		uint8_t* data = (uint8_t*) m_recvBuf;
		char headerChar0 = (char) *(data + 0);
		char headerChar1 = (char) *(data + 1);
		char headerChar2 = (char) *(data + 2);
		char headerChar3 = (char) *(data + 3);
		uint8_t followingBytes4 = *(data + 4);
		uint8_t followingBytes5 = *(data + 5);
		uint8_t followingBytes6 = *(data + 6);
		uint8_t followingBytes7 = *(data + 7);
		ROS_WARN("[GameController] Received and rejected a bad packet with header '%c%c%c%c' and following four hex bytes %02" PRIx8 " %02" PRIx8 " %02" PRIx8 " %02" PRIx8 "!", headerChar0, headerChar1, headerChar2, headerChar3, followingBytes4, followingBytes5, followingBytes6, followingBytes7);
	}

	// Return that no parser could parse the packet
	return false;
}

// Reset the packet received variables
void GameController::resetPacketReceived()
{
	// Reset the packet received variables
	ros::WallTime now = ros::WallTime::now();
	m_lastPacketWallTime = now;
	m_lastPacketWarnTime = now;
	m_haveHadPacket = false;
}

// Handle updates to the enable game controller configuration parameter
void GameController::handleEnableGameController()
{
	// Warn if the game controller is enabled or disabled
	if(m_enabled && !config.enableGameController())
		ROS_WARN("[GameController] The game controller node was just disabled");
	if(!m_enabled && config.enableGameController())
		ROS_WARN("[GameController] The game controller node was just enabled");

	// Update the enabled flag
	m_enabled = config.enableGameController();

	// Reset the packet received variables
	resetPacketReceived();
}

// Handle updates to the set last server IP configuration parameter
void GameController::handleUseLastServerIP()
{
	// If activated, set the server IP to the last seen server IP
	if(config.useLastServerIP())
	{
		if(!m_lastServerIP.empty())
		{
			config.serverIP.set(m_lastServerIP);
			ROS_INFO("[GameController] Server IP set to %s!", m_lastServerIP.c_str());
		}
		config.useLastServerIP.set(false);
	}
}

//
// Main function
//

// Main function
int main(int argc, char** argv)
{
	// Initialise ROS
	ros::init(argc, argv, "game_controller");

	// Create an instance of the game controller class
	GameController GC;

	// Initialise the game controller class
	if(!GC.init())
	{
		ROS_FATAL("[GameController] Failed to initialise the game controller class => Exiting game controller!");
		return EXIT_FAILURE;
	}

	// Main loop
	while(ros::ok())
	{
		// Do all ROS callbacks
		ros::spinOnce();

		// Listen for packets, and process and publish them as required
		if(GC.listenForPacket(LOOP_WAIT_TIME_MS))
		{
			if(GC.processPacket())
				GC.publishProcessed();
		}

		// Sleep for a tiny amount of time to enforce a maximum loop rate in case we are getting bombarded by packets
		usleep(1000);
	}

	// Return success
	return EXIT_SUCCESS;
}
// EOF