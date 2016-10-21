// Interface to receive Robocup game control data
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Sebastian Schüller <schuell1@cs.uni-bonn.de>
//         Hafez Farazi <farazi@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAME_CONTROLLER_H
#define GAME_CONTROLLER_H

// Includes
#include <rcup_game_controller/game_controller_config.h>
#include <rcup_game_controller/packet_parser.h>
#include <rcup_game_controller/GCData.h>
#include <boost/shared_ptr.hpp>
#include <ros/node_handle.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ros/time.h>
#include <string>

// General defines that should be true for all packet versions
#define GAMECONTROLLER_PORT 3838

// Game controller namespace
namespace rcup_game_controller
{
	// Game controller class
	class GameController
	{
	public:
		// Constructor
		GameController();

		// Initialisation function
		bool init();

		// Packet processing pipeline
		bool listenForPacket(int msTimeout);
		bool processPacket();
		void publishProcessed() { m_pub_gc.publish(m_msg); }

		// Get functions
		bool enabled() const { return m_enabled; }

	private:
		// Initialise the network connection
		bool initNetwork();

		// Configuration parameters
		GCConfig config;

		// Game controller enabled flag
		bool m_enabled;

		// Silent mode flag (no return packets)
		bool m_silentMode;

		// Packet parsers
		typedef boost::shared_ptr<PacketParser> PacketParserPtr;
		typedef std::vector<PacketParserPtr> ParserList;
		ParserList m_parsers;

		// Published ROS data
		ros::Publisher m_pub_gc;
		GCDataPtr m_msg;

		// Packet received variables
		void resetPacketReceived();
		ros::WallTime m_lastPacketWallTime;
		ros::WallTime m_lastPacketWarnTime;
		bool m_haveHadPacket;

		// Incoming packet buffer
		void* m_recvBuf;
		std::size_t m_recvBufSize;
		int m_recvPacketSize;

		// Network connection variables
		int m_socket;
		sockaddr_in6 m_ret_addr;
		socklen_t m_ret_addr_len;

		// Last server IP
		std::string m_lastServerIP;

		// Configuration parameter callbacks
		void handleEnableGameController();
		void handleUseLastServerIP();
	};
}

#endif
// EOF