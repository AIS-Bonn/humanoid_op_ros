// Packet parsing base class for game controller communications
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

// Includes
#include <rcup_game_controller/game_controller_config.h>
#include <rcup_game_controller/GCRobotInfo.h>
#include <rcup_game_controller/GCTeamInfo.h>
#include <rcup_game_controller/GCData.h>

// Game controller namespace
namespace rcup_game_controller
{
	// Packet parser class
	class PacketParser
	{
	public:
		// Constructor
		explicit PacketParser(const GCConfig& config) : m_config(config) {}
		virtual ~PacketParser() {}

		// Return whether the parser is enabled
		virtual bool enabled() const = 0;

		// Return the expected packet size
		virtual std::size_t recvPacketSize() const = 0;

		// Return whether a received packet can be parsed by this parser (this should not be influenced by whether the parser is enabled or not)
		virtual bool canParsePacket(const void* data, std::size_t bytes) = 0;

		// Parse a received packet for the information that it contains
		virtual bool parsePacket(const void* data, std::size_t bytes, const GCDataPtr& msg) = 0;

		// Initialise the internal game controller response struct
		virtual void initGCResponse() = 0;

		// Update the response packet and return it
		virtual const void* getGCResponse(std::size_t& bytes) = 0;

		// Initialise the common game controller ROS message for any specific needs of this packet parser
		virtual void initGCRosData(const GCDataPtr& msg) = 0;

	protected:
		// Configuration parameters
		const GCConfig& m_config;
	};
};

#endif
// EOF