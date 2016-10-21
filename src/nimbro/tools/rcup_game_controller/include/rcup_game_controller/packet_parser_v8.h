// Packet parsing class for version 8 of the game controller communications
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef PACKET_PARSER_V8_H
#define PACKET_PARSER_V8_H

// Includes
#include <rcup_game_controller/packet_parser.h>

// Gamecontroller namespace
namespace rcup_game_controller
{
	// Struct declarations
	struct RoboCupGameControlData;
	struct RoboCupGameControlReturnData;
	struct RobotInfo;
	struct TeamInfo;

	// Packet parser version 8 class
	class PacketParserV8 : public PacketParser
	{
	public:
		// Constructor
		explicit PacketParserV8(const GCConfig& config);
		virtual ~PacketParserV8();

		// Return whether the parser is enabled
		virtual bool enabled() const { return m_config.enableVersion8(); }

		// Return the expected packet size
		virtual std::size_t recvPacketSize() const;

		// Return whether a received packet can be parsed by this parser (this should not be influenced by whether the parser is enabled or not)
		virtual bool canParsePacket(const void* data, std::size_t bytes);

		// Parse a received packet for the information that it contains
		virtual bool parsePacket(const void* data, std::size_t bytes, const GCDataPtr& msg);

		// Initialise the internal game controller response struct
		virtual void initGCResponse();

		// Update the response packet and return it
		virtual const void* getGCResponse(std::size_t& bytes);

		// Initialise the common game controller ROS message for any specific needs of this packet parser
		virtual void initGCRosData(const GCDataPtr& msg);

	private:
		// Parse helper functions
		bool parseRoboCupGameControlData(rcup_game_controller::GCData& data, const RoboCupGameControlData& RGCD);
		void parseTeamInfo(rcup_game_controller::GCTeamInfo& teamInfo, const TeamInfo& TI);
		void parseRobotInfo(rcup_game_controller::GCRobotInfo& robotInfo, const RobotInfo& RI);

		// Game controller response packet
		RoboCupGameControlReturnData* m_response;

		// Secondary time filter variables
		ros::Time m_filterLastExtraTime;
		int16_t m_filterLastAcceptValue;
		ros::Time m_filterLastAcceptTime;
		uint8_t m_filterLastAcceptState;
		uint8_t m_filterLastAcceptSecState;
		unsigned int m_filterCount0;
	};
};

#endif
// EOF