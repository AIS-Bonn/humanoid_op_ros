// Packet parsing class for version 8 of the game controller communications
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rcup_game_controller/packet_parser_v8.h>

// Include the RoboCup game control data header version 8
namespace rcup_game_controller
{
#include <rcup_game_controller/contrib/version8/RoboCupGameControlData.h>
}

// Namespaces
using namespace rcup_game_controller;

//
// PacketParserV8 class
//

// Constructor
PacketParserV8::PacketParserV8(const GCConfig& config) : PacketParser(config), m_response(NULL)
{
	// Allocate memory for the game controller response packet
	m_response = new RoboCupGameControlReturnData();

	// Initialise the secondary time filter variables
	m_filterLastExtraTime.fromNSec(0);
	m_filterLastAcceptValue = 0;
	m_filterLastAcceptTime.fromNSec(0);
	m_filterLastAcceptState = 0;
	m_filterLastAcceptSecState = 0;
	m_filterCount0 = 0;
}

// Destructor
PacketParserV8::~PacketParserV8()
{
	// Free memory
	if(m_response)
		delete m_response;
}

// Return the expected packet size
std::size_t PacketParserV8::recvPacketSize() const
{
	// Return the expected size of received game controller packets
	return sizeof(RoboCupGameControlData);
}

// Return whether a received packet can be parsed by this parser (this should not be influenced by whether the parser is enabled or not)
bool PacketParserV8::canParsePacket(const void* data, std::size_t bytes)
{
	// Cannot parse if the size of the packet is wrong
	if(bytes != recvPacketSize())
		return false;

	// Cast the data pointer a pointer to the required struct
	const RoboCupGameControlData* structData = (const RoboCupGameControlData*) data;

	// Check whether the header of the packet is correct
	if(memcmp(structData->header, GAMECONTROLLER_STRUCT_HEADER, 4*sizeof(char)) != 0)
		return false;

	// Check whether the version of the packet is correct
	if(structData->version != GAMECONTROLLER_STRUCT_VERSION)
		return false;

	// Return that the packet can be parsed by this parser
	return true;
}

// Parse a received packet for the information that it contains
bool PacketParserV8::parsePacket(const void* data, std::size_t bytes, const GCDataPtr& msg)
{
	// Check that this parser can parse the packet
	if(!canParsePacket(data, bytes))
		return false;

	// Cast the data pointer a pointer to the required struct
	const RoboCupGameControlData* structData = (const RoboCupGameControlData*) data;

	// Parse the required data
	return parseRoboCupGameControlData(*msg, *structData);
}

// Initialise the internal game controller response struct
void PacketParserV8::initGCResponse()
{
	// Initialise fields of the game controller response packet that never change
	memcpy(m_response->header, GAMECONTROLLER_RETURN_STRUCT_HEADER, 4*sizeof(char));
	m_response->version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
}

// Update the response packet and return it
const void* PacketParserV8::getGCResponse(std::size_t& bytes)
{
	// Update the game controller response packet
	m_response->team = (uint8_t) m_config.teamNumber();
	m_response->player = (uint8_t) m_config.robotNumber();
	m_response->message = GAMECONTROLLER_RETURN_MSG_ALIVE;

	// Indicate the number of bytes in the game controller response packet
	bytes = sizeof(RoboCupGameControlReturnData);

	// Return a pointer to the required game controller response packet
	return m_response;
}

// Initialise the common game controller ROS message for any specific needs of this packet parser
void PacketParserV8::initGCRosData(const GCDataPtr& msg)
{
	// Ensure that the vectors of own and opponent players are large enough
	if(msg->ownTeam.players.size() < MAX_NUM_PLAYERS)
		msg->ownTeam.players.resize(MAX_NUM_PLAYERS);
	if(msg->oppTeam.players.size() < MAX_NUM_PLAYERS)
		msg->oppTeam.players.resize(MAX_NUM_PLAYERS);
}

// Parse information from a received game controller data struct
bool PacketParserV8::parseRoboCupGameControlData(rcup_game_controller::GCData& data, const RoboCupGameControlData& RGCD)
{
	// Get the robot and team numbers
	int teamNumber = m_config.teamNumber();
	int robotNumber = m_config.robotNumber();

	// See which team is us
	const TeamInfo* ownTeam = NULL;
	const TeamInfo* oppTeam = NULL;
	bool team0IsUs = (RGCD.teams[0].teamNumber == teamNumber);
	bool team1IsUs = (RGCD.teams[1].teamNumber == teamNumber);
	if(team0IsUs && !team1IsUs)
	{
		ownTeam = &(RGCD.teams[0]);
		oppTeam = &(RGCD.teams[1]);
	}
	else if(team1IsUs && !team0IsUs)
	{
		ownTeam = &(RGCD.teams[1]);
		oppTeam = &(RGCD.teams[0]);
	}
	else
	{
		if(team0IsUs && team1IsUs)
			ROS_WARN_THROTTLE(5.0, "[GameController] Ignoring game controller packet as both teams are us (team number %d)", teamNumber);
		else
			ROS_WARN_THROTTLE(5.0, "[GameController] Ignoring game controller packet as neither team is us (team number %d)", teamNumber);
		return false;
	}

	// Trust any secondary time value that another parser wrote into the data message
	if(data.stampExtra != m_filterLastExtraTime || m_filterLastExtraTime.isZero())
	{
		m_filterLastAcceptValue = data.secondaryTime;
		m_filterLastAcceptTime = data.stampExtra;
		m_filterLastAcceptState = data.state;
		m_filterLastAcceptSecState = data.secondaryState;
		m_filterCount0 = 0;
	}

	// Transcribe the main game controller packet data
	data.seq++;
	ros::Time now = ros::Time::now();
	data.stampBase = now;
	data.stampExtra = now;
	data.extraOutOfDate = false;
	data.playersPerTeam = RGCD.playersPerTeam;
	data.state = RGCD.state;
	data.firstHalf = (RGCD.firstHalf != 0);
	if(RGCD.kickOffTeam == TEAM_CYAN || RGCD.kickOffTeam == TEAM_MAGENTA)
	{
		data.ownKickoff = (RGCD.kickOffTeam == ownTeam->teamColour);
		data.isDropBall = false;
	}
	else
	{
		data.ownKickoff = true;
		data.isDropBall = true;
	}
	data.secondaryState = RGCD.secondaryState;
	data.dropInDueToOwnTeam = (RGCD.dropInTeam == ownTeam->teamColour);
	data.dropInTime = (int16_t) RGCD.dropInTime;
	data.secsRemaining = (int16_t) RGCD.secsRemaining;

	// Handle the secondary time
	int16_t secondaryTime = (int16_t) RGCD.secondaryTime;
	bool acceptValue = true;
	if(m_config.enableSecondaryTimeFilter() && secondaryTime == 0 && m_filterLastAcceptValue != 0 && data.state == m_filterLastAcceptState && data.secondaryState == m_filterLastAcceptSecState && m_filterCount0 < 2 && !m_filterLastAcceptTime.isZero())
	{
		m_filterCount0++;
		acceptValue = false;
		double timeSinceAccept = (data.stampExtra - m_filterLastAcceptTime).toSec();
		if(timeSinceAccept > 0.0 && timeSinceAccept <= 32767.0)
		{
			data.secondaryTime = m_filterLastAcceptValue - ((int16_t) (timeSinceAccept + 0.5));
			if(data.secondaryTime <= 1)
			{
				secondaryTime = 0;
				acceptValue = true;
			}
		}
	}
	if(acceptValue)
	{
		data.secondaryTime = secondaryTime;
		m_filterLastAcceptValue = secondaryTime;
		m_filterLastAcceptTime = data.stampExtra;
		m_filterLastAcceptState = data.state;
		m_filterLastAcceptSecState = data.secondaryState;
		m_filterCount0 = 0;
	}
	m_filterLastExtraTime = data.stampExtra;

	// Parse the team information
	parseTeamInfo(data.ownTeam, *ownTeam);
	parseTeamInfo(data.oppTeam, *oppTeam);

	// Retrieve the robot information about us
	if(robotNumber >= 1 && robotNumber <= MAX_NUM_PLAYERS)
		data.ownRobot = data.ownTeam.players[robotNumber - 1];
	else
	{
		data.ownRobot.penaltyState = PENALTY_NONE;
		data.ownRobot.secsUntilUnpenalised = 0;
	}

	// Return that the packet was successfully parsed
	return true;
}

// Parse team information from a received team info struct
void PacketParserV8::parseTeamInfo(rcup_game_controller::GCTeamInfo& teamInfo, const TeamInfo& TI)
{
	// Transcribe the main team information data
	teamInfo.teamNumber = TI.teamNumber;
	teamInfo.teamColour = TI.teamColour;
	teamInfo.score = TI.score;
	teamInfo.penaltyShot = TI.penaltyShot;
	teamInfo.singleShots = TI.singleShots;

	// Transcribe the robot information data
	for(int i = 0; i < MAX_NUM_PLAYERS; i++)
		parseRobotInfo(teamInfo.players[i], TI.players[i]);
}

// Parse robot information from a received robot info struct
void PacketParserV8::parseRobotInfo(rcup_game_controller::GCRobotInfo& robotInfo, const RobotInfo& RI)
{
	// Transcribe the robot information
	robotInfo.penaltyState = RI.penalty;
	robotInfo.secsUntilUnpenalised = RI.secsTillUnpenalised;
}
// EOF