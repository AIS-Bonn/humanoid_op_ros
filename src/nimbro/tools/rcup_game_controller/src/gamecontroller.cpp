//Interface to receive Robocup Game control data & options
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <ros/init.h>
#include <stdio.h>


#include "gamecontroller.h"

using namespace rcup_game_controller;

GameController::GameController()
 : m_teamNumber("/gc/teamNumber", 0, 1, 80, 0)
 , m_robotNumber("/gc/robotNumber", 0, 1, 5, 0)
{
	ros::NodeHandle nh("~");

	m_pubData = boost::make_shared<GCData>();

	m_pubGC = nh.advertise<GCData>("data", 1);

	initResponse();
}

bool GameController::initNetwork()
{
	memset(&m_addr, 0, sizeof(m_addr));
	m_addr.sin6_family = AF_INET;
	m_addr.sin6_port = htons(GAMECONTROLLER_PORT);
	m_addr.sin6_addr = in6addr_any;

	if ((m_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		ROS_ERROR("GameController: creating socket failed: %s", strerror(errno));
		return false;
	}
	int bind_err;

	if ((bind_err = bind(m_fd, (sockaddr *)&m_addr, sizeof(sockaddr_in6))) != 0)
	{
		ROS_ERROR("GameController: binding socket failed: %s", strerror(errno));
		close(m_fd);
		return false;
	}

	ROS_INFO("GameController: Initialized UDP  Socket. Listening on Port: %d", ntohs(m_addr.sin6_port));
	return true;
}


bool GameController::update()
{
	if (! communicate())
	{
		return false;
	}

	if (memcmp(m_data.header, GAMECONTROLLER_STRUCT_HEADER, 4 * sizeof(char)) != 0)
	{
		ROS_WARN("GameController: corrupt package");
		return false;
	}

	parseData();
	return true;
}

bool GameController::communicate()
{
	int ret, req, send_err;

	timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;

	FD_ZERO(&m_rfds);
	FD_SET(m_fd, &m_rfds);

	req = select(m_fd + 1, &m_rfds, 0, 0, &timeout);
	if (req < 0)
	{
		ROS_ERROR("GameController: Select failed!");
		return false;
	}
	else if (req == 0)
	{
		return false;
	}

	m_ret_addr_len = sizeof(m_ret_addr);
	if ((ret = recvfrom(m_fd, &m_data, sizeof(m_data), 0, (sockaddr *)&m_ret_addr, &m_ret_addr_len) < 0))
	{
		ROS_ERROR("GameController: Recieving a packet failed.: %s", strerror(errno));
		return false;
	}

	createResponse();

	// Reply to the sending IP, but with the correct port.
	if(m_ret_addr_len == sizeof(sockaddr_in))
	{
		sockaddr_in* in_addr = (sockaddr_in*)&m_ret_addr;
		in_addr->sin_port = htons(GAMECONTROLLER_PORT);
	}
	else if(m_ret_addr_len == sizeof(sockaddr_in6))
	{
		m_ret_addr.sin6_port = htons(GAMECONTROLLER_PORT);
	}

	if ((send_err = sendto(m_fd, &m_response, sizeof(m_response), 0, (sockaddr *)&m_ret_addr, m_ret_addr_len)) < 0)
	{
		ROS_ERROR("GameController: Sending a response failed.");
		return false;
	}

	return true;
}

void GameController::parseData()
{
	const int numberOfTeams = 2;
	m_pubData->playerPerTeam = m_data.playersPerTeam;
	m_pubData->state = m_data.state;
	m_pubData->secondaryState = m_data.secondaryState;
	m_pubData->firstHalf = m_data.firstHalf;
	m_pubData->remainingSeconds = m_data.secondaryState;
	m_pubData->kickOffTeam = m_data.kickOffTeam;
	m_pubData->dropInTeam = m_data.dropInTeam;
	m_pubData->dropInTime = m_data.dropInTime;
	m_pubData->timeStamp = ros::Time::now();

	m_pubData->teams.resize(numberOfTeams);
	for (int i = 0; i < numberOfTeams; i++)
	{
		GCTeamInfo tInfo;
		tInfo.goalDirection = m_data.teams[i].goalColour;
		tInfo.score = m_data.teams[i].score;
		tInfo.teamColour = m_data.teams[i].teamColour;
		tInfo.teamNumber = m_data.teams[i].teamNumber;
		tInfo.player.resize(MAX_NUM_PLAYERS);
		for (int j = 0; j < MAX_NUM_PLAYERS; j++)
		{
			GCRobotInfo rInfo;
			rInfo.penalty = m_data.teams[i].players[j].penalty;
			rInfo.remainingPenaltySec = m_data.teams[i].players[j].secsTillUnpenalised;
			tInfo.player[j] = rInfo;
		}
		m_pubData->teams[i] = tInfo;
	}
}



void GameController::publish()
{
	m_pubGC.publish(m_pubData);
}

void GameController::createResponse(uint32 msg)
{
	m_response.team = (uint16)m_teamNumber();
	m_response.player = (uint16)m_robotNumber();
	m_response.message = msg;
}

void GameController::initResponse()
{
	memcpy(m_response.header, GAMECONTROLLER_RETURN_STRUCT_HEADER, 4 * sizeof(char));
	m_response.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "game_controller");

	GameController ctrl;
	if (! ctrl.initNetwork())
		return EXIT_FAILURE;


	ctrl.createResponse();

	while(true)
	{

		if(ctrl.update())
			ctrl.publish();

		ros::spinOnce();
		if (! ros::ok())
			break;
	}
	return 0;
}