//Interface to receive Robocup Game control data & options
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef GAMECONTROLLER_H
#define GAMECONTROLLER_H

#include <netinet/in.h>
#include <ros/node_handle.h>

#include <rcup_game_controller/GCData.h>
#include <rcup_game_controller/GCRobotInfo.h>
#include <rcup_game_controller/GCTeamInfo.h>

#include <config_server/parameter.h>

#include "RoboCupGameControlData.h"

class GameController
{
public:
	GameController();
	~GameController(){};

	bool initNetwork();
	void createResponse(uint32 msg = 2);
	void initResponse();

	bool update();
	void publish();

private:

	int m_fd;
	sockaddr_in6 m_addr;
	sockaddr_in6 m_ret_addr;
	socklen_t m_ret_addr_len;

	fd_set m_rfds;

	RoboCupGameControlData m_data;
	RoboCupGameControlReturnData m_response;

	config_server::Parameter< int > m_teamNumber;
	config_server::Parameter< int > m_robotNumber;

	ros::Publisher m_pubGC;
	rcup_game_controller::GCDataPtr m_pubData;

	bool communicate();
	void parseData();
};

#endif