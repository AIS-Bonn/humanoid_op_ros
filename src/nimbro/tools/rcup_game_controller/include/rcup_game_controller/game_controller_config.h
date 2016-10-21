// Game controller configuration parameters
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAME_CONTROLLER_CONFIG_H
#define GAME_CONTROLLER_CONFIG_H

// Includes
#include <config_server/parameter.h>

// Game controller namespace
namespace rcup_game_controller
{
	// Game controller config class
	class GCConfig
	{
	public:
		// Constructor
		GCConfig()
		 : CONFIG_PARAM_PATH("/game_controller/")
		 , enableGameController(CONFIG_PARAM_PATH + "enableGameController", false)
		 , teamNumber(CONFIG_PARAM_PATH + "teamNumber", 0, 1, 80, 0)
		 , robotNumber(CONFIG_PARAM_PATH + "robotNumber", 0, 1, 5, 0)
		 , enableVersion8(CONFIG_PARAM_PATH + "enableVersion8", true)
		 , serverIP(CONFIG_PARAM_PATH + "serverIP", "192.168.101.10")
		 , useLastServerIP(CONFIG_PARAM_PATH + "useLastServerIP", false)
		 , enableSecsRemainingFilter(CONFIG_PARAM_PATH + "enableSecsRemainingFilter", true)
		 , enableSecondaryTimeFilter(CONFIG_PARAM_PATH + "enableSecondaryTimeFilter", true)
		{
		}

		// Constants
		const std::string CONFIG_PARAM_PATH;

		// Configuration parameters
		config_server::Parameter<bool> enableGameController;
		config_server::Parameter<int> teamNumber;
		config_server::Parameter<int> robotNumber;
		config_server::Parameter<bool> enableVersion8;
		config_server::Parameter<std::string> serverIP;
		config_server::Parameter<bool> useLastServerIP;
		config_server::Parameter<bool> enableSecsRemainingFilter;
		config_server::Parameter<bool> enableSecondaryTimeFilter;
	};
}

#endif
// EOF