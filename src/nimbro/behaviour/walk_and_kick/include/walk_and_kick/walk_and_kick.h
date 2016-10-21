// Walk and kick: Main walk and kick class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
// Ensure header is only included once
#ifndef WALK_AND_KICK_H
#define WALK_AND_KICK_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_sensor_vars.h>
#include <walk_and_kick/wak_ros_interface.h>
#include <walk_and_kick/wak_game_manager.h>
#include <walk_and_kick/wak_beh_manager.h>
#include <walk_and_kick/wak_utils.h>
#include <walk_and_kick/TeamCommsData.h>
#include <walk_and_kick/VisualiseDBH.h>
#include <std_srvs/Empty.h>

// TODO: Get WAK working with Gazebo

/**
* @namespace walk_and_kick
*
* @brief This namespace defines everything that is required for the walk and kick behaviour node.
**/
namespace walk_and_kick
{
	/**
	* @class WalkAndKick
	* 
	* @brief The main walk and kick behaviour class.
	**/
	class WalkAndKick
	{
	public:
		// Constructor
		WalkAndKick();

		// Reset functions
		void reset();
		void resetAll();

		// Initialisation function
		bool init();

		// Step function
		void step();

		// Publish functions
		bool publishLEDPending() { return (m_lastBlinkLED != shouldBlinkLED()); }
		void publishLED() { m_lastBlinkLED = shouldBlinkLED(); RI.updateRGBLED(m_lastBlinkLED); }
		void publishNeutral() { RI.writeNeutral(); }
		void publishState();
		void publishTeamComms(const ros::Time& now);

		// Service handlers
		bool handleVisualiseClear(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool handleVisualiseDBH(walk_and_kick::VisualiseDBHRequest& req, walk_and_kick::VisualiseDBHResponse& resp);
		bool handleVisualiseDbApp(walk_and_kick::VisualiseDbAppRequest& req, walk_and_kick::VisualiseDbAppResponse& resp);
		bool handleVisualiseGcvXY(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

		// Config parameters
		WAKConfig config;

	private:
		// ROS interface
		WAKRosInterface RI;

		// Sensor variables
		SensorVars SV;

		// Finite state machine managers
		WAKGameManager GM;
		WAKBehManager BM;

		// Field dimensions
		const FieldDimensions field;

		// Cycle number
		cycle_t m_wakCycle;

		// Team communications
		TeamCommsData m_teamPacket;
		int m_teamPacketCount;

		// LED blinking
		bool shouldBlinkLED() const { return SV.listenToGC; }
		bool m_lastBlinkLED;
	};
}

#endif
// EOF
