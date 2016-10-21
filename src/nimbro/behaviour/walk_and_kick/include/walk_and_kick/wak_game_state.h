// Walk and kick: Game states base implementation
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_GAME_STATE_H
#define WAK_GAME_STATE_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_game_vars.h>
#include <walk_and_kick/wak_game_shared.h>
#include <walk_and_kick/wak_sensor_vars.h>
#include <walk_and_kick/wak_ros_interface.h>
#include <walk_and_kick/wak_beh_manager.h> // Note: Included here to give access to WAKBehManager::BehStateID in all game states
#include <walk_and_kick/wak_utils.h>

// Walk and kick namespace
namespace walk_and_kick
{
	/**
	* @class WAKGameState
	* 
	* @brief The base class for all walk and kick game states.
	**/
	class WAKGameState
	{
	public:
		// Constructor
		WAKGameState(WAKConfig& config, const SensorVars& SV, const WAKGameShared& WGS, int ID);
		virtual ~WAKGameState() {}

		// Config parameters
		WAKConfig& config;

		// Sensor variables
		const SensorVars& SV;

		// Shared game variables
		const WAKGameShared& WGS;

		// Field dimensions
		const FieldDimensions& field;

		// Plot manager
		plot_msgs::PlotManagerFS& PM;

		// Marker manager
		WAKMarkerMan& MM;

		// Reset function
		void reset() { deactivate(); }

		// Get functions
		int id() const { return m_id; }
		std::string name() const { return m_name; }
		const std::string& nameRef() const { return m_name; }
		bool isActive() const { return m_active; }

		// Activation and deactivation functions
		void activate() { m_active = true; handleActivation(true); }
		void deactivate() { m_active = false; handleActivation(false); }

		// Execute function
		virtual void execute(GameVars& GV, const GameVars& lastGV, bool justActivated) { GV.reset(); }

	private:
		// Handle activation function
		virtual void handleActivation(bool nowActive) {}

		// Internal variables
		int m_id;
		std::string m_name;
		bool m_active;
	};
}

#endif
// EOF