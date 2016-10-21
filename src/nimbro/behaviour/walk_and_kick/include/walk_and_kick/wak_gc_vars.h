// Walk and kick: Game controller variables
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_GC_VARS_H
#define WAK_GC_VARS_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <rcup_game_controller/GCData.h>

// Defines
#define DEFAULT_GCFRESH_TIME          5.0f
#define DEFAULT_SMOOTHING_TIME        1.0f
#define DEFAULT_TIMEOUT_KICKOFF_TYPE  11.0f

// Walk and kick namespace
namespace walk_and_kick
{
	// Class declarations
	class WAKMarkerMan;

	/**
	* @class GCVars
	* 
	* @brief A class that encapsulates all of the game controller input data to the walk and kick node.
	**/
	class GCVars
	{
	public:
		//
		// General
		//

		// Constructor
		GCVars(WAKConfig* config, plot_msgs::PlotManagerFS* PM, WAKMarkerMan* MM);

		// Config parameters
		WAKConfig* const config;

		// Update function
		bool update(const rcup_game_controller::GCData& data, const ros::Time& now); // Returns whether the game controller base data is fresh

		//
		// Enumerations
		//

		// Game state enumeration
		enum GameState
		{
			GS_INITIAL  = rcup_game_controller::GCData::STATE_INITIAL,  // Initial state of undetermined duration, active at all times prior to the start of the game
			GS_READY    = rcup_game_controller::GCData::STATE_READY,    // Ready state of nominal duration 30 seconds (counted down by the secondary time), during which the robots should position themselves to be ready for the start of play
			GS_SET      = rcup_game_controller::GCData::STATE_SET,      // Set state of undetermined duration, at which point the robots should stop where they are and wait for the signal to start playing
			GS_PLAYING  = rcup_game_controller::GCData::STATE_PLAYING,  // Playing state of certain nominal duration (counted down by the time remaining, with the first 10 seconds until the ball is definitely in play being counted down by the secondary time), during which the robots should play soccer and try to score as many goals as possible
			GS_FINISHED = rcup_game_controller::GCData::STATE_FINISHED  // Final state of undetermined duration, active at all times after the end of the game
		};

		// Game phase enumeration
		enum GamePhase
		{
			GP_NORMAL   = rcup_game_controller::GCData::SECSTATE_NORMAL,        // Normal soccer gameplay, playing state has nominal duration 10 minutes
			GP_PENALTY  = rcup_game_controller::GCData::SECSTATE_PENALTYSHOOT,  // Playing state has nominal duration 1 minute, the ready state is skipped and initial passes directly to set, no secondary time countdown of 10 seconds until the ball is in play as the ball is always immediately in play
			GP_OVERTIME = rcup_game_controller::GCData::SECSTATE_OVERTIME,      // Playing state has nominal duration 5 minutes, the same as SECSTATE_NORMAL in all other aspects
			GP_TIMEOUT  = rcup_game_controller::GCData::SECSTATE_TIMEOUT        // Should only happen in conjunction with a game state of STATE_INITIAL, and involves no gameplay
		};

		// Penalty state enumeration
		enum PenaltyState
		{
			PS_NONE               = rcup_game_controller::GCRobotInfo::PENALTY_NONE,                          // Normal unpenalised robot state
			PS_BALL_MANIPULATION  = rcup_game_controller::GCRobotInfo::PENALTY_BALL_MANIPULATION,             // Ball manipulation penalty
			PS_PHYSICAL_CONTACT   = rcup_game_controller::GCRobotInfo::PENALTY_PHYSICAL_CONTACT,              // Physical contact penalty
			PS_ILLEGAL_ATTACK     = rcup_game_controller::GCRobotInfo::PENALTY_ILLEGAL_ATTACK,                // Illegal attack penalty
			PS_ILLEGAL_DEFENSE    = rcup_game_controller::GCRobotInfo::PENALTY_ILLEGAL_DEFENSE,               // Illegal defense penalty
			PS_REQ_PICKUP         = rcup_game_controller::GCRobotInfo::PENALTY_REQUEST_FOR_PICKUP,            // Robot has been picked up
			PS_REQ_SERVICE        = rcup_game_controller::GCRobotInfo::PENALTY_REQUEST_FOR_SERVICE,           // Robot is being serviced
			PS_REQ_PICKUP_SERVICE = rcup_game_controller::GCRobotInfo::PENALTY_REQUEST_FOR_PICKUP_2_SERVICE,  // Robot was picked up and is now additionally being serviced
			PS_ON_THE_BENCH       = rcup_game_controller::GCRobotInfo::PENALTY_SUBSTITUTE,                    // The robot is a substitute and not an active member of the game
			PS_MANUAL             = rcup_game_controller::GCRobotInfo::PENALTY_MANUAL                         // The robot itself manually requested a penalisation
		};

		//
		// Classes
		//

		// Team state class
		class TeamState
		{
		public:
			int teamNumber;             // Team number
			bool isCyan;                // Team colour
			unsigned int score;         // Number of goals scored
			unsigned int numNotOnBench; // Number of players not on the bench
			unsigned int numPenalised;  // Number of players not on the bench that are penalised
			unsigned int numPlaying;    // Number of players without any penalisation
		};

		// Smooth time class (Note: This class assumes that the raw time is ticking down, *not* up)
		class SmoothTime
		{
		public:
			// Constructor
			explicit SmoothTime(WAKConfig* config) : config(config) { reset(ros::Time()); }

			// Reset function
			void reset(const ros::Time& now);

			// Set functions (needs to be called in every cycle or the smoothed time estimate is not updated)
			void setValue(int timeRemaining, const ros::Time& timestamp, const ros::Time& now);

			// Config parameters
			WAKConfig* const config;

			// Time data variables
			int raw;               // The last set raw time data value
			float smooth;          // The last calculated smoothed time data value
			ros::Time timestamp;   // The timestamp used in the last update of the time data (i.e. the value of timestamp in the last call to setValue(), which is *not* necessarily the ros::Time::now() of the last update at all!)
			ros::Time elapseTime;  // The absolute time at which the raw/smooth values are expected to hit zero (can be in the future or the past)

		private:
			// Internal variables
			ros::Time m_rawTimestamp;
		};

		//
		// Data variables
		//

		// Message information
		unsigned int seqID;
		ros::Time stampBase;
		ros::Time stampExtra;
		bool extraOutOfDate;

		// Game state
		unsigned int playersPerTeam;
		GameState gameState;
		GamePhase gamePhase;
		KickoffType kickoffType;
		bool isPenaltyTaker;
		SmoothTime timeRemaining;
		SmoothTime secondaryTime;    // Extra: This variable has timestamp stampExtra
		SmoothTime timeToBallInPlay; // Extra: This variable has timestamp stampExtra

		// Robot state
		PenaltyState ownPenaltyState;
		SmoothTime ownPenaltyTimeRemaining;
		bool ownIsPenalised;
		bool ownIsOnBench;
		bool ownIsPlaying;

		// Team states
		TeamState ownTeam;
		TeamState oppTeam;

		//
		// Helper functions
		//

		// Data is fresh function
		bool baseDataIsFresh(const ros::Time& now) const { return (!stampBase.isZero() && (now - stampBase).toSec() <= (config ? config->gcFreshTime() : DEFAULT_GCFRESH_TIME)); }
		bool extraDataIsFresh(const ros::Time& now) const { return (!extraOutOfDate && !stampExtra.isZero() && (now - stampExtra).toSec() <= (config ? config->gcFreshTime() : DEFAULT_GCFRESH_TIME)); }

	private:
		// Plot manager
		plot_msgs::PlotManagerFS* PM;

		// Marker manager
		WAKMarkerMan* MM;

		// Update team state function
		static void updateTeamState(TeamState& team, const rcup_game_controller::GCTeamInfo& data);

		// Internal variables
		ros::Time m_stampStartPlaying;
	};
}

#endif
// EOF