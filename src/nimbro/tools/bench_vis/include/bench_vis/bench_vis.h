// Bench visualisation node
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         André Brandenburger <andre.brandenburger@uni-bonn.de>

// Ensure header is only included once
#ifndef BENCH_VIS_H
#define BENCH_VIS_H

// Includes
#include <bench_vis/bench_vis_config.h>
#include <walk_and_kick/wak_gc_ros.h>
#include <walk_and_kick/wak_gc_vars.h>
#include <walk_and_kick/wak_tc_ros.h>
#include <walk_and_kick/wak_tc_vars.h>
#include <rrlogger/LoggerHeartbeat.h>
#include <vis_utils/marker_manager.h>
#include <map>
#include <math.h>
#include <string>
#include <sstream>
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_beh_manager.h>
#include <walk_and_kick/wak_game_manager.h>
#include <boost/lexical_cast.hpp>
#include <rot_conv/rot_conv.h>
#include <plot_msgs/plot_manager.h>
#include <walk_and_kick/wak_utils.h>

// ATTENTION: If you change this, you need to add extra colors to the table!
#define ROBOT_LIMIT 10


// Bench visualisation namespace
namespace bench_vis
{
	class GeneralMarkerManager : public vis_utils::MarkerManager
	{
	public: 
	  explicit GeneralMarkerManager();
	  const std::string behField;
	  walk_and_kick::FieldDimensions field_dim;
	  
	  
	  config_server::Parameter<bool> m_plotData; 
	  plot_msgs::PlotManagerFS m_PM;
	  void configurePlotManager();
	  
	  enum PMIDS
	  {
		PM_GC_SEQID,
		PM_GC_TIMESINCEPACKETBASE,
		PM_GC_TIMESINCEPACKETEXTRA,
		PM_GC_EXTRAOUTOFDATE,
		PM_GC_GAMEPHASE,
		PM_GC_GAMESTATE,
		PM_GC_TIMEPLAYING,
		PM_GC_KICKOFFTYPE,
		PM_GC_TIMEREMAINING_RAW,
		PM_GC_TIMEREMAINING_SMOOTH,
		PM_GC_SECONDARYTIME_RAW,
		PM_GC_SECONDARYTIME_SMOOTH,
		PM_GC_TIMETOBALLINPLAY_RAW,
		PM_GC_TIMETOBALLINPLAY_SMOOTH,
		PM_GC_OWNPENALTYSTATE,
		PM_GC_OWNPENALTYTIME_RAW,
		PM_GC_OWNPENALTYTIME_SMOOTH,
		PM_GC_OWNSCORE,
		PM_GC_OPPSCORE,
		PM_GC_OWNNUMPLAYING,
		PM_GC_OPPNUMPLAYING,
		PM_COUNT
	  };
	};
  
  
	// Robot Marker Manager
	class RobotMarkerManager : public vis_utils::MarkerManager
	{
	public:
	  explicit RobotMarkerManager(std::string robot);
	  void updateMarkerXYZ(vis_utils::GenMarker& marker, bool show, float x = 0.0f, float y = 0.0f, float z = 0.0f, float rot = 0.0f, float a = 1.0f);
	  
	  const std::string behField;
	  walk_and_kick::FieldDimensions field_dim;
	  
	  vis_utils::BoxMarker robot_trunk;
	  vis_utils::SphereMarker robot_head;
	  vis_utils::TextMarker robot_name;
	  vis_utils::SphereMarker ball;
	  vis_utils::BoxMarker goalMarker;
	  vis_utils::ArrowMarker compass;
	  vis_utils::SphereMarker eye1;
	  vis_utils::SphereMarker eye2;
	  vis_utils::TextMarker ModeStateText;
	  vis_utils::TextMarker GameStateText;
	  vis_utils::TextMarker BehStateText;
	  vis_utils::TextMarker RoleText;
	  vis_utils::TextMarker StableText;
	  vis_utils::BoxMarker wt_cross1;
	  vis_utils::BoxMarker wt_cross2;
	  vis_utils::TextMarker northLabel;
	  vis_utils::CylinderMarker obstacle;
	  
	  config_server::Parameter<bool> m_plotData; 
	  plot_msgs::PlotManagerFS m_PM;
	  
	  
	  
	  void configurePlotManager();
	  enum PMIDS
	  {
	  	PM_ENABLED,
		PM_TIME_SINCE_LAST_RECEIVED,
		PM_FIELD_TYPE,
		PM_ON_YELLOW,
		PM_AS_CYAN,
		PM_LISTEN_TO_GC,
		PM_LISTEN_TO_TC,
		PM_IS_PENALTY_SHOOT,
		PM_KICKOFF_TYPE,
		PM_BUTTON_STATE,
		PM_GAME_COMMAND,
		PM_GAME_ROLE,
		PM_PLAY_STATE,
		PM_GAME_STATE,
		PM_BEH_STATE,
		PM_FALLEN,
		PM_COMPASS_HEADING,
		PM_ROBOT_POSE_X,
		PM_ROBOT_POSE_Y,
		PM_ROBOT_POSE_T,
		PM_ROBOT_POSE_CONF,
		PM_ROBOT_POSE_VALID,
		PM_BALL_POSE_X,
		PM_BALL_POSE_Y,
		PM_BALL_POSE_CONF,
		PM_BALL_POSE_VALID,
		PM_BALL_POSE_STABLE,
		PM_OBSTACLE_CLOSEST_X,
		PM_OBSTACLE_CLOSEST_Y,
		PM_OBSTACLE_CLOSEST_CONF,
		PM_OBSTACLE_CLOSEST_VALID,
		PM_WALKING_TARGET_X,
		PM_WALKING_TARGET_Y,
		PM_WALKING_TARGET_VALID,
		PM_TIME_SINCE_GC_BASE,
		PM_TIME_SINCE_GC_EXTRA,
		PM_TIME_SINCE_TC,
		PM_NUM_FRESH_TC,
		PM_COUNT
	  };
	  
	  ros::Time last_recieved;
	private:
	  
	};


	// Robot Marker Manager typedefs
	typedef boost::shared_ptr<RobotMarkerManager> RMM_ptr;
	typedef std::map<unsigned int, RMM_ptr> RMM_map;
	typedef std::map<unsigned int, uint16_t> PackID_to_UID_map;
	
	// BenchVis class
	class BenchVis
	{
	public:
		// Constructor
		explicit BenchVis(ros::NodeHandle& nh);
		// Step function
		void step();

	private:
		// Configuration parameters
		BVConfig config;

		// ROS node handle
		ros::NodeHandle m_nh;

		// Game controller variables
		walk_and_kick::GCRosInterface GCRI;
		walk_and_kick::GCVars GC;
		bool m_GCDataFresh;

		// Team communications variables
		walk_and_kick::TCRosInterface TCRI;
		walk_and_kick::TCVars TC;

		GeneralMarkerManager gmm;
		
		RMM_map rmm_map;
		PackID_to_UID_map pID_to_UID;
		
		
		
		// Logger functions and variables
		void updateLoggerState();
		void publishLoggerState(bool log);
		rrlogger::LoggerHeartbeat m_loggerMsg;
		ros::Publisher m_pub_logger;
		ros::Time m_loggerStamp;
		uint16_t last_packetID;
		
		const static float colors[ROBOT_LIMIT+1][3];
		
		
	};
}

#endif
// EOF