// Bench vis
// Author: André Brandenburger <andre.brandenburger@uni-bonn.de>

// Includes
#include <bench_vis/bench_vis.h>
#include <rc_utils/ros_time.h>
#include <motiontimer.h>

// Namespaces
using namespace bench_vis;
using namespace walk_and_kick;
using namespace rc_utils;


#define ROBOT_HEIGHT         .4
#define ROBOT_WIDTH          .15
#define ROBOT_DY             .30
#define HEAD_DIAMETER        .156
#define NAME_TEXT_SIZE       .15
#define TEXT_MARKER_DIST     NAME_TEXT_SIZE
#define TEXT_SIZE            .10
#define TEXT_LINE_PADDING    .03
#define BALL_DIAMETER        .25
#define GOAL_MARKER_HEIGHT   .1
#define GOAL_MARKER_WIDTH    .15
#define HEADING_SHAFT_DIAM   .025
#define HEADING_HEAD_DIAM    .05
#define HEADING_HEIGHT       .03
#define HEADING_DISTANCE     .25
#define EYE_DIAMETER          .05
#define EYE_DISTANCE         .065
#define EYE_INLINE           .03
#define TEXT_POSZ_NAME       ROBOT_HEIGHT+HEAD_DIAMETER+0.*TEXT_SIZE+1.*TEXT_LINE_PADDING+TEXT_MARKER_DIST
#define TEXT_POSZ_MODE       ROBOT_HEIGHT+HEAD_DIAMETER+0.*TEXT_SIZE+2.*TEXT_LINE_PADDING+TEXT_MARKER_DIST
#define TEXT_POSZ_GAME       ROBOT_HEIGHT+HEAD_DIAMETER+1.*TEXT_SIZE+3.*TEXT_LINE_PADDING+TEXT_MARKER_DIST
#define TEXT_POSZ_BEH        ROBOT_HEIGHT+HEAD_DIAMETER+2.*TEXT_SIZE+4.*TEXT_LINE_PADDING+TEXT_MARKER_DIST
#define TEXT_POSZ_ROLE       ROBOT_HEIGHT+HEAD_DIAMETER+3.*TEXT_SIZE+5.*TEXT_LINE_PADDING+TEXT_MARKER_DIST
#define STABLE_TEXT_SIZE     .10
#define NORTH_TEXT_SIZE      TEXT_SIZE
#define LINE_HEIGHT          .01
#define CROSS_WIDTH          .05
#define CROSS_LENGTH         .25
#define OBSTACLE_HEIGHT      .6
#define OBSTACLE_DIAMETER    .2


#define SOFT_ERROR_PAD       .4
#define HARD_ERROR_PAD       1

RobotMarkerManager::RobotMarkerManager(const std::string robot)
 : MarkerManager("~"+robot+"_marker", 1, true)
 , behField("/beh_field")
 , field_dim()
 , robot_trunk(this, behField, ROBOT_WIDTH, ROBOT_DY, ROBOT_HEIGHT, "robot")
 , robot_head(this, behField, HEAD_DIAMETER, "robot")
 , robot_name(this, behField, NAME_TEXT_SIZE, "name")
 , ball(this, behField, field_dim.ballDiameter(), "ball_target")
 , goalMarker(this, behField, GOAL_MARKER_WIDTH, field_dim.fieldWidth()/float(ROBOT_LIMIT), GOAL_MARKER_HEIGHT, "goal_target")
 , compass(this, behField, HEADING_SHAFT_DIAM, HEADING_HEAD_DIAM, 0., "compass")
 , eye1(this, behField, EYE_DIAMETER, "robot")
 , eye2(this, behField, EYE_DIAMETER, "robot")
 , ModeStateText(this, behField, TEXT_SIZE, "WAK_labels")
 , GameStateText(this, behField, TEXT_SIZE, "WAK_labels")
 , BehStateText(this, behField, TEXT_SIZE, "WAK_labels")
 , RoleText(this, behField, TEXT_SIZE, "WAK_labels")
 , StableText(this, behField, STABLE_TEXT_SIZE, "ball_target")
 , wt_cross1(this, behField, CROSS_LENGTH, CROSS_WIDTH, LINE_HEIGHT, "walk_target")
 , wt_cross2(this, behField, CROSS_WIDTH, CROSS_LENGTH, LINE_HEIGHT, "walk_target")
 , northLabel(this, behField, NORTH_TEXT_SIZE, "compass")
 , obstacle(this, behField, OBSTACLE_HEIGHT, OBSTACLE_DIAMETER, "obstacle")
 , m_plotData("/bench_vis/"+robot+"/plotData", false)
 , m_PM(PM_COUNT, "/bench_vis/"+robot)
{
	// Remember time of creation as first packet arriving
	last_recieved = ros::Time::now();
	configurePlotManager();
	
	
	// Set Lifetimes
	robot_trunk.setLifetime(2.);
	robot_head.setLifetime(2.);
	robot_name.setLifetime(2.);
	ball.setLifetime(2.);
	goalMarker.setLifetime(2.);
	compass.setLifetime(2.);
	eye1.setLifetime(2.);
	eye2.setLifetime(2.);
	ModeStateText.setLifetime(2.);
	GameStateText.setLifetime(2.);
	BehStateText.setLifetime(2.);
	RoleText.setLifetime(2.);
	StableText.setLifetime(2.);
	wt_cross1.setLifetime(2.);
	wt_cross2.setLifetime(2.);
	northLabel.setLifetime(2.);
	obstacle.setLifetime(2.);
	
	// Init. Trunk
	robot_trunk.setColor(0.0, 0.0, 0.0);
	robot_trunk.setPosition(1.,1.,0.+ROBOT_HEIGHT/2.);

	// Init. Head
	robot_head.setColor(0.0, 0.0, 0.0);
	robot_head.setPosition(0.,0.,0.);

	// Init. Name Label
	robot_name.setText(robot);
	robot_name.setPosition(0., 0., 0.);
	robot_name.setColor(0.,0.,0.);

	//Init. Ball Target Marker
	ball.setPosition(0.,0.,0.);
	ball.setColor(1.,1.,1.);
	
	// Init. eye markers
	eye1.setColor(0.,0.,0.);
	eye2.setColor(0.,0.,0.);
	
	// Init. compass
	compass.setColor(.25, .25, .25);
	northLabel.setColor(.25, .25, .25);
	northLabel.setText("N");
	
	// Init. Walk Target cross
	wt_cross1.setColor(1., 0., 0.);
	wt_cross2.setColor(1., 0., 0.);
	
	// Init. Stable text
	StableText.setText("Stable");
	
	// Init. WAK Text Markers
	ModeStateText.setPosition(0.0, 0.0, TEXT_POSZ_MODE);
	GameStateText.setPosition(0.0, 0.0, TEXT_POSZ_GAME);
	BehStateText.setPosition(0.0, 0.0, TEXT_POSZ_BEH);
	RoleText.setPosition(0., 0., TEXT_POSZ_ROLE);
	ModeStateText.setColor(0.8, 0.0, 0.0);
	GameStateText.setColor(0.0, 0.25, 0.0);
	BehStateText.setColor(0.6, 0.0, 1.0);
	RoleText.setColor(.3, .3, 1.);
	//robot_head.setScale(HEAD_DIAMETER);
}

void RobotMarkerManager::configurePlotManager()
{
	m_PM.setName(PM_ENABLED,                  "enabled");
	m_PM.setName(PM_TIME_SINCE_LAST_RECEIVED, "time_last_received");
	m_PM.setName(PM_FIELD_TYPE,               "config/field_type");
	m_PM.setName(PM_ON_YELLOW,                "config/play_on_yellow");
	m_PM.setName(PM_AS_CYAN,                  "config/play_as_cyan");
	m_PM.setName(PM_LISTEN_TO_GC,             "config/listen_to_GC");
	m_PM.setName(PM_LISTEN_TO_TC,             "config/listen_to_TC");
	m_PM.setName(PM_IS_PENALTY_SHOOT,         "config/is_penalty_shoot");
	m_PM.setName(PM_KICKOFF_TYPE,             "config/kickoff_type");
	m_PM.setName(PM_BUTTON_STATE,             "play_state/button_state");
	m_PM.setName(PM_GAME_COMMAND,             "play_state/game_command");
	m_PM.setName(PM_GAME_ROLE,                "play_state/game_role");
	m_PM.setName(PM_PLAY_STATE,               "play_state/play_state");
	m_PM.setName(PM_GAME_STATE,               "play_state/game_state");
	m_PM.setName(PM_BEH_STATE,                "play_state/beh_state");
	m_PM.setName(PM_FALLEN,                   "play_state/fallen");
	m_PM.setName(PM_COMPASS_HEADING,          "pose/compass_heading");
	m_PM.setName(PM_ROBOT_POSE_X,             "pose/x");
	m_PM.setName(PM_ROBOT_POSE_Y,             "pose/y");
	m_PM.setName(PM_ROBOT_POSE_T,             "pose/t");
	m_PM.setName(PM_ROBOT_POSE_CONF,          "pose/conf");
	m_PM.setName(PM_ROBOT_POSE_VALID,         "pose/valid");
	m_PM.setName(PM_BALL_POSE_X,              "ball/x");
	m_PM.setName(PM_BALL_POSE_Y,              "ball/y");
	m_PM.setName(PM_BALL_POSE_CONF,           "ball/conf");
	m_PM.setName(PM_BALL_POSE_VALID,          "ball/valid");
	m_PM.setName(PM_BALL_POSE_STABLE,         "ball/stable");
	m_PM.setName(PM_OBSTACLE_CLOSEST_X,       "obstacle_closest/x");
	m_PM.setName(PM_OBSTACLE_CLOSEST_Y,       "obstacle_closest/y");
	m_PM.setName(PM_OBSTACLE_CLOSEST_CONF,    "obstacle_closest/conf");
	m_PM.setName(PM_OBSTACLE_CLOSEST_VALID,   "obstacle_closest/valid");
	m_PM.setName(PM_WALKING_TARGET_X,         "walking_target/x");
	m_PM.setName(PM_WALKING_TARGET_Y,         "walking_target/y");
	m_PM.setName(PM_WALKING_TARGET_VALID,     "walking_target/valid");
	m_PM.setName(PM_TIME_SINCE_GC_BASE,       "gc/time_since_base");
	m_PM.setName(PM_TIME_SINCE_GC_EXTRA,      "gc/time_since_extra");
	m_PM.setName(PM_TIME_SINCE_TC,            "tc/time_since");
	m_PM.setName(PM_NUM_FRESH_TC,             "tc/num_fresh");
	
	if(!m_PM.checkNames())
		ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
}


void RobotMarkerManager::updateMarkerXYZ(vis_utils::GenMarker& marker, bool show, float x, float y, float z, float rot, float a)
{
	// Update the marker as required
	if(show)
	{
		marker.marker.pose.position.x = x;
		marker.marker.pose.position.y = y;
		marker.marker.pose.position.z = z;
		
		marker.marker.pose.orientation.x = 0;
		marker.marker.pose.orientation.y = 0;
		marker.marker.pose.orientation.z = sin(rot/2.);
		marker.marker.pose.orientation.w = cos(rot/2.);
		marker.marker.color.a = std::max(a, float(.4)); // Make sure stuff does not just disappear
		marker.show();
	}
	else
		marker.hide();
	marker.updateAdd();
}

//
// General Marker Manager
//

GeneralMarkerManager::GeneralMarkerManager()
 : MarkerManager("~general_markers", 1, true)
 , behField("/beh_field")
 , field_dim()
 , m_plotData("/bench_vis/general/plotData", false)
 , m_PM(PM_COUNT, "/bench_vis/general")
{
	configurePlotManager();
}


void GeneralMarkerManager::configurePlotManager()
{
	m_PM.setName(PM_GC_SEQID,                   "gc/seqid");
	m_PM.setName(PM_GC_TIMESINCEPACKETBASE,     "gc/time_since_base");
	m_PM.setName(PM_GC_TIMESINCEPACKETEXTRA,    "gc/time_since_extra");
	m_PM.setName(PM_GC_EXTRAOUTOFDATE,          "gc/extra_out_of_date");
	m_PM.setName(PM_GC_GAMEPHASE,               "gc/gamephase");
	m_PM.setName(PM_GC_GAMESTATE,               "gc/gamestate");
	m_PM.setName(PM_GC_TIMEPLAYING,             "gc/time_playing");
	m_PM.setName(PM_GC_KICKOFFTYPE,             "gc/kickoff_type");
	m_PM.setName(PM_GC_TIMEREMAINING_RAW,       "gc/time_remaining_raw");
	m_PM.setName(PM_GC_TIMEREMAINING_SMOOTH,    "gc/time_remaining_smooth");
	m_PM.setName(PM_GC_SECONDARYTIME_RAW,       "gc/secondary_time_raw");
	m_PM.setName(PM_GC_SECONDARYTIME_SMOOTH,    "gc/secondary_time_smooth");
	m_PM.setName(PM_GC_TIMETOBALLINPLAY_RAW,    "gc/time_to_ball_in_play_raw");
	m_PM.setName(PM_GC_TIMETOBALLINPLAY_SMOOTH, "gc/time_to_bal_in_play_smooth");
	m_PM.setName(PM_GC_OWNPENALTYSTATE,         "gc/own_penalty_state");
	m_PM.setName(PM_GC_OWNPENALTYTIME_RAW,      "gc/own_penalty_time_raw");
	m_PM.setName(PM_GC_OWNPENALTYTIME_SMOOTH,   "gc/own_penalty_time_smooth");
	m_PM.setName(PM_GC_OWNSCORE,                "gc/own_score");
	m_PM.setName(PM_GC_OPPSCORE,                "gc/opp_score");
	m_PM.setName(PM_GC_OWNNUMPLAYING,           "gc/own_num_playing");
	m_PM.setName(PM_GC_OPPNUMPLAYING,           "gc/opp_num_playing");
	
	if(!m_PM.checkNames())
		ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
}

//
// BenchVis class
//

// Constructor
BenchVis::BenchVis(ros::NodeHandle& nh)
 : m_nh(nh)
 , GCRI(nh)
 , GC(NULL, NULL, NULL)
 , m_GCDataFresh(false)
 , TCRI(nh)
 , TC(TCRI, NULL, NULL, NULL)
 , gmm()
 , rmm_map()
{
  
	// Reset the logger state variables
	m_loggerMsg.sourceNode = "bench_vis";
	m_loggerMsg.enableLogging = false;
	zeroRosTime(m_loggerStamp);

	// ROS publishers
	m_pub_logger = m_nh.advertise<rrlogger::LoggerHeartbeat>("/bvlogger/heartbeat", 1);

	// Start listening to the game controller packets
	GCRI.startListening();

	// Start listening to the team communications packets
	TCRI.startListening(true);
	
	last_packetID = 0;
}

// Declare the robot colors. Inspired by London Underground map.
const float BenchVis::colors[ROBOT_LIMIT+1][3] = {
  {17.,57.,155.},   // Liverpool St. 
  {228.,55.,31.},   // Waterloo
  {78.,154.,104.},  // Euston
  {132.,40.,111.},  // Marylebone
  {132.,103.,45.},  // London Bridge
  {170.,205.,109.}, // Fenchurch St.
  {207.,75.,135.},  // King's Cross
  {109.,51.,52.},   // Paddington
  {56.,120.,132.},  // Cannon St.
  {220.,153.,174.}, // St. Pancras Int.
  {48.,118.,135.}   // Victoria
};


// Step function
void BenchVis::step()
{
	// Retrieve the current ROS time
	ros::Time now = ros::Time::now();

	// Update the game controller data
	m_GCDataFresh = GC.update(GCRI.data(), now);
	
	// Update the team communications
	TC.update(now, NULL);

	// Test that the game controller data is arriving
	static unsigned int lastSeqID = -1;
	
	if(GC.seqID != lastSeqID)
	{
		gmm.m_PM.clear(now);
		if(gmm.m_plotData() == true){
		gmm.m_PM.plotScalar((GC.seqID % 11) * 0.1, PM_GC_SEQID);
		gmm.m_PM.plotScalar((GC.stampBase.isZero() ? INFINITY : (now - GC.stampBase).toSec()), PM_GC_TIMESINCEPACKETBASE);
		gmm.m_PM.plotScalar((GC.stampExtra.isZero() ? INFINITY : (now - GC.stampExtra).toSec()), PM_GC_TIMESINCEPACKETEXTRA);
		gmm.m_PM.plotScalar(GC.extraOutOfDate, PM_GC_EXTRAOUTOFDATE);
		gmm.m_PM.plotScalar(GC.gamePhase, PM_GC_GAMEPHASE);
		gmm.m_PM.plotScalar(GC.gameState, PM_GC_GAMESTATE);
		gmm.m_PM.plotScalar(GC.kickoffType, PM_GC_KICKOFFTYPE);
		gmm.m_PM.plotScalar(GC.timeRemaining.raw, PM_GC_TIMEREMAINING_RAW);
		gmm.m_PM.plotScalar(GC.timeRemaining.smooth, PM_GC_TIMEREMAINING_SMOOTH);
		gmm.m_PM.plotScalar(GC.secondaryTime.raw, PM_GC_SECONDARYTIME_RAW);
		gmm.m_PM.plotScalar(GC.secondaryTime.smooth, PM_GC_SECONDARYTIME_SMOOTH);
		gmm.m_PM.plotScalar(GC.timeToBallInPlay.raw, PM_GC_TIMETOBALLINPLAY_RAW);
		gmm.m_PM.plotScalar(GC.timeToBallInPlay.smooth, PM_GC_TIMETOBALLINPLAY_SMOOTH);
		gmm.m_PM.plotScalar(GC.ownPenaltyState, PM_GC_OWNPENALTYSTATE);
		gmm.m_PM.plotScalar(GC.ownPenaltyTimeRemaining.raw, PM_GC_OWNPENALTYTIME_RAW);
		gmm.m_PM.plotScalar(GC.ownPenaltyTimeRemaining.smooth, PM_GC_OWNPENALTYTIME_SMOOTH);
		gmm.m_PM.plotScalar(GC.ownTeam.score, PM_GC_OWNSCORE);
		gmm.m_PM.plotScalar(GC.ownTeam.score, PM_GC_OPPSCORE);
		gmm.m_PM.plotScalar(GC.ownTeam.numPlaying, PM_GC_OWNNUMPLAYING);
		gmm.m_PM.plotScalar(GC.ownTeam.numPlaying, PM_GC_OPPNUMPLAYING);
		
		gmm.m_PM.publish();
	}
		lastSeqID = GC.seqID;
	}

	for(TCRobotDataMap::iterator it = TC.robotDataMap.begin(); it != TC.robotDataMap.end(); ++it)
	{
		
		// Check if there was ever a packet recieved
		if(pID_to_UID.find(it->second->robotUID) == pID_to_UID.end()){
			pID_to_UID.insert(PackID_to_UID_map::value_type(it->second->robotUID, it->second->data.packetID));
		}
		
		// Is the recieved packetID new? 
		if(it->second->data.packetID > pID_to_UID.at(it->second->robotUID) || it->second->data.packetID == 0){
			// Do stuff and be happy: New packet!

			// Increment packet count for next iteration
			pID_to_UID[it->second->robotUID] = it->second->data.packetID;
			
			// Look up robot UID in RMM Map
			if(rmm_map.find(it->second->robotUID) == rmm_map.end()){
				// RMM was not found in map:
				// So create one and insert it!
				RMM_ptr new_rmm(new RobotMarkerManager(it->second->robot));
				
				// Update Colors, matching the robots UID
				float r, g, b;
				
				// Paint the robot black if his UID is out of range.
				if(it->second->robotUID > ROBOT_LIMIT || it->second->robotUID < 0){
					r = 0.;
					g = 0.;
					b = 0.;
				}
				else{
					r = colors[it->second->robotUID][0]/0xFF;
					g = colors[it->second->robotUID][1]/0xFF;
					b = colors[it->second->robotUID][2]/0xFF;
				}
				
				new_rmm->robot_trunk.setColor(r,g,b);
				new_rmm->robot_head.setColor(r,g,b);
				new_rmm->ball.setColor(r,g,b);
				new_rmm->goalMarker.setColor(r,g,b);
				new_rmm->StableText.setColor(r,g,b);
				new_rmm->wt_cross1.setColor(r,g,b);
				new_rmm->wt_cross2.setColor(r,g,b);
				new_rmm->obstacle.setColor(r,g,b);
				
				ROS_INFO("Registering Robot Marker for %s (UID %d - Color (%f, %f, %f))", it->second->robot.c_str(), it->second->robotUID, r,g,b);
				
				rmm_map.insert(RMM_map::value_type(it->second->robotUID, new_rmm));
			}
			
			
			// Pick out the RMM pointer that belongs to the iterator
			RMM_ptr found_rmm =  rmm_map.at(it->second->robotUID);

			
			
			// ************************* 1 - Plot raw data *************************
			
			// Clear plot
			found_rmm->m_PM.clear(now);
			if(found_rmm->m_plotData() == true){
				found_rmm->last_recieved = it->second->data.timestamp;
				
				
				found_rmm->m_PM.plotScalar(it->second->fieldType, RobotMarkerManager::PM_FIELD_TYPE);
				found_rmm->m_PM.plotScalar(it->second->data.playOnYellow, RobotMarkerManager::PM_ON_YELLOW);
				found_rmm->m_PM.plotScalar(it->second->data.playAsCyan, RobotMarkerManager::PM_AS_CYAN);
				found_rmm->m_PM.plotScalar(it->second->data.listenToGC, RobotMarkerManager::PM_LISTEN_TO_GC);
				found_rmm->m_PM.plotScalar(it->second->data.listenToTC, RobotMarkerManager::PM_LISTEN_TO_TC);
				found_rmm->m_PM.plotScalar(it->second->data.isPenaltyShoot, RobotMarkerManager::PM_IS_PENALTY_SHOOT);
				found_rmm->m_PM.plotScalar(it->second->kickoffType, RobotMarkerManager::PM_KICKOFF_TYPE);
				found_rmm->m_PM.plotScalar(it->second->buttonState, RobotMarkerManager::PM_BUTTON_STATE);
				found_rmm->m_PM.plotScalar(it->second->gameCommand, RobotMarkerManager::PM_GAME_COMMAND);
				found_rmm->m_PM.plotScalar(it->second->gameRole, RobotMarkerManager::PM_GAME_ROLE);
				found_rmm->m_PM.plotScalar(it->second->playState, RobotMarkerManager::PM_PLAY_STATE);
				found_rmm->m_PM.plotScalar(it->second->data.gameState, RobotMarkerManager::PM_GAME_STATE);
				found_rmm->m_PM.plotScalar(it->second->data.behState, RobotMarkerManager::PM_BEH_STATE);
				found_rmm->m_PM.plotScalar(it->second->data.fallen, RobotMarkerManager::PM_FALLEN);
				found_rmm->m_PM.plotScalar(it->second->data.compassHeading, RobotMarkerManager::PM_COMPASS_HEADING);
				found_rmm->m_PM.plotScalar(it->second->data.robotPoseX, RobotMarkerManager::PM_ROBOT_POSE_X);
				found_rmm->m_PM.plotScalar(it->second->data.robotPoseY, RobotMarkerManager::PM_ROBOT_POSE_Y);
				found_rmm->m_PM.plotScalar(it->second->data.robotPoseT, RobotMarkerManager::PM_ROBOT_POSE_T);
				found_rmm->m_PM.plotScalar(it->second->data.robotPoseConf, RobotMarkerManager::PM_ROBOT_POSE_CONF);
				found_rmm->m_PM.plotScalar(it->second->data.robotPoseValid, RobotMarkerManager::PM_ROBOT_POSE_VALID);
				found_rmm->m_PM.plotScalar(it->second->data.ballPoseX, RobotMarkerManager::PM_BALL_POSE_X);
				found_rmm->m_PM.plotScalar(it->second->data.ballPoseY, RobotMarkerManager::PM_BALL_POSE_Y);
				found_rmm->m_PM.plotScalar(it->second->data.ballPoseConf, RobotMarkerManager::PM_BALL_POSE_CONF);
				found_rmm->m_PM.plotScalar(it->second->data.ballPoseValid, RobotMarkerManager::PM_BALL_POSE_VALID);
				found_rmm->m_PM.plotScalar(it->second->data.ballPoseStable, RobotMarkerManager::PM_BALL_POSE_STABLE);
				found_rmm->m_PM.plotScalar(it->second->data.obstClosestPoseX, RobotMarkerManager::PM_OBSTACLE_CLOSEST_X);
				found_rmm->m_PM.plotScalar(it->second->data.obstClosestPoseY, RobotMarkerManager::PM_OBSTACLE_CLOSEST_Y);
				found_rmm->m_PM.plotScalar(it->second->data.obstClosestPoseConf, RobotMarkerManager::PM_OBSTACLE_CLOSEST_CONF);
				found_rmm->m_PM.plotScalar(it->second->data.obstClosestPoseValid, RobotMarkerManager::PM_OBSTACLE_CLOSEST_VALID);
				found_rmm->m_PM.plotScalar(it->second->data.walkingTargetPoseX, RobotMarkerManager::PM_WALKING_TARGET_X);
				found_rmm->m_PM.plotScalar(it->second->data.walkingTargetPoseY, RobotMarkerManager::PM_WALKING_TARGET_Y);
				found_rmm->m_PM.plotScalar(it->second->data.walkingTargetPoseValid, RobotMarkerManager::PM_WALKING_TARGET_VALID);
				found_rmm->m_PM.plotScalar(it->second->data.timeSinceGCBase, RobotMarkerManager::PM_TIME_SINCE_GC_BASE);
				found_rmm->m_PM.plotScalar(it->second->data.timeSinceGCExtra, RobotMarkerManager::PM_TIME_SINCE_GC_EXTRA);
				found_rmm->m_PM.plotScalar(it->second->data.timeSinceTC, RobotMarkerManager::PM_TIME_SINCE_TC);
				found_rmm->m_PM.plotScalar(it->second->data.numFreshTC, RobotMarkerManager::PM_NUM_FRESH_TC);
			}
			
			// ************************* 2 - Work with markers *************************
			
			// Clear Markers of the RMM
			found_rmm->clear(now);
			
			// Set up the pose coordinates 
			float rx = it->second->data.robotPoseX;
			float ry = it->second->data.robotPoseY;
			float rw = it->second->data.robotPoseT;
			std::string err = "";
			
			// Move robot out if he is completely invalid
			if(!bool(it->second->dataValid)){
				rx = std::pow(-1, it->second->robotUID-1)*(it->second->robotUID-1)*ROBOT_DY/.2;
				ry = found_rmm->field_dim.fieldWidth()+HARD_ERROR_PAD;
				rw = -M_PI_2;
				err = " ("+ it->second->reasonsInvalidString(it->second->reasonInvalid, ", ")+(it->second->data.robotPoseValid == 0 ? ", NotLocalised)" : ")");
			}
			
			// Move robot to the field border if his pose is invalid
			else if(!bool(it->second->data.robotPoseValid)){
				rx = std::pow(-1, it->second->robotUID-1)*(it->second->robotUID-1)*ROBOT_DY/.2;
				ry = found_rmm->field_dim.fieldWidth()+SOFT_ERROR_PAD;
				rw = it->second->data.compassHeading;
				err = " (NotLocalised)";
			}
			
			// Display Robot name with Player Number (and potential error!)
			found_rmm->robot_name.setText((it->second->data.playAsCyan != 0 ? "Blue " : "Red ")+boost::lexical_cast<std::string>(int(it->second->data.robotNumber))+": "+it->second->robot+err);
			
			// WAK Label Text update
			found_rmm->BehStateText.setText(WAKBehManager::behStateName(it->second->data.behState));
			found_rmm->GameStateText.setText(WAKGameManager::gameStateName(it->second->data.gameState)+": "+ playStateName(PlayState(it->second->data.playState)));
			found_rmm->ModeStateText.setText((bool(it->second->data.listenToGC) ? "Extern " : "Local ") + buttonStateName(ButtonState(it->second->data.buttonState)));
			found_rmm->RoleText.setText((bool(it->second->data.isPenaltyShoot) ? "Penalty " : "") + gameRoleName(GameRole(it->second->data.gameRole))+ ": "+kickoffTypeName(KickoffType(it->second->data.kickoffType)));
			
			// Change Eye-Color depending on Team Color
			if(bool(it->second->data.playAsCyan)){
				found_rmm->eye1.setColor(0.,1.,1.);
				found_rmm->eye2.setColor(0.,1.,1.);
			}
			else{
				found_rmm->eye1.setColor(1.,0.,1.);
				found_rmm->eye2.setColor(1.,0.,1.);
			}
			
			
			// NOTE: Keep everything that manually changes marker attributes above here. (to keep things clean)
			// Update the corresponding Markers
			
			// Robot
			
			if(!bool(it->second->data.fallen)){
				found_rmm->updateMarkerXYZ(found_rmm->robot_trunk, true, rx, ry, ROBOT_HEIGHT/2., rw, it->second->data.robotPoseConf);
				found_rmm->updateMarkerXYZ(found_rmm->robot_head, true, rx, ry, ROBOT_HEIGHT+1./2.*HEAD_DIAMETER, 0., it->second->data.robotPoseConf);
				found_rmm->updateMarkerXYZ(
					found_rmm->robot_name, 
					true, 
					rx, 
					ry, 
					ROBOT_HEIGHT+1./2.*HEAD_DIAMETER+TEXT_MARKER_DIST);
				found_rmm->updateMarkerXYZ(found_rmm->eye1, 
										true, 
										rx+(HEAD_DIAMETER/2.-EYE_INLINE)*cos((-1.)*rw)+(0+EYE_DISTANCE/2.)*sin((-1.)*rw),
										ry-(HEAD_DIAMETER/2.-EYE_INLINE)*sin((-1.)*rw)+(0+EYE_DISTANCE/2.)*cos((-1.)*rw),
										ROBOT_HEIGHT+1./2.*HEAD_DIAMETER+EYE_DIAMETER/2.,
										0.,
										it->second->data.robotPoseConf
										);
				found_rmm->updateMarkerXYZ(found_rmm->eye2, 
										true, 
										rx+(HEAD_DIAMETER/2.-EYE_INLINE)*cos((-1.)*rw)+(0-EYE_DISTANCE/2.)*sin((-1.)*rw),
										ry-(HEAD_DIAMETER/2.-EYE_INLINE)*sin((-1.)*rw)+(0-EYE_DISTANCE/2.)*cos((-1.)*rw),
										ROBOT_HEIGHT+1./2.*HEAD_DIAMETER+EYE_DIAMETER/2.,
										0.,
										it->second->data.robotPoseConf
										);
			}
			else{
				rot_conv::Quat q;
				q = rot_conv::QuatFromFused(it->second->data.robotPoseT,M_PI/2.,0.);
				
				found_rmm->robot_trunk.setOrientation(q.x(), q.y(), q.z(), q.w());
				found_rmm->robot_trunk.marker.pose.position.x = rx;
				found_rmm->robot_trunk.marker.pose.position.y = ry;
				found_rmm->robot_trunk.marker.pose.position.z = ROBOT_WIDTH/2.;
				found_rmm->robot_trunk.updateAdd();
				
				found_rmm->robot_head.marker.pose.position.x = rx+(ROBOT_HEIGHT/2.+HEAD_DIAMETER/2.)*cos(-1.*rw);
				found_rmm->robot_head.marker.pose.position.y = ry-(ROBOT_HEIGHT/2.+HEAD_DIAMETER/2.)*sin(-1.*rw);
				found_rmm->robot_head.marker.pose.position.z = HEAD_DIAMETER/2.;
				found_rmm->robot_head.updateAdd();
				
				found_rmm->eye1.marker.pose.position.x = rx+(ROBOT_HEIGHT/2.+HEAD_DIAMETER/2.)*cos(-1.*rw)+(0-EYE_DISTANCE/2.)*sin((-1.)*rw);
				found_rmm->eye1.marker.pose.position.y = ry-(ROBOT_HEIGHT/2.+HEAD_DIAMETER/2.)*sin(-1.*rw)+(0-EYE_DISTANCE/2.)*cos((-1.)*rw);
				found_rmm->eye1.marker.pose.position.z = EYE_INLINE;
				found_rmm->eye1.updateAdd();
				
				found_rmm->eye2.marker.pose.position.x = rx+(ROBOT_HEIGHT/2.+HEAD_DIAMETER/2.)*cos(-1.*rw)+(0+EYE_DISTANCE/2.)*sin((-1.)*rw);
				found_rmm->eye2.marker.pose.position.y = ry-(ROBOT_HEIGHT/2.+HEAD_DIAMETER/2.)*sin(-1.*rw)+(0+EYE_DISTANCE/2.)*cos((-1.)*rw);
				found_rmm->eye2.marker.pose.position.z = EYE_INLINE;
				found_rmm->eye2.updateAdd();
			}
			
			
			// WAK Text marker pos. update
			
			found_rmm->updateMarkerXYZ(
				found_rmm->RoleText,
				true,
				rx,
				ry,
				TEXT_POSZ_ROLE,
				0.,
				1.
			);
			found_rmm->updateMarkerXYZ(
				found_rmm->ModeStateText,
				true,
				rx,
				ry,
				TEXT_POSZ_MODE,
				0.,
				1.
			);
			
			found_rmm->updateMarkerXYZ(
				found_rmm->GameStateText,
				true,
				rx,
				ry,
				TEXT_POSZ_GAME,
				0.,
				1.
			);
			
			found_rmm->updateMarkerXYZ(
				found_rmm->BehStateText,
				true,
				rx,
				ry,
				TEXT_POSZ_BEH,
				0.,
				1.
			);
			
			
			if(bool(it->second->data.walkingTargetPoseValid)){
				found_rmm->wt_cross1.marker.pose.position.x = it->second->data.walkingTargetPoseX;
				found_rmm->wt_cross1.marker.pose.position.y = it->second->data.walkingTargetPoseY;
				found_rmm->wt_cross1.marker.pose.orientation.x = 0;
				found_rmm->wt_cross1.marker.pose.orientation.y = 0;
				found_rmm->wt_cross1.marker.pose.orientation.z = sin(rw/2.);
				found_rmm->wt_cross1.marker.pose.orientation.w = cos(rw/2.);
				
				
				found_rmm->wt_cross2.marker.pose.position.x = it->second->data.walkingTargetPoseX;
				found_rmm->wt_cross2.marker.pose.position.y = it->second->data.walkingTargetPoseY;
				found_rmm->wt_cross2.marker.pose.orientation.x = 0;
				found_rmm->wt_cross2.marker.pose.orientation.y = 0;
				found_rmm->wt_cross2.marker.pose.orientation.z = sin(rw/2.);
				found_rmm->wt_cross2.marker.pose.orientation.w = cos(rw/2.);
				
				found_rmm->wt_cross1.updateAdd();
				found_rmm->wt_cross2.updateAdd();
			}
			
			
			// Rest
			
			found_rmm->updateMarkerXYZ(found_rmm->StableText,
						   bool(it->second->data.ballPoseStable),
						   it->second->data.ballPoseX,
						   it->second->data.ballPoseY,
						   BALL_DIAMETER+STABLE_TEXT_SIZE,
						   0.,
						   1.
			);
			
			found_rmm->updateMarkerXYZ(found_rmm->ball, 
						   bool(it->second->data.ballPoseValid), 
						   it->second->data.ballPoseX, 
						   it->second->data.ballPoseY, 
						   (1./2.)*BALL_DIAMETER, 
						   0., 
						   it->second->data.ballPoseConf);

			float northAngle = rw - it->second->data.compassHeading;
			found_rmm->compass.setPosition(rx+cos(northAngle)*HEADING_DISTANCE, ry+sin(northAngle)*HEADING_DISTANCE, HEADING_HEIGHT);
			found_rmm->compass.update((1./2.)*cos(northAngle), (1./2.)*sin(northAngle), 0.0);
			found_rmm->compass.updateAdd();
			
			found_rmm->northLabel.update(rx+cos(northAngle)*(HEADING_DISTANCE+1./2.), ry+sin(northAngle)*(HEADING_DISTANCE+1./2.), HEADING_HEIGHT+HEADING_HEAD_DIAM);
			found_rmm->northLabel.updateAdd();
			
			found_rmm->updateMarkerXYZ(found_rmm->obstacle,
						   bool(it->second->data.obstClosestPoseValid),
						   it->second->data.obstClosestPoseX,
						   it->second->data.obstClosestPoseY,
						   OBSTACLE_HEIGHT/2.,
						   0.,
						   it->second->data.obstClosestPoseConf
			);
			
			
			// Update marker that indicates the goal target
			float goalmarker_y;
			goalmarker_y = found_rmm->field_dim.fieldWidthH() - (found_rmm->field_dim.fieldWidth()/float(ROBOT_LIMIT))*(float(it->second->robotUID));
			
			if(it->second->data.playOnYellow){
				found_rmm->updateMarkerXYZ(found_rmm->goalMarker, true, found_rmm->field_dim.fieldLengthH()+found_rmm->field_dim.boundary()/2., goalmarker_y, GOAL_MARKER_HEIGHT/2.);
			}
			else{
				found_rmm->updateMarkerXYZ(found_rmm->goalMarker, true, (-1.)*found_rmm->field_dim.fieldLengthH()-found_rmm->field_dim.boundary()/2., goalmarker_y, GOAL_MARKER_HEIGHT/2.);
			}
			
			
			// Publish the updates
			found_rmm->publish();
		}
	}
	
	for(RMM_map::iterator it = rmm_map.begin(); it != rmm_map.end(); ++it){
		if(it->second->m_plotData() == true){
			float time_diff = now.toSec()-it->second->last_recieved.toSec();
			it->second->m_PM.plotScalar(time_diff, RobotMarkerManager::PM_TIME_SINCE_LAST_RECEIVED);
			
			// Publish all plotters!
			it->second->m_PM.publish();
		}
	}
	
	// Update the logger state
	updateLoggerState();
}

// Publish the default calculated logger state
void BenchVis::updateLoggerState()
{
	// Initialise whether we should be logging based on what we are already doing
	bool shouldLog = m_loggerMsg.enableLogging;

	// Decide whether we should be logging
	if(m_GCDataFresh)
		shouldLog = (GC.gamePhase != GCVars::GP_TIMEOUT && GC.gameState != GCVars::GS_INITIAL && GC.gameState != GCVars::GS_FINISHED);

	// Force logging if the relevant config says so
	if(config.forceLogging())
		shouldLog = true;

	// Publish the required logger state
	publishLoggerState(shouldLog);
}

// Publish a forced logger state
void BenchVis::publishLoggerState(bool log)
{
	// Get the current ROS time
	ros::Time now = ros::Time::now();

	// See whether we have new information to publish
	bool newState = (log != m_loggerMsg.enableLogging);
	bool newTime = ((now - m_loggerStamp).toSec() >= 0.4);

	// Publish the required logger state if we have reason to
	if(newState || newTime)
	{
		m_loggerStamp = now;
		m_loggerMsg.enableLogging = log;
		m_pub_logger.publish(m_loggerMsg);
	}
}

//
// Main function
//

// Main function
int main(int argc, char** argv)
{
	// Initialise ROS
	ros::init(argc, argv, "bench_vis");

	// Retrieve a ROS node handle
	ros::NodeHandle nh("~");

	// Construct a bench vis class object
	BenchVis BV(nh);

	// Initialise timer object for loop rate timing
	MotionTimer timer(1/20.0);

	// Main loop
	while(ros::ok())
	{
		// Do all ROS callbacks
		ros::spinOnce();

		// Sleep for the required duration
		uint64_t expirations = timer.sleep();

		// Execute the bench visualisation
		BV.step();

		// Check whether any cycles were missed
		if(expirations > 1)
			ROS_WARN("Bench visualisation missed %lu timer cycles", expirations - 1);
	}

	// Return success
	return 0;
}
// EOF