// Walk and kick: Plotter variable definitions
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_PLOT_H
#define WAK_PLOT_H

// Includes
#include <plot_msgs/plot_manager.h>
#include <ros/time.h>
#include <string>

// Defines - Plot variable scalers
#define PMSCALE_CYCLE               1000      // Scaler for the cycle plot variables
#define PMSCALE_HALT                0.9       // Scaler for the halt plot variables
#define PMSCALE_KICK                0.8       // Scaler for the kick plot variables
#define PMSCALE_DRIBBLE             1.0       // Scaler for the dribble plot variables
#define PMSCALE_HAVE                1.1       // Scaler for the have plot variables
#define PMSCALE_STABLE              0.8       // Scaler for the stable plot variables
#define PMSCALE_HAVETARGET          1.2       // Scaler for the have target plot variables
#define PMSCALE_OKTOKDB             0.7       // Scaler for the ok to kick/dribble plot variables
#define PMSCALE_STILLOKTOKDB        0.65      // Scaler for the still ok to kick/dribble plot variables
#define PMSCALE_COULDKDB            0.6       // Scaler for the could kick/dribble plot variables
#define PMSCALE_ALLOWKDB            0.5       // Scaler for the allow kick/dribble plot variables
#define PMSCALE_BALLACT             0.4       // Scaler for the ball action plot variables
#define PMSCALE_WTGPBOOL            0.95      // Scaler for the walk to global pose boolean plot variables
#define PMSCALE_WALK                0.85      // Scaler for the walk plot variables
#define PMSCALE_MOTION              0.75      // Scaler for the motion plot variables
#define PMSCALE_FOOTSEL             0.45      // Scaler for the foot selection plot variables
#define PMSCALE_LOCK                0.35      // Scaler for the lock plot variables
#define PMSCALE_FACINGOK            1.1       // Scaler for the facing ok plot variables
#define PMSCALE_FOOTOK              1.2       // Scaler for the foot ok plot variables
#define PMSCALE_ARRIVED             1.25      // Scaler for the arrived plot variables
#define PMSCALE_ISRESUMED           1.30      // Scaler for the is resumed plot variables
#define PMSCALE_COUNTER             0.01      // Scaler for the counter plot variables
#define PMSCALE_KICKOFF             0.8       // Scaler for the kickoff plot variables
#define PMSCALE_DROPBALL            0.9       // Scaler for the drop ball plot variables
#define PMSCALE_COMMSOK             1.15      // Scaler for the communications ok plot variables
#define PMSCALE_COLOR               0.85      // Scaler for the color plot variables
#define PMSCALE_PENALTY             0.95      // Scaler for the penalty plot variables
#define PMSCALE_LEGAL               0.75      // Scaler for the legal plot variables

// Walk and kick namespace
namespace walk_and_kick
{
	// Walk and kick plot manager IDs enumeration
	enum WAKPMIDS
	{
		PM_WAK_TRUEDT = 0,
		PM_WAK_CYCLE,

		PM_RI_COMMSOK,
		PM_RI_PUBGAZEPITCH,
		PM_RI_PUBGAZEYAW,
		PM_RI_PUBGCVX,
		PM_RI_PUBGCVY,
		PM_RI_PUBGCVZ,
		PM_RI_PUBWALK,
		PM_RI_PUBKICKLEFT,
		PM_RI_PUBKICKRIGHT,

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

		PM_TC_NUMFRESH,
		PM_TC_NUMFRESHVALID,
		PM_TC_TIMESINCEDATA,

		PM_SV_ISPENALTYSHOOT,
		PM_SV_ISPENALTYTAKER,
		PM_SV_KICKOFFTYPE,
		PM_SV_GOALSIGN,
		PM_SV_PLAYASCYAN,
		PM_SV_GAMECOMMAND,
		PM_SV_GAMEROLE,
		PM_SV_PLAYSTATE,
		PM_SV_PLAYSTATEFIRSTIN,
		PM_SV_PLAYSTATEFIRSTTOLD,
		PM_SV_PLAYSTATELASTTOLD,
		PM_SV_PLAYSTATETIMEOUT,
		PM_SV_TIMEPLAYING,
		PM_SV_COMPASSHEADING,
		PM_SV_ROBOTPOSECONF,
		PM_SV_HAVEROBOTPOSE,
		PM_SV_ROBOTPOSETIMEAGO,
		PM_SV_ROBOTPOSEDUR,
		PM_SV_ROBOTPOSEX,
		PM_SV_ROBOTPOSEY,
		PM_SV_ROBOTPOSEZ,
		PM_SV_BALLCONF,
		PM_SV_HAVEBALL,
		PM_SV_BALLTIMEAGO,
		PM_SV_BALLDUR,
		PM_SV_BALLX,
		PM_SV_BALLY,
		PM_SV_BALLANGLE,
		PM_SV_BALLDIST,
		PM_SV_BALLSTABLE,
		PM_SV_HAVEBALLPOSE,
		PM_SV_BALLPOSETIMEAGO,
		PM_SV_BALLPOSEDUR,
		PM_SV_BALLPOSEX,
		PM_SV_BALLPOSEY,
		PM_SV_BALLPOSESTABLE,
		PM_SV_BALLHASMOVED,
		PM_SV_BALLINPLAY,
		PM_SV_DIRECTGOALALLOWED,

		PM_GV_FORCEBEHSTATE,
		PM_GV_SUGGESTFOOT,
		PM_GV_DRIBBLEIFPOSSIBLE,
		PM_GV_KICKIFPOSSIBLE,
		PM_GV_DIVEIFPOSSIBLE,
		PM_GV_HAVEBALLTARGET,
		PM_GV_BALLTARGETX,
		PM_GV_BALLTARGETY,
		PM_GV_BALLTARGETCONF,
		PM_GV_BALLTARGETDIST,
		PM_GV_BALLTARGETANGLE,
		PM_GV_BALLTARGETWEDGE,
		PM_GV_BALLTARGETTYPE,
		PM_GV_BALLTOTARGETDIST,
		PM_GV_BALLTOTARGETANGLE,
		PM_GV_BTTANGLEOFFSETKICK,
		PM_GV_TARGETPOSEX,
		PM_GV_TARGETPOSEY,
		PM_GV_TARGETPOSEZ,
		PM_GV_TARGETPOSETOL,
		PM_GV_TARGETPOSEVALID,

		PM_AV_GAZEPITCH,
		PM_AV_GAZEYAW,
		PM_AV_GCVX,
		PM_AV_GCVY,
		PM_AV_GCVZ,
		PM_AV_HALT,
		PM_AV_KICKLEFT,
		PM_AV_KICKRIGHT,
		PM_AV_DIVE,

		PM_GM_STATECYCLE,
		PM_GM_CURSTATE,
		PM_GM_POSELEGAL,

		PM_DG_SCORINGDIST,

		PM_OBH_ACTIVE,
		PM_OBH_BALLTOOBSTDIST,
		PM_OBH_ANGLEATBALLLOW,
		PM_OBH_ANGLEATBALLHIGH,
		PM_OBH_ANGLEATBALLSTD,
		PM_OBH_TARGETANGLEADJUST,
		PM_OBH_ANGLEADJUSTFORDRIBBLE,
		PM_OBH_ANGLEADJUSTFORFOOTSEL,
		PM_OBH_KICKIFPOSSIBLE,
		PM_OBH_SUGGESTFOOT,

		PM_BM_STATECYCLE,
		PM_BM_CURSTATE,
		PM_BM_OKTOKICK,
		PM_BM_OKTODRIBBLE,
		PM_BM_OKTODRIBBLESTILL,
		PM_BM_COULDKICKNOW,
		PM_BM_COULDDRIBBLENOW,
		PM_BM_ALLOWKICK,
		PM_BM_ALLOWDRIBBLE,
		PM_BM_BALLACTIONISKICK,

		PM_WTGP_GBLTARGETX,
		PM_WTGP_GBLTARGETY,
		PM_WTGP_GBLTARGETZ,
		PM_WTGP_USEZ,
		PM_WTGP_TARGETX,
		PM_WTGP_TARGETY,
		PM_WTGP_TARGETZERR,
		PM_WTGP_TARGETDIST,
		PM_WTGP_TARGETANGLE,
		PM_WTGP_DISTCOST,

		PM_WTP_DISTCOST,
		PM_WTP_ARRIVED,

		PM_GAB_TS90,

		PM_LA_GAZEMAG,
		PM_LA_GAZETARGETID,
		PM_LA_GAZETARGETX,
		PM_LA_GAZETARGETY,

		PM_LLR_GAZEFREQ,

		PM_SFB_SFBSTATE,
		PM_SFB_ISREQUEST,
		PM_SFB_ISRESUMED,
		PM_SFB_SFBSTATETIME,
		PM_SFB_BALLHYPTYPE,
		PM_SFB_TIMEOUT,
		PM_SFB_TARGETX,
		PM_SFB_TARGETY,
		PM_SFB_DIRN,
		PM_SFB_FACTOR,
		PM_SFB_FAILCOUNTER,
		PM_SFB_DONECOUNTER,

		PM_GBB_BALLDIST,
		PM_GBB_BETA,
		PM_GBB_RADIUS_MIN,
		PM_GBB_RADIUS_DES,
		PM_GBB_RADIUS_HALO,
		PM_GBB_PATH_LEN,
		PM_GBB_WALK_SPEED,
		PM_GBB_PSI_BALL,
		PM_GBB_PSI_TARGET,
		PM_GBB_PSI_DES,
		PM_GBB_PROXIMITY_VALUE,
		PM_GBB_NEAR_FACTOR,
		PM_GBB_FOOTDUETOLESSDIST,
		PM_GBB_FOOTDUETOBALLPOSE,
		PM_GBB_RECONSIDERFOOT,
		PM_GBB_USERIGHTFOOT,
		PM_GBB_REQBALLX,
		PM_GBB_REQBALLY,
		PM_GBB_GCVX,
		PM_GBB_GCVY,
		PM_GBB_GCVZ,
		PM_GBB_GCVN,
		PM_GBB_BALLACTIONTIP,

		PM_DB_WEDGETOL,
		PM_DB_WEDGETOLSTILL,
		PM_DB_FACINGTARGET,
		PM_DB_FACINGTARGETSTILL,
		PM_DB_BALLXOK,
		PM_DB_BALLXOKSTILL,
		PM_DB_BALLYOK,
		PM_DB_BALLYOKSTILL,
		PM_DB_OKTODRIBBLE,
		PM_DB_OKTODRIBBLESTILL,
		PM_DBAPP_PREFERRIGHTFOOT,
		PM_DBAPP_RECONSIDERFOOT,
		PM_DBAPP_USERIGHTFOOT,
		PM_DBAPP_ROBOTEX,
		PM_DBAPP_ROBOTEY,
		PM_DBAPP_LOCALPATHANGLE,
		PM_DBAPP_UFACTOR,
		PM_DBAPP_RATIOXY,

		PM_KB_WEDGETOL,
		PM_KB_BALLERRORLEFTX,
		PM_KB_BALLERRORLEFTY,
		PM_KB_BALLERRORRIGHTX,
		PM_KB_BALLERRORRIGHTY,
		PM_KB_FACINGTARGET,
		PM_KB_FACINGTARGETSTILL,
		PM_KB_LEFTFOOTOK,
		PM_KB_LEFTFOOTOKSTILL,
		PM_KB_RIGHTFOOTOK,
		PM_KB_RIGHTFOOTOKSTILL,
		PM_KB_OKTOKICK,
		PM_KB_OKTOKICKSTILL,
		PM_KB_DOKICK,
		PM_KB_BESTKICKRIGHT,
		PM_KB_KICKLOCK,

		PM_DFB_DIVELOCK,

		PM_COUNT
	};

	// Plot manager configuration
	inline void configurePlotManager(plot_msgs::PlotManagerFS* PM)
	{
		// Walk and kick plot variables
		PM->setName(PM_WAK_TRUEDT, "WalkAndKick/truedT");
		PM->setName(PM_WAK_CYCLE,  "WalkAndKick/wakCycle");

		// ROS interface plot variables
		PM->setName(PM_RI_COMMSOK,      "WAKRosInterface/commsOk");
		PM->setName(PM_RI_PUBGAZEPITCH, "WAKRosInterface/published/gazePitch");
		PM->setName(PM_RI_PUBGAZEYAW,   "WAKRosInterface/published/gazeYaw");
		PM->setName(PM_RI_PUBGCVX,      "WAKRosInterface/published/gcvX");
		PM->setName(PM_RI_PUBGCVY,      "WAKRosInterface/published/gcvY");
		PM->setName(PM_RI_PUBGCVZ,      "WAKRosInterface/published/gcvZ");
		PM->setName(PM_RI_PUBWALK,      "WAKRosInterface/published/walk");
		PM->setName(PM_RI_PUBKICKLEFT,  "WAKRosInterface/published/kickLeft");
		PM->setName(PM_RI_PUBKICKRIGHT, "WAKRosInterface/published/kickRight");

		// Game controller plot variables
		PM->setName(PM_GC_SEQID,                   "GameController/seqID");
		PM->setName(PM_GC_TIMESINCEPACKETBASE,     "GameController/timeSincePacketBase");
		PM->setName(PM_GC_TIMESINCEPACKETEXTRA,    "GameController/timeSincePacketExtra");
		PM->setName(PM_GC_EXTRAOUTOFDATE,          "GameController/extraOutOfDate");
		PM->setName(PM_GC_GAMEPHASE,               "GameController/gamePhase");
		PM->setName(PM_GC_GAMESTATE,               "GameController/gameState");
		PM->setName(PM_GC_TIMEPLAYING,             "GameController/timePlaying");
		PM->setName(PM_GC_KICKOFFTYPE,             "GameController/kickoffType");
		PM->setName(PM_GC_TIMEREMAINING_RAW,       "GameController/timeRemaining/raw");
		PM->setName(PM_GC_TIMEREMAINING_SMOOTH,    "GameController/timeRemaining/smooth");
		PM->setName(PM_GC_SECONDARYTIME_RAW,       "GameController/secondaryTime/raw");
		PM->setName(PM_GC_SECONDARYTIME_SMOOTH,    "GameController/secondaryTime/smooth");
		PM->setName(PM_GC_TIMETOBALLINPLAY_RAW,    "GameController/timeToBallInPlay/raw");
		PM->setName(PM_GC_TIMETOBALLINPLAY_SMOOTH, "GameController/timeToBallInPlay/smooth");
		PM->setName(PM_GC_OWNPENALTYSTATE,         "GameController/ownRobot/penaltyState");
		PM->setName(PM_GC_OWNPENALTYTIME_RAW,      "GameController/ownRobot/penaltyTime/raw");
		PM->setName(PM_GC_OWNPENALTYTIME_SMOOTH,   "GameController/ownRobot/penaltyTime/smooth");
		PM->setName(PM_GC_OWNSCORE,                "GameController/ownTeam/score");
		PM->setName(PM_GC_OPPSCORE,                "GameController/oppTeam/score");
		PM->setName(PM_GC_OWNNUMPLAYING,           "GameController/ownTeam/numPlaying");
		PM->setName(PM_GC_OPPNUMPLAYING,           "GameController/oppTeam/numPlaying");

		// Team communications plot variable
		PM->setName(PM_TC_NUMFRESH,      "TeamComms/numFresh");
		PM->setName(PM_TC_NUMFRESHVALID, "TeamComms/numFreshAndValid");
		PM->setName(PM_TC_TIMESINCEDATA, "TeamComms/timeSinceData");

		// SensorVars plot variables
		PM->setName(PM_SV_ISPENALTYSHOOT,     "SensorVars/isPenaltyShoot");
		PM->setName(PM_SV_ISPENALTYTAKER,     "SensorVars/isPenaltyTaker");
		PM->setName(PM_SV_KICKOFFTYPE,        "SensorVars/kickoffType");
		PM->setName(PM_SV_GOALSIGN,           "SensorVars/goalSign");
		PM->setName(PM_SV_PLAYASCYAN,         "SensorVars/playAsCyan");
		PM->setName(PM_SV_GAMECOMMAND,        "SensorVars/gameCommand");
		PM->setName(PM_SV_GAMEROLE,           "SensorVars/gameRole");
		PM->setName(PM_SV_PLAYSTATE,          "SensorVars/playState/playState");
		PM->setName(PM_SV_PLAYSTATEFIRSTIN,   "SensorVars/playState/timeSinceFirstIn");
		PM->setName(PM_SV_PLAYSTATEFIRSTTOLD, "SensorVars/playState/timeSinceFirstTold");
		PM->setName(PM_SV_PLAYSTATELASTTOLD,  "SensorVars/playState/timeSinceLastTold");
		PM->setName(PM_SV_PLAYSTATETIMEOUT,   "SensorVars/playState/timeUntilTimeout");
		PM->setName(PM_SV_TIMEPLAYING,        "SensorVars/timePlaying");
		PM->setName(PM_SV_COMPASSHEADING,     "SensorVars/compassHeading");
		PM->setName(PM_SV_ROBOTPOSECONF,      "SensorVars/robotPose/conf");
		PM->setName(PM_SV_HAVEROBOTPOSE,      "SensorVars/robotPose/have");
		PM->setName(PM_SV_ROBOTPOSETIMEAGO,   "SensorVars/robotPose/timeAgo");
		PM->setName(PM_SV_ROBOTPOSEDUR,       "SensorVars/robotPose/duration");
		PM->setName(PM_SV_ROBOTPOSEX,         "SensorVars/robotPose/x");
		PM->setName(PM_SV_ROBOTPOSEY,         "SensorVars/robotPose/y");
		PM->setName(PM_SV_ROBOTPOSEZ,         "SensorVars/robotPose/z");
		PM->setName(PM_SV_BALLCONF,           "SensorVars/ball/conf");
		PM->setName(PM_SV_HAVEBALL,           "SensorVars/ball/have");
		PM->setName(PM_SV_BALLTIMEAGO,        "SensorVars/ball/timeAgo");
		PM->setName(PM_SV_BALLDUR,            "SensorVars/ball/duration");
		PM->setName(PM_SV_BALLX,              "SensorVars/ball/x");
		PM->setName(PM_SV_BALLY,              "SensorVars/ball/y");
		PM->setName(PM_SV_BALLANGLE,          "SensorVars/ball/angle");
		PM->setName(PM_SV_BALLDIST,           "SensorVars/ball/dist");
		PM->setName(PM_SV_BALLSTABLE,         "SensorVars/ball/stable");
		PM->setName(PM_SV_HAVEBALLPOSE,       "SensorVars/ballPose/have");
		PM->setName(PM_SV_BALLPOSETIMEAGO,    "SensorVars/ballPose/timeAgo");
		PM->setName(PM_SV_BALLPOSEDUR,        "SensorVars/ballPose/duration");
		PM->setName(PM_SV_BALLPOSEX,          "SensorVars/ballPose/x");
		PM->setName(PM_SV_BALLPOSEY,          "SensorVars/ballPose/y");
		PM->setName(PM_SV_BALLPOSESTABLE,     "SensorVars/ballPose/stable");
		PM->setName(PM_SV_BALLHASMOVED,       "SensorVars/ballHasMoved");
		PM->setName(PM_SV_BALLINPLAY,         "SensorVars/ballInPlay");
		PM->setName(PM_SV_DIRECTGOALALLOWED,  "SensorVars/directGoalAllowed");

		// GameVars plot variables
		PM->setName(PM_GV_FORCEBEHSTATE,      "GameVars/forceBehState");
		PM->setName(PM_GV_SUGGESTFOOT,        "GameVars/suggestFoot");
		PM->setName(PM_GV_DRIBBLEIFPOSSIBLE,  "GameVars/dribbleIfPossible");
		PM->setName(PM_GV_KICKIFPOSSIBLE,     "GameVars/kickIfPossible");
		PM->setName(PM_GV_DIVEIFPOSSIBLE,     "GameVars/diveIfPossible");
		PM->setName(PM_GV_HAVEBALLTARGET,     "GameVars/haveBallTarget");
		PM->setName(PM_GV_BALLTARGETX,        "GameVars/ballTarget/x");
		PM->setName(PM_GV_BALLTARGETY,        "GameVars/ballTarget/y");
		PM->setName(PM_GV_BALLTARGETCONF,     "GameVars/ballTarget/conf");
		PM->setName(PM_GV_BALLTARGETDIST,     "GameVars/ballTarget/dist");
		PM->setName(PM_GV_BALLTARGETANGLE,    "GameVars/ballTarget/angle");
		PM->setName(PM_GV_BALLTARGETWEDGE,    "GameVars/ballTarget/wedge");
		PM->setName(PM_GV_BALLTARGETTYPE,     "GameVars/ballTarget/type");
		PM->setName(PM_GV_BALLTOTARGETDIST,   "GameVars/ballToTarget/dist");
		PM->setName(PM_GV_BALLTOTARGETANGLE,  "GameVars/ballToTarget/angle");
		PM->setName(PM_GV_BTTANGLEOFFSETKICK, "GameVars/ballToTarget/angleOffsetKick");
		PM->setName(PM_GV_TARGETPOSEX,        "GameVars/targetPose/x");
		PM->setName(PM_GV_TARGETPOSEY,        "GameVars/targetPose/y");
		PM->setName(PM_GV_TARGETPOSEZ,        "GameVars/targetPose/z");
		PM->setName(PM_GV_TARGETPOSETOL,      "GameVars/targetPose/tol");
		PM->setName(PM_GV_TARGETPOSEVALID,    "GameVars/targetPose/valid");

		// ActuatorVars plot variables
		PM->setName(PM_AV_GAZEPITCH, "ActuatorVars/gazePitch");
		PM->setName(PM_AV_GAZEYAW,   "ActuatorVars/gazeYaw");
		PM->setName(PM_AV_GCVX,      "ActuatorVars/gcvX");
		PM->setName(PM_AV_GCVY,      "ActuatorVars/gcvY");
		PM->setName(PM_AV_GCVZ,      "ActuatorVars/gcvZ");
		PM->setName(PM_AV_HALT,      "ActuatorVars/halt");
		PM->setName(PM_AV_KICKLEFT,  "ActuatorVars/kickLeft");
		PM->setName(PM_AV_KICKRIGHT, "ActuatorVars/kickRight");
		PM->setName(PM_AV_DIVE,      "ActuatorVars/dive");

		// Game manager plot variables
		PM->setName(PM_GM_STATECYCLE, "GameFSM/Manager/stateCycle");
		PM->setName(PM_GM_CURSTATE,   "GameFSM/Manager/curState");
		PM->setName(PM_GM_POSELEGAL,  "GameFSM/Manager/poseLegalForKickoff");

		// Default goalie variables
		PM->setName(PM_DG_SCORINGDIST, "GameFSM/DefaultGoalie/scoringDist");

		// Obstacle ball handling plot variables
		PM->setName(PM_OBH_ACTIVE,                "GameFSM/ObstacleBallHandling/active");
		PM->setName(PM_OBH_BALLTOOBSTDIST,        "GameFSM/ObstacleBallHandling/ballToObstDist");
		PM->setName(PM_OBH_ANGLEATBALLLOW,        "GameFSM/ObstacleBallHandling/angleAtBallLow");
		PM->setName(PM_OBH_ANGLEATBALLHIGH,       "GameFSM/ObstacleBallHandling/angleAtBallHigh");
		PM->setName(PM_OBH_ANGLEATBALLSTD,        "GameFSM/ObstacleBallHandling/angleAtBallWrapped");
		PM->setName(PM_OBH_TARGETANGLEADJUST,     "GameFSM/ObstacleBallHandling/targetAngleAdjust");
		PM->setName(PM_OBH_ANGLEADJUSTFORDRIBBLE, "GameFSM/ObstacleBallHandling/angleAdjustForDribble");
		PM->setName(PM_OBH_ANGLEADJUSTFORFOOTSEL, "GameFSM/ObstacleBallHandling/angleAdjustForFootSel");
		PM->setName(PM_OBH_KICKIFPOSSIBLE,        "GameFSM/ObstacleBallHandling/kickIfPossible");
		PM->setName(PM_OBH_SUGGESTFOOT,           "GameFSM/ObstacleBallHandling/suggestFoot");

		// Behaviour manager plot variables
		PM->setName(PM_BM_STATECYCLE,       "BehFSM/Manager/stateCycle");
		PM->setName(PM_BM_CURSTATE,         "BehFSM/Manager/curState");
		PM->setName(PM_BM_OKTOKICK,         "BehFSM/Manager/okToKick");
		PM->setName(PM_BM_OKTODRIBBLE,      "BehFSM/Manager/okToDribble");
		PM->setName(PM_BM_OKTODRIBBLESTILL, "BehFSM/Manager/okToDribbleStill");
		PM->setName(PM_BM_COULDKICKNOW,     "BehFSM/Manager/couldKickNow");
		PM->setName(PM_BM_COULDDRIBBLENOW,  "BehFSM/Manager/couldDribbleNow");
		PM->setName(PM_BM_ALLOWKICK,        "BehFSM/Manager/allowKick");
		PM->setName(PM_BM_ALLOWDRIBBLE,     "BehFSM/Manager/allowDribble");
		PM->setName(PM_BM_BALLACTIONISKICK, "BehFSM/Manager/ballActionIsKick");

		// Walk to global pose plot variables
		PM->setName(PM_WTGP_GBLTARGETX,  "BehFSM/WalkToGlobalPose/globalTargetX");
		PM->setName(PM_WTGP_GBLTARGETY,  "BehFSM/WalkToGlobalPose/globalTargetY");
		PM->setName(PM_WTGP_GBLTARGETZ,  "BehFSM/WalkToGlobalPose/globalTargetZ");
		PM->setName(PM_WTGP_USEZ,        "BehFSM/WalkToGlobalPose/useTargetZ");
		PM->setName(PM_WTGP_TARGETX,     "BehFSM/WalkToGlobalPose/targetX");
		PM->setName(PM_WTGP_TARGETY,     "BehFSM/WalkToGlobalPose/targetY");
		PM->setName(PM_WTGP_TARGETZERR,  "BehFSM/WalkToGlobalPose/targetZErr");
		PM->setName(PM_WTGP_TARGETDIST,  "BehFSM/WalkToGlobalPose/targetDist");
		PM->setName(PM_WTGP_TARGETANGLE, "BehFSM/WalkToGlobalPose/targetAngle");
		PM->setName(PM_WTGP_DISTCOST,    "BehFSM/WalkToGlobalPose/distCost");

		// Walk to pose plot variables
		PM->setName(PM_WTP_DISTCOST,    "BehFSM/WalkToPose/distCost");
		PM->setName(PM_WTP_ARRIVED,     "BehFSM/WalkToPose/arrived");

		// Gaze at ball plot variables
		PM->setName(PM_GAB_TS90, "BehFSM/GazeAtBall/settlingTime90%");

		// Look around plot variables
		PM->setName(PM_LA_GAZEMAG,      "BehFSM/LookAround/gazeMag");
		PM->setName(PM_LA_GAZETARGETID, "BehFSM/LookAround/gazeTargetID");
		PM->setName(PM_LA_GAZETARGETX,  "BehFSM/LookAround/gazeTargetX");
		PM->setName(PM_LA_GAZETARGETY,  "BehFSM/LookAround/gazeTargetY");

		// Look left right plot variables
		PM->setName(PM_LLR_GAZEFREQ, "BehFSM/LookLeftRight/gazeFreq");

		// Search for ball plot variables
		PM->setName(PM_SFB_SFBSTATE,     "BehFSM/SearchForBall/sfbState");
		PM->setName(PM_SFB_ISREQUEST,    "BehFSM/SearchForBall/stateIsRequested");
		PM->setName(PM_SFB_ISRESUMED,    "BehFSM/SearchForBall/stateIsResumed");
		PM->setName(PM_SFB_SFBSTATETIME, "BehFSM/SearchForBall/sfbStateElapsed");
		PM->setName(PM_SFB_BALLHYPTYPE,  "BehFSM/SearchForBall/ballHypType");
		PM->setName(PM_SFB_TIMEOUT,      "BehFSM/SearchForBall/timeout");
		PM->setName(PM_SFB_TARGETX,      "BehFSM/SearchForBall/targetX");
		PM->setName(PM_SFB_TARGETY,      "BehFSM/SearchForBall/targetY");
		PM->setName(PM_SFB_DIRN,         "BehFSM/SearchForBall/dirn");
		PM->setName(PM_SFB_FACTOR,       "BehFSM/SearchForBall/factor");
		PM->setName(PM_SFB_FAILCOUNTER,  "BehFSM/SearchForBall/failCounter");
		PM->setName(PM_SFB_DONECOUNTER,  "BehFSM/SearchForBall/doneCounter");

		// Go behind ball plot variables
		PM->setName(PM_GBB_BALLDIST,          "BehFSM/GoBehindBall/ballDist");
		PM->setName(PM_GBB_BETA,              "BehFSM/GoBehindBall/beta");
		PM->setName(PM_GBB_RADIUS_MIN,        "BehFSM/GoBehindBall/radius/minDesired");
		PM->setName(PM_GBB_RADIUS_DES,        "BehFSM/GoBehindBall/radius/desiredHalo");
		PM->setName(PM_GBB_RADIUS_HALO,       "BehFSM/GoBehindBall/radius/actualHalo");
		PM->setName(PM_GBB_PATH_LEN,          "BehFSM/GoBehindBall/near/pathLength");
		PM->setName(PM_GBB_WALK_SPEED,        "BehFSM/GoBehindBall/near/walkSpeed");
		PM->setName(PM_GBB_PSI_BALL,          "BehFSM/GoBehindBall/lookAt/ballAngle");
		PM->setName(PM_GBB_PSI_TARGET,        "BehFSM/GoBehindBall/lookAt/targetAngle");
		PM->setName(PM_GBB_PSI_DES,           "BehFSM/GoBehindBall/lookAt/desiredAngle");
		PM->setName(PM_GBB_PROXIMITY_VALUE,   "BehFSM/GoBehindBall/near/proximityValue");
		PM->setName(PM_GBB_NEAR_FACTOR,       "BehFSM/GoBehindBall/nearFactor");
		PM->setName(PM_GBB_FOOTDUETOLESSDIST, "BehFSM/GoBehindBall/footSel/footDueToLessDist");
		PM->setName(PM_GBB_FOOTDUETOBALLPOSE, "BehFSM/GoBehindBall/footSel/footDueToBallPose");
		PM->setName(PM_GBB_RECONSIDERFOOT,    "BehFSM/GoBehindBall/footSel/reconsiderFoot");
		PM->setName(PM_GBB_USERIGHTFOOT,      "BehFSM/GoBehindBall/footSel/useRightFoot");
		PM->setName(PM_GBB_REQBALLX,          "BehFSM/GoBehindBall/reqBallDir/x");
		PM->setName(PM_GBB_REQBALLY,          "BehFSM/GoBehindBall/reqBallDir/y");
		PM->setName(PM_GBB_GCVX,              "BehFSM/GoBehindBall/gcv/x");
		PM->setName(PM_GBB_GCVY,              "BehFSM/GoBehindBall/gcv/y");
		PM->setName(PM_GBB_GCVZ,              "BehFSM/GoBehindBall/gcv/z");
		PM->setName(PM_GBB_GCVN,              "BehFSM/GoBehindBall/gcv/norm");
		PM->setName(PM_GBB_BALLACTIONTIP,     "BehFSM/GoBehindBall/ballActionTip");

		// Dribble ball plot variables
		PM->setName(PM_DB_WEDGETOL,           "BehFSM/DribbleBall/wedgeTol");
		PM->setName(PM_DB_WEDGETOLSTILL,      "BehFSM/DribbleBall/wedgeTolStill");
		PM->setName(PM_DB_FACINGTARGET,       "BehFSM/DribbleBall/facingTarget");
		PM->setName(PM_DB_FACINGTARGETSTILL,  "BehFSM/DribbleBall/facingTargetStill");
		PM->setName(PM_DB_BALLXOK,            "BehFSM/DribbleBall/ballXOk");
		PM->setName(PM_DB_BALLXOKSTILL,       "BehFSM/DribbleBall/ballXOkStill");
		PM->setName(PM_DB_BALLYOK,            "BehFSM/DribbleBall/ballYOk");
		PM->setName(PM_DB_BALLYOKSTILL,       "BehFSM/DribbleBall/ballYOkStill");
		PM->setName(PM_DB_OKTODRIBBLE,        "BehFSM/DribbleBall/okToDribble");
		PM->setName(PM_DB_OKTODRIBBLESTILL,   "BehFSM/DribbleBall/okToDribbleStill");
		PM->setName(PM_DBAPP_PREFERRIGHTFOOT, "BehFSM/DribbleBall/footSel/preferRightFoot");
		PM->setName(PM_DBAPP_RECONSIDERFOOT,  "BehFSM/DribbleBall/footSel/reconsiderFoot");
		PM->setName(PM_DBAPP_USERIGHTFOOT,    "BehFSM/DribbleBall/footSel/useRightFoot");
		PM->setName(PM_DBAPP_ROBOTEX,         "BehFSM/DribbleBall/approach/robotEx");
		PM->setName(PM_DBAPP_ROBOTEY,         "BehFSM/DribbleBall/approach/robotEy");
		PM->setName(PM_DBAPP_LOCALPATHANGLE,  "BehFSM/DribbleBall/approach/localPathAngle");
		PM->setName(PM_DBAPP_UFACTOR,         "BehFSM/DribbleBall/approach/sideStepFactorU");
		PM->setName(PM_DBAPP_RATIOXY,         "BehFSM/DribbleBall/approach/gcvRatioXY");

		// Kick ball plot variables
		PM->setName(PM_KB_WEDGETOL,          "BehFSM/KickBall/wedgeTol");
		PM->setName(PM_KB_BALLERRORLEFTX,    "BehFSM/KickBall/ballError/leftX");
		PM->setName(PM_KB_BALLERRORLEFTY,    "BehFSM/KickBall/ballError/leftY");
		PM->setName(PM_KB_BALLERRORRIGHTX,   "BehFSM/KickBall/ballError/rightX");
		PM->setName(PM_KB_BALLERRORRIGHTY,   "BehFSM/KickBall/ballError/rightY");
		PM->setName(PM_KB_FACINGTARGET,      "BehFSM/KickBall/facingTarget");
		PM->setName(PM_KB_FACINGTARGETSTILL, "BehFSM/KickBall/facingTargetStill");
		PM->setName(PM_KB_LEFTFOOTOK,        "BehFSM/KickBall/leftFootInPosition");
		PM->setName(PM_KB_LEFTFOOTOKSTILL,   "BehFSM/KickBall/leftFootInPositionStill");
		PM->setName(PM_KB_RIGHTFOOTOK,       "BehFSM/KickBall/rightFootInPosition");
		PM->setName(PM_KB_RIGHTFOOTOKSTILL,  "BehFSM/KickBall/rightFootInPositionStill");
		PM->setName(PM_KB_OKTOKICK,          "BehFSM/KickBall/okToKick");
		PM->setName(PM_KB_OKTOKICKSTILL,     "BehFSM/KickBall/okToKickStill");
		PM->setName(PM_KB_DOKICK,            "BehFSM/KickBall/doKick");
		PM->setName(PM_KB_BESTKICKRIGHT,     "BehFSM/KickBall/bestKickRight");
		PM->setName(PM_KB_KICKLOCK,          "BehFSM/KickBall/kickLock");

		// Dive for ball plot variables
		PM->setName(PM_DFB_DIVELOCK, "BehFSM/DiveForBall/diveLock");

		// Check that we have been thorough
		if(!PM->checkNames())
			ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
	}
}

#endif
// EOF