// Walk and Kick node
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WALK_AND_KICK_H
#define WALK_AND_KICK_H

// Includes
#include <config_server/parameter.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nimbro_op_interface/Button.h>
#include <nimbro_op_interface/LEDCommand.h>
#include <head_control/LookAtTarget.h>
#include <robotcontrol/State.h>
#include <gait_msgs/GaitCommand.h>
#include <field_model/field_model.h>
#include <Eigen/Core>
#include <robotcontrol/RobotHeading.h>
#include <nimbro_utils/math_funcs.h>
#include <nimbro_utils/math_spline.h>
#include <walk_and_kick/Vec2f.h>
#include <walk_and_kick/Vec3f.h>

// Node namespace
namespace walkandkick
{
	// WalkAndKick class
	class WalkAndKick
	{
	public:
		// Constructors/destructors
		WalkAndKick();
		virtual ~WalkAndKick();
		
		// Reset function
		void reset();

		// Initialisation function
		bool init();

		// Cycle step function (called as long as WAK is enabled in the configs)
		void step();
		
		//
		// Types
		//

		// Walk and kick state enumeration (if you add a state XXX then see executeState(), activateState(), deactivateState(), StateName[], changedXXX(), executeXXX(), XXX vars/funcs)
		enum WAKState
		{
			STATE_UNKNOWN = 0,
			STATE_STOPPED,         // Abbreviation: STP
			STATE_POSITIONING,     // Abbreviation: POS
			STATE_SEARCH_FOR_BALL, // Abbreviation: SFB
			STATE_GO_BEHIND_BALL,  // Abbreviation: GBB
			STATE_DRIBBLE_BALL,    // Abbreviation: DB
			STATE_KICK_BALL,       // Abbreviation: KB
			NUM_STATES
		};
		static const char* const StateName[NUM_STATES];
		
		// Game commands and roles
		enum GameCommands
		{
			CMD_STOP,
			CMD_POSE,
			CMD_PLAY,
			CMD_COUNT
		};
		enum GameRoles
		{
			ROLE_FIELDPLAYER,
			ROLE_SOCCER_GOALIE,
			ROLE_COUNT
		};

		// Config variables
		struct Config
		{
			static const float forceUseFoot;
			static const float gazeAngleLimit;
			static const float gazeVelLimit;
			static const float gazeBallFarDist;
			static const float gazeBallFarTs;
			static const float gazeBallNearTs;
			static const float sfbGazeSplineAccMax;
			static const float sfbGazeSplineVelMax;
			static const float gbbGcvSWalk;
			static const float gbbGcvSTipToe;
			static const float gbbGcvSStop;
			static const float gbbGcvSReverse;
			static const float gbbDesiredSLambda;
			static const float gbbDesiredSPhi;
			static const float gbbReqBallOffX;
			static const float gbbReqBallOffYRK;
			static const float gbbReqBallOffYLK;
			static const float gbbSlowDownOffX;
			static const float gbbSpeedLimit;
			static const float gbbAlphaGainZ;
			static const float gbbBetaGainY;
			static const float dbAppWalkToXDist;
			static const float dbAppWalkToYScaler;
			static const float dbAppSpeedLimit;
			static const float dbAppXYSpeed;
			static const float dbAppZGain;
			static const float dbAppMaxBallToTargetAngle;
			static const float dbTargetAnglePrecision;
			static const float dbMinTargetAngleTolerance;
			static const float dbBallDistXMax;
			static const float dbBallErrorYIwd;
			static const float dbBallErrorYOwd;
			static const float dbBallSpreadAngle;
			static const float dbBallSpreadAngleMore;
			static const float dbReqBallOffXExtra;
			static const float kbTargetAnglePrecision;
			static const float kbMinTargetAngleTolerance;
			static const float kbBallErrorXFwd;
			static const float kbBallErrorXFwdExtra;
			static const float kbBallErrorYIwd;
			static const float kbBallErrorYOwd;
			static const float kbBallErrorYOwdExtra;
			static const float zoneDbCentreXMax;
			static const float zoneDbCentreXMin;
			static const float zoneDbNearOppGoalXDist;
		} config;
		
		//
		// Helper classes
		//
		
		// Sensor variables
		struct SensorVars
		{
			SensorVars(WalkAndKick* wak);
			WalkAndKick* const wak;
			void update();

			enum BTType           // Ball target type (if you update this then also update BTTChar)
			{
				BTT_UNKNOWN = 0,  // Unknown source of ball target
				BTT_GOAL,         // Ball target based on goal detection
				BTT_POSE,         // Ball target based on localised robot pose
				BTT_COMPASS,      // Ball target based on compass heading
				BTT_COUNT
			};
			static const char BTTChar[BTT_COUNT];
			static char getBTTChar(BTType type) { if (type >= BTT_UNKNOWN && type < BTT_COUNT) return BTTChar[type]; else return BTTChar[BTT_UNKNOWN]; }
			char BallTargetChar() const { return getBTTChar(BallTargetType); }

			GameRoles Role;          // The role of the robot (e.g. ROLE_FIELDPLAYER, ROLE_SOCCER_GOALIE)
			GameCommands Command;    // Game command (e.g. CMD_PLAY, CMD_STOP, CMD_POSE...)
			bool PlayOnYellow;       // Boolean flag whether the robot should play on the positive yellow goal
			int GoalSign;            // Sign of the goal the robot should play on (+1 = Positive yellow goal, -1 = Negative blue goal)
			bool PlayAsCyan;         // Boolean flag whether the robot is playing as the cyan team
			bool Extern;             // Boolean flag whether the robot is listening to external information from the referee box
			Vec3f RobotPose;         // Global position and orientation of the robot on the field (X points to positive yellow goal, Y to the left, and Z is a CCW rotation relative to direction of the positive goal)
			float RobotPoseConf;     // Confidence of the robot's estimated position on the field
			float CompassHeading;    // Heading of the robot measured by the compass in the range (-pi,pi] (0 is towards the positive yellow goal and CCW from there is positive)
			float BallConf;          // Confidence of the ball detection
			Vec2f BallDir;           // Egocentric vector from the robot to the ball
			float BallAssumedConf;   // Confidence of the assumed ball detection
			Vec2f BallAssumedDir;    // Assumed egocentric vector from the robot to the ball (even if it can't see the ball)
			float GoalConf;          // Confidence of the goal detection
			Vec2f GoalDir;           // Egocentric vector from the robot to the goal he is trying to score in
			float GoalWedge;         // The width of the goal expressed as the angle it subtends at the robot's position
			float BallTargetConf;    // Confidence of the ball target detection
			Vec2f BallTargetDir;     // Egocentric vector from the robot to the ball target
			float BallTargetWedge;   // The width of the ball target expressed as the angle it subtends at the robot's position
			BTType BallTargetType;   // Enumeration value specifying the type of ball target that was calculated
			
			float BallAngle;         // Computed from BallDir (CCW from straight ahead)
			float BallDist;          // Computed from BallDir (scalar distance to the ball)
			Vec2f BallPose;          // Computed from BallDir and RobotPose (global position of ball, use only after checking BallConf and RobotPoseConf)
			float BallAssumedAngle;  // Computed from BallAssumedDir (CCW from straight ahead)
			float BallAssumedDist;   // Computed from BallAssumedDir (scalar distance to the assumed ball)
			Vec2f BallAssumedPose;   // Computed from BallAssumedDir and RobotPose (global position of assumed ball, use only after checking BallAssumedConf and RobotPoseConf)
			float BallTargetAngle;   // Computed from BallTargetDir (CCW from straight ahead)
			float BallTargetDist;    // Computed from BallTargetDir (scalar distance to the centre point of the ball target)
			float BallToTargetAngle; // Computed from BallDir and BallTargetDir (angle of vector from ball to target in body-fixed coordinates, CCW from straight ahead)
		};

		// Actuator output variables
		struct ActuatorVars
		{
			ActuatorVars() { init(); }
			void init();

			bool Halt;
			Vec3f GCV; // NimbRo convention: x forwards, y left, z CCW
			bool DoKick;
			int KickFoot;
			float GazeAngle; // Positive is CCW (rotation about positive z-axis)
		};
		
		// Field dimensions
		class Field
		{
		public:
			// Constructor
			Field() : m_field(field_model::FieldModel::getInstance()) {}
			
			// Field parameters
			float fieldLength() const { return m_field->length(); }
			float fieldWidth() const { return m_field->width(); }
			float circleDiameter() const { return m_field->centerCircleDiameter(); }
			float goalWidth() const { return m_field->goalWidth(); }
			float goalAreaLength() const { return m_field->goalAreaDepth(); }
			float goalAreaWidth() const { return m_field->goalAreaWidth(); }
			float penaltyMarkDist() const { return m_field->penaltyMarkerDist(); }
			float fieldLengthH() const { return 0.5*m_field->length(); }
			float fieldWidthH() const { return 0.5*m_field->width(); }
			float circleRadius() const { return 0.5*m_field->centerCircleDiameter(); }
			float goalWidthH() const { return 0.5*m_field->goalWidth(); }
			
			// Field model
			const field_model::FieldModel* const m_field;
		} field;

		// Counter class
		class Counter
		{
		public:
			Counter() { reset(); }
			void reset() { m_count = 0; }
			int count() const { return m_count; }
			void add(bool expr) { if (expr) m_count++; else m_count = (m_count > 0 ? m_count - 1 : 0); }
			bool reached(int limit) const { return (m_count >= limit); }
		private:
			int m_count;
		};

		// TheWorm class (https://en.wikipedia.org/wiki/Worm_%28marketing%29)
		class TheWorm
		{
		public:
			explicit TheWorm(int limit = 100) { reset(limit); }
			void reset() { m_worm = 0; }
			void reset(int limit) { reset(); setLimit(limit); }
			void setLimit(int limit) { m_limit = (limit >= 1 ? limit : 1); } // The count from zero to a positive or negative decision
			void setRange(int range) { m_limit = (range >= 2 ? range >> 1 : 1); } // The count from a decision to the opposite decision
			void setWorm(int value) { m_worm = (value >= m_limit ? m_limit : (value <= -m_limit ? -m_limit : value)); }
			void vote(bool decision) { if (decision) m_worm = (m_worm < m_limit ? m_worm + 1 : m_limit); else m_worm = (m_worm > -m_limit ? m_worm - 1 : -m_limit); }
			bool decision() const { return (m_worm >= 0); } // Balanced votes is a decision of true!
			bool unanimousTrue() const { return (m_worm >= m_limit); }
			bool unanimousFalse() const { return (m_worm <= -m_limit); }
			bool unanimous() const { return (unanimousTrue() || unanimousFalse()); }
		private:
			int m_limit;
			int m_worm;
		};

		// Spline class
		class Spline
		{
		public:
			Spline() { reset(); }
			void reset() { m_valid = false; m_x = m_v = m_t = 0.0; }
			void setState(double x, double v = 0.0) { m_x = x; m_v = v; m_t = 0.0; m_valid = false; }
			bool valid() const { return m_valid; }
			bool finished() const { return !m_valid || m_t >= m_spline.T(); }
			double curX() const { return m_x; }
			double curV() const { return m_v; }
			void newTarget(double x, double v, double maxVel, double maxAcc, bool ctsVel = true)
			{
				m_spline.setParams(m_x, (ctsVel ? m_v : 0.0), x, v, maxVel, maxAcc);
				m_valid = true;
				m_t = 0.0;
			}
			double forward(double dT)
			{
				if (!m_valid) return m_x;
				m_t += dT;
				m_x = m_spline.x(m_t);
				m_v = m_spline.v(m_t);
				return m_x;
			}
		private:
			nimbro_utils::TrapVelSpline m_spline;
			bool m_valid;
			double m_x; // Current position
			double m_v; // Current velocity
			double m_t; // Current time relative to the start of the spline (only if valid)
		};
		
		//
		// Walk and kick behaviour
		// 
		
		// State machine reset function
		void resetVars();

		// Update functions
		void updateLayer();

		// Behaviour functions
		float setActive(bool active);
		float aktivierungsfunktion();
		void targetfunction();
		float getActivationFactor() const { return lastActFact; }

		// State machine
		bool decideState();

		// State change functions
		void changedSTP(bool nowActive);
		void changedPOS(bool nowActive);
		void changedSFB(bool nowActive);
		void changedGBB(bool nowActive);
		void changedDB (bool nowActive);
		void changedKB (bool nowActive);

		// State execution functions
		void executeSTP(bool justActivated);
		void executePOS(bool justActivated);
		void executeSFB(bool justActivated);
		void executeGBB(bool justActivated);
		void executeDB (bool justActivated);
		void executeKB (bool justActivated);

		// Actuator output functions
		void writeActuators(const ActuatorVars& ActVar);

		// Helper functions
		static const char* getStateName(WAKState stateID);
		void deactivateState(WAKState stateID);
		void activateState(WAKState stateID);
		void executeState(WAKState stateID, bool justActivated);

		// Behaviour helper functions
		bool gazeAtBall();
		float walkToGlobalPose(float targetX, float targetY) { return walkToGlobalPose(targetX, targetY, 0.0f, false); }
		float walkToGlobalPose(float targetX, float targetY, float targetZ, bool useZ = true);

		//
		// Variables
		//

		// Persistent variables
		long long int cycle;
		long long int targetCycle;
		long long int stateCycle;
		float targetTime;
		float stateTime;
		float lastActFact;

		// State variables
		WAKState state; // Current walk and kick state
		WAKState nextState; // Can be set by a walk and kick state to request a transition to a particular other state

		// Layer variables (Note: These are updated prior to SV!)
		Vec2f ReqBallDirLeft;
		Vec2f ReqBallDirRight;
		
		// Sensor variables
		SensorVars SV;

		// Actuator output members
		ActuatorVars AV;
		ActuatorVars lastAV;

		// State machine types
		enum BAType
		{
			BA_KICK = 0,
			BA_DRIBBLE,
			BA_COUNT,
			BA_DEFAULT = BA_KICK
		};

		// State machine variables
		BAType dsBallAction;
		TheWorm dsKick;
		TheWorm dsDribble;
		TheWorm dsStillDribble;
		bool dsGoalieAlive;

		// Positioning members
		float posGazeFreq;
		float posGazePhaseOff;
		bool posArrived;
		Counter posDoneCounter;

		// Search for ball types
		enum SFBWalkState
		{
			SFB_WS_STAYCOOL = 0,
			SFB_WS_BACKUP,
			SFB_WS_SPIN,
			SFB_WS_GOTOCENTRE,
			SFB_WS_SPINHERE,
			SFB_WS_WALKTOMARK,
			SFB_WS_WALKFWDS,
			SFB_WS_COUNT
		};

		// Search for ball functions
		void changeSfbState(SFBWalkState newSfbState);
		void requestSfbState(SFBWalkState state, int data = 0); // A state of SFB_WS_COUNT means clear state request

		// Search for ball members
		SFBWalkState sfbReqState;
		int sfbReqData;
		Spline sfbGazeSpline;
		SFBWalkState sfbWalkState;
		float sfbWSTime;
		int sfbSpinDirn;
		float sfbFactor;
		Counter sfbDoneCounter;
		Counter sfbFailCounter;
		float sfbWalkFwdTime;
		long long int sfbLastActCycle;
		SFBWalkState sfbLastActState;

		// Go behind ball members
		bool gbbUseRightFoot;
		TheWorm gbbChangeToFoot;
		Counter gbbStuck;
		BAType gbbBallActionTip;

		// Kick ball functions
		bool okToKick() const;
		bool stillOkToKick() const;
		int bestKickFoot() const;

		// Dribble ball functions
		bool okToDribble() const;
		bool stillOkToDribble() const;

		// Dribble ball variables
		bool dbLock;

		//
		// Data variables
		//
		
		// Enumerations
		enum ButtonState
		{
			BTN_FIRST = 0,
			BTN_HALT = BTN_FIRST,
			BTN_PLAY,
			BTN_GOALIE,
			BTN_POS,
			BTN_COUNT
		};

		// Config server parameters
		config_server::Parameter<bool> m_enable_wak;

		// ROS outputs
		ros::Publisher m_pub_gaitcmd;
		ros::ServiceClient m_srv_kick;
		
		// Data outputs
		ros::Publisher m_pub_leds;
		ros::Publisher m_pub_headCmd;
		nimbro_op_interface::LEDCommand m_led;
		
		// Data subscribers
		ros::Subscriber m_sub_button;
		ros::Subscriber m_sub_ballvec;
		ros::Subscriber m_sub_goalvec;
		ros::Subscriber m_sub_robotpose;
		ros::Subscriber m_sub_heading;
		ros::Subscriber m_sub_robotstate;
		
		// Data handler functions
		void resetDataVariables();
		void handleButtonData(const nimbro_op_interface::ButtonConstPtr& msg);
		void handleBallData(const geometry_msgs::PointStamped::ConstPtr& msg);
		void handleGoalData(const geometry_msgs::PolygonStampedConstPtr& msg);
		void handleRobotPoseData(const geometry_msgs::PointStampedConstPtr& msg);
		void handleHeadingData( const robotcontrol::RobotHeadingConstPtr& msg);
		void handleStateData(const robotcontrol::StateConstPtr& msg);

		// Data variables
		ButtonState m_button;
		Eigen::Vector3d m_ball_vec;
		ros::Time m_ball_time;
		double m_ball_conf;
		Eigen::Vector3d m_goal_vec;
		ros::Time m_goal_time;
		double m_goal_conf;
		Eigen::Vector3d m_robot_pose_vec;
		ros::Time m_robot_pose_time;
		double m_robot_pose_conf;
		double m_heading;
		ros::Time m_heading_time;
		bool m_standing;
		bool m_walking;
		bool m_kicking;
	};
}

#endif /* WALK_AND_KICK_H */
// EOF
