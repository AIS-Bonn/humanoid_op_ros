// behaviour_exercise_template.h - Philipp Allgeuer - 12/07/13
// Template header file for behaviours exercise

// Ensure header is only included once
#ifndef BEHAVIOUR_EXERCISE_TEMPLATE_H
#define BEHAVIOUR_EXERCISE_TEMPLATE_H

// Includes - ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Includes - Nimbro packages
#include <state_controller/state_controller.h>
#include <test_utilities/test_utilities.h>
#include <config_server/parameter.h>
#include <field_model/field_model.h>

// Includes - Message and service types
#include <gait_msgs/SetOdom.h>
#include <plot_msgs/Plot.h>
#include <gait_msgs/GaitCommand.h>
#include <robotcontrol/State.h>
#include <config_server/SetParameter.h>
#include <visualization_msgs/MarkerArray.h>
#include <motion_player/PlayMotion.h>

// Includes C++ Standard Library
#include <iostream>
#include <cstdlib>
#include <string>
#include <cmath>

// Includes - Other
#include <Eigen/Core>

// Project namespace
namespace behaviourexercise
{
	//
	// Namespaces
	//
	using namespace statecontroller;
	using namespace testutilities;

	//
	// Class declarations
	//

	// Helper classes
	class SVector;
	class RosTimeMarker;

	// State controller
	class RobotSC;

	// States
	class IdleState;
	// TODO: Declare extra state classes here (e.g. SearchForBallState)

	//
	// Enumerations
	//

	// State ID enumeration
	enum RobotSCStateID
	{
		IDLE
		// TODO: Add ID's for extra states here (e.g. SEARCH_FOR_BALL)
	};

	// Motion player motion enumeration
	enum KeyMotionEnum
	{
		KM_NO_MOTION = 0,
		KM_STAND_UP,
		KM_RIGHT_KICK,
		KM_LEFT_KICK,
		KM_SIT,
		KM_GETUP_PRONE,
		KM_GETUP_SUPINE,
		KM_SCOOP_KICK,
		KM_NEW_RIGHT_KICK,
		KM_NEW_LEFT_KICK,
		KM_SIM_RIGHT_KICK,
		KM_SIM_LEFT_KICK,
		KM_NUM_MOTIONS
	};

	//
	// Constants
	//

	// Motion player motion names (must match up with KeyMotionEnum!)
	const std::string KeyMotionName[KM_NUM_MOTIONS + 1] =
	{
		"NO_MOTION",
		"STAND_UP",
		"RIGHT_KICK",
		"LEFT_KICK",
		"SIT",
		"getup_prone",
		"getup_supine",
		"SCOOP_KICK",
		"right_kick_straight",
		"left_kick_straight",
		"SIM_RIGHT_KICK",
		"SIM_LEFT_KICK",
		"NUM_MOTIONS"
	};

	//
	// Helper classes
	//

	// SVector class
	class SVector
	{
	public:
		// Constructors
		SVector() : timestamp(0) { vec.setZero(); }
		SVector(double x, double y, ros::Time timestamp) : timestamp(timestamp) { vec << x, y; }

		// Helper functions
		void reset() { timestamp.fromSec(0.0); }
		void setStampNow() { timestamp = ros::Time::now(); }
		bool isCurrent(double timeTol) const { return (ros::Time::now() - timestamp).toSec() < timeTol; }

		// Internal variables
		Eigen::Vector2d vec;
		ros::Time timestamp;
	};

	// ROSTimeMarker class
	class RosTimeMarker
	{
	public:
		// Constructor
		RosTimeMarker() : markerTime(0), iHaveMarker(false) {}

		// Timing functions
		void reset() { iHaveMarker = false; }
		void setMarker()
		{
			// Record the current ROS time
			markerTime = ros::Time::now();
			iHaveMarker = true;
		}
		bool haveMarker() const { return iHaveMarker; }
		double getElapsed() const { return (iHaveMarker ? (ros::Time::now() - markerTime).toSec() : -1.0); } // Returns the current elapsed time since the marker was set (returns -1.0 if no marker has been set - check this if you must as `getElapsed() < 0.0`)
		bool hasElapsed(double duration) const { return !iHaveMarker || (iHaveMarker && ((ros::Time::now() - markerTime).toSec() >= duration)); } // Returns whether a certain time duration has elapsed since the time marker was set (returns true if no marker has been set)

	private:
		// Internal variables
		ros::Time markerTime;
		bool iHaveMarker;
	};

	//
	// RobotSC state controller class
	//
	class RobotSC : public StateController
	{
	public:
		// Constructor
		RobotSC();
		void onConstruct();

		// Reset function
		void reset();

		// Initialisation function
		void initField();

		// Config server parameters
		config_server::Parameter<bool> m_be_run;
		config_server::Parameter<bool> m_be_use_obstacle;
		config_server::Parameter<bool> m_be_place_robot;
		config_server::Parameter<bool> m_be_rand_robot;
		config_server::Parameter<bool> m_be_rand_ball;
		config_server::Parameter<bool> m_be_rand_goal;
		config_server::Parameter<bool> m_be_rand_obst;
		config_server::Parameter<bool> m_be_imperfect_meas;

	protected:
		// ROS topics and services
		ros::Subscriber m_sub_robotState;
		ros::Publisher m_pub_plot;
		ros::Publisher m_pub_gaitCommand;
		ros::Publisher m_pub_object_markers;
		ros::ServiceClient m_srv_setOdom;
		ros::ServiceClient m_srv_setParam;
		ros::ServiceClient m_srv_playMotion;

		// ROS topic handlers
		void handleRobotState(const robotcontrol::StateConstPtr& msg);

		// ROS interface functions
		bool setOdom(double x, double y);
		bool setParam(const std::string& name, const std::string& value);
		bool playMotion(KeyMotionEnum motionID);

		// State controller callbacks
		virtual bool preStepCallback();
		bool preStepCallbackUser();
		virtual void preExecuteCallback();
		virtual void postStepCallback();
		void postStepCallbackUser();

		// Data retrieval members
		field_model::FieldModel* m_field;
		tf::TransformListener m_tflistener;
		bool getRobotTransform(tf::StampedTransform& transform, bool suppressWarn = false) const;

		// Detection simulation functions
		void calcRelativeVector(const Eigen::Vector2d& globalPos, Eigen::Vector2d& relativePos) const;
		void addDetectionNoise(Eigen::Vector2d& objVec, double magnitude = 0.1) const;
		bool isInView(const Eigen::Vector2d& objVec, double objAperture = 0.0) const;
		bool isDetected(const Eigen::Vector2d& objVec, double minDist, double maxDist) const;

		// Placement functions
		void placeGoal();
		void placeRobot();
		void placeBall();
		void placeObst();

		// Update functions
		bool updateRobotPos();
		void updateBallVector();
		void updateGoalVector();
		void updateObstVector();

		// Visualisation functions
		void publishMarkers();
		void publishPlotter();

		// Internal variables
		std::string m_robotState;
		RosTimeMarker m_lastPlayCall;
		RosTimeMarker m_lastOdomCall;
		bool m_usingPositiveGoal;
		plot_msgs::Plot m_plot;
		gait_msgs::GaitCommand m_lastGaitCmd;
		visualization_msgs::MarkerArray m_markerArray;
		tf::StampedTransform m_robotTransform;
		tf::Transform m_robotTransformInv;
		double m_robotOrient;
		Eigen::Vector2d m_robotPos;
		Eigen::Vector2d m_ballPos;
		Eigen::Vector2d m_goalPos;
		Eigen::Vector2d m_obstPos;
		SVector m_ballVec;
		SVector m_goalVec;
		SVector m_obstVec;

	public: // Note: You can call any function below here... (Do not modify any functions declared above)
		// Get functions
		std::string robotState() const { return m_robotState; }
		bool robotIsStanding()   const { return m_robotState == "standing"  ; }
		bool robotIsWalking()    const { return m_robotState == "walking"   ; }
		bool robotIsKicking()    const { return (m_robotState == "SIM_RIGHT_KICK") || (m_robotState == "SIM_LEFT_KICK"); }

		// Gait functions
		void sendGaitCommand(gait_msgs::GaitCommand cmd);
		void sendGaitCommand(bool walk, double Vx = 0.0, double Vy = 0.0, double W = 0.0);

		// Play motion functions
		bool playLeftKick()  { return playMotion(KM_SIM_LEFT_KICK); }
		bool playRightKick() { return playMotion(KM_SIM_RIGHT_KICK); }

		// Plotting functions
		void plotScalar(double value, const std::string& name);
		void plotVector(const Eigen::Vector2d& value, const std::string& name);
		void plotVector(const Eigen::Vector3d& value, const std::string& name);

		// Field state functions
		SVector ballVector() const { return m_ballVec; }
		SVector goalVector() const { return m_goalVec; }
		SVector obstVector() const { return m_obstVec; }
		bool haveBall(double timeTol = 0.2) const { return m_ballVec.isCurrent(timeTol); }
		bool haveGoal(double timeTol = 0.2) const { return m_goalVec.isCurrent(timeTol); }
		bool haveObst(double timeTol = 0.2) const { return m_obstVec.isCurrent(timeTol); }

		// Shared variables
		// TODO: Declare any shared variables you need here
	};

	//
	// State classes
	//

	// IdleState class
	class IdleState : public GenState<RobotSC>
	{
	public:
		// Constructor
		IdleState(RobotSC* sc) : GenState(sc, IDLE, "IDLE") {}

	protected:
		// State callbacks
		virtual action_t execute(cycle_t cyc);
	};

	// TODO: Define all other required state classes here (e.g. class SearchForBallState : public GenState<RobotSC>)
}

#endif /* BEHAVIOUR_EXERCISE_TEMPLATE_H */
// EOF