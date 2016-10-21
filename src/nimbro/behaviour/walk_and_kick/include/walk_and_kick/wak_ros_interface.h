// Walk and kick: ROS interface class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_ROS_INTERFACE_H
#define WAK_ROS_INTERFACE_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <walk_and_kick/wak_config.h>
#include <walk_and_kick/wak_actuator_vars.h>
#include <walk_and_kick/wak_gc_ros.h>
#include <walk_and_kick/wak_tc_ros.h>
#include <walk_and_kick/wak_vis.h>
#include <walk_and_kick/BehaviourState.h>
#include <walk_and_kick/TeamCommsData.h>
#include <walk_and_kick/VisualiseDBH.h>
#include <walk_and_kick/VisualiseDbApp.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nimbro_op_interface/LEDCommand.h>
#include <nimbro_op_interface/Button.h>
#include <vision_module/vision_outputs.h>
#include <head_control/LookAtTarget.h>
#include <robotcontrol/RobotHeading.h>
#include <robotcontrol/Diagnostics.h>
#include <robotcontrol/State.h>
#include <plot_msgs/plot_manager.h>
#include <tf/transform_broadcaster.h>
#include <gait_msgs/GaitCommand.h>
#include <boost/shared_ptr.hpp>
#include <gait/gait_common.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

// Walk and kick namespace
namespace walk_and_kick
{
	// Using declaration
	using nimbro_op_interface::LEDCommand;

	// Class declarations
	class WalkAndKick;
	class WAKGameState;
	class WAKBehState;

	/**
	* @class WAKRosInterface
	* 
	* @brief An interface class between the walk and kick and ROS worlds.
	**/
	class WAKRosInterface
	{
	public:
		// Constructor
		explicit WAKRosInterface(WalkAndKick* wak);
		virtual ~WAKRosInterface();

		// ROS node handle
		ros::NodeHandle nh;

	private:
		// Main walk and kick object (don't use this for anything other than to forward callbacks if necessary, but in general avoid using at all cost)
		WalkAndKick* wak;

	public:
		// Config parameters
		WAKConfig& config;

		// Reset function
		void resetData();

		// Return whether this robot is a fake one (e.g. xs0)
		bool isFakeRobot() const { return m_isFakeRobot; }

		// Update function
		void update(const ros::Time& now, bool LED4);

		// Actuator output functions
		void writeActuators(const ActuatorVars& actVar);
		void writeNeutralHead();
		void writeZeroGcv();
		void writeNeutral() { writeNeutralHead(); writeZeroGcv(); writeRGBLEDState(); }

		// LED functions
		void clearLEDState() { writeLEDCommand(0xFF, 0x00); }
		void writeLEDCommand(int mask, int state);
		void updateRGBLED(bool blink);
		void clearRGBLED() const;

		// Behaviour state
		void publishBehaviourState(const WAKGameState* gameState, const WAKBehState* behState);
		void publishTeamCommsPacket(const TeamCommsData& packet) const { m_pub_teamComms.publish(packet); }

		// Robot state functions
		bool stateRelaxed() const { return m_relaxed; }
		bool stateIniting() const { return m_initing; }
		bool stateStanding() const { return m_standing; }
		bool stateWalking() const { return m_walking; }
		bool stateKicking() const { return m_kicking; }
		bool stateFallen() const { return m_fallen; }

		// Game controller
		GCRosInterface GCRI;

		// Team communications
		TCRosInterface TCRI;

		// Button variables
		ButtonState button;

		// Ball variables
		Vec2f ballVec;
		float ballConf;
		bool ballDetected;
		ros::Time ballTime;

		// Goal post variables
		GoalPostList goalPostList;
		ros::Time goalPostTime;

		// Obstacle variables
		ObstacleList obstacleList;
		ros::Time obstacleTime;

		// Robot pose variables
		Vec3f robotPoseVec;
		float robotPoseConf;
		ros::Time robotPoseTime;

		// Robot heading variables
		float robotHeading;
		ros::Time robotHeadingTime;

		// Robotcontrol variables
		bool robotcontrolCommsOk;

		// Dive variables
		DiveDirection diveDecision;

		// TF transforms
		void sendTransform(const Vec3f& RobotPose);

		// Plot manager
		plot_msgs::PlotManagerFS* PM;
		plot_msgs::PlotManagerFS& getPM() const { return *PM; }

		// Marker manager
		WAKMarkerMan* MM;
		WAKMarkerMan& getMM() const { return *MM; }
		WAKBagMarkerMan* MMB;
		WAKBagMarkerMan& getMMB() const { return *MMB; }

	private:
		// TF transforms
		tf::TransformBroadcaster m_tfBroadcaster;
		tf::StampedTransform m_tfBehField;

		// Plot manager
		void callbackPlotData();

		// Config param callbacks
		void updateModeStateText() { MM->ModeStateText.setText((config.sListenToGC() ? "Extern " : "Local ") + m_buttonName); }
		void handleBlockGCPackets() { GCRI.setEnabled(!config.debugBlockGCPackets()); }
		void handleNoStoppedGCV() { gait::resetGaitCommand(m_stoppedGaitCmd); }

		// ROS publishers
		ros::Publisher m_pub_gaitCmd;
		ros::Publisher m_pub_headCmd;
		ros::Publisher m_pub_leds;
		ros::Publisher m_pub_state;
		ros::Publisher m_pub_teamComms;

		// ROS subscribers
		ros::Subscriber m_sub_button;
		ros::Subscriber m_sub_robotState;
		ros::Subscriber m_sub_robotHeading;
		ros::Subscriber m_sub_stoppedGaitCmd;
		ros::Subscriber m_sub_robotDiagnostics;
		ros::Subscriber m_sub_visionOutput;
		ros::Subscriber m_sub_diveDecision;

		// ROS service servers
		ros::ServiceServer m_srv_visualiseClear;
		ros::ServiceServer m_srv_visualiseDBH;
		ros::ServiceServer m_srv_visualiseDbApp;
		ros::ServiceServer m_srv_visualiseGcvXY;

		// ROS data handlers
		void handleButtonData(const nimbro_op_interface::ButtonConstPtr& msg);
		void handleRobotStateData(const robotcontrol::StateConstPtr& msg);
		void handleRobotHeadingData(const robotcontrol::RobotHeadingConstPtr& msg);
		void handleStoppedGaitCommand(const gait_msgs::GaitCommandConstPtr& msg);
		void handleRobotDiagnosticsData(const robotcontrol::DiagnosticsConstPtr& msg);
		void handleVisionOutputData(const vision_module::vision_outputsConstPtr& msg);
		void handleDiveDecision(const std_msgs::StringConstPtr& msg);

		// ROS service handlers
		bool handleVisualiseClear(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
		bool handleVisualiseDBH(walk_and_kick::VisualiseDBHRequest& req, walk_and_kick::VisualiseDBHResponse& resp);
		bool handleVisualiseDbApp(walk_and_kick::VisualiseDbAppRequest& req, walk_and_kick::VisualiseDbAppResponse& resp);
		bool handleVisualiseGcvXY(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

		// Robot state variables
		bool m_relaxed;
		bool m_initing;
		bool m_standing;
		bool m_walking;
		bool m_kicking;
		bool m_fallen;

		// LED functions
		void writeRGBLEDState() const;

		// LED variables
		int m_lastLEDState;
		int m_lastLEDMask;
		ros::Time m_lastLEDTime;
		bool m_blinkRGBLED;

		// Miscellaneous variables
		bool m_isFakeRobot;
		ButtonState m_lastButton;
		std::string m_buttonName;
		bool m_btnHeadCmd;
		head_control::LookAtTarget m_headCmd;
		gait_msgs::GaitCommand m_stoppedGaitCmd;
		BehaviourState m_behState;
		bool m_publishedNeutral; // Note: This must be reset in the constructor and *not* in resetData()!
	};
}

#endif
// EOF
