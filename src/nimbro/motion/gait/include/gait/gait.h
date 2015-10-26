// Generic gait motion module
// File: gait.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef GAIT_H
#define GAIT_H

// Includes
#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/robotmodel.h>
#include <nimbro_utils/math_spline.h>
#include <config_server/parameter.h>
#include <plot_msgs/plot_manager.h>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

// Includes - Local
#include <gait/gait_common.h>
#include <gait/gait_command.h>
#include <gait/gait_engine.h>

// Includes - Messages
#include <tf/transform_broadcaster.h>
#include <gait_msgs/GaitCommand.h>
#include <gait_msgs/GaitOdom.h>
#include <gait_msgs/SetOdom.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

// Includes - ROS
#include <ros/subscriber.h>

// Includes - C++ Standard Library
#include <string>

/**
* @namespace gait
*
* @brief Contains all classes necessary for the generic gait motion module.
**/
namespace gait
{
	/**
	* @class Gait
	*
	* @brief Generic gait motion module that can execute a given `GaitEngine`.
	*
	* The gait engine to use for an instance of the Gait motion module is specified via the
	* motion module parameter string.
	**/
	class Gait : public robotcontrol::MotionModule
	{
	public:
		// Constructor
		Gait(); //!< Default constructor
		virtual ~Gait(); //!< Destructor

		// Initialisation function
		virtual bool init(robotcontrol::RobotModel* model);

		// Trigger function
		virtual bool isTriggered();

		// Step function
		virtual void step();

		// Publish transforms function
		virtual void publishTransforms();

	protected:
		// Get function for gait name
		const std::string& gaitName() const { return m_gaitName; } //!< Retrieve the name of the gait (e.g. if the parameter string is `cpg_gait::CPGGait` then the gait name is everything past the first double colon, i.e. `CPGGait`)

	private:
		// Constants
		const std::string CONFIG_PARAM_PATH;

		// Configuration parameters
		boost::shared_ptr<config_server::Parameter<bool> > m_enableGait; // Flag whether to enable this gait motion module. The config parameter name is specific to the gait engine name.
		config_server::Parameter<bool> m_enableJoystick;                 // Flag whether to globally enable the use of the joystick to control the gait command vector. When enabled, the first button on the joystick toggles the joystick mode on and off.
		config_server::Parameter<bool> m_plotData;                       // Flag whether to plot gait data to the plotter visualisation.
		config_server::Parameter<bool> m_publishOdometry;                // Flag whether to publish the gait odometry.
		config_server::Parameter<bool> m_publishTransforms;              // Flag whether to publish the ego_floor and gait odometry TF transforms.
		config_server::Parameter<float> m_gaitCmdVecNormP;               // The p parameter with which to calculate the gait command vector norm (i.e. using the p-norm).
		config_server::Parameter<float> m_gaitCmdVecNormMax;             // The maximum allowed gait command vector norm, beyond which normalisation to this maximum value is performed.

		// Robot model
		robotcontrol::RobotModel* m_model;                   // Pointer to the RobotModel object to work with
		int m_jointMap[NUM_JOINTS];                          // Maps a JointID enum to its corresponding joint index in RobotModel
		boost::shared_ptr<const urdf::Link> m_trunkLink;     // Trunk URDF link (used for setting support coefficients)
		boost::shared_ptr<const urdf::Link> m_leftFootLink;  // Left foot URDF link (used for setting support coefficients)
		boost::shared_ptr<const urdf::Link> m_rightFootLink; // Right foot URDF link (used for setting support coefficients)

		// Step execution timing
		ros::Time m_now;
		ros::Time m_lastNow;
		double m_dT;

		// Robot states
		robotcontrol::RobotModel::State m_state_standing;
		robotcontrol::RobotModel::State m_state_walking;
		
		// Gait name (valid after init() has been called)
		std::string m_gaitName;

		// Transforms
		void updateTransforms();

		// Gait engine
		boost::shared_ptr<GaitEngine> m_engine;
		pluginlib::ClassLoader<GaitEngine> m_enginePluginLoader;
		bool loadGaitEngine();
		void updateHaltPose();
		void updateGaitEngineInputs(GaitEngineInput& in);
		void processGaitEngineOutputs(const GaitEngineOutput& out);
		void writeJointCommands(const GaitEngineOutput& out);
		void setSupportCoefficients(double leftLegCoeff, double rightLegCoeff);
		void plotGaitEngineInputs(const GaitEngineInput& in);
		void plotGaitEngineOutputs(const GaitEngineOutput& out);

		// Gait command
		GaitCommand m_gaitCmd;
		ros::Subscriber m_sub_gaitCommand;
		void handleGaitCommand(const gait_msgs::GaitCommandConstPtr& cmd);
		void plotRawGaitCommand();

		// Motions
		void setPendingMotion(MotionID ID, MotionStance stance = STANCE_DEFAULT, bool adjustLeft = true, bool adjustRight = true);
		void clearPendingMotion();
		MotionID m_motionPending;     // The ID of a pending motion, if there is one, otherwise MID_NONE
		MotionID m_oldMotionPending;  // The value of m_motionPending in the last cycle, for algorithmic purposes
		MotionStance m_motionStance;  // The required stance of the robot before executing the motion
		bool m_motionAdjustLeftFoot;  // Flag whether the left foot should be adjusted to attain the required foot separation (how the required foot separation for various motions is defined is up to the gait engine)
		bool m_motionAdjustRightFoot; // Flag whether the right foot should be adjusted to attain the required foot separation (how the required foot separation for various motions is defined is up to the gait engine)

		// Halt pose
		nimbro_utils::TrapVelSpline m_jointSpline[NUM_JOINTS];
		nimbro_utils::LinearSpline m_jointEffortSpline[NUM_JOINTS];
		ros::Time m_reachStartTime;
		double m_reachDuration;
		bool m_reachedHalt;
		bool m_updatedHalt;
		void startReachHaltPose();
		void continueReachHaltPose();
		void stopReachHaltPose();

		// Joystick data
		bool m_joystickEnabled;
		bool m_joystickGaitCmdLock;
		bool m_joystickButton0Pressed;
		bool m_joystickButton1Pressed;
		bool m_joystickButton2Pressed;
		bool m_joystickButton3Pressed;
		ros::Subscriber m_sub_joystickData;
		void handleJoystickData(const sensor_msgs::JoyConstPtr& joy);
		void setJoystickGaitCmdLock(bool lock);
		void callbackEnableJoystick();

		// TF transforms
		tf::TransformBroadcaster m_tf_broadcaster;
		std::vector<tf::StampedTransform> m_tf_transforms;
		tf::StampedTransform* m_tf_ego_floor;
		tf::StampedTransform* m_tf_odom;
		gait_msgs::GaitOdom m_gait_odom;
		ros::Publisher m_pub_odom;
		void configureTransforms();

		// Gait odometry
		ros::ServiceServer m_srv_resetOdom;
		ros::ServiceServer m_srv_setOdom;
		bool handleResetOdometry(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
		bool handleSetOdometry(gait_msgs::SetOdomRequest &req, gait_msgs::SetOdomResponse &res);

		// Joint functions
		bool constructJointMap();

		// Gait state
		enum GaitState
		{
			GS_INACTIVE,
			GS_REACHING_HALT_POSE,
			GS_STARTING_WALKING,
			GS_WALKING,
			GS_STOPPING_WALKING
		};
		GaitState m_gaitState;
		void resetGait();
		void plotGaitStateEvent();

		// Plot manager
		plot_msgs::PlotManagerFS m_PM;
		void configurePlotManager();
		void callbackPlotData();
		enum PMIDS
		{
			PM_NOMINAL_DT = 0,
			PM_TRUE_DT,
			PM_GAITCMD_LIN_VEL_X,
			PM_GAITCMD_LIN_VEL_Y,
			PM_GAITCMD_ANG_VEL_Z,
			PM_GAITCMD_WALK,
			PM_JOINTCMD_FIRST,
			PM_JOINTCMD_LAST = PM_JOINTCMD_FIRST + NUM_JOINTS - 1,
			PM_JOINTEFFORT_FIRST,
			PM_JOINTEFFORT_LAST = PM_JOINTEFFORT_FIRST + NUM_JOINTS - 1,
			PM_USE_RAW_JOINT_CMDS,
			PM_WALKING,
			PM_LEFT_SUPPORT_COEFF,
			PM_RIGHT_SUPPORT_COEFF,
			PM_GAITCMDRAW_LIN_VEL_X,
			PM_GAITCMDRAW_LIN_VEL_Y,
			PM_GAITCMDRAW_ANG_VEL_Z,
			PM_GAITCMDRAW_WALK,
			PM_GAIT_STATE,
			PM_GAIT_ODOM_X,
			PM_GAIT_ODOM_Y,
			PM_GAIT_ODOM_Z,
			PM_GAIT_ODOM_ID,
			PM_COUNT
		};
	};
}

#endif /* GAIT_H */
// EOF