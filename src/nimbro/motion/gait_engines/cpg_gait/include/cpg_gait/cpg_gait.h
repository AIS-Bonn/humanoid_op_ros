// Central pattern generated gait
// File: cpg_gait.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CPG_GAIT_H
#define CPG_GAIT_H

// Includes - Local
#include <cpg_gait/cpg_gait_config.h>

// Includes - Gait
#include <gait/gait_common.h>
#include <gait/gait_engine.h>
#include <gait/util/gait_joint_pose.h>
#include <gait/util/gait_abstract_pose.h>
#include <gait/util/gait_inverse_pose.h>

// Includes - Misc
#include <plot_msgs/plot_manager.h>

// Includes - Library
#include <Eigen/Core>

// CPG gait namespace
namespace cpg_gait
{
	/**
	* @class CPGGait
	*
	* @brief A central pattern generated gait, implemented as a `GaitEngine` plugin for the `Gait` motion module.
	**/
	class CPGGait : public gait::GaitEngine
	{
	public:
		// Constructor
		CPGGait(); //!< Default constructor

		// Reset function
		virtual void reset();

		// Halt pose function
		virtual void updateHaltPose();

		// Step function
		virtual void step();

	private:
		// Reset walking function (a lighter version of reset(), used for internal resets)
		void resetWalking(bool walking, const Eigen::Vector3d& gcvBias);

		// Step worker functions
		void processInputs();
		void abstractArmMotion(gait::AbstractArmPose& arm);
		void abstractLegMotion(gait::AbstractLegPose& leg);
		void inverseLegMotion(gait::InverseLegPose& leg);
		void updateOutputs();

		// Gait configuration struct
		CPGConfig config;

		// Constants
		const std::string CONFIG_PARAM_PATH;

		// Configuration parameters
		config_server::Parameter<bool> m_plotData;  

		// Pose variables
		gait::JointPose m_jointPose;           // Joint representation of the pose to command in a step
		gait::JointPose m_jointHaltPose;       // Joint representation of the gait halt pose
		gait::InversePose m_inversePose;       // Inverse representation of the pose to command in a step
		gait::AbstractPose m_abstractPose;     // Abstract representation of the pose to command in a step
		gait::AbstractPose m_abstractHaltPose; // Abstract representation of the gait halt pose

		// Gait command vector variables
		Eigen::Vector3d m_gcv;      // Gait command velocity vector (slope-limited command velocities actually followed by the gait engine)
		Eigen::Vector3d m_gcvInput; // Gait command velocity vector input (raw velocities commanded by the gait motion module)

		// Gait flags
		bool m_walk;                // True if the gait engine should walk, false if it should not walk
		bool m_walking;             // True if the gait engine is currently producing a walking motion output
		bool m_leftLegFirst;        // True if the left leg should be the first to be lifted when starting walking
		bool m_walkStartBlending;   // True if walk start blending is currently active

		// Step motion variables
		double m_gaitPhase;         // Current walking phase of the gait motion
		double m_virtualSlope;      // Current virtual slope factor

		// Plot manager
		plot_msgs::PlotManagerFS m_PM;
		void configurePlotManager();
		void callbackPlotData();
		enum PMIDS
		{
			PM_GCV = 0,
			PM_GCV_X = PM_GCV,
			PM_GCV_Y,
			PM_GCV_Z,
			PM_GAIT_PHASE,
			PM_ARM_SWING_ANGLE_R,
			PM_ARM_SWING_ANGLE_L,
			PM_ARM_SAG_SWING_R,
			PM_ARM_SAG_SWING_L,
			PM_LEG_EXTENSION_R,
			PM_LEG_EXTENSION_L,
			PM_LEG_SWING_ANGLE_R,
			PM_LEG_SWING_ANGLE_L,
			PM_LEG_SAG_SWING_R,
			PM_LEG_SAG_SWING_L,
			PM_LEG_LAT_SWING_R,
			PM_LEG_LAT_SWING_L,
			PM_LEG_ROT_SWING_R,
			PM_LEG_ROT_SWING_L,
			PM_LEG_LAT_HIP_SWING_R,
			PM_LEG_LAT_HIP_SWING_L,
			PM_LEG_VIRTUAL_COMP_R,
			PM_LEG_VIRTUAL_COMP_L,
			PM_LEG_VIRTUAL_SLOPE_R,
			PM_LEG_VIRTUAL_SLOPE_L,
			PM_COUNT
		};
	};
}

#endif /* CPG_GAIT_H */
// EOF