// Capture step gait
// File: cap_gait.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CAP_GAIT_H
#define CAP_GAIT_H

// Includes - Local
#include <cap_gait/cap_gait_config.h>
#include <cap_gait/cap_com_filter.h>

// Includes - Gait
#include <gait/gait_common.h>
#include <gait/gait_engine.h>
#include <gait/util/gait_joint_pose.h>
#include <gait/util/gait_inverse_pose.h>
#include <gait/util/gait_abstract_pose.h>

// Includes - Misc
#include <plot_msgs/plot_manager.h>
#include <nimbro_utils/ew_integrator.h>
#include <nimbro_utils/mean_filter.h>
#include <nimbro_utils/wlbf_filter.h>

// Includes - Library
#include <Eigen/Core>

// Includes - Contrib [These are included last because they potentially contain Qt includes that break the Boost signals library... (e.g. used by TF)]
#include <cap_gait/contrib/RobotModel.h>
#include <cap_gait/contrib/RobotModelVis.h>
#include <cap_gait/contrib/LimpModel.h>
#include <cap_gait/contrib/Limp.h>
#include <cap_gait/contrib/ComFilter.h>

// Capture step gait namespace
namespace cap_gait
{
	/**
	* @class CapGait
	*
	* @brief A capture step gait, implemented as a `GaitEngine` plugin for the `Gait` motion module.
	**/
	class CapGait : public gait::GaitEngine
	{
	public:
		//
		// Public functions
		//

		// Constructor
		CapGait(); //!< Default constructor

		// Reset function
		virtual void reset();

		// Step function
		virtual void step();

		// Halt pose function
		virtual void updateHaltPose();

		// Set odometry function
		virtual void setOdometry(double posX, double posY, double rotZ);

		// Update odometry function
		virtual void updateOdometry();

	private:
		//
		// Structs
		//
		
		// Common motion data struct
		struct CommonMotionData
		{
			// Constructor
			CommonMotionData() // All data that is initialised here is just to avoid divisions by zero etc in case a bug causes an uninitialised value in this struct to be used by accident
			 : gcvX(0.0)
			 , gcvY(0.0)
			 , gcvZ(0.0)
			 , absGcvX(0.0)
			 , absGcvY(0.0)
			 , absGcvZ(0.0)
			 , gaitPhase(0.0)
			 , oppGaitPhase(M_PI)
			 , limbPhase(0.0)
			 , absPhase(0.0)
			 , doubleSupportPhase(0.1)
			 , swingStartPhase(0.0)
			 , swingStopPhase(M_PI)
			 , suppTransStartPhase(0.0)
			 , suppTransStopPhase(doubleSupportPhase)
			 , liftingPhaseLen(M_PI - doubleSupportPhase)
			 , suppPhaseLen(M_PI + doubleSupportPhase)
			 , nonSuppPhaseLen(M_2PI - suppPhaseLen)
			 , sinusoidPhaseLen(M_PI)
			 , linearPhaseLen(M_PI)
			 , swingAngle(0.0)
			{}
			
			// Gait command vector variables
			double gcvX;
			double gcvY;
			double gcvZ;
			double absGcvX;
			double absGcvY;
			double absGcvZ;
			
			// Gait phase variables
			double gaitPhase;
			double oppGaitPhase;
			double limbPhase;
			double absPhase;
			
			// Phase marks
			double doubleSupportPhase;
			double swingStartPhase;
			double swingStopPhase;
			double suppTransStartPhase;
			double suppTransStopPhase;
			
			// Extra swing variables
			double liftingPhaseLen;
			double suppPhaseLen;
			double nonSuppPhaseLen;
			double sinusoidPhaseLen;
			double linearPhaseLen;
			
			// Swing angle
			double swingAngle;
		};
		
		//
		// Private functions
		//

		// Reset walking function (a lighter version of reset(), used for internal resets when starting and stopping walking)
		void resetWalking(bool walking, const Eigen::Vector3d& gcvBias);

		// Input processing
		void processInputs();
		void updateRobot(const Eigen::Vector3d& gcvBias);

		// Helper functions
		CommonMotionData calcCommonMotionData(bool isFirst) const;

		// Motion functions
		void abstractLegMotion(gait::AbstractLegPose& leg);
		void abstractArmMotion(gait::AbstractArmPose& arm);
		void inverseLegMotion(gait::InverseLegPose& leg);
		
		// Coercion functions
		void coerceAbstractPose(gait::AbstractPose& pose);
		void coerceAbstractArmPose(gait::AbstractArmPose& arm);
		void coerceAbstractLegPose(gait::AbstractLegPose& leg);

		// Output processing
		void updateOutputs();

		// Blending functions
		void resetBlending(double b = USE_HALT_POSE);
		void setBlendTarget(double target, double phaseTime);
		double blendFactor();

		// Motion stance functions
		void resetMotionStance();

		//
		// Gait variables
		//

		// Constants
		const std::string CONFIG_PARAM_PATH;
		static const double USE_HALT_POSE = 1.0;
		static const double USE_CALC_POSE = 0.0;

		// Gait configuration struct
		CapConfig config;

		// Pose variables
		gait::JointPose m_jointPose;           // Joint representation of the pose to command in a step
		gait::JointPose m_jointHaltPose;       // Joint representation of the gait halt pose
		gait::JointPose m_lastJointPose;       // The last joint pose to have been commanded during walking
		gait::InversePose m_inversePose;       // Inverse representation of the pose to command in a step
		gait::AbstractPose m_abstractPose;     // Abstract representation of the pose to command in a step
		gait::AbstractPose m_abstractHaltPose; // Abstract representation of the gait halt pose

		// Gait command vector variables
		Eigen::Vector3d m_gcv;      // Gait command velocity vector (slope-limited command velocities actually followed by the gait engine)
		Eigen::Vector3d m_gcvInput; // Gait command velocity vector input (raw velocities commanded by the gait motion module)
		robotcontrol::GolayDerivative<Eigen::Vector3d, 1, 5, Eigen::aligned_allocator<Eigen::Vector3d> > m_gcvDeriv; // Derivative filter for the GCV to calculate the gait acceleration
		nimbro_utils::MeanFilter m_gcvAccSmoothX;
		nimbro_utils::MeanFilter m_gcvAccSmoothY;
		nimbro_utils::MeanFilter m_gcvAccSmoothZ;
		Eigen::Vector3d m_gcvAcc;

		// Gait flags
		bool m_walk;                // True if the gait engine should walk, false if it should not walk
		bool m_walking;             // True if the gait engine is currently producing a walking motion output
		bool m_leftLegFirst;        // True if the left leg should be the first to be lifted when starting walking

		// Step motion variables
		double m_gaitPhase;         // Current walking phase of the gait motion

		// Blending variables
		bool m_blending;
		double m_b_current;
		double m_b_initial;
		double m_b_target;
		double m_blendPhase;
		double m_blendEndPhase;
		
		// Motion stance variables
		double m_motionLegAngleXFact; // Interpolation factor between feet narrow (= 0) and feet normal (= 1) legAngleX values
		
		//
		// Basic feedback variables
		//
		
		// Basic feedback filters
		nimbro_utils::MeanFilter fusedXFeedFilter;
		nimbro_utils::MeanFilter fusedYFeedFilter;
		nimbro_utils::WLBFFilter dFusedXFeedFilter;
		nimbro_utils::WLBFFilter dFusedYFeedFilter;
		nimbro_utils::MeanFilter iFusedXFeedFilter;
		nimbro_utils::MeanFilter iFusedYFeedFilter;
		nimbro_utils::WLBFFilter gyroXFeedFilter;
		nimbro_utils::WLBFFilter gyroYFeedFilter;
		
		// Integrators
		nimbro_utils::EWIntegrator iFusedXFeedIntegrator;
		nimbro_utils::EWIntegrator iFusedYFeedIntegrator;
		config_server::Parameter<bool> m_resetIntegrators;    // Rising edge triggered flag to reset any integrated or learned values in the gait that are not necessarily reset during start/stop of walking
		config_server::Parameter<bool> m_saveIFeedToHaltPose; // Rising edge triggered flag to save the current integrated feedback values as offsets to the halt pose (only the ones in current use)
		bool m_savedLegIFeed;   // Flag that specifies within a cycle whether the integrated leg feedback has already been saved
		bool m_savedArmIFeed;   // Flag that specifies within a cycle whether the integrated arm feedback has already been saved
		double iFusedXLastTime; // The last in.timestamp where iFusedX made a non-zero contribution to the CPG gait
		double iFusedYLastTime; // The last in.timestamp where iFusedY made a non-zero contribution to the CPG gait
		bool haveIFusedXFeed;   // Boolean flag whether the iFusedX integrator had any effect on iFusedXFeed in the current cycle
		bool haveIFusedYFeed;   // Boolean flag whether the iFusedY integrator had any effect on iFusedYFeed in the current cycle
		bool usedIFusedX;       // Boolean flag whether the iFusedX integrator had any effect on the produced joint commands in the current cycle
		bool usedIFusedY;       // Boolean flag whether the iFusedY integrator had any effect on the produced joint commands in the current cycle
		void resetIntegrators();
		void resetSaveIntegrals();
		
		// Callbacks for updating the filter sizes
		void resizeFusedFilters (int numPoints) { if(numPoints < 1) numPoints = 1; fusedXFeedFilter.resize(numPoints);  fusedYFeedFilter.resize(numPoints);  }
		void resizeDFusedFilters(int numPoints) { if(numPoints < 1) numPoints = 1; dFusedXFeedFilter.resize(numPoints); dFusedYFeedFilter.resize(numPoints); }
		void resizeIFusedFilters(int numPoints) { if(numPoints < 1) numPoints = 1; iFusedXFeedFilter.resize(numPoints); iFusedYFeedFilter.resize(numPoints); }
		void resizeGyroFilters  (int numPoints) { if(numPoints < 1) numPoints = 1; gyroXFeedFilter.resize(numPoints);   gyroYFeedFilter.resize(numPoints);   }
		
		// Basic feedback values
		double fusedXFeed;
		double fusedYFeed;
		double dFusedXFeed;
		double dFusedYFeed;
		double iFusedXFeed;
		double iFusedYFeed;
		double gyroXFeed;
		double gyroYFeed;

		//
		// Capture step variables
		//

		// Reset function
		void resetCaptureSteps(bool resetRobotModel);

		// Capture step robot model
		margait_contrib::RobotModel rxRobotModel;
		margait_contrib::RobotModelVis m_rxVis;
		config_server::Parameter<bool> m_showRxVis;
		void callbackShowRxVis();

		// Linear inverted pendulum robot models
		margait_contrib::LimpModel rxModel;
		margait_contrib::LimpModel mxModel;
		margait_contrib::LimpModel txModel;

		// Linear inverted pendulum model class for isolated calculations
		margait_contrib::Limp limp;

		// CoM filter
		ComFilter<5> m_comFilter;

		// Miscellaneous
		margait_contrib::Vec2f adaptation;
		double lastSupportOrientation;
		double oldGcvTargetY;
		double virtualSlope;
		double stepTimeCount;
		double lastStepDuration;
		int stepCounter;
		int resetCounter;
		int cycleNumber;

		//
		// Plotting
		//

		// Note: To add a variable for plotting you need to do the following...
		// 1) Add an enum value below ==> PM_MODEL_MY_DATA,
		// 2) Set the name of the plotted data in configurePlotManager() ==> m_PM.setName(PM_MODEL_MY_DATA, "model/myData");
		// 3) Actually plot the required data in the function where you calculate it ==> m_PM.plotScalar(myData, PM_MODEL_MY_DATA);
		// It is recommended to enclose the plotScaler() call in ==> if(m_PM.getEnabled()) { ... }

		// Plot manager
		config_server::Parameter<bool> m_plotData; 
		plot_msgs::PlotManagerFS m_PM;
		void configurePlotManager();
		void callbackPlotData();
		enum PMIDS
		{
			PM_GCV = 0,
			PM_GCV_X = PM_GCV,
			PM_GCV_Y,
			PM_GCV_Z,
			PM_GCV_ACC,
			PM_GCV_ACC_X = PM_GCV_ACC,
			PM_GCV_ACC_Y,
			PM_GCV_ACC_Z,
			PM_GAIT_PHASE,
			PM_USED_IFUSEDX,
			PM_USED_IFUSEDY,
			PM_HALT_BLEND_FACTOR,
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
			PM_LEG_SAG_LEAN_R,
			PM_LEG_SAG_LEAN_L,
			PM_LEG_LAT_LEAN_R,
			PM_LEG_LAT_LEAN_L,
			PM_LEG_FEED_HIPANGLEX_R,
			PM_LEG_FEED_HIPANGLEX_L,
			PM_LEG_FEED_HIPANGLEY_R,
			PM_LEG_FEED_HIPANGLEY_L,
			PM_LEG_FEED_FOOTANGLEX_R,
			PM_LEG_FEED_FOOTANGLEX_L,
			PM_LEG_FEED_FOOTANGLEY_R,
			PM_LEG_FEED_FOOTANGLEY_L,
			PM_LEG_FEED_FOOTANGLECX_R,
			PM_LEG_FEED_FOOTANGLECX_L,
			PM_LEG_FEED_FOOTANGLECY_R,
			PM_LEG_FEED_FOOTANGLECY_L,
			PM_LEG_ABS_LEGEXT_R,
			PM_LEG_ABS_LEGEXT_L,
			PM_LEG_ABS_LEGANGLEX_R,
			PM_LEG_ABS_LEGANGLEX_L,
			PM_LEG_ABS_LEGANGLEY_R,
			PM_LEG_ABS_LEGANGLEY_L,
			PM_LEG_ABS_LEGANGLEZ_R,
			PM_LEG_ABS_LEGANGLEZ_L,
			PM_LEG_ABS_FOOTANGLEX_R,
			PM_LEG_ABS_FOOTANGLEX_L,
			PM_LEG_ABS_FOOTANGLEY_R,
			PM_LEG_ABS_FOOTANGLEY_L,
			PM_LEG_FEED_COMSHIFTX_R,
			PM_LEG_FEED_COMSHIFTX_L,
			PM_LEG_FEED_COMSHIFTY_R,
			PM_LEG_FEED_COMSHIFTY_L,
			PM_LEG_VIRTUAL_SLOPE_R,
			PM_LEG_VIRTUAL_SLOPE_L,
			PM_LEG_VIRTUAL_COMP_R,
			PM_LEG_VIRTUAL_COMP_L,
			PM_ARM_SWING_ANGLE_R,
			PM_ARM_SWING_ANGLE_L,
			PM_ARM_SAG_SWING_R,
			PM_ARM_SAG_SWING_L,
			PM_ARM_FEED_ARMANGLEX_R,
			PM_ARM_FEED_ARMANGLEX_L,
			PM_ARM_FEED_ARMANGLEY_R,
			PM_ARM_FEED_ARMANGLEY_L,
			PM_ARM_ABS_ARMEXT_R,
			PM_ARM_ABS_ARMEXT_L,
			PM_ARM_ABS_ARMANGLEX_R,
			PM_ARM_ABS_ARMANGLEX_L,
			PM_ARM_ABS_ARMANGLEY_R,
			PM_ARM_ABS_ARMANGLEY_L,
			PM_RXRMODEL_SUPPVEC_X,
			PM_RXRMODEL_SUPPVEC_Y,
			PM_RXRMODEL_SUPPVEC_Z,
			PM_RXRMODEL_STEPVEC_X,
			PM_RXRMODEL_STEPVEC_Y,
			PM_RXRMODEL_STEPVEC_Z,
			PM_RXRMODEL_STEPVEC_FYAW,
			PM_FUSED_X,
			PM_FUSED_Y,
			PM_COMFILTER_X,
			PM_COMFILTER_Y,
			PM_COMFILTER_VX,
			PM_COMFILTER_VY,
			PM_RXMODEL_X,
			PM_RXMODEL_Y,
			PM_RXMODEL_VX,
			PM_RXMODEL_VY,
			PM_RXMODEL_SUPPLEG,
			PM_RXMODEL_TIMETOSTEP,
			PM_MXMODEL_X,
			PM_MXMODEL_Y,
			PM_MXMODEL_VX,
			PM_MXMODEL_VY,
			PM_MXMODEL_SUPPLEG,
			PM_MXMODEL_TIMETOSTEP,
			PM_MXMODEL_ZMP_X,
			PM_MXMODEL_ZMP_Y,
			PM_TXMODEL_X,
			PM_TXMODEL_Y,
			PM_TXMODEL_VX,
			PM_TXMODEL_VY,
			PM_TXMODEL_SUPPLEG,
			PM_TXMODEL_TIMETOSTEP,
			PM_ADAPTATION_X,
			PM_ADAPTATION_Y,
			PM_EXP_FUSED_X,
			PM_EXP_FUSED_Y,
			PM_DEV_FUSED_X,
			PM_DEV_FUSED_Y,
			PM_FEEDBACK_FUSED_X,
			PM_FEEDBACK_FUSED_Y,
			PM_FEEDBACK_DFUSED_X,
			PM_FEEDBACK_DFUSED_Y,
			PM_FEEDBACK_IFUSED_X,
			PM_FEEDBACK_IFUSED_Y,
			PM_FEEDBACK_GYRO_X,
			PM_FEEDBACK_GYRO_Y,
			PM_TIMING_FEED_WEIGHT,
			PM_TIMING_FREQ_DELTA,
			PM_GAIT_FREQUENCY,
			PM_REM_GAIT_PHASE,
			PM_TIMETOSTEP,
			PM_STEPSIZE_X,
			PM_STEPSIZE_Y,
			PM_STEPSIZE_Z,
			PM_LAST_STEP_DURATION,
			PM_COUNT
		};
	};
}

#endif
// EOF