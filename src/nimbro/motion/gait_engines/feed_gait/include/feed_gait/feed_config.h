// Feedback gait configuration parameters
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_CONFIG_H
#define FEED_CONFIG_H

// Includes
#include <feed_gait/feed_common.h>
#include <config_server/parameter.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @class FeedConfig
	*
	* @brief Configuration parameters for the feedback gait.
	**/
	class FeedConfig
	{
	public:
		// Constructor
		FeedConfig()
		 : CONFIG_PARAM_PATH(ROOT_CONFIG_PARAM_PATH)

		 , kinematicsType       (KIN_CONFIG_PARAM_PATH + "Type", KT_FIRST, 1, KT_COUNT - 1, KT_DEFAULT)
		 , trajectoryType       (TRAJ_CONFIG_PARAM_PATH + "Type", TT_FIRST, 1, TT_COUNT - 1, TT_DEFAULT)
		 , odometryType         (ODOM_CONFIG_PARAM_PATH + "Type", OT_FIRST, 1, OT_COUNT - 1, OT_DEFAULT)

		 , cmdUseActions        (CONFIG_PARAM_PATH + "cmd/useActions", false)
		 , cmdUseStepSize       (CONFIG_PARAM_PATH + "cmd/useStepSize", false)
		 , cmdUseStepSizeX      (CONFIG_PARAM_PATH + "cmd/useStepSizeX", false)
		 , cmdUseStepSizeY      (CONFIG_PARAM_PATH + "cmd/useStepSizeY", false)
		 , cmdUseStepSizeZ      (CONFIG_PARAM_PATH + "cmd/useStepSizeZ", false)
		 , cmdUseTiming         (CONFIG_PARAM_PATH + "cmd/useTiming", false)

		 , joystickMode         (CONFIG_PARAM_PATH + "joystick/mode", 0, 1, 8, 0)

		 , blendingStartPhaseLen(CONFIG_PARAM_PATH + "general/blending/startPhaseLen", 0.0, 0.05, 4.0*M_PI, 2.0*M_PI)
		 , blendingStopPhaseLen (CONFIG_PARAM_PATH + "general/blending/stopPhaseLen", 0.0, 0.05, 2.0*M_PI, M_PI)
		 , effortDefault        (CONFIG_PARAM_PATH + "general/effortDefault", 0.0, 0.01, 1.0, 0.2)
		 , fusedPitchOffset     (CONFIG_PARAM_PATH + "general/fusedPitchOffset", -0.5, 0.005, 0.5, 0.0)
		 , fusedRollOffset      (CONFIG_PARAM_PATH + "general/fusedRollOffset", -0.5, 0.005, 0.5, 0.0)
		 , gaitFrequency        (CONFIG_PARAM_PATH + "general/gaitFrequency", 0.3, 0.02, 5.0, 2.4)
		 , gaitFrequencyMax     (CONFIG_PARAM_PATH + "general/gaitFrequencyMax", 0.3, 0.02, 5.0, 3.0)
		 , leftLegFirst         (CONFIG_PARAM_PATH + "general/leftLegFirst", true)
		 , stoppingGcvMag       (CONFIG_PARAM_PATH + "general/stopping/gcvMag", 0.005, 0.005, 0.5, 0.1)
		 , stoppingPhaseTol     (CONFIG_PARAM_PATH + "general/stopping/phaseTol", 1.0, 0.05, 5.0, 2.0)
		 , useServoModel        (CONFIG_PARAM_PATH + "general/useServoModel", true)

		 , gcvSlopeLFForwards   (CONFIG_PARAM_PATH + "gcv/gcvSlopeLF/slopeForwards", 0.05, 0.01, 1.0, 0.4)
		 , gcvSlopeLFBackwards  (CONFIG_PARAM_PATH + "gcv/gcvSlopeLF/slopeBackwards", 0.05, 0.01, 1.0, 0.4)
		 , gcvSlopeLFSidewards  (CONFIG_PARAM_PATH + "gcv/gcvSlopeLF/slopeSidewards", 0.05, 0.01, 1.0, 0.4)
		 , gcvSlopeLFRotational (CONFIG_PARAM_PATH + "gcv/gcvSlopeLF/slopeRotational", 0.05, 0.01, 1.0, 0.4)
		 , gcvSlopeLFDecToAcc   (CONFIG_PARAM_PATH + "gcv/gcvSlopeLF/decToAccRatio", 0.4, 0.01, 2.5, 1.0)
		 , gcvSlopeHFForwards   (CONFIG_PARAM_PATH + "gcv/gcvSlopeHF/slopeForwards", 0.5, 0.05, 5.0, 2.0)
		 , gcvSlopeHFSidewards  (CONFIG_PARAM_PATH + "gcv/gcvSlopeHF/slopeSidewards", 0.5, 0.05, 5.0, 2.0)
		 , gcvSlopeHFRotational (CONFIG_PARAM_PATH + "gcv/gcvSlopeHF/slopeRotational", 0.5, 0.05, 5.0, 2.0)
		 , gcvLFAccFilterN      (CONFIG_PARAM_PATH + "gcv/gcvAccLF/wlbfFilterN", 1, 1, 50, 30)

		 , plotData             (CONFIG_PARAM_PATH + "plotData", false)
		{
			// Note: The following parameters have callbacks outside of the FeedConfig class:
			//       plotData: FeedGait::callbackPlotData
		}

		// Constants
		const std::string CONFIG_PARAM_PATH;

		// Dynamic object parameters
		config_server::Parameter<int>   kinematicsType;        //!< @brief Kinematics type to use (see KinematicsType enum)
		config_server::Parameter<int>   trajectoryType;        //!< @brief Trajectory type to use (see TrajectoryType enum)
		config_server::Parameter<int>   odometryType;          //!< @brief Odometry type to use (see OdometryType enum)

		// Command parameters
		config_server::Parameter<bool>  cmdUseActions;         //!< @brief Boolean flag whether computed closed loop actions should be used to control the gait, or whether they should just be zeroed
		config_server::Parameter<bool>  cmdUseStepSize;        //!< @brief Boolean flag whether computed closed loop step sizes should be used to control the gait, or whether the internal gcv should just be controlled manually from the external gcv input (via maximum gcv acceleration rates)
		config_server::Parameter<bool>  cmdUseStepSizeX;       //!< @brief Boolean flag whether computed closed loop step sizes in the linear x-direction should be used to control the gait (#cmdUseStepSize must be enabled as well for this to have effect)
		config_server::Parameter<bool>  cmdUseStepSizeY;       //!< @brief Boolean flag whether computed closed loop step sizes in the linear y-direction should be used to control the gait (#cmdUseStepSize must be enabled as well for this to have effect)
		config_server::Parameter<bool>  cmdUseStepSizeZ;       //!< @brief Boolean flag whether computed closed loop step sizes in the angular z-direction should be used to control the gait (#cmdUseStepSize must be enabled as well for this to have effect)
		config_server::Parameter<bool>  cmdUseTiming;          //!< @brief Boolean flag whether computed closed loop step timings should be used to control the gait, or whether the timing should just remain fixed at the nominal gait frequency

		// Joystick parameters
		config_server::Parameter<int>   joystickMode;          //!< @brief The mode in which to interpret custom joystick button presses

		// General parameters
		config_server::Parameter<float> blendingStartPhaseLen; //!< @brief The amount of time in units of gait phase to take to blend from the halt pose to the moving calculated gait pose during start of walking
		config_server::Parameter<float> blendingStopPhaseLen;  //!< @brief The amount of time in units of gait phase to take to blend from the moving calculated gait pose to the halt pose after stopping walking
		config_server::Parameter<float> effortDefault;         //!< @brief Default effort to use for unmapped joints (default position is zero)
		config_server::Parameter<float> fusedPitchOffset;      //!< @brief Additive offset that is applied to the fused pitch measurements before use
		config_server::Parameter<float> fusedRollOffset;       //!< @brief Additive offset that is applied to the fused roll measurements before use
		config_server::Parameter<float> gaitFrequency;         //!< @brief Nominal frequency of the gait
		config_server::Parameter<float> gaitFrequencyMax;      //!< @brief Maximum allowed frequency of the gait
		config_server::Parameter<bool>  leftLegFirst;          //!< @brief Flag specifying whether the first leg to step with when starting walking should be the left leg
		config_server::Parameter<float> stoppingGcvMag;        //!< @brief Unbiased gait command vector 2-norm below which immediate walk stopping is allowed
		config_server::Parameter<float> stoppingPhaseTol;      //!< @brief Gait phase tolerance below 0 and &pi;, in units of nominal phase increments (see gaitFrequency and the robotcontrol cycle time), within which intelligent walking stopping is allowed
		config_server::Parameter<bool>  useServoModel;         //!< @brief Boolean flag whether to use the servo model

		// Gait command vector parameters
		config_server::Parameter<float> gcvSlopeLFForwards;    //!< @brief Gait command vector low frequency slope limit for ramping of positive linear x command velocities (scale by #gcvSlopeLFDecToAcc to get the deceleration limit)
		config_server::Parameter<float> gcvSlopeLFBackwards;   //!< @brief Gait command vector low frequency slope limit for ramping of negative linear x command velocities (scale by #gcvSlopeLFDecToAcc to get the deceleration limit)
		config_server::Parameter<float> gcvSlopeLFSidewards;   //!< @brief Gait command vector low frequency slope limit for ramping of linear y command velocities (scale by #gcvSlopeLFDecToAcc to get the deceleration limit)
		config_server::Parameter<float> gcvSlopeLFRotational;  //!< @brief Gait command vector low frequency slope limit for ramping of angular z command velocities (scale by #gcvSlopeLFDecToAcc to get the deceleration limit)
		config_server::Parameter<float> gcvSlopeLFDecToAcc;    //!< @brief Multiplicative scale factor to obtain the gait command vector deceleration (towards zero) slope limits from the corresponding acceleration (away from zero) limits
		config_server::Parameter<float> gcvSlopeHFForwards;    //!< @brief Gait command vector high frequency slope limit for preventing step changes in linear x command velocities
		config_server::Parameter<float> gcvSlopeHFSidewards;   //!< @brief Gait command vector high frequency slope limit for preventing step changes in linear y command velocities
		config_server::Parameter<float> gcvSlopeHFRotational;  //!< @brief Gait command vector high frequency slope limit for preventing step changes in angular z command velocities
		config_server::Parameter<int>   gcvLFAccFilterN;       //!< @brief Weighted line of best fit filter size for calculating the low frequency gait command acceleration

		// Miscellaneous parameters
		config_server::Parameter<bool>  plotData;              //!< @brief Boolean flag whether to plot data
	};
}

#endif
// EOF