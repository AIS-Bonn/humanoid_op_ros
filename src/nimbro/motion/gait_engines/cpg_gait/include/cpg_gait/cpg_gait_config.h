// Central pattern generated gait configuration parameters
// File: cpg_gait_config.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CPG_GAIT_CONFIG_H
#define CPG_GAIT_CONFIG_H

// Includes
#include <config_server/parameter.h>
#include <string>

// CPG gait namespace
namespace cpg_gait
{
	/**
	* @struct CPGConfig
	*
	* @brief Configuration struct for the central pattern generated gait.
	*
	* TODO: Explain the gait phase and that it's (-pi,pi] and crucial that it's always wrapped to this range
	* TODO: Explain the swing and support phase that's driving the whole gait
	* TODO: Explain the main components => What is pushout, what is swing, etc...
	* TODO: Recommended order for gait tuning
	* TODO: Swing phase must not be smaller than the support phase! (should not be possible with the config variable ranges supplied)
	* TODO: Explain Mag/Lat/Sag/etc and the GradX/Y/Z system
	**/
	struct CPGConfig
	{
		//! Constructor
		CPGConfig()
		 : CONFIG_PARAM_PATH("/cpg_gait/config/")
		 
		 , nominalStepTime       (CONFIG_PARAM_PATH + "nominalStepTime", 0.2, 0.02, 5.0, 1.0)
		 , leftLegFirst          (CONFIG_PARAM_PATH + "leftLegFirst", true)
		 , swingStartPhase       (CONFIG_PARAM_PATH + "swingStartPhase", 0.0, 0.01, M_PI_2, 0.1*M_PI)
		 , swingStopPhase        (CONFIG_PARAM_PATH + "swingStopPhase", M_PI_2, 0.01, M_PI, 0.9*M_PI)
		 , stoppingGcvMag        (CONFIG_PARAM_PATH + "stoppingGcvMag", 0.001, 0.001, 0.15, 0.001)
		 , stoppingPhaseTolLB    (CONFIG_PARAM_PATH + "stoppingPhaseTolLB", 2.0, 0.1, 20.0, 5.0)
		 , stoppingPhaseTolUB    (CONFIG_PARAM_PATH + "stoppingPhaseTolUB", 2.0, 0.1, 20.0, 5.0)
		 
		 , gcvBiasLinVelX        (CONFIG_PARAM_PATH + "gcvBias/linVelX", -1.0, 0.01, 1.0, 0.0)
		 , gcvBiasLinVelY        (CONFIG_PARAM_PATH + "gcvBias/linVelY", -1.0, 0.01, 1.0, 0.0)
		 , gcvBiasAngVelZ        (CONFIG_PARAM_PATH + "gcvBias/angVelZ", -1.0, 0.01, 1.0, 0.0)
		 , gcvAccForwards        (CONFIG_PARAM_PATH + "gcvAccLimits/accForwards", 0.05, 0.01, 1.0, 0.2)
		 , gcvAccBackwards       (CONFIG_PARAM_PATH + "gcvAccLimits/accBackwards", 0.05, 0.01, 1.0, 0.2)
		 , gcvAccSidewards       (CONFIG_PARAM_PATH + "gcvAccLimits/accSidewards", 0.05, 0.01, 1.0, 0.2)
		 , gcvAccRotational      (CONFIG_PARAM_PATH + "gcvAccLimits/accRotational", 0.05, 0.01, 1.0, 0.2)
		 , gcvDecToAccRatio      (CONFIG_PARAM_PATH + "gcvAccLimits/decToAccRatio", 0.4, 0.01, 2.5, 1.0)
		 
		 , haltArmExtension      (CONFIG_PARAM_PATH + "haltPose/armExtension", 0.0, 0.005, 0.8, 0.0)
		 , haltArmAngleX         (CONFIG_PARAM_PATH + "haltPose/armAngleX", -0.1, 0.01, 0.6, 0.0)
		 , haltArmAngleY         (CONFIG_PARAM_PATH + "haltPose/armAngleY", -0.6, 0.01, 0.6, 0.0)
		 , haltLegExtension      (CONFIG_PARAM_PATH + "haltPose/legExtension", 0.0, 0.005, 0.8, 0.0)
		 , haltLegAngleX         (CONFIG_PARAM_PATH + "haltPose/legAngleX", -0.1, 0.01, 0.6, 0.0)
		 , haltLegAngleY         (CONFIG_PARAM_PATH + "haltPose/legAngleY", -0.6, 0.005, 0.6, 0.0)
		 , haltLegAngleZ         (CONFIG_PARAM_PATH + "haltPose/legAngleZ", -0.6, 0.005, 0.6, 0.0)
		 , haltFootAngleX        (CONFIG_PARAM_PATH + "haltPose/footAngleX", -0.4, 0.005, 0.4, 0.0)
		 , haltFootAngleY        (CONFIG_PARAM_PATH + "haltPose/footAngleY", -0.4, 0.005, 0.4, 0.0)
		 , haltEffortArm         (CONFIG_PARAM_PATH + "haltPose/effortArm", 0.0, 0.01, 1.0, 0.0)
		 , haltEffortHipYaw      (CONFIG_PARAM_PATH + "haltPose/effortHipYaw", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortHipRoll     (CONFIG_PARAM_PATH + "haltPose/effortHipRoll", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortHipPitch    (CONFIG_PARAM_PATH + "haltPose/effortHipPitch", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortKneePitch   (CONFIG_PARAM_PATH + "haltPose/effortKneePitch", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortAnklePitch  (CONFIG_PARAM_PATH + "haltPose/effortAnklePitch", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortAnkleRoll   (CONFIG_PARAM_PATH + "haltPose/effortAnkleRoll", 0.0, 0.01, 1.5, 0.0)

		 , armSagSwingMag        (CONFIG_PARAM_PATH + "arm/sagSwingMag", 0.0, 0.01, 1.0, 0.0)
		 , armSagSwingMagGradX   (CONFIG_PARAM_PATH + "arm/sagSwingMagGradX", 0.0, 0.01, 1.0, 0.25)
		 
		 , legStepHeight         (CONFIG_PARAM_PATH + "leg/stepHeight", 0.0, 0.005, 0.4, 0.1)
		 , legStepHeightGradX    (CONFIG_PARAM_PATH + "leg/stepHeightGradX", 0.0, 0.005, 0.4, 0.0)
		 , legStepHeightGradY    (CONFIG_PARAM_PATH + "leg/stepHeightGradY", 0.0, 0.005, 0.4, 0.0)
		 , legPushHeight         (CONFIG_PARAM_PATH + "leg/pushHeight", 0.0, 0.005, 0.4, 0.05)
		 , legPushHeightGradX    (CONFIG_PARAM_PATH + "leg/pushHeightGradX", 0.0, 0.005, 0.4, 0.0)
		 , legSagSwingMagGradXBwd(CONFIG_PARAM_PATH + "leg/sagSwingMagGradXBwd", 0.0, 0.005, 0.4, 0.10)
		 , legSagSwingMagGradXFwd(CONFIG_PARAM_PATH + "leg/sagSwingMagGradXFwd", 0.0, 0.005, 0.4, 0.10)
		 , legSagLeanGradXBwd    (CONFIG_PARAM_PATH + "leg/sagLeanGradXBwd", -0.2, 0.005, 0.3, 0.0)
		 , legSagLeanGradXFwd    (CONFIG_PARAM_PATH + "leg/sagLeanGradXFwd", -0.2, 0.005, 0.3, 0.0)
		 , legSagLeanGradZ       (CONFIG_PARAM_PATH + "leg/sagLeanGradZ", -0.3, 0.005, 0.3, 0.0)
		 , legLatSwingMagGradY   (CONFIG_PARAM_PATH + "leg/latSwingMagGradY", 0.0, 0.005, 0.4, 0.10)
		 , legLatHipSwingBias    (CONFIG_PARAM_PATH + "leg/latHipSwingBias", -0.3, 0.005, 0.3, 0.0)
		 , legLatHipSwingMag     (CONFIG_PARAM_PATH + "leg/latHipSwingMag", 0.0, 0.005, 0.3, 0.05)
		 , legLatHipSwingMagGradX(CONFIG_PARAM_PATH + "leg/latHipSwingMagGradX", -0.15, 0.005, 0.3, 0.0)
		 , legLatHipSwingMagGradY(CONFIG_PARAM_PATH + "leg/latHipSwingMagGradY", 0.0, 0.005, 0.3, 0.0)
		 , legLatPushoutMagGradX (CONFIG_PARAM_PATH + "leg/latPushoutMagGradX", 0.0, 0.005, 0.3, 0.0)
		 , legLatPushoutMagGradY (CONFIG_PARAM_PATH + "leg/latPushoutMagGradY", 0.0, 0.005, 0.3, 0.0)
		 , legLatPushoutMagGradZ (CONFIG_PARAM_PATH + "leg/latPushoutMagGradZ", 0.0, 0.005, 0.3, 0.0)
		 , legLatLeanGradXZ      (CONFIG_PARAM_PATH + "leg/latLeanGradXZ", 0.0, 0.01, 1.0, 0.0)
		 , legRotSwingMagGradZ   (CONFIG_PARAM_PATH + "leg/rotSwingMagGradZ", 0.0, 0.005, 0.4, 0.10)
		 , legRotVPushoutMagGradZ(CONFIG_PARAM_PATH + "leg/rotVPushoutMagGradZ", 0.0, 0.005, 0.4, 0.0)

		 , tuningNoArms          (CONFIG_PARAM_PATH + "tuning/noArms", false)
		 , tuningNoArmSwing      (CONFIG_PARAM_PATH + "tuning/noArmSwing", false)
		 , tuningNoLegs          (CONFIG_PARAM_PATH + "tuning/noLegs", false)
		 , tuningNoLegLifting    (CONFIG_PARAM_PATH + "tuning/noLegLifting", false)
		 , tuningNoLegSwing      (CONFIG_PARAM_PATH + "tuning/noLegSwing", false)
		 , tuningNoLegHipSwing   (CONFIG_PARAM_PATH + "tuning/noLegHipSwing", false)
		 , tuningNoLegPushout    (CONFIG_PARAM_PATH + "tuning/noLegPushout", false)
		 , tuningNoLegLeaning    (CONFIG_PARAM_PATH + "tuning/noLegLeaning", false)

		 , cmdVirtualSlope       (CONFIG_PARAM_PATH + "cmd/VirtualSlope", false)
		 , mgVirtualSlopeOffset  (CONFIG_PARAM_PATH + "mg/VirtualSlopeOffset", -0.5, 0.01, 0.5, 0.0)
		 , mgVirtualSlopeGainAsc (CONFIG_PARAM_PATH + "mg/VirtualSlopeGainAsc", 0.0, 0.02, 2.0, 0.0)
		 , mgVirtualSlopeGainDsc (CONFIG_PARAM_PATH + "mg/VirtualSlopeGainDsc", 0.0, 0.02, 2.0, 0.0)
		 , mgVirtualSlopeMidAngle(CONFIG_PARAM_PATH + "mg/VirtualSlopeMidAngle", -0.5, 0.01, 0.5, 0.0)
		 , mgVirtualSlopeMinAngle(CONFIG_PARAM_PATH + "mg/VirtualSlopeMinAngle", 0.0, 0.01, 1.0, 0.1)
		{}

		//! @name Class constants
		///@{
		const std::string CONFIG_PARAM_PATH;                    //!< @brief Path for the CPG gait configuration parameters on the config server
		///@}

		//! @name Gait parameters
		///@{
		config_server::Parameter<float> nominalStepTime;        //!< @brief The nominal time of each step in the gait motion (i.e. half the true period of the gait, as each cycle consists of two steps)
		config_server::Parameter<bool>  leftLegFirst;           //!< @brief Flag specifying whether the first leg to step with when starting walking should be the left leg
		config_server::Parameter<float> swingStartPhase;        //!< @brief Limb phase at which to start the forwards swing of a limb (marks the start of the swing phase of that limb, and the end of the support phase)
		config_server::Parameter<float> swingStopPhase;         //!< @brief Limb phase at which to end the forwards swing of a limb (marks the end of the swing phase of that limb, and the start of the support phase)
		config_server::Parameter<float> stoppingGcvMag;         //!< @brief Unbiased gait command velocity 2-norm below which immediate walk stopping is allowed
		config_server::Parameter<float> stoppingPhaseTolLB;     //!< @brief Gait phase tolerance below 0 and &pi;, in units of nominal phase increments (see #nominalStepTime and the robotcontrol cycle time), within which intelligent walking stopping is allowed
		config_server::Parameter<float> stoppingPhaseTolUB;     //!< @brief Gait phase tolerance above 0 and -&pi;, in units of nominal phase increments (see #nominalStepTime and the robotcontrol cycle time), within which intelligent walking stopping is allowed
		///@}

		//! @name Gait command parameters
		///@{
		config_server::Parameter<float> gcvBiasLinVelX;         //!< @brief Bias added to the linear x-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
		config_server::Parameter<float> gcvBiasLinVelY;         //!< @brief Bias added to the linear y-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
		config_server::Parameter<float> gcvBiasAngVelZ;         //!< @brief Bias added to the angular z-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
		config_server::Parameter<float> gcvAccForwards;         //!< @brief Gait command acceleration limit for positive linear x command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
		config_server::Parameter<float> gcvAccBackwards;        //!< @brief Gait command acceleration limit for negative linear x command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
		config_server::Parameter<float> gcvAccSidewards;        //!< @brief Gait command acceleration limit for linear y command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
		config_server::Parameter<float> gcvAccRotational;       //!< @brief Gait command acceleration limit for angular z command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
		config_server::Parameter<float> gcvDecToAccRatio;       //!< @brief Multiplicative scale factor to obtain the gait command deceleration limits from the gait command acceleration limits
		///@}

		//! @name Halt pose parameters
		///@{
		config_server::Parameter<float> haltArmExtension;       //!< @brief Halt pose: Extension of the arms (0 = Fully extended, 1 = Fully contracted, see @ref gait::AbstractArmPose "AbstractArmPose")
		config_server::Parameter<float> haltArmAngleX;          //!< @brief Halt pose: Roll angle of the arms (positive is away from the body for both arms)
		config_server::Parameter<float> haltArmAngleY;          //!< @brief Halt pose: Pitch angle of the central axis of the arms (positive is moving the arms towards the back for both arms)
		config_server::Parameter<float> haltLegExtension;       //!< @brief Halt pose: Extension of the legs (0 = Fully extended, 1 = Fully contracted, see @ref gait::AbstractLegPose "AbstractLegPose")
		config_server::Parameter<float> haltLegAngleX;          //!< @brief Halt pose: Roll angle of the legs (positive is away from the body for both legs)
		config_server::Parameter<float> haltLegAngleY;          //!< @brief Halt pose: Pitch angle of the central axis of the legs (positive is moving the legs towards the back for both legs)
		config_server::Parameter<float> haltLegAngleZ;          //!< @brief Halt pose: Yaw angle of the legs (toe-out is positive for both legs)
		config_server::Parameter<float> haltFootAngleX;         //!< @brief Halt pose: Roll angle of the feet relative to the trunk (positive is tilting onto the inner feet for both feet)
		config_server::Parameter<float> haltFootAngleY;         //!< @brief Halt pose: Pitch angle of the feet relative to the trunk (positive is what would make the robot lean back for both feet)
		config_server::Parameter<float> haltEffortArm;          //!< @brief Halt pose: Joint effort to use for the arms (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipYaw;       //!< @brief Halt pose: Joint effort to use for the leg hip yaw (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipRoll;      //!< @brief Halt pose: Joint effort to use for the leg hip roll (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipPitch;     //!< @brief Halt pose: Joint effort to use for the leg hip pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortKneePitch;    //!< @brief Halt pose: Joint effort to use for the leg knee pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortAnklePitch;   //!< @brief Halt pose: Joint effort to use for the leg ankle pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortAnkleRoll;    //!< @brief Halt pose: Joint effort to use for the leg ankle roll (in the range `[0,1]`)
		///@}

		//! @name Arm parameters
		///@{
		config_server::Parameter<float> armSagSwingMag;         //!< @brief Magnitude in radians of the arm swing to use at zero biased gait command velocity (for a total peak-to-peak swing of double this value)
		config_server::Parameter<float> armSagSwingMagGradX;    //!< @brief Gradient of #armSagSwingMag with respect to the biased gait command x-velocity
		///@}

		//! @name Leg parameters
		///@{
		config_server::Parameter<float> legStepHeight;          //!< @brief Nominal swing leg step height (out of the ground, in units of leg extension) to use at zero biased gait command velocity
		config_server::Parameter<float> legStepHeightGradX;     //!< @brief Gradient of #legStepHeight with respect to the absolute biased gait command x-velocity
		config_server::Parameter<float> legStepHeightGradY;     //!< @brief Gradient of #legStepHeight with respect to the absolute biased gait command y-velocity
		config_server::Parameter<float> legPushHeight;          //!< @brief Nominal support leg push height (into the ground, in units of leg extension) to use at zero biased gait command velocity
		config_server::Parameter<float> legPushHeightGradX;     //!< @brief Gradient of #legPushHeight with respect to the absolute biased gait command x-velocity
		config_server::Parameter<float> legSagSwingMagGradXBwd; //!< @brief Gradient of the sagittal leg swing magnitude (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is negative (backwards walking)
		config_server::Parameter<float> legSagSwingMagGradXFwd; //!< @brief Gradient of the sagittal leg swing magnitude (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is positive (forwards walking)
		config_server::Parameter<float> legSagLeanGradXBwd;     //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is negative (backwards walking)
		config_server::Parameter<float> legSagLeanGradXFwd;     //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is positive (forwards walking)
		config_server::Parameter<float> legSagLeanGradZ;        //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the biased gait command z-velocity
		config_server::Parameter<float> legLatSwingMagGradY;    //!< @brief Gradient of the lateral leg swing magnitude (nominally zero radians) with respect to the biased gait command y-velocity
		config_server::Parameter<float> legLatHipSwingBias;     //!< @brief Constant bias to the lateral hip swing waveform (hip roll angle, units of radians), used to try to correct for robot walking asymmetries
		config_server::Parameter<float> legLatHipSwingMag;      //!< @brief Nominal lateral hip swing magnitude (hip roll angle, units of radians) to use at zero biased gait command velocity
		config_server::Parameter<float> legLatHipSwingMagGradX; //!< @brief Gradient of the lateral hip swing magnitude with respect to the absolute biased gait command x-velocity
		config_server::Parameter<float> legLatHipSwingMagGradY; //!< @brief Gradient of the lateral hip swing magnitude with respect to the absolute biased gait command y-velocity
		config_server::Parameter<float> legLatPushoutMagGradX;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command x-velocity
		config_server::Parameter<float> legLatPushoutMagGradY;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command y-velocity
		config_server::Parameter<float> legLatPushoutMagGradZ;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command z-velocity
		config_server::Parameter<float> legLatLeanGradXZ;       //!< @brief Gradient of the lateral lean (nominally zero radians) with respect to the biased gait command x (absolute) and z (signed) velocities
		config_server::Parameter<float> legRotSwingMagGradZ;    //!< @brief Gradient of the rotational leg swing magnitude (nominally zero radians) with respect to the biased gait command z-velocity
		config_server::Parameter<float> legRotVPushoutMagGradZ; //!< @brief Gradient of the rotational leg V pushout magnitude (nominally zero radians) with respect to the absolute biased gait command z-velocity
		///@}

		//! @name Tuning parameters
		///@{
		config_server::Parameter<bool> tuningNoArms;            //!< @brief Disable all gait arm motions
		config_server::Parameter<bool> tuningNoArmSwing;        //!< @brief Disable all arm swing components
		config_server::Parameter<bool> tuningNoLegs;            //!< @brief Disable all gait leg motions
		config_server::Parameter<bool> tuningNoLegLifting;      //!< @brief Disable all leg lifting
		config_server::Parameter<bool> tuningNoLegSwing;        //!< @brief Disable all leg swing components
		config_server::Parameter<bool> tuningNoLegHipSwing;     //!< @brief Disable all leg hip swing components
		config_server::Parameter<bool> tuningNoLegPushout;      //!< @brief Disable all leg pushout components
		config_server::Parameter<bool> tuningNoLegLeaning;      //!< @brief Disable all leg leaning components
		///@}

		//! @name Capture step parameters
		///@{
		config_server::Parameter<bool>  cmdVirtualSlope;
		config_server::Parameter<float> mgVirtualSlopeOffset;
		config_server::Parameter<float> mgVirtualSlopeGainAsc;
		config_server::Parameter<float> mgVirtualSlopeGainDsc;
		config_server::Parameter<float> mgVirtualSlopeMidAngle;
		config_server::Parameter<float> mgVirtualSlopeMinAngle;
		///@}
	};
}

#endif /* CPG_GAIT_CONFIG_H */
// EOF