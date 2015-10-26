// Capture step gait configuration parameters
// File: cap_gait_config.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CAP_GAIT_CONFIG_H
#define CAP_GAIT_CONFIG_H

// Includes
#include <config_server/parameter.h>
#include <string>

// Capture step gait namespace
namespace cap_gait
{
	/**
	* @struct CapConfig
	*
	* @brief Configuration struct for the capture step gait.
	**/
	struct CapConfig
	{
	public:
		//! Constructor
		CapConfig()
		 : CONFIG_PARAM_PATH("/cap_gait/config/")
		 
		 , armLinkLength          (CONFIG_PARAM_PATH + "robotSpec/armLinkLength", 0.01, 0.005, 0.6, 0.2)
		 , legLinkLength          (CONFIG_PARAM_PATH + "robotSpec/legLinkLength", 0.01, 0.005, 0.6, 0.2)
		 , shoulderWidth          (CONFIG_PARAM_PATH + "robotSpec/shoulderWidth", 0.01, 0.005, 0.6, 0.2)
		 , hipWidth               (CONFIG_PARAM_PATH + "robotSpec/hipWidth", 0.01, 0.005, 0.6, 0.2)
		 , trunkHeight            (CONFIG_PARAM_PATH + "robotSpec/trunkHeight", 0.01, 0.005, 1.0, 0.4)
		 , trunkLinkOffsetX       (CONFIG_PARAM_PATH + "robotSpec/trunkLinkOffsetX", -0.3, 0.005, 0.3, 0.0)
		 , trunkLinkOffsetY       (CONFIG_PARAM_PATH + "robotSpec/trunkLinkOffsetY", -0.3, 0.005, 0.3, 0.0)
		 , trunkLinkOffsetZ       (CONFIG_PARAM_PATH + "robotSpec/trunkLinkOffsetZ", -0.3, 0.005, 0.7, 0.2)
		 , comOffsetX             (CONFIG_PARAM_PATH + "robotSpec/comOffsetX", -0.25, 0.005, 0.25, 0.0)
		 , comOffsetZ             (CONFIG_PARAM_PATH + "robotSpec/comOffsetZ", 0.0, 0.005, 1.0, 0.2)
		 , footWidth              (CONFIG_PARAM_PATH + "robotSpec/footWidth", 0.01, 0.005, 0.3, 0.1)
		 , footLength             (CONFIG_PARAM_PATH + "robotSpec/footLength", 0.01, 0.005, 0.5, 0.2)
		 , footOffsetX            (CONFIG_PARAM_PATH + "robotSpec/footOffsetX", -0.25, 0.005, 0.25, 0.0)
		 , footOffsetY            (CONFIG_PARAM_PATH + "robotSpec/footOffsetY", -0.15, 0.005, 0.15, 0.0)
		 , footOffsetZ            (CONFIG_PARAM_PATH + "robotSpec/footOffsetZ", 0.0, 0.005, 0.2, 0.0)
		 , neckHeight             (CONFIG_PARAM_PATH + "robotSpec/neckHeight", 0.0, 0.005, 0.3, 0.0)
		 , headOffsetX            (CONFIG_PARAM_PATH + "robotSpec/headOffsetX", -0.2, 0.005, 0.2, 0.0)
		 , headOffsetZ            (CONFIG_PARAM_PATH + "robotSpec/headOffsetZ", 0.0, 0.005, 0.3, 0.1)
		 
		 , armThickness           (CONFIG_PARAM_PATH + "robotVis/armThickness", 0.005, 0.005, 0.2, 0.03)
		 , legThickness           (CONFIG_PARAM_PATH + "robotVis/legThickness", 0.005, 0.005, 0.3, 0.05)
		 , headDiameter           (CONFIG_PARAM_PATH + "robotVis/headDiameter", 0.01, 0.005, 0.4, 0.15)
		 , jointDiameter          (CONFIG_PARAM_PATH + "robotVis/jointDiameter", 0.005, 0.002, 0.1, 0.025)
		 , visOffsetX             (CONFIG_PARAM_PATH + "robotVis/visOffsetX", -2.0, 0.01, 2.0, 0.0)
		 , visOffsetY             (CONFIG_PARAM_PATH + "robotVis/visOffsetY", -2.0, 0.01, 2.0, -0.5)
		 , visOffsetZ             (CONFIG_PARAM_PATH + "robotVis/visOffsetZ", -2.0, 0.01, 2.0, 0.0)
		 , markVectors            (CONFIG_PARAM_PATH + "robotVis/markVectors", false)
		 
		 , footHeightHysteresis   (CONFIG_PARAM_PATH + "robotModel/footHeightHysteresis", 0.0, 0.001, 0.1, 0.005)
		 
		 , enableMotionStances    (CONFIG_PARAM_PATH + "general/enableMotionStances", false)
		 , gaitFrequency          (CONFIG_PARAM_PATH + "general/gaitFrequency", 0.3, 0.02, 5.0, 3.0)
		 , gaitFrequencyMax       (CONFIG_PARAM_PATH + "general/gaitFrequencyMax", 0.3, 0.02, 5.0, 3.0)
		 , leftLegFirst           (CONFIG_PARAM_PATH + "general/leftLegFirst", true)
		 , stanceAdjustGcvMax     (CONFIG_PARAM_PATH + "general/stanceAdjustGcvMax", 0.0, 0.01, 1.0, 0.4)
		 , stanceAdjustRate       (CONFIG_PARAM_PATH + "general/stanceAdjustRate", 0.1, 0.05, 5.0, 1.0)
		 , stoppingGcvMag         (CONFIG_PARAM_PATH + "general/stoppingGcvMag", 0.001, 0.001, 0.15, 0.001)
		 , stoppingPhaseTolLB     (CONFIG_PARAM_PATH + "general/stoppingPhaseTolLB", 2.0, 0.1, 20.0, 5.0)
		 , stoppingPhaseTolUB     (CONFIG_PARAM_PATH + "general/stoppingPhaseTolUB", 2.0, 0.1, 20.0, 5.0)
		 , supportCoeffRange      (CONFIG_PARAM_PATH + "general/supportCoeffRange", 0.0, 0.01, 1.0, 1.0)
		 , useServoModel          (CONFIG_PARAM_PATH + "general/useServoModel", true)
		 
		 , gcvBiasLinVelX         (CONFIG_PARAM_PATH + "gcv/gcvBias/linVelX", -1.0, 0.01, 1.0, 0.0)
		 , gcvBiasLinVelY         (CONFIG_PARAM_PATH + "gcv/gcvBias/linVelY", -1.0, 0.01, 1.0, 0.0)
		 , gcvBiasAngVelZ         (CONFIG_PARAM_PATH + "gcv/gcvBias/angVelZ", -1.0, 0.01, 1.0, 0.0)
		 , gcvAccForwards         (CONFIG_PARAM_PATH + "gcv/gcvAccLimits/accForwards", 0.05, 0.01, 1.0, 0.2)
		 , gcvAccBackwards        (CONFIG_PARAM_PATH + "gcv/gcvAccLimits/accBackwards", 0.05, 0.01, 1.0, 0.2)
		 , gcvAccSidewards        (CONFIG_PARAM_PATH + "gcv/gcvAccLimits/accSidewards", 0.05, 0.01, 1.0, 0.2)
		 , gcvAccRotational       (CONFIG_PARAM_PATH + "gcv/gcvAccLimits/accRotational", 0.05, 0.01, 1.0, 0.2)
		 , gcvDecToAccRatio       (CONFIG_PARAM_PATH + "gcv/gcvAccLimits/decToAccRatio", 0.4, 0.01, 2.5, 1.0)
		 , gcvAccJerkLimitX       (CONFIG_PARAM_PATH + "gcv/gcvAccJerk/accJerkLimitX", 0.005, 0.005, 0.2, 0.05)
		 , gcvAccJerkLimitY       (CONFIG_PARAM_PATH + "gcv/gcvAccJerk/accJerkLimitY", 0.005, 0.005, 0.2, 0.05)
		 , gcvAccJerkLimitZ       (CONFIG_PARAM_PATH + "gcv/gcvAccJerk/accJerkLimitZ", 0.005, 0.005, 0.2, 0.05)
		 , gcvPrescalerLinVelX    (CONFIG_PARAM_PATH + "gcv/gcvPrescaler/linVelX", 0.0, 0.05, 4.0, 1.0)
		 , gcvPrescalerLinVelY    (CONFIG_PARAM_PATH + "gcv/gcvPrescaler/linVelY", 0.0, 0.05, 4.0, 1.0)
		 , gcvPrescalerAngVelZ    (CONFIG_PARAM_PATH + "gcv/gcvPrescaler/angVelZ", 0.0, 0.05, 4.0, 1.0)

		 , limArmAngleXBuf        (CONFIG_PARAM_PATH + "poseLimits/armAngleXBuf", 0.0, 0.01, 0.5, 0.1)
		 , limArmAngleXMax        (CONFIG_PARAM_PATH + "poseLimits/armAngleXMax", 0.2, 0.02, 2.0, 0.8)
		 , limArmAngleXMin        (CONFIG_PARAM_PATH + "poseLimits/armAngleXMin", -2.0, 0.02, 0.2, 0.0)
		 , limArmAngleXUseLimits  (CONFIG_PARAM_PATH + "poseLimits/armAngleXUseLimits", true)
		 , limArmAngleYBuf        (CONFIG_PARAM_PATH + "poseLimits/armAngleYBuf", 0.0, 0.01, 0.5, 0.1)
		 , limArmAngleYMax        (CONFIG_PARAM_PATH + "poseLimits/armAngleYMax", 0.0, 0.02, 2.0, 0.8)
		 , limArmAngleYMin        (CONFIG_PARAM_PATH + "poseLimits/armAngleYMin", -2.0, 0.02, 0.0, -0.8)
		 , limArmAngleYUseLimits  (CONFIG_PARAM_PATH + "poseLimits/armAngleYUseLimits", true)
		 , limFootAngleXBuf       (CONFIG_PARAM_PATH + "poseLimits/footAngleXBuf", 0.0, 0.01, 0.5, 0.1)
		 , limFootAngleXMax       (CONFIG_PARAM_PATH + "poseLimits/footAngleXMax", 0.2, 0.01, 1.0, 0.4)
		 , limFootAngleXMin       (CONFIG_PARAM_PATH + "poseLimits/footAngleXMin", -1.0, 0.01, 0.2, -0.4)
		 , limFootAngleXUseLimits (CONFIG_PARAM_PATH + "poseLimits/footAngleXUseLimits", true)
		 , limFootAngleYBuf       (CONFIG_PARAM_PATH + "poseLimits/footAngleYBuf", 0.0, 0.01, 0.5, 0.1)
		 , limFootAngleYMax       (CONFIG_PARAM_PATH + "poseLimits/footAngleYMax", 0.0, 0.01, 1.0, 0.4)
		 , limFootAngleYMin       (CONFIG_PARAM_PATH + "poseLimits/footAngleYMin", -1.0, 0.01, 0.0, -0.4)
		 , limFootAngleYUseLimits (CONFIG_PARAM_PATH + "poseLimits/footAngleYUseLimits", true)
		 , limLegAngleXBuf        (CONFIG_PARAM_PATH + "poseLimits/legAngleXBuf", 0.0, 0.01, 0.5, 0.1)
		 , limLegAngleXMax        (CONFIG_PARAM_PATH + "poseLimits/legAngleXMax", 0.2, 0.01, 1.0, 0.5)
		 , limLegAngleXMin        (CONFIG_PARAM_PATH + "poseLimits/legAngleXMin", -1.0, 0.01, 0.2, -0.2)
		 , limLegAngleXUseLimits  (CONFIG_PARAM_PATH + "poseLimits/legAngleXUseLimits", true)
		 , limLegAngleYBuf        (CONFIG_PARAM_PATH + "poseLimits/legAngleYBuf", 0.0, 0.01, 0.5, 0.1)
		 , limLegAngleYMax        (CONFIG_PARAM_PATH + "poseLimits/legAngleYMax", 0.0, 0.01, 1.0, 0.6)
		 , limLegAngleYMin        (CONFIG_PARAM_PATH + "poseLimits/legAngleYMin", -1.0, 0.01, 0.0, -0.5)
		 , limLegAngleYUseLimits  (CONFIG_PARAM_PATH + "poseLimits/legAngleYUseLimits", true)
		 
		 , startBlendPhaseLen     (CONFIG_PARAM_PATH + "OL/phase/startBlendPhaseLen", 0.0, 0.05, 4.0*M_PI, 2.0*M_PI)
		 , stopBlendPhaseLen      (CONFIG_PARAM_PATH + "OL/phase/stopBlendPhaseLen", 0.0, 0.05, 2.0*M_PI, M_PI)
		 , doubleSupportPhaseLen  (CONFIG_PARAM_PATH + "OL/phase/doubleSupportPhaseLen", 0.0, 0.01, 1.5, 0.1*M_PI)
		 , swingStartPhaseOffset  (CONFIG_PARAM_PATH + "OL/phase/swingStartPhaseOffset", 0.0, 0.01, 1.0, 0.0)
		 , swingStopPhaseOffset   (CONFIG_PARAM_PATH + "OL/phase/swingStopPhaseOffset", 0.0, 0.01, 1.0, 0.1*M_PI)
		 , swingMinPhaseLen       (CONFIG_PARAM_PATH + "OL/phase/swingMinPhaseLen", 0.1, 0.01, 1.0, 0.3)
		 , suppTransStartRatio    (CONFIG_PARAM_PATH + "OL/phase/suppTransStartRatio", 0.0, 0.01, 1.0, 0.5)
		 , suppTransStopRatio     (CONFIG_PARAM_PATH + "OL/phase/suppTransStopRatio", 0.0, 0.01, 1.0, 0.5)
		 , filletStepPhaseLen     (CONFIG_PARAM_PATH + "OL/phase/filletStepPhaseLen", 0.0, 0.01, 1.5, 0.0)
		 , filletPushPhaseLen     (CONFIG_PARAM_PATH + "OL/phase/filletPushPhaseLen", 0.0, 0.01, 1.5, 0.0)

		 , haltArmExtension       (CONFIG_PARAM_PATH + "OL/haltPose/armExtension", 0.0, 0.005, 0.8, 0.0)
		 , haltArmAngleX          (CONFIG_PARAM_PATH + "OL/haltPose/armAngleX", -0.1, 0.01, 0.6, 0.0)
		 , haltArmAngleXBias      (CONFIG_PARAM_PATH + "OL/haltPose/armAngleXBias", -0.2, 0.005, 0.2, 0.0)
		 , haltArmAngleY          (CONFIG_PARAM_PATH + "OL/haltPose/armAngleY", -1.3, 0.01, 1.3, 0.0)
		 , haltLegExtension       (CONFIG_PARAM_PATH + "OL/haltPose/legExtension", 0.0, 0.005, 0.8, 0.0)
		 , haltLegExtensionBias   (CONFIG_PARAM_PATH + "OL/haltPose/legExtensionBias", -0.3, 0.005, 0.3, 0.0)
		 , haltLegAngleX          (CONFIG_PARAM_PATH + "OL/haltPose/legAngleX", -0.1, 0.01, 0.6, 0.0)
		 , haltLegAngleXBias      (CONFIG_PARAM_PATH + "OL/haltPose/legAngleXBias", -0.2, 0.005, 0.2, 0.0)
		 , haltLegAngleXNarrow    (CONFIG_PARAM_PATH + "OL/haltPose/legAngleXNarrow", -0.1, 0.01, 0.6, 0.0)
		 , haltLegAngleY          (CONFIG_PARAM_PATH + "OL/haltPose/legAngleY", -1.0, 0.005, 1.0, 0.0)
		 , haltLegAngleZ          (CONFIG_PARAM_PATH + "OL/haltPose/legAngleZ", -0.6, 0.005, 0.6, 0.0)
		 , haltFootAngleX         (CONFIG_PARAM_PATH + "OL/haltPose/footAngleX", -0.4, 0.005, 0.4, 0.0)
		 , haltFootAngleXBias     (CONFIG_PARAM_PATH + "OL/haltPose/footAngleXBias", -0.2, 0.005, 0.2, 0.0)
		 , haltFootAngleY         (CONFIG_PARAM_PATH + "OL/haltPose/footAngleY", -0.4, 0.005, 0.4, 0.0)
		 , haltEffortArm          (CONFIG_PARAM_PATH + "OL/haltPose/effortArm", 0.0, 0.01, 1.0, 0.0)
		 , haltEffortHipYaw       (CONFIG_PARAM_PATH + "OL/haltPose/effortHipYaw", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortHipRoll      (CONFIG_PARAM_PATH + "OL/haltPose/effortHipRoll", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortHipPitch     (CONFIG_PARAM_PATH + "OL/haltPose/effortHipPitch", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortKneePitch    (CONFIG_PARAM_PATH + "OL/haltPose/effortKneePitch", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortAnklePitch   (CONFIG_PARAM_PATH + "OL/haltPose/effortAnklePitch", 0.0, 0.01, 1.5, 0.0)
		 , haltEffortAnkleRoll    (CONFIG_PARAM_PATH + "OL/haltPose/effortAnkleRoll", 0.0, 0.01, 1.5, 0.0)
		 
		 , armSagSwingMag         (CONFIG_PARAM_PATH + "OL/armMotion/sagSwingMag", 0.0, 0.01, 1.0, 0.0)
		 , armSagSwingMagGradX    (CONFIG_PARAM_PATH + "OL/armMotion/sagSwingMagGradX", 0.0, 0.01, 1.0, 0.25)
		 
		 , legExtToAngleYGain     (CONFIG_PARAM_PATH + "OL/legMotion/extToAngleYGain", -1.0, 0.05, 3.0, 0.0)
		 , legHipAngleXLegExtGain (CONFIG_PARAM_PATH + "OL/legMotion/hipAngleXLegExtGain", 0.0, 0.005, 0.5, 0.2)
		 , legStepHeight          (CONFIG_PARAM_PATH + "OL/legMotion/stepHeight", 0.0, 0.005, 0.4, 0.1)
		 , legStepHeightGradX     (CONFIG_PARAM_PATH + "OL/legMotion/stepHeightGradX", 0.0, 0.005, 0.4, 0.0)
		 , legStepHeightGradY     (CONFIG_PARAM_PATH + "OL/legMotion/stepHeightGradY", 0.0, 0.005, 0.4, 0.0)
		 , legPushHeight          (CONFIG_PARAM_PATH + "OL/legMotion/pushHeight", 0.0, 0.005, 0.4, 0.05)
		 , legPushHeightGradX     (CONFIG_PARAM_PATH + "OL/legMotion/pushHeightGradX", 0.0, 0.005, 0.4, 0.0)
		 , legSagSwingMagGradXBwd (CONFIG_PARAM_PATH + "OL/legMotion/sagSwingMagGradXBwd", 0.0, 0.005, 0.4, 0.10)
		 , legSagSwingMagGradXFwd (CONFIG_PARAM_PATH + "OL/legMotion/sagSwingMagGradXFwd", 0.0, 0.005, 0.4, 0.10)
		 , legSagLeanGradAccXBwd  (CONFIG_PARAM_PATH + "OL/legMotion/sagLeanGradAccXBwd", -0.5, 0.02, 3.0, 0.0)
		 , legSagLeanGradAccXFwd  (CONFIG_PARAM_PATH + "OL/legMotion/sagLeanGradAccXFwd", -0.5, 0.02, 3.0, 0.0)
		 , legSagLeanGradVelXBwd  (CONFIG_PARAM_PATH + "OL/legMotion/sagLeanGradVelXBwd", -0.05, 0.005, 0.15, 0.0)
		 , legSagLeanGradVelXFwd  (CONFIG_PARAM_PATH + "OL/legMotion/sagLeanGradVelXFwd", -0.05, 0.005, 0.15, 0.0)
		 , legSagLeanGradVelZAbs  (CONFIG_PARAM_PATH + "OL/legMotion/sagLeanGradVelZAbs", -0.15, 0.005, 0.15, 0.0)
		 , legLatSwingMagGradY    (CONFIG_PARAM_PATH + "OL/legMotion/latSwingMagGradY", 0.0, 0.005, 0.4, 0.10)
		 , legLatHipSwingBias     (CONFIG_PARAM_PATH + "OL/legMotion/latHipSwingBias", -0.3, 0.005, 0.3, 0.0)
		 , legLatHipSwingMag      (CONFIG_PARAM_PATH + "OL/legMotion/latHipSwingMag", 0.0, 0.005, 0.3, 0.05)
		 , legLatHipSwingMagGradX (CONFIG_PARAM_PATH + "OL/legMotion/latHipSwingMagGradX", -0.15, 0.005, 0.3, 0.0)
		 , legLatHipSwingMagGradY (CONFIG_PARAM_PATH + "OL/legMotion/latHipSwingMagGradY", 0.0, 0.005, 0.3, 0.0)
		 , legLatPushoutMagGradX  (CONFIG_PARAM_PATH + "OL/legMotion/latPushoutMagGradX", 0.0, 0.005, 0.3, 0.0)
		 , legLatPushoutMagGradY  (CONFIG_PARAM_PATH + "OL/legMotion/latPushoutMagGradY", 0.0, 0.005, 0.3, 0.0)
		 , legLatPushoutMagGradZ  (CONFIG_PARAM_PATH + "OL/legMotion/latPushoutMagGradZ", 0.0, 0.005, 0.3, 0.0)
		 , legLatLeanGradXZBwd    (CONFIG_PARAM_PATH + "OL/legMotion/latLeanGradXZBwd", 0.0, 0.01, 1.0, 0.0)
		 , legLatLeanGradXZFwd    (CONFIG_PARAM_PATH + "OL/legMotion/latLeanGradXZFwd", 0.0, 0.01, 1.0, 0.0)
		 , legRotSwingMagGradZ    (CONFIG_PARAM_PATH + "OL/legMotion/rotSwingMagGradZ", 0.0, 0.005, 0.4, 0.10)
		 , legRotVPushoutMagGradZ (CONFIG_PARAM_PATH + "OL/legMotion/rotVPushoutMagGradZ", 0.0, 0.005, 0.4, 0.0)

		 , tuningNoArms           (CONFIG_PARAM_PATH + "tuning/noArms", false)
		 , tuningNoArmSwing       (CONFIG_PARAM_PATH + "tuning/noArmSwing", false)
		 , tuningNoArmFeedback    (CONFIG_PARAM_PATH + "tuning/noArmBasicFeedback", false)
		 , tuningNoLegs           (CONFIG_PARAM_PATH + "tuning/noLegs", false)
		 , tuningNoLegLifting     (CONFIG_PARAM_PATH + "tuning/noLegLifting", false)
		 , tuningNoLegSwing       (CONFIG_PARAM_PATH + "tuning/noLegSwing", false)
		 , tuningNoLegHipSwing    (CONFIG_PARAM_PATH + "tuning/noLegHipSwing", false)
		 , tuningNoLegPushout     (CONFIG_PARAM_PATH + "tuning/noLegPushout", false)
		 , tuningNoLegLeaning     (CONFIG_PARAM_PATH + "tuning/noLegLeaning", false)
		 , tuningNoLegVirtual     (CONFIG_PARAM_PATH + "tuning/noLegVirtual", false)
		 , tuningNoLegFeedback    (CONFIG_PARAM_PATH + "tuning/noLegBasicFeedback", false)
		 , tuningNoLegSuppCoeff   (CONFIG_PARAM_PATH + "tuning/noLegSupportCoeff", false)
		 
		 , virtualSlopeEnabled    (CONFIG_PARAM_PATH + "CL/virtualSlope/virtualSlopeEnabled", false)
		 , virtualSlopeOffset     (CONFIG_PARAM_PATH + "CL/virtualSlope/virtualSlopeOffset", -0.2, 0.005, 0.2, 0.0)
		 , virtualSlopeGainAsc    (CONFIG_PARAM_PATH + "CL/virtualSlope/virtualSlopeGainAsc", 0.0, 0.02, 2.0, 0.0)
		 , virtualSlopeGainDsc    (CONFIG_PARAM_PATH + "CL/virtualSlope/virtualSlopeGainDsc", 0.0, 0.02, 2.0, 0.0)
		 , virtualSlopeMidAngle   (CONFIG_PARAM_PATH + "CL/virtualSlope/virtualSlopeMidAngle", -0.5, 0.01, 0.5, 0.0)
		 , virtualSlopeMinAngle   (CONFIG_PARAM_PATH + "CL/virtualSlope/virtualSlopeMinAngle", 0.0, 0.01, 1.0, 0.1)
		 
		 , basicEnableArmAngleX   (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableArmAngleX", false)
		 , basicEnableArmAngleY   (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableArmAngleY", false)
		 , basicEnableComShiftX   (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableComShiftX", false)
		 , basicEnableComShiftY   (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableComShiftY", false)
		 , basicEnableFootAngleCX (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableFootAngleCtsX", false)
		 , basicEnableFootAngleCY (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableFootAngleCtsY", false)
		 , basicEnableFootAngleX  (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableFootAngleX", false)
		 , basicEnableFootAngleY  (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableFootAngleY", false)
		 , basicEnableHipAngleX   (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableHipAngleX", false)
		 , basicEnableHipAngleY   (CONFIG_PARAM_PATH + "CL/basicFeedback/enable/enableHipAngleY", false)
		 
		 , basicFeedBiasArmAngleX (CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasArmAngleX", -0.5, 0.01, 0.5, 0.0)
		 , basicFeedBiasArmAngleY (CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasArmAngleY", -0.5, 0.01, 0.5, 0.0)
		 , basicFeedBiasComShiftX (CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasComShiftX", -0.1, 0.001, 0.1, 0.0)
		 , basicFeedBiasComShiftY (CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasComShiftY", -0.1, 0.001, 0.1, 0.0)
		 , basicFeedBiasFootAngleX(CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasFootAngleX", -0.5, 0.01, 0.5, 0.0)
		 , basicFeedBiasFootAngleY(CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasFootAngleY", -0.5, 0.01, 0.5, 0.0)
		 , basicFeedBiasFootAngCX (CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasFootAngleCtsX", -0.5, 0.01, 0.5, 0.0)
		 , basicFeedBiasFootAngCY (CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasFootAngleCtsY", -0.5, 0.01, 0.5, 0.0)
		 , basicFeedBiasHipAngleX (CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasHipAngleX", -0.5, 0.01, 0.5, 0.0)
		 , basicFeedBiasHipAngleY (CONFIG_PARAM_PATH + "CL/basicFeedback/feedbackBias/biasHipAngleY", -0.5, 0.01, 0.5, 0.0)
		 
		 , basicFusedEnabledLat   (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/enabledLat", false)
		 , basicFusedEnabledSag   (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/enabledSag", false)
		 , basicFusedFilterN      (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/filterN", 1, 1, 50, 10)
		 , basicFusedDeadRadiusX  (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/deadbandRadiusX", 0.0, 0.01, 1.0, 0.0)
		 , basicFusedDeadRadiusY  (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/deadbandRadiusY", 0.0, 0.01, 1.0, 0.0)
		 , basicFusedExpXSinMag   (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/expectedXSinMag", 0.0, 0.005, 0.3, 0.0)
		 , basicFusedExpYSinMag   (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/expectedYSinMag", 0.0, 0.005, 0.3, 0.0)
		 , basicFusedExpXSinOffset(CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/expectedXSinOffset", -0.3, 0.005, 0.3, 0.0)
		 , basicFusedExpYSinOffset(CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/expectedYSinOffset", -0.3, 0.005, 0.3, 0.0)
		 , basicFusedExpXSinPhase (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/expectedXSinPhase", -M_PI, 0.05, M_PI, 0.0)
		 , basicFusedExpYSinPhase (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/expectedYSinPhase", -M_PI, 0.05, M_PI, 0.0)
		 , basicFusedGainAllLat   (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/gainAllLat", 0.0, 0.05, 4.0, 1.0)
		 , basicFusedGainAllSag   (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/gainAllSag", 0.0, 0.05, 4.0, 1.0)
		 , basicFusedArmAngleX    (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/gainArmAngleX", 0.0, 0.05, 4.0, 0.0)
		 , basicFusedArmAngleY    (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/gainArmAngleY", 0.0, 0.05, 4.0, 0.0)
		 , basicFusedComShiftX    (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/gainComShiftX", 0.0, 0.01, 1.0, 0.0)
		 , basicFusedComShiftY    (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/gainComShiftY", 0.0, 0.01, 1.0, 0.0)
		 , basicFusedFootAngleX   (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/gainFootAngleX", 0.0, 0.01, 1.0, 0.0)
		 , basicFusedFootAngleY   (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/gainFootAngleY", 0.0, 0.01, 1.0, 0.0)
		 , basicFusedHipAngleX    (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/lat/gainHipAngleX", 0.0, 0.01, 1.0, 0.0)
		 , basicFusedHipAngleY    (CONFIG_PARAM_PATH + "CL/basicFeedback/fused/sag/gainHipAngleY", 0.0, 0.01, 1.0, 0.0)
		 
		 , basicDFusedEnabledLat  (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/enabledLat", false)
		 , basicDFusedEnabledSag  (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/enabledSag", false)
		 , basicDFusedFilterN     (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/filterN", 1, 1, 50, 35)
		 , basicDFusedDeadRadiusX (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/lat/deadbandRadiusX", 0.0, 0.01, 1.0, 0.0)
		 , basicDFusedDeadRadiusY (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/sag/deadbandRadiusY", 0.0, 0.01, 1.0, 0.0)
		 , basicDFusedGainAllLat  (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/lat/gainAllLat", 0.0, 0.05, 4.0, 1.0)
		 , basicDFusedGainAllSag  (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/sag/gainAllSag", 0.0, 0.05, 4.0, 1.0)
		 , basicDFusedArmAngleX   (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/lat/gainArmAngleX", 0.0, 0.05, 4.0, 0.0)
		 , basicDFusedArmAngleY   (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/sag/gainArmAngleY", 0.0, 0.05, 4.0, 0.0)
		 , basicDFusedComShiftX   (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/sag/gainComShiftX", 0.0, 0.01, 1.0, 0.0)
		 , basicDFusedComShiftY   (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/lat/gainComShiftY", 0.0, 0.01, 1.0, 0.0)
		 , basicDFusedFootAngleX  (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/lat/gainFootAngleX", 0.0, 0.01, 1.0, 0.0)
		 , basicDFusedFootAngleY  (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/sag/gainFootAngleY", 0.0, 0.01, 1.0, 0.0)
		 , basicDFusedHipAngleX   (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/lat/gainHipAngleX", 0.0, 0.01, 1.0, 0.0)
		 , basicDFusedHipAngleY   (CONFIG_PARAM_PATH + "CL/basicFeedback/dFused/sag/gainHipAngleY", 0.0, 0.01, 1.0, 0.0)
		 
		 , basicIFusedEnabledLat  (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/enabledLat", false)
		 , basicIFusedEnabledSag  (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/enabledSag", false)
		 , basicIFusedFilterN     (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/filterN", 1, 1, 50, 1)
		 , basicIFusedHalfLifeTime(CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/halfLifeTime", 0.05, 0.05, 20.0, 5.0)
		 , basicIFusedTimeToDecay (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/timeToDecay", 0.5, 0.5, 60.0, 20.0)
		 , basicIFusedTimeToFreeze(CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/timeToFreeze", 0.1, 0.05, 2.0, 0.1)
		 , basicIFusedGainAllLat  (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/lat/gainAllLat", 0.0, 0.01, 1.0, 0.0) // Note: This gets multiplied by 1e-3
		 , basicIFusedGainAllSag  (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/sag/gainAllSag", 0.0, 0.01, 1.0, 0.0) // Note: This gets multiplied by 1e-3
		 , basicIFusedArmAngleX   (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/lat/gainArmAngleX", 0.0, 0.05, 4.0, 0.0)
		 , basicIFusedArmAngleY   (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/sag/gainArmAngleY", 0.0, 0.05, 4.0, 0.0)
		 , basicIFusedComShiftX   (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/sag/gainComShiftX", 0.0, 0.01, 1.0, 0.0)
		 , basicIFusedComShiftY   (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/lat/gainComShiftY", 0.0, 0.01, 1.0, 0.0)
		 , basicIFusedFootAngleCX (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/lat/gainFootAngleCtsX", 0.0, 0.01, 1.0, 0.0)
		 , basicIFusedFootAngleCY (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/sag/gainFootAngleCtsY", 0.0, 0.01, 1.0, 0.0)
		 , basicIFusedFootAngleX  (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/lat/gainFootAngleX", 0.0, 0.01, 1.0, 0.0)
		 , basicIFusedFootAngleY  (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/sag/gainFootAngleY", 0.0, 0.01, 1.0, 0.0)
		 , basicIFusedHipAngleX   (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/lat/gainHipAngleX", 0.0, 0.01, 1.0, 0.0)
		 , basicIFusedHipAngleY   (CONFIG_PARAM_PATH + "CL/basicFeedback/iFused/sag/gainHipAngleY", 0.0, 0.01, 1.0, 0.0)
		 
		 , basicGyroEnabledLat    (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/enabledLat", false)
		 , basicGyroEnabledSag    (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/enabledSag", false)
		 , basicGyroFilterN       (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/filterN", 1, 1, 50, 35)
		 , basicGyroDeadRadiusX   (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/lat/deadbandRadiusX", 0.0, 0.01, 1.0, 0.0)
		 , basicGyroDeadRadiusY   (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/sag/deadbandRadiusY", 0.0, 0.01, 1.0, 0.0)
		 , basicGyroExpX          (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/lat/expectedGyroX", -1.0, 0.01, 1.0, 0.0)
		 , basicGyroExpY          (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/sag/expectedGyroY", -1.0, 0.01, 1.0, 0.0)
		 , basicGyroGainAllLat    (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/lat/gainAllLat", 0.0, 0.05, 4.0, 1.0)
		 , basicGyroGainAllSag    (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/sag/gainAllSag", 0.0, 0.05, 4.0, 1.0)
		 , basicGyroArmAngleX     (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/lat/gainArmAngleX", 0.0, 0.05, 4.0, 0.0)
		 , basicGyroArmAngleY     (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/sag/gainArmAngleY", 0.0, 0.05, 4.0, 0.0)
		 , basicGyroComShiftX     (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/sag/gainComShiftX", 0.0, 0.01, 1.0, 0.0)
		 , basicGyroComShiftY     (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/lat/gainComShiftY", 0.0, 0.01, 1.0, 0.0)
		 , basicGyroFootAngleX    (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/lat/gainFootAngleX", 0.0, 0.01, 1.0, 0.0)
		 , basicGyroFootAngleY    (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/sag/gainFootAngleY", 0.0, 0.01, 1.0, 0.0)
		 , basicGyroHipAngleX     (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/lat/gainHipAngleX", 0.0, 0.01, 1.0, 0.0)
		 , basicGyroHipAngleY     (CONFIG_PARAM_PATH + "CL/basicFeedback/gyro/sag/gainHipAngleY", 0.0, 0.01, 1.0, 0.0)
		 
		 , basicTimingEnabled     (CONFIG_PARAM_PATH + "CL/basicFeedback/timing/enabled", false)
		 , basicTimingFeedDeadRad (CONFIG_PARAM_PATH + "CL/basicFeedback/timing/feedDeadRadius", 0.0, 0.01, 1.0, 0.0)
		 , basicTimingGainSlowDown(CONFIG_PARAM_PATH + "CL/basicFeedback/timing/gainSlowDown", 0.0, 0.1, 20.0, 5.0)
		 , basicTimingGainSpeedUp (CONFIG_PARAM_PATH + "CL/basicFeedback/timing/gainSpeedUp", 0.0, 0.1, 20.0, 3.0)
		 , basicTimingWeightFactor(CONFIG_PARAM_PATH + "CL/basicFeedback/timing/weightFactor", 0.0, 0.05, 5.0, 1.0)
		 
		 , basicComShiftXBuf      (CONFIG_PARAM_PATH + "CL/basicFeedback/comShift/comShiftXBuf", 0.0, 0.001, 0.1, 0.0)
		 , basicComShiftXMax      (CONFIG_PARAM_PATH + "CL/basicFeedback/comShift/comShiftXMax", 0.0, 0.001, 0.1, 0.0)
		 , basicComShiftXMin      (CONFIG_PARAM_PATH + "CL/basicFeedback/comShift/comShiftXMin", -0.1, 0.001, 0.0, 0.0)
		 , basicComShiftXUseLimits(CONFIG_PARAM_PATH + "CL/basicFeedback/comShift/comShiftXUseLimits", true)
		 , basicComShiftYBuf      (CONFIG_PARAM_PATH + "CL/basicFeedback/comShift/comShiftYBuf", 0.0, 0.001, 0.1, 0.0)
		 , basicComShiftYMax      (CONFIG_PARAM_PATH + "CL/basicFeedback/comShift/comShiftYMax", 0.0, 0.001, 0.1, 0.0)
		 , basicComShiftYMin      (CONFIG_PARAM_PATH + "CL/basicFeedback/comShift/comShiftYMin", -0.1, 0.001, 0.0, 0.0)
		 , basicComShiftYUseLimits(CONFIG_PARAM_PATH + "CL/basicFeedback/comShift/comShiftYUseLimits", true)
		 , basicFootAnglePhaseLen (CONFIG_PARAM_PATH + "CL/basicFeedback/footAngle/transitionPhaseLen", 0.3, 0.01, 1.5, 0.5)
		 
		 , cmdUseCLStepSize       (CONFIG_PARAM_PATH + "CL/cmd/useCLStepSize", false)
		 , cmdUseCLTiming         (CONFIG_PARAM_PATH + "CL/cmd/useCLTiming", false)
		 , cmdUseNonZeroZMP       (CONFIG_PARAM_PATH + "CL/cmd/useNonZeroZMP", false)
		 , cmdUseRXFeedback       (CONFIG_PARAM_PATH + "CL/cmd/useRXFeedback", false)
		 , cmdUseTXStepSize       (CONFIG_PARAM_PATH + "CL/cmd/useTXStepSize", false)
		 , cmdUseTXTiming         (CONFIG_PARAM_PATH + "CL/cmd/useTXTiming", false)
		 , mgC                    (CONFIG_PARAM_PATH + "CL/mg/LIPM/C", 0.0, 0.2, 40.0, 10.0)
		 , mgAlpha                (CONFIG_PARAM_PATH + "CL/mg/LIPM/Alpha", -1.0, 0.01, 1.0, 0.0)
		 , mgDelta                (CONFIG_PARAM_PATH + "CL/mg/LIPM/Delta", -1.0, 0.01, 1.0, 0.0)
		 , mgOmega                (CONFIG_PARAM_PATH + "CL/mg/LIPM/Omega", -1.0, 0.01, 1.0, 0.0)
		 , mgSigma                (CONFIG_PARAM_PATH + "CL/mg/LIPM/Sigma", -2.0, 0.02, 2.0, 0.0)
		 , mgGamma                (CONFIG_PARAM_PATH + "CL/mg/LIPM/Gamma", -1.0, 0.02, 3.0, 0.0)
		 , mgLatency              (CONFIG_PARAM_PATH + "CL/mg/latency", 0.0, 0.002, 0.4, 0.050)
		 , mgZmpXMin              (CONFIG_PARAM_PATH + "CL/mg/zmpXMin", -1.0, 0.01, 1.0, 0.0)
		 , mgZmpXMax              (CONFIG_PARAM_PATH + "CL/mg/zmpXMax", -1.0, 0.01, 1.0, 0.0)
		 , mgZmpYMin              (CONFIG_PARAM_PATH + "CL/mg/zmpYMin", -1.0, 0.01, 1.0, 0.0)
		 , mgZmpYMax              (CONFIG_PARAM_PATH + "CL/mg/zmpYMax", -1.0, 0.01, 1.0, 0.0)
		 , mgComOffsetX           (CONFIG_PARAM_PATH + "CL/mg/comOffsetX", -1.0, 0.01, 1.0, 0.0)
		 , mgComOffsetY           (CONFIG_PARAM_PATH + "CL/mg/comOffsetY", -1.0, 0.01, 1.0, 0.0)
		 , mgFusedOffsetX         (CONFIG_PARAM_PATH + "CL/mg/fusedOffsetX", -0.5, 0.005, 0.5, 0.0)
		 , mgFusedOffsetY         (CONFIG_PARAM_PATH + "CL/mg/fusedOffsetY", -0.5, 0.005, 0.5, 0.0)
		 , mgMaxStepRadiusX       (CONFIG_PARAM_PATH + "CL/mg/maxStepRadiusX", 0.0, 0.01, 2.0, 0.4)
		 , mgMaxStepRadiusY       (CONFIG_PARAM_PATH + "CL/mg/maxStepRadiusY", 0.0, 0.01, 2.0, 0.4)
		 , mgMaxComPositionX      (CONFIG_PARAM_PATH + "CL/mg/maxComPositionX", -1.0, 0.01, 1.0, 0.0)
		 , mgPostStepStateCorrAng (CONFIG_PARAM_PATH + "CL/mg/postStepStateCorrAng", -1.0, 0.01, 1.0, 0.0)
		 , nsGain                 (CONFIG_PARAM_PATH + "CL/noiseSuppression/gain", 0.0, 0.01, 2.0, 1.0)
		 , nsFusedXRangeLBnd      (CONFIG_PARAM_PATH + "CL/noiseSuppression/fusedXRangeLBnd", -0.5, 0.01, 0.5, 0.0)
		 , nsFusedXRangeUBnd      (CONFIG_PARAM_PATH + "CL/noiseSuppression/fusedXRangeUBnd", -0.5, 0.01, 0.5, 0.0)
		 , nsFusedYRangeLBnd      (CONFIG_PARAM_PATH + "CL/noiseSuppression/fusedYRangeLBnd", -0.5, 0.01, 0.5, 0.0)
		 , nsFusedYRangeUBnd      (CONFIG_PARAM_PATH + "CL/noiseSuppression/fusedYRangeUBnd", -0.5, 0.01, 0.5, 0.0)
		 , nsMaxAdaptation        (CONFIG_PARAM_PATH + "CL/noiseSuppression/maxAdaptation", 0.0, 0.01, 1.0, 1.0)
		 , nsAdaptationGain       (CONFIG_PARAM_PATH + "CL/noiseSuppression/adaptationGain", 0.0, 0.05, 5.0, 1.0)
		 , nsStepNoiseTime        (CONFIG_PARAM_PATH + "CL/noiseSuppression/stepNoiseTime", 0.05, 0.01, 1.0, 0.1)
		{
			// Set up robot specification callbacks
			boost::function<void (const float&)> robotSpecCallback = boost::bind(&CapConfig::callRobotSpecCallbacks, this);
			armLinkLength.setCallback(robotSpecCallback);
			legLinkLength.setCallback(robotSpecCallback);
			shoulderWidth.setCallback(robotSpecCallback);
			hipWidth.setCallback(robotSpecCallback);
			trunkHeight.setCallback(robotSpecCallback);
			trunkLinkOffsetX.setCallback(robotSpecCallback);
			trunkLinkOffsetY.setCallback(robotSpecCallback);
			trunkLinkOffsetZ.setCallback(robotSpecCallback);
			comOffsetX.setCallback(robotSpecCallback);
			comOffsetZ.setCallback(robotSpecCallback);
			footWidth.setCallback(robotSpecCallback);
			footLength.setCallback(robotSpecCallback);
			footOffsetX.setCallback(robotSpecCallback);
			footOffsetY.setCallback(robotSpecCallback);
			footOffsetZ.setCallback(robotSpecCallback);
			neckHeight.setCallback(robotSpecCallback);
			headOffsetX.setCallback(robotSpecCallback);
			headOffsetZ.setCallback(robotSpecCallback);
		}

		//! @name Class constants
		///@{
		const std::string CONFIG_PARAM_PATH;                    //!< @brief Path for the capture step gait configuration parameters on the config server
		///@}
		
		//! @name Robot specifications
		///@{
		config_server::Parameter<float> armLinkLength;          //!< @brief Length of each/both the upper and lower arm links
		config_server::Parameter<float> legLinkLength;          //!< @brief Length of each/both the upper and lower leg links
		config_server::Parameter<float> shoulderWidth;          //!< @brief Horizontal separation between the two hip joints (length of the hip line)
		config_server::Parameter<float> hipWidth;               //!< @brief Horizontal separation between the two hip joints (length of the hip line)
		config_server::Parameter<float> trunkHeight;            //!< @brief Vertical height of the trunk from the hip line to the shoulder line
		config_server::Parameter<float> trunkLinkOffsetX;       //!< @brief Forward offset of the trunk link tf frame from the hip midpoint
		config_server::Parameter<float> trunkLinkOffsetY;       //!< @brief Leftward offset of the trunk link tf frame from the hip midpoint
		config_server::Parameter<float> trunkLinkOffsetZ;       //!< @brief Upward offset of the trunk link tf frame from the hip midpoint
		config_server::Parameter<float> comOffsetX;             //!< @brief Forward offset of the CoM in front of the hip line
		config_server::Parameter<float> comOffsetZ;             //!< @brief Height of the CoM above the hip line
		config_server::Parameter<float> footWidth;              //!< @brief Width of the robot foot (along y-axis, the foot plate is assumed to be rectangular)
		config_server::Parameter<float> footLength;             //!< @brief Length of the robot foot (along x-axis, the foot plate is assumed to be rectangular)
		config_server::Parameter<float> footOffsetX;            //!< @brief Backward offset from the foot plate geometric center to the ankle joint (along x-axis)
		config_server::Parameter<float> footOffsetY;            //!< @brief Inward offset from the foot plate geometric center to the ankle joint (along y-axis)
		config_server::Parameter<float> footOffsetZ;            //!< @brief Upward offset from the foot plate geometric center to the ankle joint (along z-axis)
		config_server::Parameter<float> neckHeight;             //!< @brief Height of the neck joint vertically above the center of the shoulder line (not for analytic calculation)
		config_server::Parameter<float> headOffsetX;            //!< @brief Forward offset from the neck joint to the center of the head (not for analytic calculation)
		config_server::Parameter<float> headOffsetZ;            //!< @brief Upward offset from the neck joint to the center of the head (not for analytic calculation)
		///@}

		//! @name Robot visualisation parameters
		///@{
		config_server::Parameter<float> armThickness;           //!< @brief Approximate thickness of the arm segments (not for analytic calculation)
		config_server::Parameter<float> legThickness;           //!< @brief Approximate thickness of the leg segments (not for analytic calculation)
		config_server::Parameter<float> headDiameter;           //!< @brief Approximate diameter of the head (not for analytic calculation)
		config_server::Parameter<float> jointDiameter;          //!< @brief Diameter of the joints (not for analytic calculation)
		config_server::Parameter<float> visOffsetX;             //!< @brief Global x offset to the robot model where the RobotModelVis visualisation should be displayed
		config_server::Parameter<float> visOffsetY;             //!< @brief Global y offset to the robot model where the RobotModelVis visualisation should be displayed
		config_server::Parameter<float> visOffsetZ;             //!< @brief Global z offset to the robot model where the RobotModelVis visualisation should be displayed
		config_server::Parameter<bool>  markVectors;            //!< @brief Boolean flag whether to publish arrow markers showing the various vector quantities of the robot model
		///@}

		//! @name Robot model parameters
		///@{
		config_server::Parameter<float> footHeightHysteresis;   //!< @brief The minimum required foot height difference in the robot model to unlock the possibility of the model performing a support exchange
		///@}

		//! @name General gait parameters
		///@{
		config_server::Parameter<bool>  enableMotionStances;    //!< @brief Boolean flag whether to enable the use of motion stances (changes to the halt pose during stopping to allow a particular follow-up motion to be played)
		config_server::Parameter<float> gaitFrequency;          //!< @brief Nominal frequency of the gait
		config_server::Parameter<float> gaitFrequencyMax;       //!< @brief Maximum allowed frequency of the gait
		config_server::Parameter<bool>  leftLegFirst;           //!< @brief Flag specifying whether the first leg to step with when starting walking should be the left leg
		config_server::Parameter<float> stanceAdjustGcvMax;     //!< @brief The maximum GCV at which stance adjustments are allowed to occur during stopping
		config_server::Parameter<float> stanceAdjustRate;       //!< @brief The dimensionless rate at which motion stance adjustment occurs while stopping walking
		config_server::Parameter<float> stoppingGcvMag;         //!< @brief Unbiased gait command velocity 2-norm below which immediate walk stopping is allowed
		config_server::Parameter<float> stoppingPhaseTolLB;     //!< @brief Gait phase tolerance below 0 and &pi;, in units of nominal phase increments (see gaitFrequency and the robotcontrol cycle time), within which intelligent walking stopping is allowed
		config_server::Parameter<float> stoppingPhaseTolUB;     //!< @brief Gait phase tolerance above 0 and -&pi;, in units of nominal phase increments (see gaitFrequency and the robotcontrol cycle time), within which intelligent walking stopping is allowed
		config_server::Parameter<float> supportCoeffRange;      //!< @brief The required difference between the symmetric support coefficients during walking (e.g. if this is 0.8 the support coefficients transition between 0.1 and 0.9, as 0.9 - 0.1 = 0.8 and 0.9 + 0.1 = 1.0)
		config_server::Parameter<bool>  useServoModel;          //!< @brief Flag specifying whether to use raw joint commands (i.e. no servo model, `false`), or use the servo model (`true`)
		///@}

		//! @name Gait command vector parameters
		///@{
		config_server::Parameter<float> gcvBiasLinVelX;         //!< @brief Bias added to the linear x-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
		config_server::Parameter<float> gcvBiasLinVelY;         //!< @brief Bias added to the linear y-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
		config_server::Parameter<float> gcvBiasAngVelZ;         //!< @brief Bias added to the angular z-velocity component of the received gait command vector so as to produce zero observable robot velocity for zero velocity command
		config_server::Parameter<float> gcvAccForwards;         //!< @brief Gait command acceleration limit for positive linear x command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
		config_server::Parameter<float> gcvAccBackwards;        //!< @brief Gait command acceleration limit for negative linear x command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
		config_server::Parameter<float> gcvAccSidewards;        //!< @brief Gait command acceleration limit for linear y command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
		config_server::Parameter<float> gcvAccRotational;       //!< @brief Gait command acceleration limit for angular z command velocities (scale by #gcvDecToAccRatio to get the deceleration limit)
		config_server::Parameter<float> gcvDecToAccRatio;       //!< @brief Multiplicative scale factor to obtain the gait command deceleration limits from the gait command acceleration limits
		config_server::Parameter<float> gcvAccJerkLimitX;       //!< @brief Jerk (acceleration slope) limit for the calculation of the gait command acceleration from m_gcv (linear X direction)
		config_server::Parameter<float> gcvAccJerkLimitY;       //!< @brief Jerk (acceleration slope) limit for the calculation of the gait command acceleration from m_gcv (linear Y direction)
		config_server::Parameter<float> gcvAccJerkLimitZ;       //!< @brief Jerk (acceleration slope) limit for the calculation of the gait command acceleration from m_gcv (angular Z direction)
		config_server::Parameter<float> gcvPrescalerLinVelX;    //!< @brief Prescaler for the gcv linear velocity X that is then used by the open loop gait (allows easy scaling of the dynamic range of the gait)
		config_server::Parameter<float> gcvPrescalerLinVelY;    //!< @brief Prescaler for the gcv linear velocity Y that is then used by the open loop gait (allows easy scaling of the dynamic range of the gait)
		config_server::Parameter<float> gcvPrescalerAngVelZ;    //!< @brief Prescaler for the gcv angular velocity Z that is then used by the open loop gait (allows easy scaling of the dynamic range of the gait)
		///@}
		
		//! @name Limb limit parameters
		///@{
		config_server::Parameter<float> limArmAngleXBuf;        //!< @brief Angle buffer for soft limiting of the arm angle X to be commanded by the gait (see `nimbro_utils::softCoerce`)
		config_server::Parameter<float> limArmAngleXMax;        //!< @brief Maximum allowed arm angle X to be commanded by the gait (for left arm)
		config_server::Parameter<float> limArmAngleXMin;        //!< @brief Minimum allowed arm angle X to be commanded by the gait (for left arm)
		config_server::Parameter<bool>  limArmAngleXUseLimits;  //!< @brief Boolean flag whether to use the specified arm angle X min/max limits
		config_server::Parameter<float> limArmAngleYBuf;        //!< @brief Angle buffer for soft limiting of the arm angle Y to be commanded by the gait (see `nimbro_utils::softCoerce`)
		config_server::Parameter<float> limArmAngleYMax;        //!< @brief Maximum allowed arm angle Y to be commanded by the gait
		config_server::Parameter<float> limArmAngleYMin;        //!< @brief Minimum allowed arm angle Y to be commanded by the gait
		config_server::Parameter<bool>  limArmAngleYUseLimits;  //!< @brief Boolean flag whether to use the specified arm angle Y min/max limits
		config_server::Parameter<float> limFootAngleXBuf;       //!< @brief Angle buffer for soft limiting of the foot angle X to be commanded by the gait (see `nimbro_utils::softCoerce`)
		config_server::Parameter<float> limFootAngleXMax;       //!< @brief Maximum allowed foot angle X to be commanded by the gait (for left foot)
		config_server::Parameter<float> limFootAngleXMin;       //!< @brief Minimum allowed foot angle X to be commanded by the gait (for left foot)
		config_server::Parameter<bool>  limFootAngleXUseLimits; //!< @brief Boolean flag whether to use the specified foot angle X min/max limits
		config_server::Parameter<float> limFootAngleYBuf;       //!< @brief Angle buffer for soft limiting of the foot angle Y to be commanded by the gait (see `nimbro_utils::softCoerce`)
		config_server::Parameter<float> limFootAngleYMax;       //!< @brief Maximum allowed foot angle Y to be commanded by the gait
		config_server::Parameter<float> limFootAngleYMin;       //!< @brief Minimum allowed foot angle Y to be commanded by the gait
		config_server::Parameter<bool>  limFootAngleYUseLimits; //!< @brief Boolean flag whether to use the specified foot angle Y min/max limits
		config_server::Parameter<float> limLegAngleXBuf;        //!< @brief Angle buffer for soft limiting of the leg angle X to be commanded by the gait (see `nimbro_utils::softCoerce`)
		config_server::Parameter<float> limLegAngleXMax;        //!< @brief Maximum allowed leg angle X to be commanded by the gait (for left leg)
		config_server::Parameter<float> limLegAngleXMin;        //!< @brief Minimum allowed leg angle X to be commanded by the gait (for left leg)
		config_server::Parameter<bool>  limLegAngleXUseLimits;  //!< @brief Boolean flag whether to use the specified leg angle X min/max limits
		config_server::Parameter<float> limLegAngleYBuf;        //!< @brief Angle buffer for soft limiting of the leg angle Y to be commanded by the gait (see `nimbro_utils::softCoerce`)
		config_server::Parameter<float> limLegAngleYMax;        //!< @brief Maximum allowed leg angle Y to be commanded by the gait
		config_server::Parameter<float> limLegAngleYMin;        //!< @brief Minimum allowed leg angle Y to be commanded by the gait
		config_server::Parameter<bool>  limLegAngleYUseLimits;  //!< @brief Boolean flag whether to use the specified leg angle Y min/max limits
		///@}
		
		//! @name Gait phase parameters
		///@{
		config_server::Parameter<float> startBlendPhaseLen;     //!< @brief The amount of time, in terms of gait phase, to take to blend from the halt pose to the moving calculated gait pose during start of walking
		config_server::Parameter<float> stopBlendPhaseLen;      //!< @brief The amount of time, in terms of gait phase, to take to blend from the moving calculated gait pose to the halt pose after cessation of walking
		config_server::Parameter<float> doubleSupportPhaseLen;  //!< @brief Length of the double support phase
		config_server::Parameter<float> swingStartPhaseOffset;  //!< @brief Offset from the end of the double support phase to the start of the swing phase
		config_server::Parameter<float> swingStopPhaseOffset;   //!< @brief Offset from the start of the double support phase back to the end of the previous swing phase (a positive value means the double support phase starts that amount after the end of swing)
		config_server::Parameter<float> swingMinPhaseLen;       //!< @brief Minimum allowed length of the swing phase (used as a check only, should have no effect on gait phase timing if there is no violation)
		config_server::Parameter<float> suppTransStartRatio;    //!< @brief Ratio that governs the start of support transitioning, via linear interpolation (0 => At start of double support phase, 1 => At end of immediately preceding end of swing phase)
		config_server::Parameter<float> suppTransStopRatio;     //!< @brief Ratio that governs the stop of support transitioning, via linear interpolation (0 => At end of double support phase, 1 => At start of immediately following start of swing phase)
		config_server::Parameter<float> filletStepPhaseLen;     //!< @brief Nominal size, in terms of approximate phase duration, of the fillets to the leg extension stepping waveform
		config_server::Parameter<float> filletPushPhaseLen;     //!< @brief Nominal size, in terms of approximate phase duration, of the fillets to the leg extension pushing waveform
		///@}

		//! @name Halt pose parameters
		///@{
		config_server::Parameter<float> haltArmExtension;       //!< @brief Halt pose: Extension of the arms (0 = Fully extended, 1 = Fully contracted, see @ref gait::AbstractArmPose "AbstractArmPose")
		config_server::Parameter<float> haltArmAngleX;          //!< @brief Halt pose: Roll angle of the arms (positive is away from the body for both arms)
		config_server::Parameter<float> haltArmAngleXBias;      //!< @brief Halt pose: Additive anti-symmetric roll angle of the arms (positive is a roll rotation about the positive x axis)
		config_server::Parameter<float> haltArmAngleY;          //!< @brief Halt pose: Pitch angle of the central axis of the arms (positive is moving the arms towards the back for both arms)
		config_server::Parameter<float> haltLegExtension;       //!< @brief Halt pose: Extension of the legs (0 = Fully extended, 1 = Fully contracted, see @ref gait::AbstractLegPose "AbstractLegPose")
		config_server::Parameter<float> haltLegExtensionBias;   //!< @brief Halt pose: Additive one-sided bias of the extension of one of the legs (+ve = Left leg only is shortened by this amount, -ve = Right leg only is shortened by this amount)
		config_server::Parameter<float> haltLegAngleX;          //!< @brief Halt pose: Roll angle of the legs (positive is away from the body for both legs)
		config_server::Parameter<float> haltLegAngleXBias;      //!< @brief Halt pose: Additive anti-symmetric roll angle of the legs (positive is a roll rotation about the positive x axis)
		config_server::Parameter<float> haltLegAngleXNarrow;    //!< @brief Halt pose: Roll angle of the legs (positive is away from the body for both legs) for the narrow feet halt pose
		config_server::Parameter<float> haltLegAngleY;          //!< @brief Halt pose: Pitch angle of the central axis of the legs (positive is moving the legs towards the back for both legs)
		config_server::Parameter<float> haltLegAngleZ;          //!< @brief Halt pose: Yaw angle of the legs (toe-out is positive for both legs)
		config_server::Parameter<float> haltFootAngleX;         //!< @brief Halt pose: Roll angle of the feet relative to the trunk (positive is tilting onto the inner feet for both feet)
		config_server::Parameter<float> haltFootAngleXBias;     //!< @brief Halt pose: Additive anti-symmetric roll angle of the feet relative to the trunk (positive is a roll rotation about the positive x axis)
		config_server::Parameter<float> haltFootAngleY;         //!< @brief Halt pose: Pitch angle of the feet relative to the trunk (positive is what would make the robot lean back for both feet)
		config_server::Parameter<float> haltEffortArm;          //!< @brief Halt pose: Joint effort to use for the arms (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipYaw;       //!< @brief Halt pose: Joint effort to use for the leg hip yaw (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipRoll;      //!< @brief Halt pose: Joint effort to use for the leg hip roll (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortHipPitch;     //!< @brief Halt pose: Joint effort to use for the leg hip pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortKneePitch;    //!< @brief Halt pose: Joint effort to use for the leg knee pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortAnklePitch;   //!< @brief Halt pose: Joint effort to use for the leg ankle pitch (in the range `[0,1]`)
		config_server::Parameter<float> haltEffortAnkleRoll;    //!< @brief Halt pose: Joint effort to use for the leg ankle roll (in the range `[0,1]`)
		///@}

		//! @name Arm motion parameters
		///@{
		config_server::Parameter<float> armSagSwingMag;         //!< @brief Magnitude in radians of the arm swing to use at zero biased gait command velocity (for a total peak-to-peak swing of double this value)
		config_server::Parameter<float> armSagSwingMagGradX;    //!< @brief Gradient of #armSagSwingMag with respect to the biased gait command x-velocity
		///@}

		//! @name Leg motion parameters
		///@{
		config_server::Parameter<float> legExtToAngleYGain;     //!< @brief Gain that determines how much the leg extension is also applied to the leg angle Y in order to modify the angle at which the robot lifts its feet, and thereby trim walking on the spot in the sagittal direction
		config_server::Parameter<float> legHipAngleXLegExtGain; //!< @brief Gain that determines how much the appropriate leg is shortened to try to keep the feet level vertically when applying hip angle X
		config_server::Parameter<float> legStepHeight;          //!< @brief Nominal swing leg step height (out of the ground, in units of leg extension) to use at zero biased gait command velocity
		config_server::Parameter<float> legStepHeightGradX;     //!< @brief Gradient of #legStepHeight with respect to the absolute biased gait command x-velocity
		config_server::Parameter<float> legStepHeightGradY;     //!< @brief Gradient of #legStepHeight with respect to the absolute biased gait command y-velocity
		config_server::Parameter<float> legPushHeight;          //!< @brief Nominal support leg push height (into the ground, in units of leg extension) to use at zero biased gait command velocity
		config_server::Parameter<float> legPushHeightGradX;     //!< @brief Gradient of #legPushHeight with respect to the absolute biased gait command x-velocity
		config_server::Parameter<float> legSagSwingMagGradXBwd; //!< @brief Gradient of the sagittal leg swing magnitude (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is negative (backwards walking)
		config_server::Parameter<float> legSagSwingMagGradXFwd; //!< @brief Gradient of the sagittal leg swing magnitude (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is positive (forwards walking)
		config_server::Parameter<float> legSagLeanGradAccXBwd;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the gait command x-acceleration, when this acceleration is negative (backwards acceleration)
		config_server::Parameter<float> legSagLeanGradAccXFwd;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the gait command x-acceleration, when this acceleration is positive (forwards acceleration)
		config_server::Parameter<float> legSagLeanGradVelXBwd;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is negative (backwards velocity)
		config_server::Parameter<float> legSagLeanGradVelXFwd;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the biased gait command x-velocity, when this velocity is positive (forwards velocity)
		config_server::Parameter<float> legSagLeanGradVelZAbs;  //!< @brief Gradient of the sagittal lean (nominally zero radians) with respect to the absolute value of the biased gait command z-velocity
		config_server::Parameter<float> legLatSwingMagGradY;    //!< @brief Gradient of the lateral leg swing magnitude (nominally zero radians) with respect to the biased gait command y-velocity
		config_server::Parameter<float> legLatHipSwingBias;     //!< @brief Constant bias to the lateral hip swing waveform (hip roll angle, units of radians), used to try to correct for robot walking asymmetries
		config_server::Parameter<float> legLatHipSwingMag;      //!< @brief Nominal lateral hip swing magnitude (hip roll angle, units of radians) to use at zero biased gait command velocity
		config_server::Parameter<float> legLatHipSwingMagGradX; //!< @brief Gradient of the lateral hip swing magnitude with respect to the absolute biased gait command x-velocity
		config_server::Parameter<float> legLatHipSwingMagGradY; //!< @brief Gradient of the lateral hip swing magnitude with respect to the absolute biased gait command y-velocity
		config_server::Parameter<float> legLatPushoutMagGradX;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command x-velocity
		config_server::Parameter<float> legLatPushoutMagGradY;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command y-velocity
		config_server::Parameter<float> legLatPushoutMagGradZ;  //!< @brief Gradient of the outwards hip-roll-based lateral leg pushout (nominally zero radians) with respect to the absolute biased gait command z-velocity
		config_server::Parameter<float> legLatLeanGradXZBwd;    //!< @brief Gradient of the lateral lean (nominally zero radians) with respect to the biased gait command x (absolute) and z (signed) velocities, when the x-velocity is negative
		config_server::Parameter<float> legLatLeanGradXZFwd;    //!< @brief Gradient of the lateral lean (nominally zero radians) with respect to the biased gait command x (absolute) and z (signed) velocities, when the x-velocity is positive
		config_server::Parameter<float> legRotSwingMagGradZ;    //!< @brief Gradient of the rotational leg swing magnitude (nominally zero radians) with respect to the biased gait command z-velocity
		config_server::Parameter<float> legRotVPushoutMagGradZ; //!< @brief Gradient of the rotational leg V pushout magnitude (nominally zero radians) with respect to the absolute biased gait command z-velocity
		///@}

		//! @name Tuning parameters
		///@{
		config_server::Parameter<bool> tuningNoArms;            //!< @brief Disable all gait arm motions
		config_server::Parameter<bool> tuningNoArmSwing;        //!< @brief Disable all arm swing components
		config_server::Parameter<bool> tuningNoArmFeedback;     //!< @brief Disable all arm basic feedback components
		config_server::Parameter<bool> tuningNoLegs;            //!< @brief Disable all gait leg motions
		config_server::Parameter<bool> tuningNoLegLifting;      //!< @brief Disable all leg lifting
		config_server::Parameter<bool> tuningNoLegSwing;        //!< @brief Disable all leg swing components
		config_server::Parameter<bool> tuningNoLegHipSwing;     //!< @brief Disable all leg hip swing components
		config_server::Parameter<bool> tuningNoLegPushout;      //!< @brief Disable all leg pushout components
		config_server::Parameter<bool> tuningNoLegLeaning;      //!< @brief Disable all leg leaning components
		config_server::Parameter<bool> tuningNoLegVirtual;      //!< @brief Disable all leg virtual slope components
		config_server::Parameter<bool> tuningNoLegFeedback;     //!< @brief Disable all leg basic feedback components
		config_server::Parameter<bool> tuningNoLegSuppCoeff;    //!< @brief Disable all variations in the leg support coefficients (makes them equal by default instead)
		///@}

		//! @name Virtual slope parameters
		///@{
		config_server::Parameter<bool>  virtualSlopeEnabled;    //!< @brief Boolean flag whether use of the virtual slope method is enabled
		config_server::Parameter<float> virtualSlopeOffset;     //!< @brief A constant offset to the virtual slope to continuously apply while walking
		config_server::Parameter<float> virtualSlopeGainAsc;    //!< @brief Gradient of the virtual slope with respect to the fused pitch angle (after deadband has been applied) when walking up a virtual slope
		config_server::Parameter<float> virtualSlopeGainDsc;    //!< @brief Gradient of the virtual slope with respect to the fused pitch angle (after deadband has been applied) when walking down a virtual slope
		config_server::Parameter<float> virtualSlopeMidAngle;   //!< @brief Fused pitch angle for which the (offset-less) virtual slope should be zero
		config_server::Parameter<float> virtualSlopeMinAngle;   //!< @brief Minimum radius of the fused pitch angle from virtualSlopeMidAngle before virtual slope starts being applied with a certain gradient (i.e. radius of deadband)
		///@}
		
		//! @name Basic feedback parameters
		///@{
		config_server::Parameter<bool>  basicEnableArmAngleX;
		config_server::Parameter<bool>  basicEnableArmAngleY;
		config_server::Parameter<bool>  basicEnableComShiftX;
		config_server::Parameter<bool>  basicEnableComShiftY;
		config_server::Parameter<bool>  basicEnableFootAngleCX;
		config_server::Parameter<bool>  basicEnableFootAngleCY;
		config_server::Parameter<bool>  basicEnableFootAngleX;
		config_server::Parameter<bool>  basicEnableFootAngleY;
		config_server::Parameter<bool>  basicEnableHipAngleX;
		config_server::Parameter<bool>  basicEnableHipAngleY;
		
		config_server::Parameter<float> basicFeedBiasArmAngleX;
		config_server::Parameter<float> basicFeedBiasArmAngleY;
		config_server::Parameter<float> basicFeedBiasComShiftX;
		config_server::Parameter<float> basicFeedBiasComShiftY;
		config_server::Parameter<float> basicFeedBiasFootAngleX;
		config_server::Parameter<float> basicFeedBiasFootAngleY;
		config_server::Parameter<float> basicFeedBiasFootAngCX;
		config_server::Parameter<float> basicFeedBiasFootAngCY;
		config_server::Parameter<float> basicFeedBiasHipAngleX;
		config_server::Parameter<float> basicFeedBiasHipAngleY;
		
		config_server::Parameter<bool>  basicFusedEnabledLat;
		config_server::Parameter<bool>  basicFusedEnabledSag;
		config_server::Parameter<int>   basicFusedFilterN;
		config_server::Parameter<float> basicFusedDeadRadiusX;
		config_server::Parameter<float> basicFusedDeadRadiusY;
		config_server::Parameter<float> basicFusedExpXSinMag;
		config_server::Parameter<float> basicFusedExpYSinMag;
		config_server::Parameter<float> basicFusedExpXSinOffset;
		config_server::Parameter<float> basicFusedExpYSinOffset;
		config_server::Parameter<float> basicFusedExpXSinPhase;
		config_server::Parameter<float> basicFusedExpYSinPhase;
		config_server::Parameter<float> basicFusedGainAllLat;
		config_server::Parameter<float> basicFusedGainAllSag;
		config_server::Parameter<float> basicFusedArmAngleX;
		config_server::Parameter<float> basicFusedArmAngleY;
		config_server::Parameter<float> basicFusedComShiftX;
		config_server::Parameter<float> basicFusedComShiftY;
		config_server::Parameter<float> basicFusedFootAngleX;
		config_server::Parameter<float> basicFusedFootAngleY;
		config_server::Parameter<float> basicFusedHipAngleX;
		config_server::Parameter<float> basicFusedHipAngleY;
		 
		config_server::Parameter<bool>  basicDFusedEnabledLat;
		config_server::Parameter<bool>  basicDFusedEnabledSag;
		config_server::Parameter<int>   basicDFusedFilterN;
		config_server::Parameter<float> basicDFusedDeadRadiusX;
		config_server::Parameter<float> basicDFusedDeadRadiusY;
		config_server::Parameter<float> basicDFusedGainAllLat;
		config_server::Parameter<float> basicDFusedGainAllSag;
		config_server::Parameter<float> basicDFusedArmAngleX;
		config_server::Parameter<float> basicDFusedArmAngleY;
		config_server::Parameter<float> basicDFusedComShiftX;
		config_server::Parameter<float> basicDFusedComShiftY;
		config_server::Parameter<float> basicDFusedFootAngleX;
		config_server::Parameter<float> basicDFusedFootAngleY;
		config_server::Parameter<float> basicDFusedHipAngleX;
		config_server::Parameter<float> basicDFusedHipAngleY;
		 
		config_server::Parameter<bool>  basicIFusedEnabledLat;
		config_server::Parameter<bool>  basicIFusedEnabledSag;
		config_server::Parameter<int>   basicIFusedFilterN;
		config_server::Parameter<float> basicIFusedHalfLifeTime;
		config_server::Parameter<float> basicIFusedTimeToDecay;
		config_server::Parameter<float> basicIFusedTimeToFreeze;
		config_server::Parameter<float> basicIFusedGainAllLat;
		config_server::Parameter<float> basicIFusedGainAllSag;
		config_server::Parameter<float> basicIFusedArmAngleX;
		config_server::Parameter<float> basicIFusedArmAngleY;
		config_server::Parameter<float> basicIFusedComShiftX;
		config_server::Parameter<float> basicIFusedComShiftY;
		config_server::Parameter<float> basicIFusedFootAngleCX;
		config_server::Parameter<float> basicIFusedFootAngleCY;
		config_server::Parameter<float> basicIFusedFootAngleX;
		config_server::Parameter<float> basicIFusedFootAngleY;
		config_server::Parameter<float> basicIFusedHipAngleX;
		config_server::Parameter<float> basicIFusedHipAngleY;
		
		config_server::Parameter<bool>  basicGyroEnabledLat;
		config_server::Parameter<bool>  basicGyroEnabledSag;
		config_server::Parameter<int>   basicGyroFilterN;
		config_server::Parameter<float> basicGyroDeadRadiusX;
		config_server::Parameter<float> basicGyroDeadRadiusY;
		config_server::Parameter<float> basicGyroExpX;
		config_server::Parameter<float> basicGyroExpY;
		config_server::Parameter<float> basicGyroGainAllLat;
		config_server::Parameter<float> basicGyroGainAllSag;
		config_server::Parameter<float> basicGyroArmAngleX;
		config_server::Parameter<float> basicGyroArmAngleY;
		config_server::Parameter<float> basicGyroComShiftX;
		config_server::Parameter<float> basicGyroComShiftY;
		config_server::Parameter<float> basicGyroFootAngleX;
		config_server::Parameter<float> basicGyroFootAngleY;
		config_server::Parameter<float> basicGyroHipAngleX;
		config_server::Parameter<float> basicGyroHipAngleY;
		
		config_server::Parameter<bool>  basicTimingEnabled; // Note: To enable basic timing feedback, cmdUseCLTiming must also be true
		config_server::Parameter<float> basicTimingFeedDeadRad;
		config_server::Parameter<float> basicTimingGainSlowDown;
		config_server::Parameter<float> basicTimingGainSpeedUp;
		config_server::Parameter<float> basicTimingWeightFactor;
		 
		config_server::Parameter<float> basicComShiftXBuf;
		config_server::Parameter<float> basicComShiftXMax;
		config_server::Parameter<float> basicComShiftXMin;
		config_server::Parameter<bool>  basicComShiftXUseLimits;
		config_server::Parameter<float> basicComShiftYBuf;
		config_server::Parameter<float> basicComShiftYMax;
		config_server::Parameter<float> basicComShiftYMin;
		config_server::Parameter<bool>  basicComShiftYUseLimits;
		config_server::Parameter<float> basicFootAnglePhaseLen; // Determines how quickly the foot angle feedback fades in (and out) after a foot becomes the support foot
		///@}
		
		//! @name Capture step parameters
		///@{
		config_server::Parameter<bool>  cmdUseCLStepSize;       //!< @brief Boolean flag whether computed closed loop step sizes should be used to control the gait, or whether the internal gcv should just be controlled directly from the external gcv input (via maximum gcv acceleration rates)
		config_server::Parameter<bool>  cmdUseCLTiming;         //!< @brief Boolean flag whether computed closed loop step timing should be used to control the gait, or whether the timing should just remain fixed at the nominal OL frequency
		config_server::Parameter<bool>  cmdUseNonZeroZMP;       //!< @brief Boolean flag whether the target ZMPs calculated in the LimpModel should be used during walking (i.e. not artificially zeroed right after they are calculated)
		config_server::Parameter<bool>  cmdUseRXFeedback;       //!< @brief Boolean flag whether the capture gait feedback from RX to MX should be activated (i.e. whether adaptation is used)
		config_server::Parameter<bool>  cmdUseTXStepSize;       //!< @brief If `cmdUseCLStepSize` is true: Boolean flag whether the closed loop step size calculated by the TX model should be used, or just a fixed nominal CL step size
		config_server::Parameter<bool>  cmdUseTXTiming;         //!< @brief If `cmdUseCLTiming` is true: Boolean flag whether the closed loop timing calculated by the TX model should be used, or just a fixed nominal CL timing
		config_server::Parameter<float> mgC;
		config_server::Parameter<float> mgAlpha;
		config_server::Parameter<float> mgDelta;
		config_server::Parameter<float> mgOmega;
		config_server::Parameter<float> mgSigma;
		config_server::Parameter<float> mgGamma;
		config_server::Parameter<float> mgLatency;
		config_server::Parameter<float> mgZmpXMin;
		config_server::Parameter<float> mgZmpXMax;
		config_server::Parameter<float> mgZmpYMin;
		config_server::Parameter<float> mgZmpYMax;
		config_server::Parameter<float> mgComOffsetX;
		config_server::Parameter<float> mgComOffsetY;
		config_server::Parameter<float> mgFusedOffsetX;
		config_server::Parameter<float> mgFusedOffsetY;
		config_server::Parameter<float> mgMaxStepRadiusX;
		config_server::Parameter<float> mgMaxStepRadiusY;
		config_server::Parameter<float> mgMaxComPositionX;
		config_server::Parameter<float> mgPostStepStateCorrAng;
		config_server::Parameter<float> nsGain;
		config_server::Parameter<float> nsFusedXRangeLBnd;
		config_server::Parameter<float> nsFusedXRangeUBnd;
		config_server::Parameter<float> nsFusedYRangeLBnd;
		config_server::Parameter<float> nsFusedYRangeUBnd;
		config_server::Parameter<float> nsMaxAdaptation;
		config_server::Parameter<float> nsAdaptationGain;
		config_server::Parameter<float> nsStepNoiseTime;
		///@}
		
		// Manage callbacks for robot specifications
		void resetRobotSpecCallbacks() { m_robotSpecCallbacks.clear(); }
		void addRobotSpecCallback(const boost::function<void ()>& callback) { m_robotSpecCallbacks.push_back(callback); callback(); }
		void callRobotSpecCallbacks() { for(std::vector<boost::function<void ()> >::iterator it = m_robotSpecCallbacks.begin(); it != m_robotSpecCallbacks.end(); it++) (*it)(); }
		
	private:
		// Callback list for robot specifications
		std::vector<boost::function<void ()> > m_robotSpecCallbacks;
	};
}

#endif /* CAP_GAIT_CONFIG_H */
// EOF