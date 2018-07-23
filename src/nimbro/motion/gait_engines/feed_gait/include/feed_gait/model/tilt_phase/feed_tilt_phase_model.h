// Feedback gait tilt phase model
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_TILT_PHASE_MODEL_H
#define FEED_TILT_PHASE_MODEL_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/model/feed_model_base.h>
#include <feed_gait/model/tilt_phase/tripendulum_model.h>
#include <humanoid_kinematics/robot_kinematics.h>
#include <config_server/parameter.h>
#include <rc_utils/ell_bnd_integrator.h>
#include <rc_utils/limited_low_pass.h>
#include <rc_utils/mean_filter_nd.h>
#include <rc_utils/wlbf_filter_nd.h>
#include <rc_utils/slope_limiter.h>
#include <rc_utils/hold_filter.h>
#include <type_traits>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @namespace tilt_phase_model
	* 
	* @brief Tilt phase model namespace.
	**/
	namespace tilt_phase_model
	{
		/**
		* @class TiltPhaseModelConfig
		* 
		* @brief Tilt phase model configuration parameters class.
		**/
		class TiltPhaseModelConfig
		{
		private:
			// Constructor
			explicit TiltPhaseModelConfig(const CommonModelConfig& cmconfig)
				: TYPE_NAME(modelTypeName(MT_TILT_PHASE))
				, CONFIG_PARAM_PATH(MODEL_CONFIG_PARAM_PATH + TYPE_NAME + "/")

				, enable                (cmconfig.ENABLE_PATH + TYPE_NAME, false)
				, useActions            (cmconfig.USE_ACTIONS_PATH + TYPE_NAME, false)
				, useStepSize           (cmconfig.USE_STEPSIZE_PATH + TYPE_NAME, false)
				, useTiming             (cmconfig.USE_TIMING_PATH + TYPE_NAME, false)

				, enablePhasePX         (CONFIG_PARAM_PATH + "enable/phasePID/enablePhaseLatP", false)
				, enablePhasePY         (CONFIG_PARAM_PATH + "enable/phasePID/enablePhaseSagP", false)
				, enablePhaseDX         (CONFIG_PARAM_PATH + "enable/phasePID/enablePhaseLatD", false)
				, enablePhaseDY         (CONFIG_PARAM_PATH + "enable/phasePID/enablePhaseSagD", false)
				, enablePhaseIX         (CONFIG_PARAM_PATH + "enable/phasePID/enablePhaseLatI", false)
				, enablePhaseIY         (CONFIG_PARAM_PATH + "enable/phasePID/enablePhaseSagI", false)
				, enableArmTiltX        (CONFIG_PARAM_PATH + "enable/actions/enableArmTiltX", false)
				, enableArmTiltY        (CONFIG_PARAM_PATH + "enable/actions/enableArmTiltY", false)
				, enableFootTiltCtsX    (CONFIG_PARAM_PATH + "enable/actions/enableFootTiltCtsX", false)
				, enableFootTiltCtsY    (CONFIG_PARAM_PATH + "enable/actions/enableFootTiltCtsY", false)
				, enableFootTiltSuppX   (CONFIG_PARAM_PATH + "enable/actions/enableFootTiltSuppX", false)
				, enableFootTiltSuppY   (CONFIG_PARAM_PATH + "enable/actions/enableFootTiltSuppY", false)
				, enableHipShiftX       (CONFIG_PARAM_PATH + "enable/actions/enableHipShiftX", false)
				, enableHipShiftY       (CONFIG_PARAM_PATH + "enable/actions/enableHipShiftY", false)
				, enableLeanTiltX       (CONFIG_PARAM_PATH + "enable/actions/enableLeanTiltX", false)
				, enableLeanTiltY       (CONFIG_PARAM_PATH + "enable/actions/enableLeanTiltY", false)
				, enablePlaneNSX        (CONFIG_PARAM_PATH + "enable/actions/enablePlaneNSX", false)
				, enablePlaneNSY        (CONFIG_PARAM_PATH + "enable/actions/enablePlaneNSY", false)
				, enableSwingOutX       (CONFIG_PARAM_PATH + "enable/actions/enableSwingOutX", false)
				, enableSwingOutY       (CONFIG_PARAM_PATH + "enable/actions/enableSwingOutY", false)
				, enableHipHeightMax    (CONFIG_PARAM_PATH + "enable/actions/enableHipHeightMax", false)
				, enableStepSizeX       (CONFIG_PARAM_PATH + "enable/stepSize/enableStepSizeX", false)
				, enableStepSizeY       (CONFIG_PARAM_PATH + "enable/stepSize/enableStepSizeY", false)
				, enableStepSizeZ       (CONFIG_PARAM_PATH + "enable/stepSize/enableStepSizeZ", false)
				, enableTiming          (CONFIG_PARAM_PATH + "enable/timing/enableTiming", false)

				, biasArmTiltX          (CONFIG_PARAM_PATH + "bias/actions/biasArmTiltX", -0.5, 0.01, 0.5, 0.0)
				, biasArmTiltY          (CONFIG_PARAM_PATH + "bias/actions/biasArmTiltY", -0.5, 0.01, 0.5, 0.0)
				, biasFootTiltCtsX      (CONFIG_PARAM_PATH + "bias/actions/biasFootTiltCtsX", -0.5, 0.01, 0.5, 0.0)
				, biasFootTiltCtsY      (CONFIG_PARAM_PATH + "bias/actions/biasFootTiltCtsY", -0.5, 0.01, 0.5, 0.0)
				, biasFootTiltSuppX     (CONFIG_PARAM_PATH + "bias/actions/biasFootTiltSuppX", -0.5, 0.01, 0.5, 0.0)
				, biasFootTiltSuppY     (CONFIG_PARAM_PATH + "bias/actions/biasFootTiltSuppY", -0.5, 0.01, 0.5, 0.0)
				, biasHipShiftX         (CONFIG_PARAM_PATH + "bias/actions/biasHipShiftX", -0.5, 0.01, 0.5, 0.0)
				, biasHipShiftY         (CONFIG_PARAM_PATH + "bias/actions/biasHipShiftY", -0.5, 0.01, 0.5, 0.0)
				, biasLeanTiltX         (CONFIG_PARAM_PATH + "bias/actions/biasLeanTiltX", -0.5, 0.01, 0.5, 0.0)
				, biasLeanTiltY         (CONFIG_PARAM_PATH + "bias/actions/biasLeanTiltY", -0.5, 0.01, 0.5, 0.0)

				, limLeanTiltAMax       (CONFIG_PARAM_PATH + "limits/leanTilt/alphaMax", 0.0, 0.01, 1.0, 0.0)
				, limLeanTiltABuf       (CONFIG_PARAM_PATH + "limits/leanTilt/alphaBuf", 0.0, 0.005, 0.2, 0.0)
				, limFootTiltSuppAMax   (CONFIG_PARAM_PATH + "limits/footTiltSupp/alphaMax", 0.0, 0.01, 1.0, 0.0)
				, limFootTiltSuppABuf   (CONFIG_PARAM_PATH + "limits/footTiltSupp/alphaBuf", 0.0, 0.005, 0.2, 0.0)
				, limFootTiltCtsAMax    (CONFIG_PARAM_PATH + "limits/footTiltCts/alphaMax", 0.0, 0.01, 1.0, 0.0)
				, limFootTiltCtsABuf    (CONFIG_PARAM_PATH + "limits/footTiltCts/alphaBuf", 0.0, 0.005, 0.2, 0.0)
				, limArmTiltAMax        (CONFIG_PARAM_PATH + "limits/armTilt/alphaMax", 0.0, 0.01, 1.55, 0.0)
				, limArmTiltABuf        (CONFIG_PARAM_PATH + "limits/armTilt/alphaBuf", 0.0, 0.005, 0.4, 0.0)
				, limHipShiftXMax       (CONFIG_PARAM_PATH + "limits/hipShift/XMax", 0.0, 0.005, 0.4, 0.0)
				, limHipShiftYMax       (CONFIG_PARAM_PATH + "limits/hipShift/YMax", 0.0, 0.005, 0.4, 0.0)
				, limHipShiftBuf        (CONFIG_PARAM_PATH + "limits/hipShift/buf", 0.0, 0.005, 0.1, 0.0)
				, limSwingPlaneNSXMax   (CONFIG_PARAM_PATH + "limits/swingPlane/NSXMax", 0.0, 0.01, 1.0, 0.0)
				, limSwingPlaneNSYMax   (CONFIG_PARAM_PATH + "limits/swingPlane/NSYMax", 0.0, 0.01, 1.0, 0.0)
				, limSwingPlaneNSBuf    (CONFIG_PARAM_PATH + "limits/swingPlane/NSBuf", 0.0, 0.01, 0.5, 0.0)
				, limSwingOutXMax       (CONFIG_PARAM_PATH + "limits/swingOut/XMax", 0.0, 0.01, 1.0, 0.0)
				, limSwingOutYMax       (CONFIG_PARAM_PATH + "limits/swingOut/YMax", 0.0, 0.01, 0.6, 0.0)
				, limSwingOutBuf        (CONFIG_PARAM_PATH + "limits/swingOut/buf", 0.0, 0.005, 0.2, 0.0)

				, expPhaseXSinMag       (CONFIG_PARAM_PATH + "expected/phaseXSinMag", 0.0, 0.005, 0.2, 0.0)
				, expPhaseXSinPhase     (CONFIG_PARAM_PATH + "expected/phaseXSinPhase", -M_PI, 0.05, M_PI, 0.0)
				, expPhaseXSinOffset    (CONFIG_PARAM_PATH + "expected/phaseXSinOffset", -0.2, 0.005, 0.2, 0.0)
				, expPhaseYSinMag       (CONFIG_PARAM_PATH + "expected/phaseYSinMag", 0.0, 0.005, 0.2, 0.0)
				, expPhaseYSinPhase     (CONFIG_PARAM_PATH + "expected/phaseYSinPhase", -M_PI, 0.05, M_PI, 0.0)
				, expPhaseYSinOffset    (CONFIG_PARAM_PATH + "expected/phaseYSinOffset", -0.2, 0.005, 0.2, 0.0)

				, syncPhaseFilterN      (CONFIG_PARAM_PATH + "general/syncFilterN", 1, 1, 50, 25)

				, phasePMeanFilterN     (CONFIG_PARAM_PATH + "phasePD/meanFilterN", 1, 1, 20, 10)
				, phaseDDerivFilterN    (CONFIG_PARAM_PATH + "phasePD/derivFilterN", 1, 1, 50, 30)
				, phasePXDeadRadius     (CONFIG_PARAM_PATH + "phasePD/deadRadius/deadRadiusPLat", 0.0, 0.005, 0.2, 0.0)
				, phasePYDeadRadius     (CONFIG_PARAM_PATH + "phasePD/deadRadius/deadRadiusPSag", 0.0, 0.005, 0.2, 0.0)
				, phaseDXDeadRadius     (CONFIG_PARAM_PATH + "phasePD/deadRadius/deadRadiusDLat", 0.0, 0.005, 0.5, 0.0)
				, phaseDYDeadRadius     (CONFIG_PARAM_PATH + "phasePD/deadRadius/deadRadiusDSag", 0.0, 0.005, 0.5, 0.0)
				, phasePXGainAll        (CONFIG_PARAM_PATH + "phasePD/gainAllPD/gainAllPLat", 0.0, 0.05, 2.0, 0.0)
				, phasePYGainAll        (CONFIG_PARAM_PATH + "phasePD/gainAllPD/gainAllPSag", 0.0, 0.05, 2.0, 0.0)
				, phaseDXGainAll        (CONFIG_PARAM_PATH + "phasePD/gainAllPD/gainAllDLat", 0.0, 0.01, 0.5, 0.0)
				, phaseDYGainAll        (CONFIG_PARAM_PATH + "phasePD/gainAllPD/gainAllDSag", 0.0, 0.01, 0.5, 0.0)
				, phasePDGainArmTiltX   (CONFIG_PARAM_PATH + "phasePD/gainAction/gainArmTiltLat", 0.0, 0.05, 3.0, 0.0)
				, phasePDGainArmTiltY   (CONFIG_PARAM_PATH + "phasePD/gainAction/gainArmTiltSag", 0.0, 0.05, 3.0, 0.0)
				, phasePDGainFootTiltSX (CONFIG_PARAM_PATH + "phasePD/gainAction/gainFootTiltSuppLat", 0.0, 0.01, 1.0, 0.0)
				, phasePDGainFootTiltSY (CONFIG_PARAM_PATH + "phasePD/gainAction/gainFootTiltSuppSag", 0.0, 0.01, 1.0, 0.0)

				, phaseIXDevMax         (CONFIG_PARAM_PATH + "phaseI/devMax/devMaxLat", 0.0, 0.005, 0.2, 0.0)
				, phaseIYDevMax         (CONFIG_PARAM_PATH + "phaseI/devMax/devMaxSag", 0.0, 0.005, 0.2, 0.0)
				, phaseIGainIntegrand   (CONFIG_PARAM_PATH + "phaseI/gainIntegrand", 0.0, 0.05, 8.0, 0.0)
				, phaseIXIntegralMax    (CONFIG_PARAM_PATH + "phaseI/integral/integralMaxLat", 0.0, 0.01, 2.0, 0.0)
				, phaseIYIntegralMax    (CONFIG_PARAM_PATH + "phaseI/integral/integralMaxSag", 0.0, 0.01, 2.0, 0.0)
				, phaseIIntegralBuf     (CONFIG_PARAM_PATH + "phaseI/integral/integralBuf", 0.0, 0.01, 0.4, 0.0)
				, phaseIMeanFilterN     (CONFIG_PARAM_PATH + "phaseI/meanFilterN", 1, 5, 301, 1)
				, phaseIGainFootTiltCts (CONFIG_PARAM_PATH + "phaseI/gainAction/gainFootTiltCts", 0.0, 0.01, 0.3, 0.0)
				, phaseIGainHipShift    (CONFIG_PARAM_PATH + "phaseI/gainAction/gainHipShift", 0.0, 0.005, 0.4, 0.0)

				, planeNSMeanFilterN    (CONFIG_PARAM_PATH + "planeS/meanFilterN", 1, 1, 20, 10)
				, planeNSDeadRadiusX    (CONFIG_PARAM_PATH + "planeS/deadRadiusNSLat", 0.0, 0.005, 0.2, 0.0)
				, planeNSDeadRadiusY    (CONFIG_PARAM_PATH + "planeS/deadRadiusNSSag", 0.0, 0.005, 0.2, 0.0)
				, planeNSScale          (CONFIG_PARAM_PATH + "planeS/scaleNS", 0.0, 0.05, 2.0, 0.0)

				, swingHoldFilterN      (CONFIG_PARAM_PATH + "swingOut/holdFilterN", 1, 1, 50, 15)
				, crossingPhaseXL       (CONFIG_PARAM_PATH + "swingOut/crossingModel/phaseXLeft", -0.5, 0.01, -0.05, -0.3)
				, crossingPhaseXR       (CONFIG_PARAM_PATH + "swingOut/crossingModel/phaseXRight", 0.05, 0.01, 0.5, 0.3)
				, crossingCsqConst      (CONFIG_PARAM_PATH + "swingOut/crossingModel/CsqConst", 2.0, 0.1, 30.0, 10.0)
				, crossingEnergyMin     (CONFIG_PARAM_PATH + "swingOut/crossingToSwingX/energyMin", -0.1, 0.001, 0.0, 0.0)
				, crossingEnergyDeadRad (CONFIG_PARAM_PATH + "swingOut/crossingToSwingX/energyDeadRadius", 0.0, 0.0002, 0.02, 0.0)
				, crossingEnergyToSwingX(CONFIG_PARAM_PATH + "swingOut/crossingToSwingX/energyToSwingXScaler", 0.0, 0.1, 10.0, 0.0)
				, swingYGammaMaxAbs     (CONFIG_PARAM_PATH + "swingOut/swingOutY/gammaMaxAbs", 0.0, 0.01, 0.7, 0.0)
				, swingYGammaBuf        (CONFIG_PARAM_PATH + "swingOut/swingOutY/gammaBuf", 0.0, 0.005, 0.2, 0.0)
				, swingYPhaseDevXLow    (CONFIG_PARAM_PATH + "swingOut/swingOutY/phaseDevXLow", 0.0, 0.005, 0.3, 0.08)
				, swingYPhaseDevXHigh   (CONFIG_PARAM_PATH + "swingOut/swingOutY/phaseDevXHigh", 0.05, 0.005, 0.4, 0.15)

				, instBasedOnX          (CONFIG_PARAM_PATH + "hipHeightMax/instability/basedOnX", false)
				, instBasedOnY          (CONFIG_PARAM_PATH + "hipHeightMax/instability/basedOnY", false)
				, instLowPassTs         (CONFIG_PARAM_PATH + "hipHeightMax/instability/lowPassTs", 1.0, 0.1, 15.0, 6.0)
				, instLowPassMaxSlope   (CONFIG_PARAM_PATH + "hipHeightMax/instability/lowPassMaxSlope", 0.01, 0.001, 0.1, 0.05)
				, instForHipHeightNom   (CONFIG_PARAM_PATH + "hipHeightMax/instability/instForHipNom", 0.0, 0.01, 0.5, 0.2)
				, instForHipHeightMin   (CONFIG_PARAM_PATH + "hipHeightMax/instability/instForHipMin", 0.1, 0.01, 0.6, 0.4)
				, hipHeightMaxSlope     (CONFIG_PARAM_PATH + "hipHeightMax/hipHeightMaxSlope", 0.0, 0.005, 0.2, 0.05)

				, timingWeightFactor    (CONFIG_PARAM_PATH + "timing/weightFactor", 0.0, 0.05, 5.0, 1.0)
				, timingFeedDeadRadius  (CONFIG_PARAM_PATH + "timing/feedDeadRadius", 0.0, 0.005, 0.2, 0.0)
				, timingGainSpeedUp     (CONFIG_PARAM_PATH + "timing/gainSpeedUp", 0.0, 0.1, 20.0, 3.0)
				, timingGainSlowDown    (CONFIG_PARAM_PATH + "timing/gainSlowDown", 0.0, 0.1, 20.0, 5.0)

				, stepSTPMCrossingPhaseB(CONFIG_PARAM_PATH + "stepSize/sagTriPendModel/crossingPhaseYB", -0.7, 0.01, 0.0, -0.4)
				, stepSTPMCrossingPhaseF(CONFIG_PARAM_PATH + "stepSize/sagTriPendModel/crossingPhaseYF", 0.0, 0.01, 0.7, 0.4)
				, stepSTPMCsqConstBF    (CONFIG_PARAM_PATH + "stepSize/sagTriPendModel/CsqConstBF", 2.0, 0.1, 30.0, 10.0)
				, stepSTPMCsqConstM     (CONFIG_PARAM_PATH + "stepSize/sagTriPendModel/CsqConstM", 2.0, 0.1, 30.0, 10.0)
				, stepSCrossingEMinB    (CONFIG_PARAM_PATH + "stepSize/sagCrossingEnergy/energyMinB", -0.1, 0.001, 0.0, 0.0)
				, stepSCrossingEMinF    (CONFIG_PARAM_PATH + "stepSize/sagCrossingEnergy/energyMinF", -0.1, 0.001, 0.0, 0.0)
				, stepSCrossingEDeadRadB(CONFIG_PARAM_PATH + "stepSize/sagCrossingEnergy/energyDeadRadiusB", 0.0, 0.0002, 0.02, 0.0)
				, stepSCrossingEDeadRadF(CONFIG_PARAM_PATH + "stepSize/sagCrossingEnergy/energyDeadRadiusF", 0.0, 0.0002, 0.02, 0.0)
				, stepSCrossingEToGcvX  (CONFIG_PARAM_PATH + "stepSize/sagCrossingEnergy/energyToGcvXScaler", 0.0, 0.2, 30.0, 0.0)
				, stepGcvXHoldFilterN   (CONFIG_PARAM_PATH + "stepSize/gcvDeltaX/holdFilterN", 1, 1, 40, 10)
				, stepGcvDeltaXMaxAbs   (CONFIG_PARAM_PATH + "stepSize/gcvDeltaX/deltaMaxAbs", 0.0, 0.05, 1.5, 1.0)
				, stepGcvDeltaXBuf      (CONFIG_PARAM_PATH + "stepSize/gcvDeltaX/deltaBuf", 0.0, 0.01, 0.2, 0.1)
			{}

			// Ensure class remains a singleton
			TiltPhaseModelConfig(const TiltPhaseModelConfig&) = delete;
			TiltPhaseModelConfig& operator=(const TiltPhaseModelConfig&) = delete;

		public:
			// Get singleton instance of class
			static TiltPhaseModelConfig& getInstance() { static thread_local TiltPhaseModelConfig tpmconfig(CommonModelConfig::getInstance()); return tpmconfig; }

			// Constants
			const std::string TYPE_NAME;
			const std::string CONFIG_PARAM_PATH;

			// Common model parameters
			config_server::Parameter<bool>  enable;                 //!< @brief Boolean flag whether the tilt phase model should be enabled
			config_server::Parameter<bool>  useActions;             //!< @brief Boolean flag whether actions should be used from the tilt phase model
			config_server::Parameter<bool>  useStepSize;            //!< @brief Boolean flag whether step sizes should be used from the tilt phase model
			config_server::Parameter<bool>  useTiming;              //!< @brief Boolean flag whether timing should be used from the tilt phase model

			// Enable parameters
			config_server::Parameter<bool>  enablePhasePX;          //!< @brief Boolean flag whether the proportional X feedback terms should be enabled
			config_server::Parameter<bool>  enablePhasePY;          //!< @brief Boolean flag whether the proportional Y feedback terms should be enabled
			config_server::Parameter<bool>  enablePhaseDX;          //!< @brief Boolean flag whether the derivative X feedback terms should be enabled
			config_server::Parameter<bool>  enablePhaseDY;          //!< @brief Boolean flag whether the derivative Y feedback terms should be enabled
			config_server::Parameter<bool>  enablePhaseIX;          //!< @brief Boolean flag whether the integral X feedback terms should be enabled
			config_server::Parameter<bool>  enablePhaseIY;          //!< @brief Boolean flag whether the integral Y feedback terms should be enabled
			config_server::Parameter<bool>  enableArmTiltX;         //!< @brief Boolean flag whether the arm tilt X feedback should be enabled
			config_server::Parameter<bool>  enableArmTiltY;         //!< @brief Boolean flag whether the arm tilt Y feedback should be enabled
			config_server::Parameter<bool>  enableFootTiltCtsX;     //!< @brief Boolean flag whether the continuous foot tilt X feedback should be enabled
			config_server::Parameter<bool>  enableFootTiltCtsY;     //!< @brief Boolean flag whether the continuous foot tilt Y feedback should be enabled
			config_server::Parameter<bool>  enableFootTiltSuppX;    //!< @brief Boolean flag whether the support foot tilt X feedback should be enabled
			config_server::Parameter<bool>  enableFootTiltSuppY;    //!< @brief Boolean flag whether the support foot tilt Y feedback should be enabled
			config_server::Parameter<bool>  enableHipShiftX;        //!< @brief Boolean flag whether the hip shift X feedback should be enabled
			config_server::Parameter<bool>  enableHipShiftY;        //!< @brief Boolean flag whether the hip shift Y feedback should be enabled
			config_server::Parameter<bool>  enableLeanTiltX;        //!< @brief Boolean flag whether the lean tilt X feedback should be enabled
			config_server::Parameter<bool>  enableLeanTiltY;        //!< @brief Boolean flag whether the lean tilt Y feedback should be enabled
			config_server::Parameter<bool>  enablePlaneNSX;         //!< @brief Boolean flag whether the swing ground plane X relative to N feedback should be enabled
			config_server::Parameter<bool>  enablePlaneNSY;         //!< @brief Boolean flag whether the swing ground plane Y relative to N feedback should be enabled
			config_server::Parameter<bool>  enableSwingOutX;        //!< @brief Boolean flag whether the swing out X feedback component should be enabled
			config_server::Parameter<bool>  enableSwingOutY;        //!< @brief Boolean flag whether the swing out Y feedback component should be enabled
			config_server::Parameter<bool>  enableHipHeightMax;     //!< @brief Boolean flag whether the maximum normalised hip height should be enabled
			config_server::Parameter<bool>  enableStepSizeX;        //!< @brief Boolean flag whether the step size X command should be enabled
			config_server::Parameter<bool>  enableStepSizeY;        //!< @brief Boolean flag whether the step size Y command should be enabled
			config_server::Parameter<bool>  enableStepSizeZ;        //!< @brief Boolean flag whether the step size Z command should be enabled
			config_server::Parameter<bool>  enableTiming;           //!< @brief Boolean flag whether the timing feedback terms should be enabled

			// Bias parameters
			config_server::Parameter<float> biasArmTiltX;           //!< @brief Bias for the arm tilt X feedback
			config_server::Parameter<float> biasArmTiltY;           //!< @brief Bias for the arm tilt Y feedback
			config_server::Parameter<float> biasFootTiltCtsX;       //!< @brief Bias for the continuous foot tilt X feedback
			config_server::Parameter<float> biasFootTiltCtsY;       //!< @brief Bias for the continuous foot tilt Y feedback
			config_server::Parameter<float> biasFootTiltSuppX;      //!< @brief Bias for the support foot tilt X feedback
			config_server::Parameter<float> biasFootTiltSuppY;      //!< @brief Bias for the support foot tilt Y feedback
			config_server::Parameter<float> biasHipShiftX;          //!< @brief Bias for the hip shift X feedback (in units of inverse leg scale)
			config_server::Parameter<float> biasHipShiftY;          //!< @brief Bias for the hip shift Y feedback (in units of inverse leg scale)
			config_server::Parameter<float> biasLeanTiltX;          //!< @brief Bias for the lean tilt X feedback
			config_server::Parameter<float> biasLeanTiltY;          //!< @brief Bias for the lean tilt Y feedback

			// Limit parameters
			config_server::Parameter<float> limLeanTiltAMax;        //!< @brief Maximum allowed tilt angle component of the commanded lean tilt
			config_server::Parameter<float> limLeanTiltABuf;        //!< @brief Buffer for soft coercion of the tilt angle component of the commanded lean tilt
			config_server::Parameter<float> limFootTiltSuppAMax;    //!< @brief Maximum allowed tilt angle component of the support foot tilt
			config_server::Parameter<float> limFootTiltSuppABuf;    //!< @brief Buffer for soft coercion of the tilt angle component of the support foot tilt
			config_server::Parameter<float> limFootTiltCtsAMax;     //!< @brief Maximum allowed tilt angle component of the continuous foot tilt
			config_server::Parameter<float> limFootTiltCtsABuf;     //!< @brief Buffer for soft coercion of the tilt angle component of the continuous foot tilt
			config_server::Parameter<float> limArmTiltAMax;         //!< @brief Maximum allowed tilt angle component of the arm tilt
			config_server::Parameter<float> limArmTiltABuf;         //!< @brief Buffer for soft coercion of the tilt angle component of the arm tilt
			config_server::Parameter<float> limHipShiftXMax;        //!< @brief Maximum allowed hip shift X (elliptical limit, in units of inverse leg scale)
			config_server::Parameter<float> limHipShiftYMax;        //!< @brief Maximum allowed hip shift Y (elliptical limit, in units of inverse leg scale)
			config_server::Parameter<float> limHipShiftBuf;         //!< @brief Buffer for soft coercion of the hip shifts (in units of inverse leg scale)
			config_server::Parameter<float> limSwingPlaneNSXMax;    //!< @brief Maximum allowed swing ground plane X relative to N (elliptical limit)
			config_server::Parameter<float> limSwingPlaneNSYMax;    //!< @brief Maximum allowed swing ground plane Y relative to N (elliptical limit)
			config_server::Parameter<float> limSwingPlaneNSBuf;     //!< @brief Buffer for soft coercion of the swing ground plane relative to N
			config_server::Parameter<float> limSwingOutXMax;        //!< @brief Maximum allowed swing out X (elliptical limit)
			config_server::Parameter<float> limSwingOutYMax;        //!< @brief Maximum allowed swing out Y (elliptical limit)
			config_server::Parameter<float> limSwingOutBuf;         //!< @brief Buffer for soft coercion of the swing out

			// Expected waveform parameters
			config_server::Parameter<float> expPhaseXSinMag;        //!< @brief Magnitude of the expected sinusoidal phase X waveform
			config_server::Parameter<float> expPhaseXSinPhase;      //!< @brief Phase offset of the expected sinusoidal phase X waveform (increasing the sine phase offset shifts the expected waveform later in time, i.e. to the right)
			config_server::Parameter<float> expPhaseXSinOffset;     //!< @brief Offset of the expected sinusoidal phase X waveform
			config_server::Parameter<float> expPhaseYSinMag;        //!< @brief Magnitude of the expected sinusoidal phase Y waveform
			config_server::Parameter<float> expPhaseYSinPhase;      //!< @brief Phase offset of the expected sinusoidal phase Y waveform (increasing the sine phase offset shifts the expected waveform later in time, i.e. to the right)
			config_server::Parameter<float> expPhaseYSinOffset;     //!< @brief Offset of the expected sinusoidal phase Y waveform

			// General parameters
			config_server::Parameter<int>   syncPhaseFilterN;       //!< @brief Number of points to use for the synchronised tilt phase value/derivative filter

			// Proportional/derivative feedback parameters
			config_server::Parameter<int>   phasePMeanFilterN;      //!< @brief Number of points to use for the proportional feedback mean filter
			config_server::Parameter<int>   phaseDDerivFilterN;     //!< @brief Number of points to use for the derivative feedback derivative filter
			config_server::Parameter<float> phasePXDeadRadius;      //!< @brief Deadband radius for the proportional X feedback
			config_server::Parameter<float> phasePYDeadRadius;      //!< @brief Deadband radius for the proportional Y feedback
			config_server::Parameter<float> phaseDXDeadRadius;      //!< @brief Deadband radius for the derivative X feedback
			config_server::Parameter<float> phaseDYDeadRadius;      //!< @brief Deadband radius for the derivative Y feedback
			config_server::Parameter<float> phasePXGainAll;         //!< @brief Extra gain for all proportional X feedback
			config_server::Parameter<float> phasePYGainAll;         //!< @brief Extra gain for all proportional Y feedback
			config_server::Parameter<float> phaseDXGainAll;         //!< @brief Extra gain for all derivative X feedback
			config_server::Parameter<float> phaseDYGainAll;         //!< @brief Extra gain for all derivative Y feedback
			config_server::Parameter<float> phasePDGainArmTiltX;    //!< @brief Gain for the arm tilt X proportional/derivative feedback
			config_server::Parameter<float> phasePDGainArmTiltY;    //!< @brief Gain for the arm tilt Y proportional/derivative feedback
			config_server::Parameter<float> phasePDGainFootTiltSX;  //!< @brief Gain for the support foot tilt X proportional/derivative feedback
			config_server::Parameter<float> phasePDGainFootTiltSY;  //!< @brief Gain for the support foot tilt Y proportional/derivative feedback

			// Integral feedback parameters
			config_server::Parameter<float> phaseIXDevMax;          //!< @brief Maximum deviation tilt X for integration
			config_server::Parameter<float> phaseIYDevMax;          //!< @brief Maximum deviation tilt Y for integration
			config_server::Parameter<float> phaseIGainIntegrand;    //!< @brief Gain of the coerced deviation tilt before integration
			config_server::Parameter<float> phaseIXIntegralMax;     //!< @brief Maximum integrated deviation tilt value in the X direction
			config_server::Parameter<float> phaseIYIntegralMax;     //!< @brief Maximum integrated deviation tilt value in the Y direction
			config_server::Parameter<float> phaseIIntegralBuf;      //!< @brief Buffer for internal soft coercion of the integrated deviation tilt
			config_server::Parameter<int>   phaseIMeanFilterN;      //!< @brief Number of points to use for the integral feedback mean filter
			config_server::Parameter<float> phaseIGainFootTiltCts;  //!< @brief Gain for the continuous foot tilt integral feedback
			config_server::Parameter<float> phaseIGainHipShift;     //!< @brief Gain for the hip shift integral feedback (scales values that are in units of inverse leg scale)

			// Swing ground plane parameters
			config_server::Parameter<int>   planeNSMeanFilterN;     //!< @brief Number of points to use for the swing ground plane mean filter
			config_server::Parameter<float> planeNSDeadRadiusX;     //!< @brief Deadband radius for the swing ground plane X relative to N feedback
			config_server::Parameter<float> planeNSDeadRadiusY;     //!< @brief Deadband radius for the swing ground plane Y relative to N feedback
			config_server::Parameter<float> planeNSScale;           //!< @brief Scale factor for the swing ground plane relative to N feedback

			// Swing out parameters
			config_server::Parameter<int>   swingHoldFilterN;       //!< @brief Number of points to use for the swing out X hold filters
			config_server::Parameter<float> crossingPhaseXL;        //!< @brief Phase X at which crossing is assumed to occur when tipping out over the left foot
			config_server::Parameter<float> crossingPhaseXR;        //!< @brief Phase X at which crossing is assumed to occur when tipping out over the right foot
			config_server::Parameter<float> crossingCsqConst;       //!< @brief Pendulum constant (C^2, in inverse units of tip leg scale) of the crossing model (theoretically this value should be ~ g = 9.81)
			config_server::Parameter<float> crossingEnergyMin;      //!< @brief Minimum crossing energy above which swing out is activated
			config_server::Parameter<float> crossingEnergyDeadRad;  //!< @brief Smooth deadband radius to apply when the minimum crossing energy has been exceeded (to smooth out the use of swing out)
			config_server::Parameter<float> crossingEnergyToSwingX; //!< @brief Scale factor to go from deadbanded crossing energy to swing out X
			config_server::Parameter<float> swingYGammaMaxAbs;      //!< @brief Maximum allowed swing out tilt axis deviation from being pure lateral swing out
			config_server::Parameter<float> swingYGammaBuf;         //!< @brief Buffer for soft coercion of the swing out tilt axis deviation
			config_server::Parameter<float> swingYPhaseDevXLow;     //!< @brief Phase deviation X value at which to start fading in use of swing out Y
			config_server::Parameter<float> swingYPhaseDevXHigh;    //!< @brief Phase deviation X value at which to completely use swing out Y

			// Hip height max parameters
			config_server::Parameter<bool>  instBasedOnX;           //!< @brief Boolean flag whether the filtered phase deviation X contributes to the estimated level of instability
			config_server::Parameter<bool>  instBasedOnY;           //!< @brief Boolean flag whether the filtered phase deviation Y contributes to the estimated level of instability
			config_server::Parameter<float> instLowPassTs;          //!< @brief Settling time (90%) of the instability limited low pass filter
			config_server::Parameter<float> instLowPassMaxSlope;    //!< @brief Maximum slope of the instability limited low pass filter
			config_server::Parameter<float> instForHipHeightNom;    //!< @brief Largest instability value that maps to the nominal hip height as a maximum hip height (linear interpolation)
			config_server::Parameter<float> instForHipHeightMin;    //!< @brief Smallest instability value that maps to the minimum hip height as a maximum hip height (linear interpolation)
			config_server::Parameter<float> hipHeightMaxSlope;      //!< @brief Maximum rate of change of the commanded maximum normalised hip height

			// Timing feedback parameters
			config_server::Parameter<float> timingWeightFactor;     //!< @brief Factor that scales the timing feedback weight sinusoid prior to clipping (trims how much of the gait cycle is spent at full timing feedback weight)
			config_server::Parameter<float> timingFeedDeadRadius;   //!< @brief Deadband radius for the timing-weighted mean-filtered raw phase X feedback
			config_server::Parameter<float> timingGainSpeedUp;      //!< @brief Multiplicative gait frequency adjustment gain when speeding up the gait cycle
			config_server::Parameter<float> timingGainSlowDown;     //!< @brief Multiplicative gait frequency adjustment gain when slowing down the gait cycle

			// Step size feedback parameters
			config_server::Parameter<float> stepSTPMCrossingPhaseB; //!< @brief Backwards crossing tilt phase Y of the sagittal tripendulum model
			config_server::Parameter<float> stepSTPMCrossingPhaseF; //!< @brief Forwards crossing tilt phase Y of the sagittal tripendulum model
			config_server::Parameter<float> stepSTPMCsqConstBF;     //!< @brief Pendulum constant (C^2, in inverse units of tip leg scale) of the forwards and backwards pendulums in the sagittal tripendulum model (theoretically this value should be ~ g = 9.81)
			config_server::Parameter<float> stepSTPMCsqConstM;      //!< @brief Pendulum constant (C^2, in inverse units of tip leg scale) of the middle pendulum in the sagittal tripendulum model
			config_server::Parameter<float> stepSCrossingEMinB;     //!< @brief Minimum backwards crossing energy above which sagittal step size adjustments are activated
			config_server::Parameter<float> stepSCrossingEMinF;     //!< @brief Minimum forwards crossing energy above which sagittal step size adjustments are activated
			config_server::Parameter<float> stepSCrossingEDeadRadB; //!< @brief Smooth deadband radius to apply when the minimum backwards crossing energy has been exceeded
			config_server::Parameter<float> stepSCrossingEDeadRadF; //!< @brief Smooth deadband radius to apply when the minimum forwards crossing energy has been exceeded
			config_server::Parameter<float> stepSCrossingEToGcvX;   //!< @brief Scale factor to go from deadbanded crossing energy to gcv X
			config_server::Parameter<int>   stepGcvXHoldFilterN;    //!< @brief Number of points to use for the sagittal step size adjustment hold filters
			config_server::Parameter<float> stepGcvDeltaXMaxAbs;    //!< @brief Maximum allowed adjustment of gcv X based on the state of the sagittal tripendulum model
			config_server::Parameter<float> stepGcvDeltaXBuf;       //!< @brief Buffer for soft coercion of the allowed adjustment of gcv X
		};

		/**
		* @class TPMHelper
		*
		* @brief Helper class for the tilt phase model class.
		**/
		class TPMHelper
		{
		public:
			// PID feedback functions
			static Quat deviationTiltPhaseN(double phaseX, double phaseY, double expectedPhaseX, double expectedPhaseY, double fusedPitchN);

			// Swing ground plane feedback functions
			static rot_conv::TiltPhase2D swingGroundPlaneN(double phaseX, double phaseY, double expectedPhaseX, double expectedPhaseY, double fusedPitchN);
			static void swingGroundPlaneAction(const rot_conv::TiltPhase2D& PNS, double fusedPitchN, double& fusedPitchS, double& fusedRollS);
		};

		// State information struct
		struct StateInfo
		{
			// Constructor and reset
			StateInfo() { reset(); }
			void reset()
			{
				phase.setZero();
				expectedPhase.setZero();
				phaseDev.setZero();
				phaseDevFilt.setZero();
				phaseDevFiltLast.setZero();
				syncPhase.setZero();
				syncPhaseD.setZero();
			}

			// Data members
			Vec2 phase;            //!< @brief Tilt phase at the current instant
			Vec2 expectedPhase;    //!< @brief Expected tilt phase at the current instant
			Vec2 phaseDev;         //!< @brief Tilt phase deviation relative to N as derived from `TPMHelper::deviationTiltPhaseN`
			Vec2 phaseDevFilt;     //!< @brief Mean-filtered version of `phaseDev` (refer to `phasePMeanFilter`)
			Vec2 phaseDevFiltLast; //!< @brief Value of `phaseDevFilt` before being updated in the current cycle
			Vec2 syncPhase;        //!< @brief Time-synchronised filtered tilt phase at the current instant
			Vec2 syncPhaseD;       //!< @brief Time-synchronised filtered tilt phase derivative at the current instant
		};

		/**
		* @class FeedTiltPhaseModel
		*
		* @brief Feedback gait tilt phase model class.
		**/
		template<class Kinematics> class FeedTiltPhaseModel : public FeedModelBase
		{
		private:
			// Static assertions
			static_assert(std::is_base_of<humanoid_kinematics::RobotKinematics, Kinematics>::value, "The Kinematics template parameter must derive from humanoid_kinematics::RobotKinematics");

		public:
			// Constructor/destructor
			explicit FeedTiltPhaseModel(FeedPlotManager* PM) : FeedModelBase(PM), tpmconfig(TiltPhaseModelConfig::getInstance()) { GaitPhaseInfo phaseInfo; ModelInput modelInput; resetMembers(phaseInfo, modelInput); }
			virtual ~FeedTiltPhaseModel() = default;

			// Configuration parameters
			const TiltPhaseModelConfig& tpmconfig;

			// Kinematics
			const Kinematics K;

			// Reset function
			virtual void reset(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput) override { FeedModelBase::reset(phaseInfo, modelInput); resetMembers(phaseInfo, modelInput); }

			// Update function
			virtual void update(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput) override;

		private:
			// Reset members function
			void resetMembers(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput);

			// General feedback
			void resetGeneral();
			void configGeneral();
			rc_utils::MeanFilter2D phasePMeanFilter;
			rc_utils::WLBFFilter2D syncPhaseFilter;

			// PID feedback
			void resetPID();
			void configPID();
			void updatePID(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo);
			rc_utils::WLBFFilter2D phaseDDerivFilter;
			rc_utils::EllBndIntegrator2D phaseIEllBndInteg;
			rc_utils::MeanFilter2D phaseIMeanFilter;

			// Swing ground plane feedback
			void resetPlaneS();
			void configPlaneS();
			void updatePlaneS(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo);
			rc_utils::MeanFilter2D planeNSMeanFilter;

			// Swing out feedback
			void resetSwingOut();
			void configSwingOut();
			void updateSwingOut(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo);
			rc_utils::HoldMinFilter swingXLHoldFilter;
			rc_utils::HoldMaxFilter swingXRHoldFilter;

			// Hip height maximum feedback
			void resetHipMax();
			void configHipMax(const ModelInput& modelInput);
			void updateHipMax(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo);
			rc_utils::LimitedLowPass instLowPassFilter;
			rc_utils::SlopeLimiter hipMaxSlopeLimiter;

			// Timing feedback
			void resetTiming();
			void configTiming();
			void updateTiming(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo);

			// Step size feedback
			void resetStepSize();
			void configStepSize();
			void updateStepSize(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput, const StateInfo& stateInfo);
			tripendulum::TriPendModel stepTriPendModelSag;
			rc_utils::HoldMinFilter stepGcvXBHoldFilter;
			rc_utils::HoldMaxFilter stepGcvXFHoldFilter;
		};
	}
}

// Include implementations that should occur in the header
#include <feed_gait/model/tilt_phase/feed_tilt_phase_model_impl.h>

#endif
// EOF