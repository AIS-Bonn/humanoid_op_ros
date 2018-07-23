// Feedback gait trivial model
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_TRIVIAL_MODEL_H
#define FEED_TRIVIAL_MODEL_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/model/feed_model_base.h>
#include <config_server/parameter.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @namespace trivial_model
	* 
	* @brief Trivial model namespace.
	**/
	namespace trivial_model
	{
		/**
		* @class TrivialModelConfig
		* 
		* @brief Trivial model configuration parameters class.
		**/
		class TrivialModelConfig
		{
		private:
			// Constructor
			explicit TrivialModelConfig(const CommonModelConfig& cmconfig)
				: TYPE_NAME(modelTypeName(MT_TRIVIAL))
				, CONFIG_PARAM_PATH(MODEL_CONFIG_PARAM_PATH + TYPE_NAME + "/")

				, enable           (cmconfig.ENABLE_PATH + TYPE_NAME, false)
				, useActions       (cmconfig.USE_ACTIONS_PATH + TYPE_NAME, false)
				, useStepSize      (cmconfig.USE_STEPSIZE_PATH + TYPE_NAME, false)
				, useTiming        (cmconfig.USE_TIMING_PATH + TYPE_NAME, false)

				, gcvX             (CONFIG_PARAM_PATH + "gcv/x", -1.0, 0.01, 1.0, 0.0)
				, gcvY             (CONFIG_PARAM_PATH + "gcv/y", -1.0, 0.01, 1.0, 0.0)
				, gcvZ             (CONFIG_PARAM_PATH + "gcv/z", -1.0, 0.01, 1.0, 0.0)

				, fusedPitchS      (CONFIG_PARAM_PATH + "fusedPitchS", -0.5, 0.005, 0.5, 0.0)
				, fusedRollS       (CONFIG_PARAM_PATH + "fusedRollS", -0.5, 0.005, 0.5, 0.0)
				, footTiltCtsAlpha (CONFIG_PARAM_PATH + "footTiltCts/alpha", -0.8, 0.01, 0.8, 0.0)
				, footTiltCtsGamma (CONFIG_PARAM_PATH + "footTiltCts/gamma", -M_PI, 0.05, M_PI, 0.0)
				, footTiltSuppAlpha(CONFIG_PARAM_PATH + "footTiltSupp/alpha", -0.8, 0.01, 0.8, 0.0)
				, footTiltSuppGamma(CONFIG_PARAM_PATH + "footTiltSupp/gamma", -M_PI, 0.05, M_PI, 0.0)
				, swingOutAlpha    (CONFIG_PARAM_PATH + "swingOut/alpha", -0.8, 0.01, 0.8, 0.0)
				, swingOutGamma    (CONFIG_PARAM_PATH + "swingOut/gamma", -M_PI, 0.05, M_PI, 0.0)
				, leanTiltAlpha    (CONFIG_PARAM_PATH + "leanTilt/alpha", -0.8, 0.01, 0.8, 0.0)
				, leanTiltGamma    (CONFIG_PARAM_PATH + "leanTilt/gamma", -M_PI, 0.05, M_PI, 0.0)
				, hipShiftX        (CONFIG_PARAM_PATH + "hipShift/x", -0.3, 0.005, 0.3, 0.0)
				, hipShiftY        (CONFIG_PARAM_PATH + "hipShift/y", -0.3, 0.005, 0.3, 0.0)
				, hipHeightMax     (CONFIG_PARAM_PATH + "hipHeightMax", 0.2, 0.01, 1.2, 1.2)
				, armTiltAlpha     (CONFIG_PARAM_PATH + "armTilt/alpha", -1.5, 0.01, 1.5, 0.0)
				, armTiltGamma     (CONFIG_PARAM_PATH + "armTilt/gamma", -M_PI, 0.05, M_PI, 0.0)
			{}

			// Ensure class remains a singleton
			TrivialModelConfig(const TrivialModelConfig&) = delete;
			TrivialModelConfig& operator=(const TrivialModelConfig&) = delete;

		public:
			// Get singleton instance of class
			static const TrivialModelConfig& getInstance() { static thread_local TrivialModelConfig tmconfig(CommonModelConfig::getInstance()); return tmconfig; }

			// Constants
			const std::string TYPE_NAME;
			const std::string CONFIG_PARAM_PATH;

			// Common model parameters
			config_server::Parameter<bool>  enable;            //!< @brief Boolean flag whether the trivial model should be enabled
			config_server::Parameter<bool>  useActions;        //!< @brief Boolean flag whether actions should be used from the trivial model
			config_server::Parameter<bool>  useStepSize;       //!< @brief Boolean flag whether step sizes should be used from the trivial model
			config_server::Parameter<bool>  useTiming;         //!< @brief Boolean flag whether timing should be used from the trivial model

			// Step size command parameters
			config_server::Parameter<float> gcvX;              //!< @brief Fixed value to command for the gait command x-velocity
			config_server::Parameter<float> gcvY;              //!< @brief Fixed value to command for the gait command y-velocity
			config_server::Parameter<float> gcvZ;              //!< @brief Fixed value to command for the gait command z-velocity

			// Actions command parameters
			config_server::Parameter<float> fusedPitchS;       //!< @brief Fixed value to command for the swing ground plane fused pitch
			config_server::Parameter<float> fusedRollS;        //!< @brief Fixed value to command for the swing ground plane fused roll
			config_server::Parameter<float> footTiltCtsAlpha;  //!< @brief Fixed value to command for the continuous foot tilt tilt angle
			config_server::Parameter<float> footTiltCtsGamma;  //!< @brief Fixed value to command for the continuous foot tilt absolute tilt axis angle
			config_server::Parameter<float> footTiltSuppAlpha; //!< @brief Fixed value to command for the support foot tilt tilt angle
			config_server::Parameter<float> footTiltSuppGamma; //!< @brief Fixed value to command for the support foot tilt absolute tilt axis angle
			config_server::Parameter<float> swingOutAlpha;     //!< @brief Fixed value to command for the swing out tilt angle
			config_server::Parameter<float> swingOutGamma;     //!< @brief Fixed value to command for the swing out absolute tilt axis angle
			config_server::Parameter<float> leanTiltAlpha;     //!< @brief Fixed value to command for the lean tilt tilt angle
			config_server::Parameter<float> leanTiltGamma;     //!< @brief Fixed value to command for the lean tilt absolute tilt axis angle
			config_server::Parameter<float> hipShiftX;         //!< @brief Fixed value to command for the hip shift in the x-axis
			config_server::Parameter<float> hipShiftY;         //!< @brief Fixed value to command for the hip shift in the y-axis
			config_server::Parameter<float> hipHeightMax;      //!< @brief Fixed value to command for the maximum hip height
			config_server::Parameter<float> armTiltAlpha;      //!< @brief Fixed value to command for the arm tilt tilt angle
			config_server::Parameter<float> armTiltGamma;      //!< @brief Fixed value to command for the arm tilt absolute tilt axis angle
		};

		/**
		* @class FeedTrivialModel
		*
		* @brief Feedback gait trivial model class.
		**/
		class FeedTrivialModel : public FeedModelBase
		{
		public:
			// Constructor/destructor
			explicit FeedTrivialModel(FeedPlotManager* PM) : FeedModelBase(PM), tmconfig(TrivialModelConfig::getInstance()) { GaitPhaseInfo phaseInfo; ModelInput modelInput; resetMembers(phaseInfo, modelInput); }
			virtual ~FeedTrivialModel() = default;

			// Configuration parameters
			const TrivialModelConfig& tmconfig;

			// Reset function
			virtual void reset(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput) override { FeedModelBase::reset(phaseInfo, modelInput); resetMembers(phaseInfo, modelInput); }

			// Update function
			virtual void update(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput) override;

		private:
			// Reset members function
			void resetMembers(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput);
		};
	}
}

#endif
// EOF