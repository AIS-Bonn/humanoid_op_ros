// Feedback gait model base class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FEED_MODEL_BASE_H
#define FEED_MODEL_BASE_H

// Includes
#include <feed_gait/feed_common.h>
#include <feed_gait/feed_plot.h>
#include <feed_gait/model/feed_model_common.h>
#include <config_server/parameter.h>

// Feedback gait namespace
namespace feed_gait
{
	/**
	* @class CommonModelConfig
	* 
	* @brief Common model configuration parameters class.
	**/
	class CommonModelConfig
	{
	private:
		// Constructor
		CommonModelConfig()
			: CONFIG_PARAM_PATH(MODEL_CONFIG_PARAM_PATH + "common/")
			, ENABLE_PATH(CONFIG_PARAM_PATH + "enable/")
			, USE_ACTIONS_PATH(CONFIG_PARAM_PATH + "useActions/")
			, USE_STEPSIZE_PATH(CONFIG_PARAM_PATH + "useStepSize/")
			, USE_TIMING_PATH(CONFIG_PARAM_PATH + "useTiming/")
		{}

		// Ensure class remains a singleton
		CommonModelConfig(const CommonModelConfig&) = delete;
		CommonModelConfig& operator=(const CommonModelConfig&) = delete;

	public:
		// Get singleton instance of class
		static const CommonModelConfig& getInstance() { static thread_local CommonModelConfig cmconfig; return cmconfig; }

		// Constants
		const std::string CONFIG_PARAM_PATH;
		const std::string ENABLE_PATH;
		const std::string USE_ACTIONS_PATH;
		const std::string USE_STEPSIZE_PATH;
		const std::string USE_TIMING_PATH;
	};

	/**
	* @class FeedModelBase
	*
	* @brief Feedback gait model base class.
	**/
	class FeedModelBase
	{
	public:
		// Constructor/destructor
		explicit FeedModelBase(FeedPlotManager* PM) : cmconfig(CommonModelConfig::getInstance()), m_PM(PM) { GaitPhaseInfo phaseInfo; ModelInput modelInput; resetMembers(phaseInfo, modelInput); }
		virtual ~FeedModelBase() = default;

		// Configuration parameters
		const CommonModelConfig& cmconfig;

		// Reset functions
		void reset() { GaitPhaseInfo phaseInfo; ModelInput modelInput; reset(phaseInfo, modelInput); }
		virtual void reset(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput) { resetMembers(phaseInfo, modelInput); } // This function should reset the entire class and all base classes

		// Update functions
		void callUpdate(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput);
		virtual void update(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput) = 0; // This function should update the model output struct based on the gait phase information, model inputs and current state

		// Output function
		ModelOutput output() const { return m_out; }

		// Timing command functions
		bool useTiming() const { return m_out.useTiming; }
		TimingCommand getTiming() const { return m_out.timingCmd; }

		// Step size command functions
		bool useStepSize() const { return m_out.useStepSize; }
		StepSizeCommand getStepSize() const { return m_out.stepSizeCmd; }

		// Actions command functions
		ACFlag useActions() const { return m_out.useActions; }
		ActionsCommand getActions() const { return m_out.actionsCmd; }
		void writeActions(ActionsCommand& actionsCmd, ACFlag& useActions) const { actionsCmd.setFrom(m_out.actionsCmd, m_out.useActions); useActions.setFlags(m_out.useActions); }

	protected:
		// Plot manager
		FeedPlotManager* const m_PM;

		// Model output
		ModelOutput m_out;

	private:
		// Reset members function
		void resetMembers(const GaitPhaseInfo& phaseInfo, const ModelInput& modelInput) { m_inited = false; m_out.reset(); }

		// Initialised flag
		bool m_inited;
	};

	// Typedefs
	typedef std::shared_ptr<FeedModelBase> FeedModelBasePtr;
}

#endif
// EOF