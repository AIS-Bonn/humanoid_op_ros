// Bench visualisation configuration parameters
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef BENCH_VIS_CONFIG_H
#define BENCH_VIS_CONFIG_H

// Includes
#include <config_server/parameter.h>

// Bench visualisation namespace
namespace bench_vis
{
	// Bench visualisation config class
	class BVConfig
	{
	public:
		// Constructor
		BVConfig()
		 : CONFIG_PARAM_PATH("/bench_vis/")
		 , forceLogging(CONFIG_PARAM_PATH + "forceLogging", false)
		{
		}

		// Constants
		const std::string CONFIG_PARAM_PATH;

		// Configuration parameters
		config_server::Parameter<bool> forceLogging;
	};
}

#endif
// EOF