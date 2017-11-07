// Injects the required configuration into RViz using LD_PRELOAD hooking
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Include for dlsym
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <dlfcn.h>

// Includes
#include <QString>
#include <iostream>

// RViz namespace
namespace rviz
{
	// VisualizationFrame class
	class VisualizationFrame
	{
	public:
		// Initialize function to hook
		void initialize(const QString& display_config_file = "");
	};

	// Definition of new initialize function
	void VisualizationFrame::initialize(const QString& display_config_file)
	{
		// Static variable to store a pointer to the original initialize function
		typedef void (VisualizationFrame::*FuncType)(const QString& display_config_file);
		static FuncType origFunc = 0;

		// Get a pointer to the original initialize function if we have not already
		if(origFunc == 0)
		{
			void* tmpPtr = dlsym(RTLD_NEXT, "_ZN4rviz18VisualizationFrame10initializeERK7QString");
			if(!tmpPtr)
			{
				std::cerr << "\033[31mrviz_config_injector: Failed to find the original 'void initialize(const QString&)' function!\033[m" << std::endl;
				std::abort();
			}
			memcpy(&origFunc, &tmpPtr, sizeof(void*));
		}

		// Retrieve the environment variable specifying which config file to use
		const char* env = getenv("RVIZ_DISPLAY_CONFIG_FILE");

		// Decide on the desired display configuration file
		QString new_config_file = display_config_file;
		if(env && strlen(env) != 0 && new_config_file.isEmpty())
			new_config_file = QString::fromUtf8(env);

		// Pass the work on to the original initialize function
		(this->*origFunc)(new_config_file);
	}
}
// EOF