
#define CATCH_CONFIG_RUNNER
#include <catch_ros/catch.hpp>

#include "ros_junit_reporter.h"

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

int main( int argc, char* const argv[] )
{
	Catch::Session session;

	int returnCode = session.applyCommandLine( argc, argv );
	if( returnCode != 0 ) // Indicates a command line error
		return returnCode;

	// The catkin scripts calling tests do not create the output directory for
	// us :-(
	if(!session.configData().outputFilename.empty())
	{
		fs::path outputPath = session.configData().outputFilename;

		fs::path outputDir = outputPath.parent_path();
		if(!fs::exists(outputDir))
		{
			fs::create_directories(outputDir);
		}
	}

	return session.run();
}
