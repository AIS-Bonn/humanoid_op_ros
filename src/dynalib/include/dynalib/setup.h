//Setup specification for different robot configurations
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef SETUP_H
#define SETUP_H

#include <vector>
#include <string>

#if YAML_NEW_API
#include <yaml-cpp/yaml.h>
#else
#include <yaml-cpp/node.h>
#include <yaml-cpp/iterator.h>
#endif

#include <dynalib/device_setup.h>

namespace dynalib
{
	class Setup
	{
	public:
			Setup(std::string filename)
			 : filename(filename)
			{};

			bool parseFile();
			bool nextDevice();

			std::string deviceType();
			std::string deviceName();
			int deviceId();

			bool writeSetup(DeviceSetup& deviceSetup);


			void setFilename(std::string filename)
			{this->filename = filename;};
	private:
		std::string filename;

		const YAML::Node& currentDevice();
		YAML::Node setup;
#if YAML_NEW_API
		YAML::iterator deviceIt;
#else
		YAML::Iterator deviceIt;
#endif
	};
}

#endif