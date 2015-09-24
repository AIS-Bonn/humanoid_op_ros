//Setup specification for different robot configurations
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <dynalib/setup.h>

#include <fstream>

#if !YAML_NEW_API
#include <yaml-cpp/parser.h>
#endif

#include <boost/iterator/iterator_concepts.hpp>
#include <dynalib/device_registry.h>


namespace dynalib
{

bool Setup::parseFile()
{
	std::ifstream fs;
	fs.open(filename);
	if (not fs.is_open())
	{
		return false;
	}

	try
	{
#if YAML_NEW_API
		setup = YAML::Load(fs);
#else
		YAML::Parser p(fs);
		p.GetNextDocument(setup);
#endif
	}
	catch (YAML::ParserException& e)
	{
		return false;
	}
	catch (YAML::RepresentationException& e)
	{
		return false;
	}
	deviceIt = setup.begin();
	return true;
}

bool Setup::nextDevice()
{
	deviceIt++;
	if (deviceIt == setup.end())
		return false;
	return true;
}

const YAML::Node& Setup::currentDevice()
{
	return *deviceIt;
}

bool Setup::writeSetup(dynalib::DeviceSetup& deviceSetup)
{
	const YAML::Node& device = currentDevice();
	
	std::string type = "";
	try
	{
#if YAML_NEW_API
		type = device["type"].as<std::string>();
#else
		device["type"] >> type;
#endif
	}
	catch(YAML::ParserException& e)
	{
		fprintf(stderr, "Error while creating device.\n");
		fprintf(stderr, "No type for device defined.\n");
		return false;
	}
	Device* dev = DeviceRegistry::device(type);
	if (not dev)
	{
		fprintf(stderr, "Device type %s is not known.\n", type.c_str());
		return false;
	}
	deviceSetup.setDevice(dev);

	int id;
	try
	{
#if YAML_NEW_API
		id = device["id"].as<int>();
#else
		device["id"] >> id;
#endif
	}
	catch(YAML::ParserException& e)
	{
		fprintf(stderr, "Error while creating device.\n");
		fprintf(stderr, "No ID for device defined.\n");
		return false;
	}
	deviceSetup.setId(id);

#if YAML_NEW_API
	YAML::const_iterator it;
#else
	YAML::Iterator it;
#endif
	for (it = device.begin(); it != device.end(); ++it)
	{
		std::string key;
		try
		{
#if YAML_NEW_API
			key = it->first.as<std::string>();
#else
			it.first() >> key;
#endif
		}
		catch (YAML::ParserException& e)
		{
			fprintf(stderr, "Yaml exception while parsing device: %s\n", e.what());
			fprintf(stderr, "Key type does not match.\n");
			return false;
		}

		if (key == "type" || key == "id" || key == "name")
			continue;

		unsigned int value;
		try
		{
#if YAML_NEW_API
			value = it->second.as<unsigned int>();
#else
			it.second() >> value;
#endif
		}
		catch (YAML::ParserException& e)
		{
			fprintf(stderr, "Yaml exception while parsing device: %s\n", e.what());
			return false;
		}
		if (not deviceSetup.setRegister(key, value))
		{
			fprintf(stderr, "Could not set register %s\n", key.c_str());
			return false;
		}
	}

	if (not deviceSetup.device())
	{
		return false;
	}

	return true;
}

int Setup::deviceId()
{
	int id;
	try
	{
#if YAML_NEW_API
		id = currentDevice()["id"].as<int>();
#else
		currentDevice()["id"] >> id;
#endif
	}
	catch (YAML::ParserException& e)
	{
		id = -1;
	}
	return id; // TODO: Temporary warning fix, feel free to scrap when continuing working on this function
}

std::string Setup::deviceName()
{
	std::string name;
	try
	{
#if YAML_NEW_API
		name = currentDevice()["name"].as<std::string>();
#else
		currentDevice()["name"] >> name;
#endif
	}
	catch(YAML::ParserException& e)
	{
		name = "";
	}
	return name;
}

std::string Setup::deviceType()
{
	std::string type;
	try
	{
#if YAML_NEW_API
		type = currentDevice()["type"].as<std::string>();
#else
		currentDevice()["type"] >> type;
#endif
	}
	catch (YAML::ParserException& e)
	{
		type = "";
	}
	return type;
}


}