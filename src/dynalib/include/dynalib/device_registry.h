// Device registry
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_DEVICE_REGISTRY_H
#define DYNALIB_DEVICE_REGISTRY_H

#include <dynalib/device.h>
#include <map>
#include <string>

namespace dynalib
{
	class DeviceRegistry
	{
	public:
		typedef std::map<uint16_t, Device*> DeviceMap;
		typedef std::map<std::string, Device*> NameDeviceMap;

		static void registerDevice(std::string name, uint16_t model_number, Device* device);

		static Device* device(uint16_t model_number);
		static Device* device(std::string name);
		static const char* deviceName(uint16_t model_number);

		static const std::map<uint16_t, Device*>& map();
		static const std::map<std::string, Device*>& name_map();
	};

	class DeviceRegisterer
	{
	public:
		DeviceRegisterer(std::string name, uint16_t model_number, Device* device)
		{
			DeviceRegistry::registerDevice(name, model_number, device);
		}
	};
}

#endif
