// Device registry
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <dynalib/device_registry.h>

#include <stdio.h>
#include <map>
#include <stdlib.h>

namespace dynalib
{

DeviceRegistry::DeviceMap* g_map = 0;
DeviceRegistry::NameDeviceMap* g_name_map = 0;

void DeviceRegistry::registerDevice(std::string name, uint16_t model_number, dynalib::Device* device)
{
	if(!g_map)
		g_map = new DeviceMap();
	if (!g_name_map)
		g_name_map = new NameDeviceMap();

	DeviceMap::iterator it = g_map->find(model_number);
	if(it != g_map->end())
	{
		fprintf(stderr, "dynalib: Tried to register device for model number %d twice!\n", model_number);
		abort();
	}

	g_map->insert(std::pair<uint16_t, Device*>(model_number, device));

	NameDeviceMap::iterator name_it = g_name_map->find(name);
	if(name_it != g_name_map->end())
	{
		fprintf(stderr, "dynalib: Tried to register device for device %s twice!\n", name.c_str());
		abort();
	}

	g_name_map->insert(std::pair<std::string, Device*>(name, device));
}

Device* DeviceRegistry::device(uint16_t model_number)
{
	if(!g_map)
		return 0;

	DeviceMap::iterator it = g_map->find(model_number);
	if(it == g_map->end())
		return 0;

	return it->second;
}

Device* DeviceRegistry::device(std::string name)
{
	if (!g_name_map)
		return 0;

	NameDeviceMap::iterator it = g_name_map->find(name);
	if(it == g_name_map->end())
		return 0;

	return it->second;
}


const char* DeviceRegistry::deviceName(uint16_t model_number)
{
	Device* dev = device(model_number);

	if(!dev)
		return 0;

	return dev->name();
}

const std::map< uint16_t, Device*>& DeviceRegistry::map()
{
	if(!g_map)
		g_map = new DeviceMap();

	return *g_map;
}

const std::map< std::string, Device* >& DeviceRegistry::name_map()
{
	if (!g_name_map)
		g_name_map = new NameDeviceMap();

	return *g_name_map;
}


}
