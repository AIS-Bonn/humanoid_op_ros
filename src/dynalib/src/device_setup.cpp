//Value storage for a Device in a setup specification
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <dynalib/device_setup.h>

namespace dynalib
{

DeviceSetup::DeviceSetup() : m_device(0)
{
}

bool DeviceSetup::setDevice(Device* device)
{
	if(not device) return false;
	m_device = device;
	m_registers.clear();
	return true;
}

bool DeviceSetup::setRegister(std::string name, unsigned int value)
{
	if(not m_device) return false;

	unsigned int i;
	for(i = 0; i < m_device->registerCount(); i++)
	{
		if(!m_device->registerInfo(i).name) continue;
		std::string regName = m_device->registerInfo(i).name;
		if(name == regName)
		{
			if(m_device->registerInfo(i).reg_type != Device::RegisterInfo::EEPROM) return true;
			m_registers.push_back(Register(m_device->registerInfo(i), value));
			return true;
		}
	}
	return false;
}

int DeviceSetup::registerCount()
{
	return m_registers.size();
}

const DeviceSetup::Register* DeviceSetup::reg(int index)
{
	if ((size_t)index > m_registers.size())
		return 0;
	return &(m_registers[index]);
}

}