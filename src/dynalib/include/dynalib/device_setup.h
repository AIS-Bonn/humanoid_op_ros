//Value storage for a Device in a setup specification
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef DEVICE_SETUP_H
#define DEVICE_SETUP_H

#include <string>
#include <dynalib/device.h>
#include <memory>
#include <vector>

namespace dynalib
{
	class DeviceSetup
	{
	public:
		struct Register
		{
		public:
			Register (Device::RegisterInfo info, unsigned int value)
			{
				this->info = info;
				this->value = value;
			};
			Device::RegisterInfo info;
			unsigned int value;
		};

		DeviceSetup();

		inline int id() {return m_devId;};
		inline Device* device() {return m_device;};
		int registerCount();
		const Register* reg(int index);

		bool setDevice(Device* device);
		bool setRegister(std::string name, unsigned int value);
		inline void setId(int id) {m_devId = id;};


	private:
		Device* m_device;
		int m_devId;

		std::vector<Register> m_registers;
	};
}

#endif