// Joint interface base class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ijointinterface.h"

#include "dxljointinterface.h"

const char* IJointInterface::CONTROL_TYPE_NAMES[] = {
	"Position",
	"Velocity",
	"Torque"
};

IJointInterface::IJointInterface(int id)
 : m_id(id)
{
}

IJointInterface::~IJointInterface()
{
}

IJointInterface* IJointInterface::create()
{
	return new DXLJointInterface;
}

void IJointInterface::setPValue(int value)
{
}

double IJointInterface::currentTemperature()
{
	return 0;
}


