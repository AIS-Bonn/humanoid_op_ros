// Hardware interface base class
// Author: David Schwarz <david.schwarz@uni-bonn.de>

#include <robotcontrol/hw/hardwareinterface.h>

namespace robotcontrol
{

bool HardwareInterface::emergencyStopActive()
{
	return false;
}

}