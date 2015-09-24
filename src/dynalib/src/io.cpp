// Dynalib IO interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <dynalib/io.h>

namespace dynalib
{

IO::~IO()
{
}

void IO::reset()
{
}

ReturnCode IO::startTransmit()
{
	return SUCCESS;
}

}
