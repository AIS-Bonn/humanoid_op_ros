// Print everything to stdout
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <dynalib/impl/debug_io.h>

#include <stdio.h>

namespace dynalib
{

void DebugIO::reset()
{
}

ReturnCode DebugIO::readByte(uint8_t* dest, uint16_t* timeout)
{
	*timeout = 0;
	return TIMEOUT;
}

ReturnCode DebugIO::startTransmit()
{
	printf(" SYNC\n");
	return SUCCESS;
}

bool DebugIO::writeByte(uint8_t data)
{
	printf("0x%02X ", data);
	return true;
}

bool DebugIO::writeSegment(uint8_t* data, uint8_t size)
{
	for(unsigned int i = 0; i < size; ++i)
		writeByte(data[i]);
	return true;
}

bool DebugIO::write(const uint8_t* data, uint8_t size)
{
	for(unsigned int i = 0; i < size; ++i)
		writeByte(data[i]);
	printf("\n");
	return true;
}

}
