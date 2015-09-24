// Print everything to stdout
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_IMPL_DEBUG_IO_H
#define DYNALIB_IMPL_DEBUG_IO_H

#include <dynalib/io.h>

namespace dynalib
{

class DebugIO
{
public:
	virtual ReturnCode readByte(uint8_t* dest, uint16_t* timeout);
	virtual void reset();
	virtual bool writeByte(uint8_t data);
	virtual bool writeSegment(uint8_t* data, uint8_t size);
	virtual ReturnCode startTransmit();

	virtual bool write(const uint8_t* data, uint8_t size);
};

}

#endif
