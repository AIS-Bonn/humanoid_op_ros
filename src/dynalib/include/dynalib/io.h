// Dynalib IO interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_IO_H
#define DYNALIB_IO_H

#include "return_code.h"

#include <stdint.h>

namespace dynalib
{

class IO
{
public:
	virtual ~IO();

	virtual void reset();
	virtual ReturnCode startTransmit();

	virtual bool writeByte(uint8_t data) = 0;
	virtual bool writeSegment(uint8_t* data, uint8_t size) = 0;
	virtual bool write(const uint8_t* data, uint8_t size) = 0;

	virtual ReturnCode readByte(uint8_t* dest, uint16_t* timeout) = 0;
};

}

#endif
