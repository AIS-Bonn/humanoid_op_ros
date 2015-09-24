// IO implementation for linux serial devices
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_IMPL_LINUX_SERIAL_H
#define DYNALIB_IMPL_LINUX_SERIAL_H

#include "../io.h"

#include <boost/circular_buffer.hpp>

#include <string.h>

namespace dynalib
{

class LinuxSerial
{
public:
	LinuxSerial();
	~LinuxSerial();

	bool init(const char* device, unsigned int baud = 1000000UL);

	inline void reset()
	{
		m_writeSize = 0;
	}
	ReturnCode startTransmit();

	ReturnCode readByte(uint8_t* dest, uint16_t* timeout);
	inline __attribute__((always_inline)) bool writeByte(uint8_t data)
	{
		if(m_writeSize >= sizeof(m_writeBuf))
			return false;

		m_writeBuf[m_writeSize] = data;
		m_writeSize++;

		return true;
	}

	bool writeSegment(uint8_t* data, uint8_t size)
	{
		if(m_writeSize + size > sizeof(m_writeBuf))
			return false;

		memcpy(m_writeBuf + m_writeSize, data, size);
		m_writeSize += size;

		return true;
	}

	bool write(const uint8_t* data, uint8_t size);

	void setDebugEnabled(bool on);
private:
	void debug(const char* prefix, const uint8_t* data, uint8_t len);

	int m_fd;
	unsigned int m_writeSize;
	uint8_t m_writeBuf[512];
	uint8_t m_readBuf[128];
	unsigned int m_readSize;
	unsigned int m_readIdx;
	bool m_debug;
	unsigned int m_baudFactor;
};

}

#endif
