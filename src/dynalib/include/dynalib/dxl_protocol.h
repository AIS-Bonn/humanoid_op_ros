// Dynamixel protocol
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_DXL_PROTOCOL_H
#define DYNALIB_DXL_PROTOCOL_H

#include "io.h"
#include "util/applicative_stream.h"
#include "static_buffer.h"

#include <endian.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <initializer_list>

#define COMMS_TIMEOUT  12000

namespace dynalib
{

template<class Buffer = StaticBuffer<256> >
class BulkReadBase
{
public:
	BulkReadBase();

	template<class Region>
	bool addRead(uint8_t id);

	void finalize();

	inline unsigned int requestSize() const
	{ return 7 + m_numParamBytes; }

	inline const uint8_t* request() const
	{ return m_buf.data(); }
private:
	Buffer m_buf;
	Buffer m_recvBuf;

	uint8_t m_numParamBytes;
};

typedef BulkReadBase<> BulkRead;

template<class MyIO = dynalib::IO>
class DXLProtocol
{
public:
	class Device
	{
	public:
		bool ping();
		bool reset();

		template<class Register>
		bool readRegister(typename Register::Type* dest);

		template<class Region>
		bool readRegion(Region* region);

		bool readRaw(uint8_t addr, uint8_t* dest, uint8_t size);

		bool writeRaw(uint8_t addr, const uint8_t* src, uint8_t size);

		template<class Region>
		bool writeRegion(const Region& region);

		template<class Register, class Value>
		bool writeRegister(Value value);
	private:
		friend class DXLProtocol<MyIO>;
		Device(int id, DXLProtocol<MyIO>* proto);

		uint8_t m_id;
		DXLProtocol* m_proto;
	};

	DXLProtocol(MyIO* io);
	~DXLProtocol();

	Device device(uint8_t id);

	template<class Region, typename PacketFiller>
	bool syncWrite(uint8_t num_devices, PacketFiller filler);

	template<class Region, class Container>
	bool syncWriteConstant(const Container& ids, const Region& region);

	template<class Region>
	bool syncWriteConstant(const std::initializer_list<uint8_t>& ids, const Region& region)
	{ return syncWriteConstant<Region, std::initializer_list<uint8_t>>(ids, region); }

	template<class Buffer>
	bool sendBulkRead(const BulkReadBase<Buffer>& bulkRead);

	template<class Part>
	bool readBulkReadPart(Part* part);
private:
	template<int NumParam>
	class DXLPacket
	{
	public:
		DXLPacket(uint8_t id, uint8_t cmd)
		{
			m_data[0] = 0xFF;
			m_data[1] = 0xFF;
			m_data[2] = id;
			m_data[3] = NumParam + 2;
			m_data[4] = cmd;
		}

		void finalize()
		{
			uint8_t c = 0;
			for(int i = 2; i < Length-1; ++i)
				c += m_data[i];
			m_data[Length-1] = ~c;
		}

		int length() const
		{ return Length; }

		const uint8_t* data() const
		{ return m_data; }

		uint8_t& operator[](unsigned int idx)
		{ return m_data[5+idx]; }
	private:
		enum { Length = 2 + 1 + 1 + 1 + NumParam + 1 };
		uint8_t m_data[Length];
	};

	struct SyncWriteHeader
	{
		uint8_t addr;
		uint8_t size;
	} __attribute__((packed));

	template<class Region>
	struct SyncWriteHelper
	{
		uint8_t id;
		Region region;
	} __attribute__((packed));


	typedef ApplicativeStream<uint8_t, MyIO, &MyIO::writeByte> Stream;

	template<int NumParam>
	bool write(const DXLPacket<NumParam>& packet);

	bool writeHeader(uint8_t id, uint8_t cmd, uint8_t num_args);
	bool writeFooter();

	bool writeByte(uint8_t c);
	bool writeSegment(uint8_t* data, uint8_t size);

	ReturnCode readByte(uint8_t* c, uint16_t* timeout);
	ReturnCode waitForStatusPacket(uint8_t* id, uint8_t* num_params, uint8_t* error, uint16_t* timeout);
	ReturnCode checkStatusChecksum(uint16_t* timeout);

	MyIO* m_io;
	uint8_t m_checksum;
};

// IMPLEMENTATION
namespace
{
	enum DXLCommand
	{
		CMD_PING = 0x01,
		CMD_READ_DATA = 0x02,
		CMD_WRITE_DATA = 0x03,
		CMD_REG_WRITE = 0x04,
		CMD_ACTION = 0x05,
		CMD_RESET = 0x06,
		CMD_DIGITAL_RESET = 0x07,
		CMD_SYNC_WRITE = 0x83,
		CMD_BULK_READ = 0x92,
	};
}

template<class MyIO>
DXLProtocol<MyIO>::DXLProtocol(MyIO* io)
 : m_io(io)
{}

template<class MyIO>
DXLProtocol<MyIO>::~DXLProtocol()
{}

template<class MyIO>
typename DXLProtocol<MyIO>::Device DXLProtocol<MyIO>::device(uint8_t id)
{
	return Device(id, this);
}

template<class MyIO>
bool DXLProtocol<MyIO>::writeHeader(uint8_t id, uint8_t cmd, uint8_t num_params)
{
	m_io->reset();
	Stream stream(m_io);

	m_checksum = id + num_params + 2 + cmd;
	stream << 0xFF << 0xFF << id << num_params+2 << cmd;

	return !stream.isBad();
}

template<class MyIO>
bool DXLProtocol<MyIO>::writeByte(uint8_t c)
{
	m_checksum += c;
	return m_io->writeByte(c);
}

template<class MyIO>
bool DXLProtocol<MyIO>::writeFooter()
{
	if(!m_io->writeByte(~m_checksum))
		return false;

	m_io->startTransmit();

	return true;
}

template<class MyIO>
template<int NumParam>
bool DXLProtocol<MyIO>::write(const DXLPacket<NumParam>& packet)
{
	return m_io->write(packet.data(), packet.length());
}

template<class MyIO>
bool DXLProtocol<MyIO>::writeSegment(uint8_t* data, uint8_t size)
{
	for(unsigned int i = 0; i < size; ++i)
	{
		m_checksum += data[i];
	}

	return m_io->writeSegment(data, size);
}

enum WaitForStatusPacketState
{
	WSTATE_INIT1,
	WSTATE_INIT2,
	WSTATE_ID,
	WSTATE_NUM_PARAMS,
	WSTATE_ERROR
};

template<class MyIO>
ReturnCode DXLProtocol<MyIO>::waitForStatusPacket(uint8_t* id, uint8_t* num_params, uint8_t* error, uint16_t* timeout)
{
	WaitForStatusPacketState state = WSTATE_INIT1;

	while(*timeout > 0)
	{
		uint8_t c;
		ReturnCode ret = m_io->readByte(&c, timeout);
		if(ret == TIMEOUT)
			continue;

		if(!ret)
			return ret;

		switch(state)
		{
			case WSTATE_INIT1:
				if(c == 0xFF)
					state = WSTATE_INIT2;
				break;
			case WSTATE_INIT2:
				if(c == 0xFF)
					state = WSTATE_ID;
				else
					state = WSTATE_INIT1;
				break;
			case WSTATE_ID:
				m_checksum = c;
				*id = c;
				state = WSTATE_NUM_PARAMS;
				break;
			case WSTATE_NUM_PARAMS:
				m_checksum += c;
				if(c >= 2)
				{
					*num_params = c - 2;
					state = WSTATE_ERROR;
				}
				else
					state = WSTATE_INIT1;
				break;
			case WSTATE_ERROR:
				m_checksum += c;
				*error = c;
				return SUCCESS;
		}
	}

	return TIMEOUT;
}

template<class MyIO>
ReturnCode DXLProtocol<MyIO>::checkStatusChecksum(uint16_t* timeout)
{
	uint8_t c;
	ReturnCode ret = m_io->readByte(&c, timeout);

	if(!ret)
		return ret;

	if((uint8_t)~m_checksum == c)
		return SUCCESS;
	else
	{
		fprintf(stderr, "Checksum mismatch\n");
		return FAILURE;
	}
}

template<class MyIO>
ReturnCode DXLProtocol<MyIO>::readByte(uint8_t* c, uint16_t* timeout)
{
	ReturnCode ret = m_io->readByte(c, timeout);
	if(ret == SUCCESS)
		m_checksum += *c;

	return ret;
}

template<class MyIO>
DXLProtocol<MyIO>::Device::Device(int id, DXLProtocol<MyIO>* proto)
 : m_id(id)
 , m_proto(proto)
{
}

template<class MyIO>
bool DXLProtocol<MyIO>::Device::ping()
{
	DXLProtocol<MyIO>::DXLPacket<0> packet(m_id, CMD_PING);
	packet.finalize();

	if(!m_proto->write(packet))
		return false;

	uint16_t timeout = COMMS_TIMEOUT;
	while(timeout != 0)
	{
		uint8_t id = 0, params = 0, error = 0;
		ReturnCode ret = m_proto->waitForStatusPacket(&id, &params, &error, &timeout);
		if(ret == TIMEOUT)
			continue;
		if(ret != SUCCESS)
			return false;

		if(id != m_id || params != 0)
			continue;

		if(!m_proto->checkStatusChecksum(&timeout))
			continue;

		return true;
	}

	if(timeout == 0)
		return false;

	return true;
}

template<class MyIO>
bool DXLProtocol<MyIO>::Device::reset()
{
	// Note: A digital reset is equivalent to a normal reset, only the device ID isn't lost in the case of servos, hence we use it here
	DXLProtocol<MyIO>::DXLPacket<0> packet(m_id, CMD_DIGITAL_RESET);
	packet.finalize();

	if(!m_proto->write(packet))
		return false;

	return true;
}

enum ReadState
{
	RSTATE_HEADER,
	RSTATE_DATA,
	RSTATE_FOOTER
};

template<class MyIO>
bool DXLProtocol<MyIO>::Device::readRaw(uint8_t addr, uint8_t* dest, uint8_t size)
{
	DXLProtocol<MyIO>::DXLPacket<2> packet(m_id, CMD_READ_DATA);
	packet[0] = addr;
	packet[1] = size;
	packet.finalize();

	if(!m_proto->write(packet))
		return false;

	ReadState state = RSTATE_HEADER;

	uint16_t timeout = COMMS_TIMEOUT;
	while(timeout != 0)
	{
		uint8_t id = 0, params = 0, error;
		ReturnCode ret;

		switch(state)
		{
			case RSTATE_HEADER:
				ret = m_proto->waitForStatusPacket(&id, &params, &error, &timeout);
				if(ret == SUCCESS && id == m_id && params == size)
					state = RSTATE_DATA;
				break;
			case RSTATE_DATA:
				ret = m_proto->readByte(dest, &timeout);
				if(ret == SUCCESS)
				{
					dest++;
					size--;
					if(size == 0)
						state = RSTATE_FOOTER;
				}
				break;
			case RSTATE_FOOTER:
				ret = m_proto->checkStatusChecksum(&timeout);
				if(ret == SUCCESS)
					return true;
				break;
			default:
				return FAILURE;
		}

		if(ret == FAILURE)
		{
			fprintf(stderr, "readRaw: failure in state %d\n", state);
			return FAILURE;
		}
	}

	if(timeout == 0)
		return false;

	return true;
}

template<class MyIO>
template<class Register>
bool DXLProtocol<MyIO>::Device::readRegister(typename Register::Type* dest)
{
	Register reg;
	if(!readRegion(&reg))
		return false;

	*dest = reg.get();

	return true;
}

template<class MyIO>
template<class Region>
bool DXLProtocol<MyIO>::Device::readRegion(Region* region)
{
	static_assert(Region::Size == sizeof(Region), "Invalid region size");

	return readRaw(Region::FirstAddress, (uint8_t*)region, sizeof(Region));
}



template<class MyIO>
bool DXLProtocol<MyIO>::Device::writeRaw(uint8_t addr, const uint8_t* src, uint8_t size)
{
	if(!m_proto->writeHeader(m_id, CMD_WRITE_DATA, 1 + size))
		return false;

	if(!m_proto->writeByte(addr))
		return false;

	for(size_t i = 0; i < size; ++i)
	{
		if(!m_proto->writeByte(src[i]))
			return false;
	}

	if(!m_proto->writeFooter())
		return false;

	return true;
}

template<class MyIO>
template<class Region>
bool DXLProtocol<MyIO>::Device::writeRegion(const Region& region)
{
	return writeRaw(Region::FirstAddress, (const uint8_t*)&region, Region::Size);
}

template<class MyIO>
template<class Register, class Value>
bool DXLProtocol<MyIO>::Device::writeRegister(Value value)
{
	Register reg;
	reg.set(value);

	return writeRaw(Register::Address, (uint8_t*)&reg, Register::Size);
}

////////////////////////////////////////////////////////////////////////////////
// SYNC WRITE

template<class MyIO>
template<class Region, typename PacketFiller>
bool DXLProtocol<MyIO>::syncWrite(uint8_t num_devices, PacketFiller filler)
{
	SyncWriteHeader header;
	SyncWriteHelper<Region> helper;

	if(!writeHeader(0xFE, CMD_SYNC_WRITE, sizeof(header) + num_devices * (1 + Region::Size)))
		return false;

	header.addr = Region::FirstAddress;
	header.size = Region::Size;

	if(!writeSegment((uint8_t*)&header, sizeof(header)))
		return false;

	for(unsigned int i = 0; i < num_devices; ++i)
	{
		if(!filler(i, &helper.id, &helper.region))
			return false;

		if(!writeSegment((uint8_t*)&helper, sizeof(helper)))
			return false;
	}

	if(!writeFooter())
		return false;

	return true;
}

template<class MyIO>
template<class Region, class Container>
bool DXLProtocol<MyIO>::syncWriteConstant(const Container& ids, const Region& region)
{
	typename Container::const_iterator it = ids.begin();

	return syncWrite<Region>(ids.size(),
		[&] (int, uint8_t* id, Region* dest)
		{
			*id = *it;
			*dest = region;

			++it;
			return true;
		}
	);
}

////////////////////////////////////////////////////////////////////////////////
// BULK READ

template<class Buffer>
BulkReadBase<Buffer>::BulkReadBase()
 : m_numParamBytes(0)
{
	uint8_t header[] = {0xFF, 0xFF, 0xFE, 0x0, CMD_BULK_READ, 0x0};
	if(!m_buf.resize(requestSize()))
		abort();

	memcpy(m_buf.data(), header, sizeof(header));
}

template<class Buffer>
template<class Region>
bool BulkReadBase<Buffer>::addRead(uint8_t id)
{
	uint8_t request[] = {Region::Size, id, Region::FirstAddress};

	if(!m_buf.resize(requestSize() + sizeof(request)))
		return false;

	uint8_t* dest = m_buf.data() + requestSize()-1;
	memcpy(dest, request, sizeof(request));

	m_numParamBytes += sizeof(request);

	return true;
}

template<class Buffer>
void BulkReadBase<Buffer>::finalize()
{
	m_buf.data()[3] = 3 + m_numParamBytes;

	uint8_t checksum = 0;
	for(unsigned int i = 2; i < requestSize()-1; ++i)
	{
		checksum += m_buf.data()[i];
	}

	m_buf.data()[requestSize()-1] = ~checksum;
}

template<class MyIO>
template<class Buffer>
bool DXLProtocol<MyIO>::sendBulkRead(const BulkReadBase<Buffer>& br)
{
	return m_io->write(br.request(), br.requestSize());
}

template<class MyIO>
template<class Region>
bool DXLProtocol<MyIO>::readBulkReadPart(Region* part)
{
	ReadState state = RSTATE_HEADER;

	uint16_t timeout = COMMS_TIMEOUT;
	uint8_t* dest = (uint8_t*)part;
	uint32_t size = Region::Size;

	while(timeout != 0)
	{
		uint8_t id = 0, params = 0, error;
		ReturnCode ret;

		switch(state)
		{
			case RSTATE_HEADER:
				ret = waitForStatusPacket(&id, &params, &error, &timeout);
				if(ret == SUCCESS && params == Region::Size)
					state = RSTATE_DATA;
				break;
			case RSTATE_DATA:
				ret = readByte(dest, &timeout);
				if(ret == SUCCESS)
				{
					dest++;
					size--;
					if(size == 0)
						state = RSTATE_FOOTER;
				}
				break;
			case RSTATE_FOOTER:
				ret = checkStatusChecksum(&timeout);
				if(ret == SUCCESS)
					return true;
				else
					state = RSTATE_HEADER;
				break;
		}

		if(ret == FAILURE)
		{
			fprintf(stderr, "Failure in state %d\n", state);
			return false;
		}
	}

	if(timeout == 0)
		return false;

	return true;
}

}

#endif
