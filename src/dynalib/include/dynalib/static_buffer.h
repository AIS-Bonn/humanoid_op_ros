// Static buffer implementation (used for BulkRead)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_STATIC_BUFFER_H
#define DYNALIB_STATIC_BUFFER_H

namespace dynalib
{

template<int Size>
class StaticBuffer
{
public:
	inline uint8_t* data()
	{ return m_data; }

	inline const uint8_t* data() const
	{ return m_data; }

	bool resize(unsigned int size)
	{
		return size <= Size;
	}
private:
	uint8_t m_data[Size];
};

}

#endif
