// Little-Endian value
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_LE_VALUE_H
#define DYNALIB_LE_VALUE_H

#include <stdint.h>
#include <endian.h>

namespace dynalib
{

template<int Width>
class LEValue
{
};

template<int Width, class _Type>
class TypedLEValue
{
public:
	typedef _Type Type;

	inline operator Type() const
	{ return Type(m_val()); }

	inline Type operator()() const
	{ return m_val(); }

	inline Type operator=(Type type)
	{ m_val = type; return type; }
private:
	LEValue<Width> m_val;
} __attribute__((packed));


template<>
class LEValue<1>
{
public:
	typedef uint8_t Type;

	inline operator uint8_t() const
	{ return m_value; }

	inline uint8_t operator()() const
	{ return m_value; }

	inline uint8_t operator=(uint8_t value)
	{ return m_value = value; }
private:
	uint8_t m_value;
} __attribute__((packed));

template<>
class LEValue<2>
{
public:
	typedef uint16_t Type;

	inline operator uint16_t() const
	{ return le16toh(m_value); }

	inline uint16_t operator()() const
	{ return le16toh(m_value); }

	inline uint16_t operator=(uint16_t value)
	{
		m_value = htole16(value);
		return value;
	}
private:
	uint16_t m_value;
} __attribute__((packed));

}

#endif
