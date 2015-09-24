// Dynamixel units
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_UNITS_H
#define DYNALIB_UNITS_H

#include <math.h>

namespace dynalib
{

template<class Type>
class Raw
{
public:
	constexpr Raw()
	{}

	constexpr Raw(Type val)
	 : m_val(val)
	{}

	constexpr void operator=(const Type& val)
	{ m_val = val; }

	constexpr operator Type() const
	{ return m_val; }

	constexpr Type value() const
	{ return m_val; }

	void setValue(const Type& val)
	{ m_val = val; }
private:
	Type m_val;
};

class Radian : public Raw<float>
{
public:
	constexpr Radian(float val) : Raw<float>(val) {}
};

class RadPerS : public Raw<float>
{
public:
	constexpr RadPerS(float val) : Raw<float>(val) {}
};

class MXPosition : public Raw<uint16_t>
{
public:
	static constexpr float TICK = 0.088 * M_PI / 180.0;

	MXPosition(uint16_t val) : Raw<uint16_t>(val) {}
	MXPosition(Radian rad) : Raw<uint16_t>(rad.value() / TICK + 2048) {}
	MXPosition() {}

	operator Radian() const
	{ return (value() - 2048) * TICK; }
};

static constexpr float MX_VEL_TICK = 0.114 / 60.0 * 2.0 * M_PI;
class MXVelocity : public Raw<uint16_t>
{
public:
	MXVelocity() {}
	MXVelocity(uint16_t val) : Raw<uint16_t>(val) {}
	MXVelocity(RadPerS vel)
	{
		double value = vel.value();
		if(value >= 0)
			setValue(value / MX_VEL_TICK);
		else
			setValue((-value / MX_VEL_TICK) + 1024);
	}

	// BUG: This interface is dangerous!
// 	// Accept anything that can be converted to rad/s
// 	template<class T>
// 	MXVelocity(T vel) : MXVelocity((RadPerS)vel)
// 	{
// 	}

	operator RadPerS() const
	{
		uint16_t val = value();
		if(val & (1 << 10))
			return RadPerS(-MX_VEL_TICK * (val & ~(1 << 10)));
		else
			return RadPerS(MX_VEL_TICK * val);
	}
};

}

inline constexpr dynalib::RadPerS operator "" _rad_per_s(long double val)
{ return dynalib::RadPerS(val); }

inline constexpr dynalib::RadPerS operator "" _rpm(long double val)
{ return dynalib::RadPerS(val / 60.0 * 2.0 * M_PI); }

#endif
