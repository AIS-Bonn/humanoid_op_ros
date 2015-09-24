// Device base class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_DEVICE_H
#define DYNALIB_DEVICE_H

#include <stdint.h>

#include "units.h"

namespace dynalib
{

class Device
{
public:
	struct RegisterInfo
	{
	public:
		enum RegisterType {EEPROM, RAM};
		enum RegisterFlags
		{
			FLAG_HIDDEN = (1 << 0),
			FLAG_SIGNED = (1 << 1),
			FLAG_HEX    = (1 << 2)
		};

		RegisterInfo(const char* name, uint8_t addr, uint8_t size, uint8_t reg_type, unsigned int flags)
		 : name(name)
		 , addr(addr)
		 , size(size)
		 , reg_type(reg_type)
		 , flags(flags)
		{}

		RegisterInfo()
		 : name(0)
		 , addr(0)
		 , size(0)
		 , reg_type(0)
		 , flags(0)
		{
		}

		const char* name;
		uint8_t addr;
		uint8_t size;
		uint8_t reg_type;
		unsigned int flags;
	};

	Device() {}
	virtual ~Device() {}
	virtual const char* name() const = 0;
	virtual uint16_t model() const = 0;
	virtual RegisterInfo registerInfo(unsigned int index) const = 0;
	virtual unsigned int registerCount() const = 0;
	virtual uint8_t readSize() const = 0;
};

template<class Base, int Count>
struct MergePred : public MergePred<typename Base::Pred, Count-1>, public Base
{
	enum { Size = Base::Size + MergePred<typename Base::Pred, Count-1>::Size };
} __attribute__((packed));

template<class XBase>
struct MergePred<XBase, 0> : public XBase
{
	enum { Size = XBase::Size };
} __attribute__((packed));

template<int Head, int... Tail>
struct RegionHelper
{
	typedef RegionHelper<Tail...> TH;
	enum
	{
		First = (Head > TH::First) ? TH::First : Head,
		Last = (Head > TH::Last) ? Head : TH::Last
	};
};

template<int Head>
struct RegionHelper<Head>
{
	enum
	{
		First = Head,
		Last = Head
	};
};

}

#endif


