// Applies a function on operator<<
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_UTIL_APPLICATIVE_STREAM_H
#define DYNALIB_UTIL_APPLICATIVE_STREAM_H

#include <dynalib/util/likely.h>

namespace dynalib
{

template<class T, class Base, bool (Base::*Func)(T)>
class ApplicativeStream
{
public:
	ApplicativeStream(Base* base)
	 : m_base(base)
	 , m_bad(false)
	{
	}

	inline __attribute__((always_inline)) ApplicativeStream& operator<<(T val)
	{
		if(likely(!m_bad))
		{
			if(unlikely(!(m_base->*Func)(val)))
				m_bad = true;
		}

		return *this;
	}

	bool isBad()
	{
		return m_bad;
	}
private:
	Base* m_base;
	bool m_bad;
};

}

#endif
