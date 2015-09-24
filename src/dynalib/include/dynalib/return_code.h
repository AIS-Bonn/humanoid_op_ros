// Unified return code for size & error
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DYNALIB_RETURN_CODE_H
#define DYNALIB_RETURN_CODE_H

namespace dynalib
{

enum ErrorCode
{
	SUCCESS = 0,
	TIMEOUT,
	FAILURE
};

struct ReturnCode
{
	ReturnCode(ErrorCode code)
		: m_code(-(int)code)
	{}

	ReturnCode(int size)
		: m_code(size)
	{}

	ReturnCode()
	{}

	inline operator bool() const
	{ return m_code == SUCCESS; }

	inline bool operator!() const
	{ return m_code != SUCCESS; }

	inline int size() const
	{ return m_code; }

	inline bool operator==(ErrorCode code)
	{
		return m_code == -(int)code;
	}
	
	inline bool operator!=(ErrorCode code)
	{
		return m_code != -(int)code;
	}
private:
	int m_code;
};

}

#endif
