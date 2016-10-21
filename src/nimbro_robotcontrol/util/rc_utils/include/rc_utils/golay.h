// Savitzky-Golay filter
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef GOLAY_H
#define GOLAY_H

#include <boost/circular_buffer.hpp>
#include <boost/utility/value_init.hpp>
#include <boost/utility/enable_if.hpp>

namespace rc_utils
{

template<class T, int DerivGrade, int WindowSize = 9, class Alloc = std::allocator<T> >
class GolayDerivative
{
public:
	GolayDerivative();
	virtual ~GolayDerivative();

	typedef boost::circular_buffer<T, Alloc> BufferType;

	void put(const T& data);
	T value() const;

	BufferType* buffer()
	{ return &m_buf; }

	inline void reset()
	{ m_buf.clear(); }
private:
	BufferType m_buf;
};

template<int DerivGrade, int WindowSize>
class GolayCoeff
{
};

template<>
class GolayCoeff<0, 1>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<0, 5>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<0, 7>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<0, 9>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<1, 5>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<1, 7>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<1, 9>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<2, 5>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<2, 7>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<>
class GolayCoeff<2, 9>
{
public:
	const static double Coefficients[];
	const static double Normalization;
};

template<class T>
typename boost::enable_if<boost::is_arithmetic<T>, T>::type initValue(const T& dummy)
{
	return 0;
}

// Eigen3 type support
#ifdef EIGEN_CORE_H
template<class T, int Rows, int Cols>
Eigen::Matrix<T, Rows, Cols> initValue(const Eigen::Matrix<T, Rows, Cols>& dummy)
{
	return Eigen::Matrix<T, Rows, Cols>::Zero();
}
#endif

template<class T, int DerivGrade, int WindowSize, class Alloc>
GolayDerivative<T, DerivGrade, WindowSize, Alloc>::GolayDerivative()
 : m_buf(WindowSize)
{
}

template<class T, int DerivGrade, int WindowSize, class Alloc>
GolayDerivative<T, DerivGrade, WindowSize, Alloc>::~GolayDerivative()
{
}

template<class T, int DerivGrade, int WindowSize, class Alloc>
void GolayDerivative<T, DerivGrade, WindowSize, Alloc>::put(const T& data)
{
	m_buf.push_back(data);
}

template<class T, int DerivGrade, int WindowSize, class Alloc>
T GolayDerivative<T, DerivGrade, WindowSize, Alloc>::value() const
{
	T val = initValue(T());

	if(m_buf.size() != WindowSize)
		return val;

	for(size_t i = 0; i < WindowSize; ++i)
		val += GolayCoeff<DerivGrade, WindowSize>::Coefficients[i] * m_buf[i];

	return val / GolayCoeff<DerivGrade, WindowSize>::Normalization;
}


}

#endif
