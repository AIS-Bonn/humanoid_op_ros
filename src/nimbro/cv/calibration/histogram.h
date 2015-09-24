// Simple histogram encapsulation
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <QtCore/QVector>

class Histogram : public QVector<int>
{
public:
	explicit Histogram(uint size);
	virtual ~Histogram();

	virtual void reset();

	inline void hit(int x)
	{
		int c = ++(operator[](x));
		if(c > m_max)
			m_max = c;
	}

	inline int maximum() const
	{ return m_max; }
private:
	int m_max;
};

#endif
