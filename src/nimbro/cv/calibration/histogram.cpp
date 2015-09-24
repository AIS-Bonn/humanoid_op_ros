// Simple histogram encapsulation
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "histogram.h"

Histogram::Histogram(uint size)
 : QVector<int>(size, 0)
 , m_max(0)
{
}

Histogram::~Histogram()
{
}

void Histogram::reset()
{
	fill(0);
	m_max = 0;
}
