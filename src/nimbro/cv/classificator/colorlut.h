// Color classification LUT
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COLORLUT_H
#define COLORLUT_H

#include <stdint.h>
#include <vector>

class ColorLUT
{
public:
	ColorLUT()
	{
		m_lut.resize(256*256);
	}

	inline uint8_t color(int y, int u, int v) const
	{
		return m_lut[256*u + v];
	}

	inline uint8_t& operator()(int u, int v)
	{
		return m_lut[256*u + v];
	}

	inline uint32_t meanColor(int color) const
	{ return m_colors[color]; }

	inline void setMeanColor(int color, int rgb)
	{
		m_colors[color] = rgb;
	}

	inline void setNumColors(int numColors)
	{
		m_colors.resize(numColors);
	}

	inline size_t numColors() const
	{ return m_colors.size(); }
private:
	std::vector<uint8_t> m_lut;
	std::vector<uint32_t> m_colors;
};

#endif
