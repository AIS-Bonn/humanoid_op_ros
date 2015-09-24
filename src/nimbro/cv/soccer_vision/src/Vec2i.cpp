// Vec2i.cpp: Implementierung der Klasse Vec2i.
// 2-dimensional integer vector
//////////////////////////////////////////////////////////////////////
#include "Vec2i.h"
#include "Vec2f.h"

//////////////////////////////////////////////////////////////////////
// Konstruktion/Destruktion
//////////////////////////////////////////////////////////////////////
Vec2i::Vec2i(const Vec2f& v):x((int) v.x), y((int) v.y)
{
}

Vec2i& Vec2i::operator=(const Vec2f& v)
{
   x = (int) v.x;
   y = (int) v.y;
   return *this;
}


/* scalar product */
int operator*(const Vec2i& v, const Vec2i& w)
{
	return v.x * w.x + v.y * w.y;
}

Vec2i operator*(int scalar, const Vec2i& v)
{
	return Vec2i(scalar * v.x, scalar * v.y);
}
Vec2i operator*(const Vec2i& v, int scalar)
{
	return Vec2i(scalar * v.x, scalar * v.y);
}


Vec2i operator*(float scalar, const Vec2i& v)
{
	return Vec2i(int(scalar * v.x), int(scalar * v.y));
}
Vec2i operator*(const Vec2i& v, float scalar)
{
	return Vec2i(int(scalar * v.x), int(scalar * v.y));
}


Vec2i operator/(const Vec2i& v, int scalar)
{
	return Vec2i(v.x/scalar, v.y/scalar);
}
