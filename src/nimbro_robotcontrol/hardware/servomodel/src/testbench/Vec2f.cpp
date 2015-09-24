#include "Vec2f.h"

Vec2f::Vec2f(const Vec2f& v):x(v.x), y(v.y)
{
}

/* scalar product */
float operator*(const Vec2f& v, const Vec2f& w)
{
	return v.x * w.x + v.y * w.y;
}

Vec2f operator*(float scalar, const Vec2f& v)
{
	return Vec2f(scalar * v.x, scalar * v.y);
}
Vec2f operator*(const Vec2f& v, const float scalar)
{
	return Vec2f(scalar * v.x, scalar * v.y);
}

Vec2f operator/(const Vec2f& v, const float scalar)
{
	return Vec2f(v.x/scalar, v.y/scalar);
}

Vec2f operator/(const float scalar, const Vec2f& v)
{
	return Vec2f(scalar / v.x, scalar / v.y);
}

Vec2f operator-(const float scalar, const Vec2f& v)
{
	return Vec2f(scalar - v.x, scalar - v.y);
}
