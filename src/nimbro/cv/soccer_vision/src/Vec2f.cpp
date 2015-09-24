#include "Vec2f.h"
#include "Vec2i.h"

template<class T> inline static T RCrangeCut(T lower_border,T value,T upper_border)
{
		return std::min(upper_border, std::max(lower_border, value));
}

template<class T> inline static T RCrangeCut(T border,T value)
{
		return std::min(border, std::max(-border, value));
}

Vec2f::Vec2f(const Vec2i& v):x((float) v.x), y((float) v.y)
{
}


Vec2f& Vec2f::operator=(const Vec2i& v)
{
   x = (float) v.x;
   y = (float) v.y;
   return *this;
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

Vec2f Vec2f::rangeCut(const float &border) const 
{ 
	return Vec2f(RCrangeCut(border, x), RCrangeCut(border, y)); 
}

Vec2f Vec2f::rangeCut(const float &lower_border, const float &upper_border) const 
{ 
	return Vec2f(RCrangeCut(lower_border, x, upper_border), RCrangeCut(lower_border, y, upper_border)); 
}

Vec2f operator/(const float scalar, const Vec2f& v)
{
	return Vec2f(scalar / v.x, scalar / v.y);
}

Vec2f operator-(const float scalar, const Vec2f& v)
{
	return Vec2f(scalar - v.x, scalar - v.y);
}
