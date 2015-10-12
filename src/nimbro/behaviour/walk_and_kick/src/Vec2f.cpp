#include <walk_and_kick/Vec2f.h>

// Vec2f::Vec2f(const Vec2i& v):x((float) v.x), y((float) v.y)
// {
// }

/*
Vec2f& Vec2f::operator=(const Vec2i& v)
{
   x = (float) v.x;
   y = (float) v.y;
   return *this;
}*/


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

// Vec2f Vec2f::rangeCut(const float &border) const 
// { 
// 	return Vec2f(RcMath::rangeCut(border, x), RcMath::rangeCut(border, y)); 
// }
// 
// Vec2f Vec2f::rangeCut(const float &lower_border, const float &upper_border) const 
// { 
// 	return Vec2f(RcMath::rangeCut(lower_border, x, upper_border), RcMath::rangeCut(lower_border, y, upper_border)); 
// }

Vec2f operator/(const float scalar, const Vec2f& v)
{
	return Vec2f(scalar / v.x, scalar / v.y);
}

Vec2f operator-(const float scalar, const Vec2f& v)
{
	return Vec2f(scalar - v.x, scalar - v.y);
}
