#include "keyframe_player/Vec2f.h"

Vec2f::Vec2f(const Vec2f& v):x(v.x), y(v.y)
{
}

/* scalar product */
double operator*(const Vec2f& v, const Vec2f& w)
{
	return v.x * w.x + v.y * w.y;
}

Vec2f operator*(double scalar, const Vec2f& v)
{
	return Vec2f(scalar * v.x, scalar * v.y);
}
Vec2f operator*(const Vec2f& v, const double scalar)
{
	return Vec2f(scalar * v.x, scalar * v.y);
}

Vec2f operator/(const Vec2f& v, const double scalar)
{
	return Vec2f(v.x/scalar, v.y/scalar);
}

Vec2f operator/(const double scalar, const Vec2f& v)
{
	return Vec2f(scalar / v.x, scalar / v.y);
}

Vec2f operator-(const double scalar, const Vec2f& v)
{
	return Vec2f(scalar - v.x, scalar - v.y);
}


QDebug operator<<(QDebug dbg, const Vec2f &v)
{
	dbg.nospace() << "(" << v.x << ", " << v.y << ")";

	return dbg.space();
};
