#include "Vec3f.h"

/* scalar product */
double operator*(const Vec3f& v, const Vec3f& w)
{
	return v.x * w.x + v.y * w.y + v.z*w.z;
}

Vec3f operator*(const double scalar, const Vec3f& v)
{
	return Vec3f(scalar * v.x, scalar * v.y, scalar * v.z);
}
Vec3f operator*(const Vec3f& v, const double scalar)
{
	return Vec3f(scalar * v.x, scalar * v.y, scalar*v.z);
}

Vec3f operator/(const Vec3f& v, const double scalar)
{
	return Vec3f(v.x/scalar, v.y/scalar, v.z/scalar);
}


QDebug operator<<(QDebug dbg, const Vec3f &v)
{
	dbg.nospace() << "(" << v.x << ", " << v.y << ", " << v.z << ")";

	return dbg.space();
};

