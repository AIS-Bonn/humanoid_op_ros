#include <walk_and_kick/Vec3f.h>

// Vec3f::Vec3f(const Vec3i& v):x((float) v.x), y((float) v.y), z((float) v.z)
// {
// }


// Vec3f& Vec3f::operator=(const Vec3i& v)
// {
//    x = (float) v.x;
//    y = (float) v.y;
//    z = (float) v.z;
//    return *this;
// }


/* scalar product */
float operator*(const Vec3f& v, const Vec3f& w)
{
	return v.x * w.x + v.y * w.y + v.z*w.z;
}

Vec3f operator*(const float scalar, const Vec3f& v)
{
	return Vec3f(scalar * v.x, scalar * v.y, scalar * v.z);
}
Vec3f operator*(const Vec3f& v, const float scalar)
{
	return Vec3f(scalar * v.x, scalar * v.y, scalar*v.z);
}

Vec3f operator/(const Vec3f& v, const float scalar)
{
	return Vec3f(v.x/scalar, v.y/scalar, v.z/scalar);
}

// Vec3f Vec3f::rangeCut(const float &border) const 
// { 
// 	return Vec3f(	RcMath::rangeCut(border, x), 
// 					RcMath::rangeCut(border, y), 
// 					RcMath::rangeCut(border, z)	); 
// }
