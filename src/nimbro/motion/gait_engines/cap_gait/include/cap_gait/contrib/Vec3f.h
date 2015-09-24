#ifndef VEC3F_H_
#define VEC3F_H_

#include <QGLViewer/vec.h>
#include <QDebug>
#include <math.h>

namespace margait_contrib
{

class Vec3f;

extern double operator*(const Vec3f& v, const Vec3f& w);
extern Vec3f operator*(const double scalar, const Vec3f& v);
extern Vec3f operator*(const Vec3f& v, const double scalar);
extern Vec3f operator/(const Vec3f& v, const double scalar);

class Vec3f
{
public:

		double x, y, z;

		Vec3f(){ x=y=z=0.f;}
		Vec3f(const double xx, const double yy, const double zz){x=xx; y=yy; z=zz;}
		Vec3f(const qglviewer::Vec& other){x=other.x; y=other.y; z=other.z;}
		Vec3f & operator=(const qglviewer::Vec& other) {x=other.x; y=other.y; z=other.z; return *this;}

		inline Vec3f operator+(const Vec3f& v) const {return Vec3f(x+v.x, y+v.y, z+v.z);}
		inline Vec3f operator-() const {return Vec3f(-x, -y, -z);}
		inline Vec3f operator-(const Vec3f& v) const {return Vec3f(x-v.x, y-v.y, z-v.z);}
		inline Vec3f operator%(const Vec3f& v) const {return Vec3f(x*v.x, y*v.y, z*v.z);}
		inline Vec3f operator%=(const Vec3f& v) const {return Vec3f(x*v.x, y*v.y, z*v.z);}
		inline bool operator==(const Vec3f& v) const {return (x==v.x) && (y==v.y) && (z==v.z);}
		inline bool operator!=(const Vec3f& v) const {return (x!=v.x) || (y!=v.y) || (z!=v.z);}
		inline Vec3f operator/(const Vec3f& v) const {return Vec3f(x/v.x, y/v.y, z/v.z);}

		inline Vec3f& operator=(const Vec3f& v){	x=v.x; y=v.y; z=v.z;	return *this;}
		inline Vec3f& operator*=(const double scalar){	x*=scalar;y*=scalar; z*=scalar; return *this;}
		inline Vec3f& operator/=(const double scalar){	x/=scalar;y/=scalar; z/=scalar; return *this;}
		inline Vec3f& operator+=(const Vec3f& v){	x+=v.x; y+=v.y; z+=v.z; return *this;}
		inline Vec3f& operator-=(const Vec3f& v){	x-=v.x; y-=v.y; z-= v.z; return *this;}

		operator const double*() const {return &x;} // For OpenGL

		/* Euklidischer Länge eines Vektors */
		inline double norm() const { return sqrt(x*x+y*y+z*z);} // Euklidische Länge des Vektors zum Quadrat

		/* Euklidischer Länge eines Vektors zum Quadrat */
		inline double norm2() const { return x*x+y*y+z*z;}  // Euklidische Länge des Vektors zum Quadrat

		/* P-Norm of a vector. */
		inline double normp(double p) const { return pow( pow(fabs(x), p) + pow(fabs(y), p) + pow(fabs(z), p), 1/p); }

		inline Vec3f& normalize()
		{
			//double n = norm(); x /= n; y /= n; return *this;
			double n = norm();
			if( n != 0.f)
			{
				x /= n; y /= n; z/= n;
			}
			else
			{
				//Vec2f returns (0,1) but we uses now (0,0,0)
				x = 0.f; y = 0.f; z= 0.f;
			}
			return *this;
		}

		inline Vec3f getNormalized() const
		{
			double n = norm();
			if( n != 0.f)
				return Vec3f(x / n, y / n, z/n);
			else
				return *this;
		}

		inline void normiereAuf( const double length)
		{
			normalize();
			*this *= length;
		}
		inline Vec3f normiertAuf( const double length)
		{
			Vec3f res = *this;
			res.normiereAuf(length);
			return res;
		}
		/* Euklidischer Abstand zu einem Vektors */
		inline double dist(const Vec3f& v) const { return (*this-v).norm();}
		/* Euklidischer Abstand zu einem Vektors zum Quadrat */
		inline double dist2(const Vec3f& v) const { return (*this-v).norm2();}


		inline void rotate(double angle)
		{
			double x_ = x;
			double y_ = y;
			x = x_ * cos(angle) + y_ * -sin(angle);
			y = x_ * sin(angle) + y_ * cos(angle);
		}

		inline bool operator<(const Vec3f& v ) const
		{
			return norm2() < v.norm2();
		}

		inline Vec3f absComponents() const
		{
			return Vec3f(fabsf(x), fabsf(y), fabsf(z));
		}

		double maxComponent()
		{
			if (fabsf(x)>fabsf(y))
				if(fabsf(x)>fabsf(z))
					return fabsf(x);
				else
					return fabsf(z);
			else
				if(fabsf(y)>fabsf(z))
					return fabsf(y);
				else
					return fabsf(z);
		}

		double minComponent()
		{
			if (fabsf(x)>fabsf(y))
				if(fabsf(y)>fabsf(z))
					return fabsf(z);
				else
					return fabsf(y);
			else
				if(fabsf(x)>fabsf(z))
					return fabsf(z);
				else
					return fabsf(x);
		}
};

QDebug operator<<(QDebug dbg, const Vec3f &v);

}

#endif /* ACTION_H_ */
