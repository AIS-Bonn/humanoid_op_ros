#ifndef VEC2F
#define VEC2F

#include <QGLViewer/vec.h>
// #include <QtGlobal>
#include <QDebug>
#include <math.h>
// #include "learner/NVec.h"
#include <cap_gait/contrib/Vec3f.h>

namespace margait_contrib
{

class Vec2f;

extern double operator*(const Vec2f& v, const Vec2f& w);
extern Vec2f operator*(const double scalar, const Vec2f& v);
extern Vec2f operator*(const Vec2f& v, const double scalar);
extern Vec2f operator/(const Vec2f& v, const double scalar);
extern Vec2f operator/(const double scalar, const Vec2f& v);
extern Vec2f operator-(const double scalar, const Vec2f& v);

class Vec2f
{
	public:
		double x,y;
		// Constructors
		Vec2f(const Vec2f& v);
		Vec2f(){ x=y=0.f;}
		Vec2f(double xx){x=xx; y=xx;}
		Vec2f(double xx, double yy){x=xx; y=yy;}
		Vec2f(double xx[2]) { x = xx[0]; y = xx[1]; }
		Vec2f(const double xx[2]) { x = xx[0]; y = xx[1]; }
		Vec2f(const Vec3f& other){x=other.x; y=other.y;}
		Vec2f(const qglviewer::Vec& other){x=other.x; y=other.y;}
// 		Vec2f(const NVec<2>& other){x=other.x; y=other.y;}
		Vec2f & operator=(const Vec3f& other) {x=other.x; y=other.y; return *this;}
		Vec2f & operator=(const qglviewer::Vec& other) {x=other.x; y=other.y; return *this;}
// 		Vec2f & operator=(const NVec<2>& other) {x=other.x; y=other.y; return *this;}

		qglviewer::Vec toVec(){return qglviewer::Vec(x,y,0);}

		//operators
		inline void setZero() { x=y=0.f; }
		inline bool isZero() const { return x==0.f && y==0.f; }
		Vec2f rangeCut(const double &border) const;
		Vec2f rangeCut(const double &lower_border, const double &upper_border) const;
		double getmax() const {return x>y ? x : y;}
		double getmin() const {return x<y ? x : y;}

		Vec2f mapped(double (*func)(double)) { return Vec2f(func(x), func(y)); }
		void map(double (*func)(double)) { x = func(x); y = func(y); }
		inline Vec2f operator+(const double& t) const {return Vec2f(x+t, y+t);}
		inline Vec2f operator-(const double& t) const {return Vec2f(x-t, y-t);}
		inline Vec2f operator+(const Vec2f& v) const {return Vec2f(x+v.x, y+v.y);}
		inline Vec2f operator-() const {return Vec2f(-x, -y);}
		inline Vec2f operator-(const Vec2f& v) const {return Vec2f(x-v.x, y-v.y);}
		inline Vec2f& operator/=(const Vec2f& v) {x/=v.x; y/=v.y; return *this;}
		inline Vec2f operator/(const Vec2f& v) const {return Vec2f(x/v.x, y/v.y);}
		inline Vec2f operator%(const Vec2f& v) const {return Vec2f(x*v.x, y*v.y);}
		inline Vec2f operator%=(const Vec2f& v) const {return Vec2f(x*v.x, y*v.y);} //auch von Felix
		inline bool operator==(const Vec2f& v) const {return (x==v.x) && (y==v.y);}
		inline bool operator!=(const Vec2f& v) const {return (x!=v.x) || (y!=v.y);}

		inline Vec2f& operator=(const Vec2f& v){x=v.x; y=v.y; return *this;}
		inline Vec2f& operator=(double v){x=v; y=v; return *this;}
		inline Vec2f& operator*=(const double scalar){x*=scalar; y*=scalar; return *this;}
		inline Vec2f& operator/=(const double scalar){x/=scalar; y/=scalar; return *this;}
		inline Vec2f& operator+=(const Vec2f& v){x+=v.x; y+=v.y; return *this;}
		inline Vec2f& operator-=(const Vec2f& v){x-=v.x; y-=v.y; return *this;}
		inline Vec2f& operator+=(double v){x+=v; y+=v; return *this;}
		inline Vec2f& operator-=(double v){x-=v; y-=v; return *this;}

		// member functions
		inline Vec2f& projectTo(const Vec2f& to)
		{
			*this = to * ( ((*this) * to) / to.norm2());
			return *this;
		}

		inline Vec2f projectionTo(const Vec2f& to) const
		{
			Vec2f v(*this);
			v.projectTo(to);
			return v;
		}

		double ellipseDist(const Vec2f &p) const
		{
			double dx = p.x/x;
			double dy = p.y/y;
			return dx*dx + dy*dy;
		}

		/* Kreuzprodukt zweier Vektoren */
		inline double cross(const Vec2f& v2) const { return x*v2.y-y*v2.x; };
		/* Manhatten Norm eines Vektors */
		inline double norm1() const { return qAbs(x) + qAbs(y);}
		/* Euklidischer Länge eines Vektors */
		inline double norm() const { return sqrt(x*x+y*y);} // Euklidische Länge des Vektors zum Quadrat
		/* Euklidischer Länge eines Vektors zum Quadrat */
		inline double norm2() const { return x*x+y*y;}  // Euklidische Länge des Vektors zum Quadrat
		/* Winkel des Vectors in rad -pi..pi */
		inline double angle() const { return atan2(y,x); }

		inline void normalize()
		{
			double n = norm();
			if( n != 0.f)
			{
				x /= n; y /= n;
			}
			else
			{
				x = 0.f; y = 1.f;
			}
		}

		inline void normalizeTo(double l)
		{
			double n = norm();
			if ( n > l)
			{
				x *= l/n;
				y *= l/n;
			}
		}

		inline Vec2f normalize() const
		{
			double n = norm();
			if( n != 0.f)
				return Vec2f(x / n, y / n);
			else
				return *this;
		}

//		inline Vec2f normalizeTo(double l)
//		{
//			double n = norm();
//			if ( n > l)
//				return Vec2f(x*l/n, y*l/n);
//			else
//				return *this;
//		}

		/* Euklidischer Abstand zu einem Vektors */
		inline double dist(const Vec2f& v) const { return (*this-v).norm();}
		/* Euklidischer Abstand zu einem Vektors zum Quadrat */
		inline double dist2(const Vec2f& v) const { return (*this-v).norm2();}

		bool isLeftOf(const Vec2f& v) const
		{
			return (v.x * y - x * v.y) < 0;
		}

		inline void rotate(double angle)
		{
			double x_ = x;
			double y_ = y;
			x = x_ * cos(angle) + y_ * -sin(angle);
			y = x_ * sin(angle) + y_ * cos(angle);
		}

		inline Vec2f rotated(double angle) const
		{
			Vec2f v(*this);
			v.rotate(angle);
			return v;
		}

		inline bool operator<(const Vec2f& v ) const
		{
			return norm2() < v.norm2();
		}
};

QDebug operator<<(QDebug dbg, const Vec2f &v);

}

#endif //VEC2F
