#pragma once

#include <assert.h>
#include <cmath>
#include <string>
#include <iostream>

#include <walk_and_kick/Vec2f.h>

class Vec3i;
class Vec3f;

extern float operator*(const Vec3f& v, const Vec3f& w);
extern Vec3f operator*(const float scalar, const Vec3f& v);
extern Vec3f operator*(const Vec3f& v, const float scalar);
extern Vec3f operator/(const Vec3f& v, const float scalar);

class Vec3f
{
   friend class Vec3i;

	public:
		union {
			struct {
				float x, y, z;
			};
			float vi[3];
		};

		// Constructors
		Vec3f(const Vec3i& v);
		Vec3f(const Vec2f& v)
		{
			x = v.x;
			y = v.y;
			z = 0.f;
		}
		Vec3f(const Vec2f& v, float zz)
		{
			x= v.x;
			y= v.y;
			z= zz;
		}

		Vec3f(){ x=y=z=0.f;}
		explicit Vec3f(const float v){ x=y=z=v;}
		explicit Vec3f(const int v){ x=y=z=(float)v;}
		Vec3f(const float xx, const float yy, const float zz){  x=xx; y=yy; z=zz;}
		Vec3f(const float xx[3])
		{
			x = xx[0];
			y = xx[1];
			z = xx[2];
		}

		//operators
		Vec3f& operator=(const Vec3i& v);
		Vec3f& operator=(const float& v) { x=y=z=v; return *this; };

//      inline float& operator[](int i){assert((0<=i)&&(i<2));	return (&x)[i];};

		Vec3f mapped(float (*func)(float)) { return Vec3f(func(x), func(y), func(z)); }
		void map(float (*func)(float)) { x = func(x); y = func(y); z = func(z); }
// 		bool isFinite()  const { return _finite(x) && _finite(y) && _finite(z); }
		inline Vec3f operator+(const Vec3f& v) const {return Vec3f(x+v.x, y+v.y, z+v.z);}
		inline Vec3f operator-() const {return Vec3f(-x, -y, -z);}
		inline Vec3f operator-(const Vec3f& v) const {return Vec3f(x-v.x, y-v.y, z-v.z);}
		inline Vec3f operator%(const Vec3f& v) const {return Vec3f(x*v.x, y*v.y, z*v.z);}
		inline Vec3f operator%=(const Vec3f& v) const {return Vec3f(x*v.x, y*v.y, z*v.z);}
		inline bool operator==(const Vec3f& v) const {return (x==v.x) && (y==v.y) && (z==v.z);}
		inline bool operator!=(const Vec3f& v) const {return (x!=v.x) || (y!=v.y) || (z!=v.z);}
		inline Vec3f operator/(const Vec3f& v) const {return Vec3f(x/v.x, y/v.y, z/v.z);}


		inline Vec3f& operator=(const Vec3f& v){	x=v.x; y=v.y; z=v.z;	return *this;}
		inline Vec3f& operator*=(const float scalar){	x*=scalar;y*=scalar; z*=scalar; return *this;}
		inline Vec3f& operator/=(const float scalar){	x/=scalar;y/=scalar; z/=scalar; return *this;}
		inline Vec3f& operator+=(const Vec3f& v){	x+=v.x; y+=v.y; z+=v.z; return *this;}
		inline Vec3f& operator-=(const Vec3f& v){	x-=v.x; y-=v.y; z-= v.z; return *this;}
		inline float& operator[](const unsigned int i){	
			if(i>=0 && i<=2) return vi[i];
			else throw;
		}
		inline float operator[](const unsigned int i) const {	
			if(i>=0 && i<=2) return vi[i];
			else throw;
		}
// 		inline std::ostream& operator<<(const Vec3f v) 
// 		{ 
// 			std::cout << "x-component: " << v.x << " y-component: " << v.y 
// 				<< "z-component " << v.z << std::endl; 
// 		}

		// member functions
		inline void setZero() { x=y=z=0.f; }
		inline bool isZero() const { return x==0.f && y==0.f && z==0.f; }
		inline void setBasis(const unsigned int i) { 
			assert(i<3);
			x=y=z=0.f;
			vi[i]=1.f; 
		}
		inline void setBasisX() { x=1.f; y=z=0.f; }
		inline void setBasisY() { y=1.f; x=z=0.f; }
		inline void setBasisZ() { z=1.f; x=y=0.f; }
		Vec3f rangeCut(const float &border) const;
		inline Vec3f& projectTo(const Vec3f& to)
		{
			*this = to  *  ( ((*this) * to) / to.norm2());
			return *this;
		}

		/* Same as projectTo */
		inline Vec3f projectionTo(const Vec3f& to) const
		{
			Vec3f v(*this);
			v.projectTo(to);
			return v;
		}

		inline float angleToXY() const {
			return float(atan2(float(z), float(sqrt(x*x+y*y))));
			//float xy = x*x+y*y;
			//return RcMath::signum(z)*acos( sqrt( xy / (xy+z*z) ) );
		}

		inline void rotateZ( const float a )
		{
			float tmp = (float)(cos( a )* x - sin( a )*y);
			y = (float)(sin( a )*x+cos(a)*y);
			x = tmp;
		}

		inline Vec3f rotatedZ( const float a ) const
		{
			return Vec3f((float)(cos( a )* x - sin( a )*y),(float)(sin( a )*x+cos(a)*y), z);
		}

		/* Kreuzprodukt dreier Vektoren */
		inline Vec3f cross(const Vec3f& v2) const { 
			return Vec3f(y*v2.z-z*v2.y, z*v2.x - x*v2.z, x*v2.y-y*v2.x); };
		
		/* Manhatten Norm eines Vektors */
		inline float norm1() const { return fabs(x) + fabs(y) + fabs(z);}
		
		/* Euklidischer L�nge eines Vektors */
		inline float norm() const { return sqrt(x*x+y*y+z*z);} // Euklidische L�nge des Vektors zum Quadrat
		
		/* Euklidischer L�nge eines Vektors zum Quadrat */
		inline float norm2() const { return x*x+y*y+z*z;}  // Euklidische L�nge des Vektors zum Quadrat

		/* P-Norm of a vector. */
		inline float normp(float p) const { return pow( pow(fabs(x), p) + pow(fabs(y), p) + pow(fabs(z), p), 1/p); }


		inline Vec3f& normalize()
		{
			//float n = norm(); x /= n; y /= n; return *this;
			float n = norm();
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
			float n = norm();
			if( n != 0.f)
				return Vec3f(x / n, y / n, z/n);
			else
				return *this;
		}

		inline void normiereAuf( const float length)
		{
			normalize();
			*this *= length;
		}
		inline Vec3f normiertAuf( const float length)
		{
			Vec3f res = *this;
			res.normiereAuf(length);
			return res;
		}
		/* Euklidischer Abstand zu einem Vektors */
		inline float dist(const Vec3f& v) const { return (*this-v).norm();}
		/* Euklidischer Abstand zu einem Vektors zum Quadrat */
		inline float dist2(const Vec3f& v) const { return (*this-v).norm2();}
			
		
		inline bool operator<(const Vec3f& v ) const
		{
			return norm2() < v.norm2();
		}

		inline Vec3f absComponents() const
		{
			return Vec3f(fabsf(x), fabsf(y), fabsf(z));
		}

		inline void saturateMagnitude(const float max_magnitude) 
		{
			assert(max_magnitude>=0);

			if(x>max_magnitude)
				x=max_magnitude;
			else if(x<-max_magnitude)
				x= -max_magnitude;

			if(y>max_magnitude)
				y=max_magnitude;
			else if(y<-max_magnitude)
				y= -max_magnitude;

			if(z>max_magnitude)
				z=max_magnitude;
			else if(z<-max_magnitude)
				z= -max_magnitude;
		}

		inline void increaseMagnitude(const float magnitude_offset) 
		{
			assert(magnitude_offset>=0);

			if(x<0)
				x -= magnitude_offset;
			else if(x>0)
				x += magnitude_offset;

			if(y<0)
				y -= magnitude_offset;
			else if(y>0)
				y += magnitude_offset;

			if(z<0)
				z -= magnitude_offset;
			else if(z>0)
				z += magnitude_offset;
		}

		inline Vec2f getXYVec() const
		{
			return Vec2f(x, y);
		}

		float maximal_component()
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

		float minimal_component()
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
