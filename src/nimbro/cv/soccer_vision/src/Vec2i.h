// Vec2i.h: Schnittstelle für die Klasse Vec2i.
// 2-dimensional integer vector
//////////////////////////////////////////////////////////////////////

#ifndef VEC2I_H
#define VEC2I_H

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string>

class Vec2f;
class Vec2i
{
   friend class Vec2f;
	public:
		int x,y;
		// Constructors
		Vec2i(const Vec2f& v);
		Vec2i(){ x=y=0;}
		Vec2i(int xx, int yy){  x=xx; y=yy;}
		Vec2i(int xx[2])
		{
			for(int i=0; i<2; i++){ (&x)[i] = xx[i];}
		}

		//operators
      Vec2i& operator=(const Vec2f& v);
		inline int& operator[](int i){assert((0<=i)&&(i<2));	return (&x)[i];};

		inline Vec2i operator+(const Vec2i& v) const {return Vec2i(x+v.x, y+v.y);}
		inline Vec2i operator-() const {return Vec2i(-x, -y);}
		inline Vec2i operator-(const Vec2i& v) const {return Vec2i(x-v.x, y-v.y);}
		inline bool operator==(const Vec2i& v) const {return (x==v.x) && (y==v.y);}
		inline bool operator!=(const Vec2i& v) const {return (x!=v.x) || (y!=v.y);}

		inline void operator=(const Vec2i& v){	x=v.x; y=v.y;}
		inline Vec2i& operator*=(float scalar){	x=(int)((float)x*scalar);y=(int)((float)y*scalar); return *this;}
		inline Vec2i& operator+=(const Vec2i& v){	x+=v.x; y+=v.y; return *this;}
		inline Vec2i& operator-=(const Vec2i& v){	x-=v.x; y-=v.y; return *this;}
		// member functions
		inline int norm1() const { return abs(x)+abs(y);}
		inline float norm() const { return sqrtf((float)(x*x+y*y));}
		inline int norm2() const { return x*x+y*y;}
		inline float dist(const Vec2i& v) const { return (*this-v).norm();}
		inline int dist2(const Vec2i& v) const { return (*this-v).norm2();}
		bool isLeftOf(const Vec2i& g1, const Vec2i& g2) const
		{
			/*
				   g1x g2x this->x
			  det( g1y g2y this->y ) < 0
					1   1    1
			*/
			return (g1.x * g2.y + g2.x * y + x * g1.y - x * g2.y - g2.x * g1.y - g1.x * y) < 0;
		};
      // drehe um 90 Grad clockwise
      inline void dreheCW90()
      {
         int tmp = -x;
         x = y;
         y=tmp;
      };
      // drehe um 90 Grad counter-clockwise
      inline void dreheCCW90()
      {
         int tmp = x;
         x = -y;
         y=tmp;
      };
	/*THK unnoetig?
	// Save method
	virtual std::ostream& save(std::ostream& out)
	{
		out  << "Vec2i: " << x << " " << y << std::endl;
		return out;
	}
	// Load method
	virtual std::istream& load(std::istream& in)
	{
		std::string txt;
		in >> std::ws >> txt >> x >> y >> std::ws;
		return in;
	}
	*/
};

extern Vec2i operator*(float scalar, const Vec2i& v);
extern Vec2i operator*(const Vec2i& v, float scalar);

extern int operator*(const Vec2i& v, const Vec2i& w);
extern Vec2i operator*(int scalar, const Vec2i& v);
extern Vec2i operator*(const Vec2i& v, int scalar);
extern Vec2i operator/(const Vec2i& v, int scalar);

#endif
