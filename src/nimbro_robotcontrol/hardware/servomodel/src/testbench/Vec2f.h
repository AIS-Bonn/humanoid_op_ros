#ifndef VEC2F
#define VEC2F

#include <math.h>
class Vec2f;

extern float operator*(const Vec2f& v, const Vec2f& w);
extern Vec2f operator*(const float scalar, const Vec2f& v);
extern Vec2f operator*(const Vec2f& v, const float scalar);
extern Vec2f operator/(const Vec2f& v, const float scalar);
extern Vec2f operator/(const float scalar, const Vec2f& v);
extern Vec2f operator-(const float scalar, const Vec2f& v);

class Vec2f
{
	public:
		float x,y;
		// Constructors
		Vec2f(const Vec2f& v);
		Vec2f(){ x=y=0.f;}
		Vec2f(float xx, float yy){  x=xx; y=yy;}
		Vec2f(float xx[2]) { x = xx[0]; y = xx[1]; }
		Vec2f(const float xx[2]) { x = xx[0]; y = xx[1]; }

		//operators
		inline void setZero() { x=y=0.f; }
		inline bool isZero() const { return x==0.f && y==0.f; }
		Vec2f rangeCut(const float &border) const;
		Vec2f rangeCut(const float &lower_border, const float &upper_border) const;
		float getmax() const { return x>y ? x : y; }
		float getmin() const { return x<y ? x : y; }

		Vec2f mapped(float (*func)(float)) { return Vec2f(func(x), func(y)); }
		void map(float (*func)(float)) { x = func(x); y = func(y); }
		inline Vec2f operator+(const float& t) const {return Vec2f(x+t, y+t);}
		inline Vec2f operator-(const float& t) const {return Vec2f(x-t, y-t);}
		inline Vec2f operator+(const Vec2f& v) const {return Vec2f(x+v.x, y+v.y);}
		inline Vec2f operator-() const {return Vec2f(-x, -y);}
		inline Vec2f operator-(const Vec2f& v) const {return Vec2f(x-v.x, y-v.y);}
		inline Vec2f& operator/=(const Vec2f& v) { x/=v.x; y/=v.y; return *this;} 
		inline Vec2f operator/(const Vec2f& v) const {return Vec2f(x/v.x, y/v.y);} 
		inline Vec2f operator%(const Vec2f& v) const {return Vec2f(x*v.x, y*v.y);}
		inline Vec2f operator%=(const Vec2f& v) const {return Vec2f(x*v.x, y*v.y);} //auch von Felix
		inline bool operator==(const Vec2f& v) const {return (x==v.x) && (y==v.y);}
		inline bool operator!=(const Vec2f& v) const {return (x!=v.x) || (y!=v.y);}

		inline Vec2f& operator=(const Vec2f& v){	x=v.x; y=v.y;	return *this;}
		inline Vec2f& operator*=(const float scalar){	x*=scalar;y*=scalar; return *this;}
		inline Vec2f& operator/=(const float scalar){	x/=scalar;y/=scalar; return *this;}
		inline Vec2f& operator+=(const Vec2f& v){	x+=v.x; y+=v.y; return *this;}
		inline Vec2f& operator-=(const Vec2f& v){	x-=v.x; y-=v.y; return *this;}

		// member functions
		inline Vec2f& projectTo(const Vec2f& to)
		{
			*this = to  *  ( ((*this) * to) / to.norm2());
			return *this;
		}

		inline Vec2f projectionTo(const Vec2f& to) const
		{
			Vec2f v(*this);
			v.projectTo(to);
			return v;
		}

		float ellipseDist(const Vec2f &p) const
		{
			float dx = p.x/x;
			float dy = p.y/y;
			return dx*dx + dy*dy;
		}

		/* Kreuzprodukt zweier Vektoren */
		inline float cross(const Vec2f& v2) const { return x*v2.y-y*v2.x; };
		/* Manhatten Norm eines Vektors */
		inline float norm1() const { return fabs(x) + fabs(y);}
		/* Euklidischer L�nge eines Vektors */
		inline float norm() const { return sqrt(x*x+y*y);} // Euklidische L�nge des Vektors zum Quadrat
		/* Euklidischer L�nge eines Vektors zum Quadrat */
		inline float norm2() const { return x*x+y*y;}  // Euklidische L�nge des Vektors zum Quadrat
		/* Winkel des Vectors in rad -pi..pi */
		inline float angle() const { return float(atan2(float(y),float(x))); }
		inline float angle_y() const { return float(atan2(float(x),float(y))); }
		inline Vec2f& normalize()
		{
			//float n = norm(); x /= n; y /= n; return *this;
			float n = norm();
			if( n != 0.f)
			{
				x /= n; y /= n;
			}
			else
			{
				x = 0.f; y = 1.f;
			}
			return *this;
		}
		inline Vec2f& normalizeSave()
		{
			float n = norm();
			if( n != 0.f)
			{
				x /= n; y /= n;
			}
			else
			{
				x = 0.f; y = 1.f;
			}
			return *this;
		}
		inline Vec2f getNormalized() const
		{
		//	float n = norm(); return Vec2f(x / n, y / n);
			float n = norm();
			if( n != 0.f)
				return Vec2f(x / n, y / n);
			else
				return *this;
		}

		inline Vec2f getNormalizedSave() const
		{
		//	float n = norm(); return Vec2f(x / n, y / n);
			float n = norm();
			if( n != 0.f)
				return Vec2f(x / n, y / n);
			else
				return Vec2f(0.f, 1.f);
		}

		inline void normiereAuf( float length)
		{
			normalizeSave();
			*this *= length;
		}
		inline Vec2f normiertAuf( float length)
		{
			Vec2f res = *this;
			res.normiereAuf(length);
			return res;
		}
		/* Euklidischer Abstand zu einem Vektors */
		inline float dist(const Vec2f& v) const { return (*this-v).norm();}
		/* Euklidischer Abstand zu einem Vektors zum Quadrat */
		inline float dist2(const Vec2f& v) const { return (*this-v).norm2();}
		bool isLeftOf(const Vec2f& g1, const Vec2f& g2) const
		{
			/*
				   g1x g2x this->x
			  det( g1y g2y this->y ) < 0
					1   1    1
			*/
			return (g1.x * g2.y + g2.x * y + x * g1.y - x * g2.y - g2.x * g1.y - g1.x * y) < 0;
		}

		bool isLeftOf(const Vec2f& v) const
		{
			/*
				   g1x g2x this->x
			  det( g1y g2y this->y ) < 0
					1   1    1
			*/
			return (v.x * y - x * v.y) < 0;
		}

		// return rotated um 90 Grad clockwise
		inline Vec2f rotatedCW90()
		{
			Vec2f v = *this;
			v.dreheCW90();
			return v;
		}

		// drehe um 90 Grad clockwise
		inline void dreheCW90()
		{
			 float tmp = -x;
			 x = y;
			 y=tmp;
		}
		// drehe um 90 Grad counter-clockwise
		inline void dreheCCW90()
		{
			 float tmp = x;
			 x = -y;
			 y=tmp;
		}

		// return rotated um 90 Grad counter-clockwise
		inline Vec2f rotatedCCW90()
		{
			Vec2f v = *this;
			v.dreheCCW90();
			return v;
		}

		inline void rotate( float a )
		{
			float tmp = (float)(cos( a )* x - sin( a )*y);
			y = (float)(sin( a )*x+cos(a)*y);
			x = tmp;
		}

		inline Vec2f rotated( float a ) const
		{
			return Vec2f((float)(cos( a )* x - sin( a )*y),(float)(sin( a )*x+cos(a)*y));
		}
		inline bool operator<(const Vec2f& v ) const
		{
			return norm2() < v.norm2();
		}

		inline Vec2f absComponents() const
		{
			return Vec2f(fabs(x), fabs(y));
		}

		inline void saturateMagnitude(float max_magnitude) 
		{
	
			if(x>max_magnitude)
				x=max_magnitude;
			else if(x<-max_magnitude)
				x= -max_magnitude;

			if(y>max_magnitude)
				y=max_magnitude;
			else if(y<-max_magnitude)
				y= -max_magnitude;
		}

		inline void increaseMagnitude(float magnitude_offset) 
		{
			if(x<0)
				x -= magnitude_offset;
			else if(x>0)
				x += magnitude_offset;

			if(y<0)
				y -= magnitude_offset;
			else if(y>0)
				y += magnitude_offset;
		}
};

#endif //VEC2F
