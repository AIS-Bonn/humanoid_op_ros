// RcAngle.h: Schnittstelle für die Klasse RcAngle.
//
//////////////////////////////////////////////////////////////////////

#ifndef RCANGLE_H
#define RCANGLE_H

#include <math.h>
	
//////////////////////////////////////////////////////////////
// Angle - Klasse
class RcAngle  
{
public:
	RcAngle() {};
	RcAngle(const float &angle) : sinus(sin(angle)), cosinus(cos(angle)) {};
	virtual ~RcAngle();

	inline RcAngle& normiere()
	{
		float len = sinus*sinus + cosinus*cosinus;
		if(len == 0)
		{
			sinus = 0;
			cosinus = 1;
		}
		else
		{
			float scale = 1.f/sqrtf(len);
			sinus *= scale;
			cosinus *= scale;
		}

		return *this;
	};

	RcAngle& operator-()
	{
		sinus = -sinus;
		return *this;
	}


	/* Ungetestet, geht bestimmt nicht, wenn es doch geht, dann zu kompliziert! 
	RcAngle& operator*(float f)
	{
		float angle = RcMath::vangle(Vec2f(1.f,0.f),Vec2f(cosinus,sinus));
		Vec2f rotated = Vec2f(cosinus,sinus).rotated(f*angle);
		cosinus = rotated.x;
		sinus = rotated.y;
		return *this;

	}
	*/

	float sinus;
	float cosinus;

};

//////////////////////////////////////////////////////////////
// Parameter, die eine raeumliche Veraenderung beschreiben
class RcAlphaBetaDist
{
public:
	RcAngle Alpha;
	RcAngle Beta;
	float Dist;
};

#endif
