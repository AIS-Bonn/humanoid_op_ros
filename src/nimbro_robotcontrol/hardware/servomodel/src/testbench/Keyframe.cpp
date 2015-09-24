#include "Keyframe.h"

Keyframe::Keyframe()
{
	t = 0;
	x = 0;
	v = 0;
	a = 0;
	highlight = false;
	type = TYPE_DEFAULT;
}

Keyframe::Keyframe(double t, double x, double v)
{
	this->t = t;
	this->x = x;
	this->v = v;
	a = 0;
	highlight = false;
	type = TYPE_DEFAULT;
}

void Keyframe::set(double t, double x, double v)
{
	this->t = t;
	this->x = x;
	this->v = v;
	a = 0;
	highlight = false;
	type = TYPE_DEFAULT;
}

Vec2f Keyframe::location()
{
	return Vec2f(t,x);
}

void Keyframe::setLocation(Vec2f v)
{
	t = v.x;
	x = v.y;
}

void Keyframe::relocateBy(Vec2f v)
{
	t += v.x;
	x += v.y;
}

QDebug operator<<(QDebug dbg, const Keyframe &k)
{
	dbg.nospace() << "(" << k.t << ", " << k.x << ", " << k.v << ", " << k.type << ")";

	return dbg.space();
};

