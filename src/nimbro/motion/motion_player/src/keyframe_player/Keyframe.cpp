#include "keyframe_player/Keyframe.h"
#include <QDebug>

namespace kf_player
{

void Keyframe::set(double t, double x, double v, double effort, double suppLeftLeg, double suppRightLeg)
{
	type = TYPE_DEFAULT;
	this->t = t;
	this->x = x;
	this->v = v;
	this->a = 0;
	this->effort = effort;
	this->suppLeftLeg = suppLeftLeg;
	this->suppRightLeg = suppRightLeg;
}

Vec2f Keyframe::location()
{
	return Vec2f(t, x);
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


}