#include "keyframe_player/Keyframe.h"
#include <QDebug>

namespace kf_player
{

void Keyframe::set(double t, double x, double v, double effort, double suppLeftLeg, double suppRightLeg, double pGain, double iGain, double dGain, double limit, gainSelectEnum gainSelect, double roll, double pitch, double yaw)
{
	type = TYPE_DEFAULT;
	this->t = t;
	this->x = x;
	this->v = v;
	this->a = 0;
	this->effort = effort;
	this->suppLeftLeg = suppLeftLeg;
	this->suppRightLeg = suppRightLeg;
	this->pGain=pGain;
	this->iGain=iGain;
	this->dGain=dGain;
	this->limit=limit;
	this->gainSelect=gainSelect;
	this->roll=roll;
	this->pitch=pitch;
	this->yaw=yaw;
	this->yaw=pGain;
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
