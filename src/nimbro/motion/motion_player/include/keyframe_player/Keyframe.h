#ifndef KEYFRAME_H_
#define KEYFRAME_H_
#include "keyframe_player/Vec2f.h"

namespace kf_player
{

class Keyframe
{
public:
	Keyframe() : type(TYPE_DEFAULT), t(0.0), x(0.0), v(0.0), a(0.0), effort(0.0), suppLeftLeg(0.0), suppRightLeg(0.0) {}
	Keyframe(double t, double x, double v) : type(TYPE_DEFAULT), t(t), x(x), v(v), a(0.0), effort(0.0), suppLeftLeg(0.0), suppRightLeg(0.0) {}
	Keyframe(double t, double x, double v, double effort) : type(TYPE_DEFAULT), t(t), x(x), v(v), a(0.0), effort(effort), suppLeftLeg(0.0), suppRightLeg(0.0) {}
	Keyframe(double t, double x, double v, double effort, double suppLeftLeg, double suppRightLeg) : type(TYPE_DEFAULT), t(t), x(x), v(v), a(0.0), effort(effort), suppLeftLeg(suppLeftLeg), suppRightLeg(suppRightLeg) {}
	virtual ~Keyframe() {};

	int type;
	double t;
	double x;
	double v;
	double a;
	double effort;
	double suppLeftLeg;
	double suppRightLeg;

	enum types
	{
		TYPE_DEFAULT,
		TYPE_AMAX,
		TYPE_VMAX,
		TYPE_UNREACHABLE
	};

	void set(double t, double x, double v, double effort, double suppLeftLeg, double suppRightLeg);
	Vec2f location();
	void setLocation(Vec2f);
	void relocateBy(Vec2f);

	inline bool operator<(const Keyframe& k) const {return (t < k.t);}
	inline bool operator<=(const Keyframe& k) const {return (t <= k.t);}
	inline bool operator>(const Keyframe& k) const {return (t > k.t);}
	inline bool operator>=(const Keyframe& v) const {return (t >= v.t);}
	inline bool operator==(const Keyframe& k) const {return (t == k.t && x == k.x && v == k.v);}
	inline bool operator!=(const Keyframe& k) const {return (t != k.t || x != k.x || v != k.v);}
};

QDebug operator<<(QDebug dbg, const Keyframe &k);

#endif /* KEYFRAME_H_ */

}