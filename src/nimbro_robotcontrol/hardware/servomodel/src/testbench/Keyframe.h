#ifndef KEYFRAME_H_
#define KEYFRAME_H_
#include "Vec2f.h"
#include <QDebug>

class Keyframe
{
public:
	Keyframe();
	Keyframe(double t, double x, double v);
	~Keyframe(){};

	int type;
	double t;
	double x;
	double v;
	double a;
	bool highlight;

	enum types
	{
		TYPE_DEFAULT,
		TYPE_AMAX,
		TYPE_VMAX,
		TYPE_UNREACHABLE
	};

	void set(double t, double x, double v);
	Vec2f location();
	void setLocation(Vec2f);
	void relocateBy(Vec2f);

	inline bool operator<(const Keyframe& v) const {return (t < v.t);}
	inline bool operator<=(const Keyframe& v) const {return (t <= v.t);}
	inline bool operator>(const Keyframe& v) const {return (t > v.t);}
	inline bool operator>=(const Keyframe& v) const {return (t >= v.t);}
	inline bool operator==(const Keyframe& v) const {return (t == v.t);}
	inline bool operator!=(const Keyframe& v) const {return (t != v.t);}
};

QDebug operator<<(QDebug dbg, const Keyframe &k);

#endif /* KEYFRAME_H_ */
