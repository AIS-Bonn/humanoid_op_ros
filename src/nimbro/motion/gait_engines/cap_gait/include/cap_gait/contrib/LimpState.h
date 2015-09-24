#ifndef LIMPSTATE_H_
#define LIMPSTATE_H_

namespace margait_contrib
{

struct LimpState;

extern LimpState operator*(const double scalar, const LimpState& v);
extern LimpState operator*(const LimpState& v, const double scalar);

struct LimpState
{
	double x;
	double vx;
	double ax;
	double y;
	double vy;
	double ay;
	double energyX;
	double energyY;
	int supportLegSign;

	LimpState()
	{
		reset();
	}

	LimpState(double x, double y, double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0)
	{
		set(x, y, vx, vy, ax, ay);
	}

	void set(double x, double y, double vx = 0.0, double vy = 0.0, double ax = 0.0, double ay = 0.0)
	{
		this->x = x;
		this->vx = vx;
		this->ax = ax;
		this->y = y;
		this->vy = vy;
		this->ay = ay;
		energyX = 0;
		energyY = 0;
		supportLegSign = 1;
	}

	void reset()
	{
		x = 0;
		vx = 0;
		ax = 0;
		y = 0;
		vy = 0;
		ay = 0;
		energyX = 0;
		energyY = 0;
		supportLegSign = 1;
	}

	inline LimpState operator+(const LimpState& v) const
	{
		LimpState lm = *this;
		lm.x += v.x;
		lm.vx += v.vx;
		lm.y += v.y;
		lm.vy += v.vy;
		return lm;
	}
	inline LimpState operator-() const
	{
		LimpState lm = *this;
		lm.x = -lm.x;
		lm.vx = -lm.vx;
		lm.y = -lm.y;
		lm.vy = -lm.vy;

		return lm;
	}
	inline LimpState operator-(const LimpState& v) const
	{
		LimpState lm = *this;
		lm.x -= v.x;
		lm.vx -= v.vx;
		lm.y -= v.y;
		lm.vy -= v.vy;
		return lm;
	}
	inline void operator+=(const LimpState& v)
	{
		x+=v.x;
		vx+=v.vx;
		y+=v.y;
		vy+=v.vy;
	}
	inline void operator-=(const LimpState& v)
	{
		x-=v.x;
		vx-=v.vx;
		y-=v.y;
		vy-=v.vy;
	}
	inline void operator*=(const double scalar)
	{
		x*=scalar;
		vx*=scalar;
		y*=scalar;
		vy*=scalar;
	}
	inline void operator/=(const double scalar)
	{
		x/=scalar;
		vx/=scalar;
		y/=scalar;
		vy/=scalar;
	}
	inline bool operator==(const LimpState& v) const
	{
		return (x==v.x) && (vx==v.vx) && (y==v.y) && (vy==v.vy);
	}
	inline bool operator!=(const LimpState& v) const
	{
		return (x!=v.x) || (vx!=v.vx) || (y!=v.y) || (vy!=v.vy);
	}
};

}

#endif
