#ifndef LIMP_H_
#define LIMP_H_

namespace margait_contrib
{

class Limp;

extern Limp operator*(const double scalar, const Limp& l);
extern Limp operator*(const Limp& l, const double scalar);
extern Limp operator/(const double scalar, const Limp& l);
extern Limp operator/(const Limp& l, const double scalar);


class Limp
{
public:

	double C;
	double x0, v0;

	Limp();
	Limp(double x, double v, double C);
	~Limp(){};

	void set(double x, double v);
	void set(double x, double v, double C);

	Limp predict(double time);
	void update(double time);
	void simUpdate(double time);
	void reset();

	double energy();
	static double energy(double x, double v, double C);
	double tLoc(double x);
	double tVel(double vx);
	double vx(double x);
	double x(double vx);
	double origin(double x, double vx);
	double z(double x, double T);

	inline Limp operator+(const Limp& l) const {return Limp(x0+l.x0, v0+l.v0, C);}
	inline Limp operator-() const {return Limp(-x0, -v0, C);}
	inline Limp operator-(const Limp& l) const {return Limp(x0-l.x0, v0-l.v0, C);}
	inline Limp& operator*=(const double scalar){x0*=scalar; v0*=scalar; return *this;}
	inline Limp& operator/=(const double scalar){x0/=scalar; v0/=scalar; return *this;}
	inline Limp& operator+=(const Limp& l){x0+=l.x0; v0+=l.v0; return *this;}
	inline Limp& operator-=(const Limp& l){x0-=l.x0; v0-=l.v0; return *this;}

};

}

#endif /* LIMP_H_ */
