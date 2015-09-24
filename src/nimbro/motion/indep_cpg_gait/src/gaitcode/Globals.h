#ifndef GLOBS_H_
#define GLOBS_H_

namespace indep_cpg_gait
{
	const double PI = 3.1415926535897932384626433832795;
	const double PI2 = 1.5707963267948965579989817342721;
	const double SPI = 1.7724538509055160272981674833411; // sqrt of pi
	const double EPSILON = 0.000001;

	template <typename T>
	inline T sgn(const T a) { return (a == 0 ? 0 : a < 0 ? -1 : 1); }
	inline double picut(double x) { return  x < 0 ? fmod(x-PI, 2*PI)+PI : fmod(x+PI, 2*PI)-PI;}
	}
#endif /* GLOBS_H_ */
