// Unit testing of the math robotcontrol utilities headers
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/math_funcs.h>
#include <rc_utils/math_spline.h>
#include <rc_utils/math_vec_mat.h>
#include <test_utilities/test_misc.h>
#include <gtest/gtest.h>
#include <cmath>

// Namespaces
using namespace rc_utils;
using namespace testutilities;

//
// Math functions tests
//

// Test: picut, picutMod, picutMax, picutMin
TEST(MathFuncsTest, test_picut)
{
	// Constants
	double eps = 1e-15; // Must be larger than epsilon(2*pi)
	double tmp = 0.0;

	// Test picut
	EXPECT_DOUBLE_EQ(0.0, picut(-M_2PI));
	EXPECT_DOUBLE_EQ(M_PI_2, picut(-3.0*M_PI_2));
	EXPECT_DOUBLE_EQ(M_PI, picut(-M_PI));
	EXPECT_DOUBLE_EQ(-M_PI + eps, picut(-M_PI + eps));
	EXPECT_DOUBLE_EQ(-M_PI_2, picut(-M_PI_2));
	EXPECT_DOUBLE_EQ(0.0, picut(0.0));
	EXPECT_DOUBLE_EQ(M_PI_2, picut(M_PI_2));
	EXPECT_DOUBLE_EQ(M_PI, picut(M_PI));
	EXPECT_DOUBLE_EQ(-M_PI + eps, picut(M_PI + eps));
	EXPECT_DOUBLE_EQ(-M_PI_2, picut(3.0*M_PI_2));
	EXPECT_DOUBLE_EQ(0.0, picut(M_2PI));

	// Test picutMod
	EXPECT_DOUBLE_EQ(M_2PI - eps, picutMod(-M_2PI - eps));
	EXPECT_DOUBLE_EQ(0.0, picutMod(-M_2PI));
	EXPECT_DOUBLE_EQ(M_PI_2, picutMod(-3.0*M_PI_2));
	EXPECT_DOUBLE_EQ(M_PI, picutMod(-M_PI));
	EXPECT_DOUBLE_EQ(3.0*M_PI_2, picutMod(-M_PI_2));
	EXPECT_DOUBLE_EQ(0.0, picutMod(0.0));
	EXPECT_DOUBLE_EQ(M_PI_2, picutMod(M_PI_2));
	EXPECT_DOUBLE_EQ(M_PI, picutMod(M_PI));
	EXPECT_DOUBLE_EQ(3.0*M_PI_2, picutMod(3.0*M_PI_2));
	EXPECT_DOUBLE_EQ(M_2PI - eps, picutMod(M_2PI - eps));
	EXPECT_DOUBLE_EQ(0.0, picutMod(M_2PI));

	// Test picutMax
	EXPECT_DOUBLE_EQ(M_2PI, picutMax(-M_2PI, 10.0));
	EXPECT_DOUBLE_EQ(M_2PI + M_PI_2, picutMax(-3.0*M_PI_2, 10.0));
	EXPECT_DOUBLE_EQ(M_2PI + M_PI, picutMax(-M_PI, 10.0));
	EXPECT_DOUBLE_EQ(M_2PI - M_PI_2, picutMax(-M_PI_2, 10.0));
	EXPECT_DOUBLE_EQ(M_2PI, picutMax(0.0, 10.0));
	EXPECT_DOUBLE_EQ(M_2PI + M_PI_2, picutMax(M_PI_2, 10.0));
	EXPECT_DOUBLE_EQ(M_2PI + M_PI, picutMax(M_PI, 10.0));
	EXPECT_DOUBLE_EQ(M_2PI - M_PI_2, picutMax(3.0*M_PI_2, 10.0));
	EXPECT_DOUBLE_EQ(M_2PI, picutMax(M_2PI, 10.0));
	EXPECT_DOUBLE_EQ(10.0, picutMax(10.0 - M_2PI, 10.0));
	EXPECT_DOUBLE_EQ((10.0 - M_2PI) + eps, picutMax((10.0 - M_2PI) + eps, 10.0));
	EXPECT_DOUBLE_EQ(10.0, picutMax(10.0, 10.0));
	EXPECT_DOUBLE_EQ((10.0 - M_2PI) + eps, picutMax(10.0 + eps, 10.0));

	// Test picutMin
	EXPECT_DOUBLE_EQ(M_2PI, picutMin(-M_2PI, 4.0));
	EXPECT_DOUBLE_EQ(M_2PI + M_PI_2, picutMin(-3.0*M_PI_2, 4.0));
	EXPECT_DOUBLE_EQ(M_2PI + M_PI, picutMin(-M_PI, 4.0));
	EXPECT_DOUBLE_EQ(M_2PI - M_PI_2, picutMin(-M_PI_2, 4.0));
	EXPECT_DOUBLE_EQ(M_2PI, picutMin(0.0, 4.0));
	EXPECT_DOUBLE_EQ(M_2PI + M_PI_2, picutMin(M_PI_2, 4.0));
	EXPECT_DOUBLE_EQ(M_2PI + M_PI, picutMin(M_PI, 4.0));
	EXPECT_DOUBLE_EQ(M_2PI - M_PI_2, picutMin(3.0*M_PI_2, 4.0));
	EXPECT_DOUBLE_EQ(M_2PI, picutMin(M_2PI, 4.0));
	EXPECT_DOUBLE_EQ(4.0 + M_2PI, picutMin(4.0, 4.0));
	EXPECT_DOUBLE_EQ(4.0 + eps, picutMin(4.0 + eps, 4.0));
	EXPECT_DOUBLE_EQ(4.0 + M_2PI, picutMin(4.0 + M_2PI, 4.0));
	EXPECT_DOUBLE_EQ(4.0 + eps, picutMin(4.0 + M_2PI + eps, 4.0));

	// Test picutVar
	tmp = -M_2PI; picutVar(tmp); EXPECT_DOUBLE_EQ(0.0, tmp);
	tmp = -3.0*M_PI_2; picutVar(tmp); EXPECT_DOUBLE_EQ(M_PI_2, tmp);
	tmp = -M_PI; picutVar(tmp); EXPECT_DOUBLE_EQ(M_PI, tmp);
	tmp = -M_PI + eps; picutVar(tmp); EXPECT_DOUBLE_EQ(-M_PI + eps, tmp);
	tmp = -M_PI_2; picutVar(tmp); EXPECT_DOUBLE_EQ(-M_PI_2, tmp);
	tmp = 0.0; picutVar(tmp); EXPECT_DOUBLE_EQ(0.0, tmp);
	tmp = M_PI_2; picutVar(tmp); EXPECT_DOUBLE_EQ(M_PI_2, tmp);
	tmp = M_PI; picutVar(tmp); EXPECT_DOUBLE_EQ(M_PI, tmp);
	tmp = M_PI + eps; picutVar(tmp); EXPECT_DOUBLE_EQ(-M_PI + eps, tmp);
	tmp = 3.0*M_PI_2; picutVar(tmp); EXPECT_DOUBLE_EQ(-M_PI_2, tmp);
	tmp = M_2PI; picutVar(tmp); EXPECT_DOUBLE_EQ(0.0, tmp);

	// Test picutVarMod
	tmp = -M_2PI - eps; picutVarMod(tmp); EXPECT_DOUBLE_EQ(M_2PI - eps, tmp);
	tmp = -M_2PI; picutVarMod(tmp); EXPECT_DOUBLE_EQ(0.0, tmp);
	tmp = -3.0*M_PI_2; picutVarMod(tmp); EXPECT_DOUBLE_EQ(M_PI_2, tmp);
	tmp = -M_PI; picutVarMod(tmp); EXPECT_DOUBLE_EQ(M_PI, tmp);
	tmp = -M_PI_2; picutVarMod(tmp); EXPECT_DOUBLE_EQ(3.0*M_PI_2, tmp);
	tmp = 0.0; picutVarMod(tmp); EXPECT_DOUBLE_EQ(0.0, tmp);
	tmp = M_PI_2; picutVarMod(tmp); EXPECT_DOUBLE_EQ(M_PI_2, tmp);
	tmp = M_PI; picutVarMod(tmp); EXPECT_DOUBLE_EQ(M_PI, tmp);
	tmp = 3.0*M_PI_2; picutVarMod(tmp); EXPECT_DOUBLE_EQ(3.0*M_PI_2, tmp);
	tmp = M_2PI - eps; picutVarMod(tmp); EXPECT_DOUBLE_EQ(M_2PI - eps, tmp);
	tmp = M_2PI; picutVarMod(tmp); EXPECT_DOUBLE_EQ(0.0, tmp);

	// Test picutVarMax
	tmp = -M_2PI; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI, tmp);
	tmp = -3.0*M_PI_2; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI + M_PI_2, tmp);
	tmp = -M_PI; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI + M_PI, tmp);
	tmp = -M_PI_2; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI - M_PI_2, tmp);
	tmp = 0.0; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI, tmp);
	tmp = M_PI_2; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI + M_PI_2, tmp);
	tmp = M_PI; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI + M_PI, tmp);
	tmp = 3.0*M_PI_2; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI - M_PI_2, tmp);
	tmp = M_2PI; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(M_2PI, tmp);
	tmp = 10.0 - M_2PI; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(10.0, tmp);
	tmp = (10.0 - M_2PI) + eps; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ((10.0 - M_2PI) + eps, tmp);
	tmp = 10.0; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ(10.0, tmp);
	tmp = 10.0 + eps; picutVarMax(tmp, 10.0); EXPECT_DOUBLE_EQ((10.0 - M_2PI) + eps, tmp);

	// Test picutVarMin
	tmp = -M_2PI; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI, tmp);
	tmp = -3.0*M_PI_2; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI + M_PI_2, tmp);
	tmp = -M_PI; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI + M_PI, tmp);
	tmp = -M_PI_2; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI - M_PI_2, tmp);
	tmp = 0.0; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI, tmp);
	tmp = M_PI_2; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI + M_PI_2, tmp);
	tmp = M_PI; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI + M_PI, tmp);
	tmp = 3.0*M_PI_2; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI - M_PI_2, tmp);
	tmp = M_2PI; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(M_2PI, tmp);
	tmp = 4.0; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(4.0 + M_2PI, tmp);
	tmp = 4.0 + eps; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(4.0 + eps, tmp);
	tmp = 4.0 + M_2PI; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(4.0 + M_2PI, tmp);
	tmp = 4.0 + M_2PI + eps; picutVarMin(tmp, 4.0); EXPECT_DOUBLE_EQ(4.0 + eps, tmp);
}

// Test: sign, sign0
TEST(MathFuncsTest, test_sign)
{
	// Check that we are getting the expected values (we want exact comparisons to be possible)
	EXPECT_EQ( 1, sign ( 5.0));
	EXPECT_EQ( 1, sign ( 0.0));
	EXPECT_EQ(-1, sign (-5.0));
	EXPECT_EQ( 1, sign0( 5.0));
	EXPECT_EQ( 0, sign0( 0.0));
	EXPECT_EQ(-1, sign0(-5.0));

	// Check the overloads based on type
	int a = 0;
	unsigned short b = 0;
	float c = 0.1;
	double d = 0.1;
	EXPECT_EQ( 1, sign(a));
	EXPECT_EQ( 0, sign0(a));
	EXPECT_EQ( 1, sign(b));
	EXPECT_EQ( 0, sign0(b));
	EXPECT_EQ( 1, sign(c));
	EXPECT_EQ( 1, sign0(c));
	EXPECT_EQ( 1, sign(d));
	EXPECT_EQ( 1, sign0(d));
}

// Test class
struct TestCoerce
{
	template<typename T> void testCoerce(T exp, bool expcoerced, T x, T min, T max)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, min, max);
		EXPECT_THAT_EQ(T, exp, coerce(x, min, max));
		EXPECT_THAT_EQ(T, exp, coerce(x, min, max, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceAbs(T exp, bool expcoerced, T x, T maxAbs)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, maxAbs);
		EXPECT_THAT_EQ(T, exp, coerceAbs(x, maxAbs));
		EXPECT_THAT_EQ(T, exp, coerceAbs(x, maxAbs, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceMax(T exp, bool expcoerced, T x, T max)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, max);
		EXPECT_THAT_EQ(T, exp, coerceMax(x, max));
		EXPECT_THAT_EQ(T, exp, coerceMax(x, max, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceMin(T exp, bool expcoerced, T x, T min)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, min);
		EXPECT_THAT_EQ(T, exp, coerceMin(x, min));
		EXPECT_THAT_EQ(T, exp, coerceMin(x, min, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceEllP(T exp, bool expcoerced, T r, T theta, T maxAbsX, T maxAbsY)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, r, theta, maxAbsX, maxAbsY);
		EXPECT_THAT_EQ(T, exp, coerceEllP(r, theta, maxAbsX, maxAbsY));
		EXPECT_THAT_EQ(T, exp, coerceEllP(r, theta, maxAbsX, maxAbsY, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceEllC(T expX, T expY, bool expcoerced, T x, T y, T maxAbsX, T maxAbsY)
	{
		T retX, retY;
		bool coerced;
		std::string args = argsAsString(typeid(T).name(), expX, expY, expcoerced, x, y, maxAbsX, maxAbsY);
		retX = x; retY = y;
		coerceEllC(retX, retY, maxAbsX, maxAbsY);
		EXPECT_THAT_UTEQ(T, expX, retX);
		EXPECT_THAT_UTEQ(T, expY, retY);
		retX = x; retY = y; coerced = !expcoerced;
		coerceEllC(retX, retY, maxAbsX, maxAbsY, coerced);
		EXPECT_THAT_UTEQ(T, expX, retX);
		EXPECT_THAT_UTEQ(T, expY, retY);
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
		retX = NAN; retY = NAN;
		coerceEllC(x, y, maxAbsX, maxAbsY, retX, retY);
		EXPECT_THAT_UTEQ(T, expX, retX);
		EXPECT_THAT_UTEQ(T, expY, retY);
		retX = NAN; retY = NAN; coerced = !expcoerced;
		coerceEllC(x, y, maxAbsX, maxAbsY, retX, retY, coerced);
		EXPECT_THAT_UTEQ(T, expX, retX);
		EXPECT_THAT_UTEQ(T, expY, retY);
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
};

// Test: coerce, coerceAbs, coerceMax, coerceMin, coerceEllP, coerceEllC
TEST(MathFuncsTest, test_coerce)
{
	// Test class
	TestCoerce T;

	// Test coerce
	T.testCoerce(2, true, 1, 2, 7);
	T.testCoerce(2, false, 2, 2, 7);
	T.testCoerce(5, false, 5, 2, 7);
	T.testCoerce(7, false, 7, 2, 7);
	T.testCoerce(7, true, 9, 2, 7);
	T.testCoerce(4, true, 6, 7, 2);
	T.testCoerce(4, true, 4, 7, 2);
	T.testCoerce(2.0, true, 1.0, 2.0, 7.0);
	T.testCoerce(2.0, false, 2.0, 2.0, 7.0);
	T.testCoerce(5.0, false, 5.0, 2.0, 7.0);
	T.testCoerce(7.0, false, 7.0, 2.0, 7.0);
	T.testCoerce(7.0, true, 9.0, 2.0, 7.0);
	T.testCoerce(4.5, true, 6.0, 7.0, 2.0);
	T.testCoerce(4.5, true, 4.5, 7.0, 2.0);

	// Test coerceAbs
	T.testCoerceAbs(-8.0, true, -9.0, 8.0);
	T.testCoerceAbs(-8.0, false, -8.0, 8.0);
	T.testCoerceAbs(1.0, false, 1.0, 8.0);
	T.testCoerceAbs(8.0, false, 8.0, 8.0);
	T.testCoerceAbs(8.0, true, 9.0, 8.0);
	T.testCoerceAbs(0.0, true, -9.0, -8.0);
	T.testCoerceAbs(0.0, true, 0.0, -8.0);

	// Test coerceMax
	T.testCoerceMax(-5.0, true, 0.0, -5.0);
	T.testCoerceMax(-5.0, false, -5.0, -5.0);
	T.testCoerceMax(-8.0, false, -8.0, -5.0);

	// Test coerceMin
	T.testCoerceMin(0.0, false, 0.0, -5.0);
	T.testCoerceMin(-5.0, false, -5.0, -5.0);
	T.testCoerceMin(-5.0, true, -8.0, -5.0);

	// Test coerceEllP
	T.testCoerceEllP(0.7, false, 0.7, 1.9, 2.0, 5.0);
	T.testCoerceEllP(4.0177692190172261, true, 7.0, 1.9, 2.0, 5.0);
	T.testCoerceEllP(-4.0177692190172261, true, -6.0, 1.9, 2.0, 5.0);
	T.testCoerceEllP(2.0, true, 7.0, 0.0, 2.0, 5.0);
	T.testCoerceEllP(5.0, true, 7.0, M_PI_2, 2.0, 5.0);
	T.testCoerceEllP(2.0, true, 7.0, M_PI, 2.0, 5.0);
	T.testCoerceEllP(5.0, true, 7.0, 3.0*M_PI_2, 2.0, 5.0);
	T.testCoerceEllP(2.0, true, 7.0, 0.0, 2.0, 0.0);
	T.testCoerceEllP(0.0, true, 7.0, 0.01, 2.0, 0.0);
	T.testCoerceEllP(0.0, true, 7.0, -0.01, 2.0, 0.0);
	T.testCoerceEllP(0.0, true, 7.0, 1.57, 0.0, 5.0);
	T.testCoerceEllP(0.0, true, 7.0, 1.571, 0.0, 5.0);
	T.testCoerceEllP(0.0, true, 7.0, 1.2, 0.0, 0.0);

	// Test coerceEllC
	T.testCoerceEllC(0.7, 1.1, false, 0.7, 1.1, 2.0, 5.0);
	T.testCoerceEllC(-1.2803687993289596, 3.8411063979868789, true, -2.2, 6.6, 2.0, 5.0);
	T.testCoerceEllC(-1.2803687993289596, -3.8411063979868789, true, -3.3, -9.9, 2.0, 5.0);
	T.testCoerceEllC(2.0, 0.0, true, 7.0, 0.0, 2.0, 5.0);
	T.testCoerceEllC(0.0, 5.0, true, 0.0, 7.0, 2.0, 5.0);
	T.testCoerceEllC(-2.0, 0.0, true, -7.0, 0.0, 2.0, 5.0);
	T.testCoerceEllC(0.0, -5.0, true, 0.0, -7.0, 2.0, 5.0);
	T.testCoerceEllC(2.0, 0.0, true, 7.0, 0.0, 2.0, 0.0);
	T.testCoerceEllC(0.0, 0.0, true, 7.0, 0.01, 2.0, 0.0);
	T.testCoerceEllC(0.0, 0.0, true, 7.0, -0.01, 2.0, 0.0);
	T.testCoerceEllC(0.0, 5.0, true, 0.0, 7.0, 0.0, 5.0);
	T.testCoerceEllC(0.0, 0.0, true, 0.01, 7.0, 0.0, 5.0);
	T.testCoerceEllC(0.0, 0.0, true, -0.01, 7.0, 0.0, 5.0);
	T.testCoerceEllC(0.0, 0.0, true, 1.3, 5.4, 0.0, 0.0);
}

// Test: EllipseAxes, collapseFlatEllipse
TEST(MathFuncsTest, test_collapseFlatEllipse)
{
	// Declare variables
	EllipseAxesd tmp;
	tmp.a = tmp.b = NAN;

	// Test a case
	const EllipseAxesd EA1(3.0, 4.0);
	EXPECT_EQ(3.0, EA1.a);
	EXPECT_EQ(4.0, EA1.b);
	tmp = EA1.collapsedFlat();
	EXPECT_EQ(3.0, tmp.a);
	EXPECT_EQ(4.0, tmp.b);
	tmp = EA1;
	EXPECT_EQ(3.0, tmp.a);
	EXPECT_EQ(4.0, tmp.b);
	tmp.collapseFlat();
	EXPECT_EQ(3.0, tmp.a);
	EXPECT_EQ(4.0, tmp.b);
	tmp = collapseFlatEllipse(EA1.a, EA1.b);
	EXPECT_EQ(3.0, tmp.a);
	EXPECT_EQ(4.0, tmp.b);

	// Test a case
	const EllipseAxesd EA2(0.0, 4.0);
	EXPECT_EQ(0.0, EA2.a);
	EXPECT_EQ(4.0, EA2.b);
	tmp = EA2.collapsedFlat();
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);
	tmp = EA2;
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(4.0, tmp.b);
	tmp.collapseFlat();
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);
	tmp = collapseFlatEllipse(EA2.a, EA2.b);
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);

	// Test a case
	const EllipseAxesd EA3(3.0, 0.0);
	EXPECT_EQ(3.0, EA3.a);
	EXPECT_EQ(0.0, EA3.b);
	tmp = EA3.collapsedFlat();
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);
	tmp = EA3;
	EXPECT_EQ(3.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);
	tmp.collapseFlat();
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);
	tmp = collapseFlatEllipse(EA3.a, EA3.b);
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);

	// Test a case
	const EllipseAxesd EA4(0.0, 0.0);
	EXPECT_EQ(0.0, EA4.a);
	EXPECT_EQ(0.0, EA4.b);
	tmp = EA4.collapsedFlat();
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);
	tmp = EA4;
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);
	tmp.collapseFlat();
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);
	tmp = collapseFlatEllipse(EA4.a, EA4.b);
	EXPECT_EQ(0.0, tmp.a);
	EXPECT_EQ(0.0, tmp.b);

	// Test a case
	EllipseAxesd EA5(3.0, 0.0, true);
	EXPECT_EQ(0.0, EA5.a);
	EXPECT_EQ(0.0, EA5.b);
	EllipseAxesd EA6(0.0, 4.0, false);
	EXPECT_EQ(0.0, EA6.a);
	EXPECT_EQ(4.0, EA6.b);
	EllipseAxesd EA7(3.0, 4.0, false);
	EXPECT_EQ(3.0, EA7.a);
	EXPECT_EQ(4.0, EA7.b);
}

// Test: ellipseRadius
TEST(MathFuncsTest, test_ellipseRadius)
{
	// Test the polar form
	EXPECT_DOUBLE_EQ(4.0, ellipseRadius(4.0, 6.0, 0.0));
	EXPECT_DOUBLE_EQ(6.0, ellipseRadius(4.0, 6.0, M_PI_2));
	EXPECT_DOUBLE_EQ(4.0, ellipseRadius(4.0, 6.0, M_PI));
	EXPECT_DOUBLE_EQ(6.0, ellipseRadius(4.0, 6.0, 3*M_PI_2));
	EXPECT_DOUBLE_EQ(2.1552636243212988, ellipseRadius(-3.0, 2.0, M_PI/3));
	EXPECT_DOUBLE_EQ(2.3533936216582081, ellipseRadius(-3.0, 2.0, 3*M_PI_4));
	EXPECT_DOUBLE_EQ(2.6186146828319088, ellipseRadius(-3.0, 2.0, 7*M_PI/6));
	EXPECT_DOUBLE_EQ(2.3533936216582081, ellipseRadius(-3.0, 2.0, -M_PI_4));
	EXPECT_DOUBLE_EQ(4.0, ellipseRadius(4.0, 0.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(4.0, 0.0, 0.01));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(4.0, 0.0, -0.01));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 6.0, 1.57));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 6.0, 1.571));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 0.0, 1.3));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 0.0, 0.0));

	// Test the cartesian form
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(4.0, 6.0, 0.0, 0.0));
	EXPECT_DOUBLE_EQ(4.0, ellipseRadius(4.0, 6.0, 2.0, 0.0));
	EXPECT_DOUBLE_EQ(6.0, ellipseRadius(4.0, 6.0, 0.0, -0.5));
	EXPECT_DOUBLE_EQ(4.7067872433164171, ellipseRadius(4.0, 6.0, 2.0, 2.0));
	EXPECT_DOUBLE_EQ(5.6568542494923806, ellipseRadius(4.0, 6.0, -1.0, 3.0));
	EXPECT_DOUBLE_EQ(4.1159660434202126, ellipseRadius(4.0, 6.0, 3.0, 1.0));
	EXPECT_DOUBLE_EQ(4.2426406871192848, ellipseRadius(4.0, -6.0, -2.0, 1.0));
	EXPECT_DOUBLE_EQ(4.0, ellipseRadius(4.0, 0.0, 5.0, 0.0));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(4.0, 0.0, 5.0, 0.01));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(4.0, 0.0, 5.0, -0.01));
	EXPECT_DOUBLE_EQ(6.0, ellipseRadius(0.0, -6.0, 0.0, 0.1));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, -6.0, 0.001, 0.1));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, -6.0, -0.001, 0.1));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 0.0, 1.4, 0.7));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 0.0, -0.3, 0.0));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 0.0, 0.0, 0.7));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 0.0, 0.0, 0.0));
}

// Test class
struct TestCoerceSoft
{
	template<typename T> void testCoerceSoft(T exp, bool expcoerced, T x, T min, T max, T buf)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, min, max, buf);
		EXPECT_THAT_EQ(T, exp, coerceSoft(x, min, max, buf));
		EXPECT_THAT_EQ(T, exp, coerceSoft(x, min, max, buf, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceSoftAbs(T exp, bool expcoerced, T x, T maxAbs, T buf)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, maxAbs, buf);
		EXPECT_THAT_EQ(T, exp, coerceSoftAbs(x, maxAbs, buf));
		EXPECT_THAT_EQ(T, exp, coerceSoftAbs(x, maxAbs, buf, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceSoftMax(T exp, bool expcoerced, T x, T max, T buf)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, max, buf);
		EXPECT_THAT_EQ(T, exp, coerceSoftMax(x, max, buf));
		EXPECT_THAT_EQ(T, exp, coerceSoftMax(x, max, buf, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceSoftMin(T exp, bool expcoerced, T x, T min, T buf)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, min, buf);
		EXPECT_THAT_EQ(T, exp, coerceSoftMin(x, min, buf));
		EXPECT_THAT_EQ(T, exp, coerceSoftMin(x, min, buf, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceSoftEllP(T exp, bool expcoerced, T r, T theta, T maxAbsX, T maxAbsY, T buf)
	{
		bool coerced = !expcoerced;
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, r, theta, maxAbsX, maxAbsY, buf);
		EXPECT_THAT_EQ(T, exp, coerceSoftEllP(r, theta, maxAbsX, maxAbsY, buf));
		EXPECT_THAT_EQ(T, exp, coerceSoftEllP(r, theta, maxAbsX, maxAbsY, buf, coerced));
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceSoftEllC(T expX, T expY, bool expcoerced, T x, T y, T maxAbsX, T maxAbsY, T buf)
	{
		T retX, retY;
		bool coerced;
		std::string args = argsAsString(typeid(T).name(), expX, expY, expcoerced, x, y, maxAbsX, maxAbsY, buf);
		retX = x; retY = y;
		coerceSoftEllC(retX, retY, maxAbsX, maxAbsY, buf);
		EXPECT_THAT_UTEQ(T, expX, retX);
		EXPECT_THAT_UTEQ(T, expY, retY);
		retX = x; retY = y; coerced = !expcoerced;
		coerceSoftEllC(retX, retY, maxAbsX, maxAbsY, buf, coerced);
		EXPECT_THAT_UTEQ(T, expX, retX);
		EXPECT_THAT_UTEQ(T, expY, retY);
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
		retX = NAN; retY = NAN;
		coerceSoftEllC(x, y, maxAbsX, maxAbsY, buf, retX, retY);
		EXPECT_THAT_UTEQ(T, expX, retX);
		EXPECT_THAT_UTEQ(T, expY, retY);
		retX = NAN; retY = NAN; coerced = !expcoerced;
		coerceSoftEllC(x, y, maxAbsX, maxAbsY, buf, retX, retY, coerced);
		EXPECT_THAT_UTEQ(T, expX, retX);
		EXPECT_THAT_UTEQ(T, expY, retY);
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
};

// Test: coerceSoft, coerceSoftAbs, coerceSoftMax, coerceSoftMin, coerceSoftEllP, coerceSoftEllC
TEST(MathFuncsTest, test_coerceSoft)
{
	// Test class
	TestCoerceSoft T;

	// Test coerceSoft
	T.testCoerceSoft(2.0248935341839318, true, 1.0, 2.0, 8.0, 0.5);
	T.testCoerceSoft(7.9751064658160677, true, 9.0, 2.0, 8.0, 0.5);
	T.testCoerceSoft(5.0, false, 5.0, 2.0, 8.0, 0.5);
	T.testCoerceSoft(2.0, true, 1.0, 2.0, 8.0, 0.0);
	T.testCoerceSoft(5.0, false, 5.0, 2.0, 8.0, 0.0);
	T.testCoerceSoft(8.0, true, 9.0, 2.0, 8.0, 0.0);
	T.testCoerceSoft(3.5472799993675128, true, 1.0, 2.0, 8.0, 10.0);
	T.testCoerceSoft(5.0000000000000000, false, 5.0, 2.0, 8.0, 10.0);
	T.testCoerceSoft(6.4527200006324872, true, 9.0, 2.0, 8.0, 10.0);

	// Test coerceSoftAbs
	T.testCoerceSoftAbs(-7.3835075901459835, true, -9.0, 8.0, 2.5);
	T.testCoerceSoftAbs(7.3835075901459835, true, 9.0, 8.0, 2.5);
	T.testCoerceSoftAbs(1.0, false, 1.0, 8.0, 2.5);
	T.testCoerceSoftAbs(-8.0, true, -9.0, 8.0, 0.0);
	T.testCoerceSoftAbs(1.0, false, 1.0, 8.0, 0.0);
	T.testCoerceSoftAbs(8.0, true, 9.0, 8.0, 0.0);
	T.testCoerceSoftAbs(0.0000000000000000, false, 0.0, 8.0, 10.0);
	T.testCoerceSoftAbs(0.7782291165556883, true, 1.0, 8.0, 10.0);
	T.testCoerceSoftAbs(3.2042274025080628, true, 5.0, 8.0, 10.0);
	T.testCoerceSoftAbs(4.8152638489847330, true, 9.0, 8.0, 10.0);

	// Test coerceSoftMax
	T.testCoerceSoftMax(-5.5518191617571633, true, -5.0, -5.0, 1.5);
	T.testCoerceSoftMax(-6.5, false, -6.5, -5.0, 1.5);
	T.testCoerceSoftMax(-5.0, true, -4.5, -5.0, 0.0);
	T.testCoerceSoftMax(-6.5, false, -6.5, -5.0, 0.0);
	T.testCoerceSoftMax(-5.0, true, -4.5, -5.0, -1.0);
	T.testCoerceSoftMax(-6.5, false, -6.5, -5.0, -1.0);

	// Test coerceSoftMin
	T.testCoerceSoftMin(-4.4481808382428367, true, -5.0, -5.0, 1.5);
	T.testCoerceSoftMin(-3.5, false, -3.5, -5.0, 1.5);
	T.testCoerceSoftMin(-5.0, true, -5.5, -5.0, 0.0);
	T.testCoerceSoftMin(-3.5, false, -3.5, -5.0, 0.0);
	T.testCoerceSoftMin(-5.0, true, -5.5, -5.0, -1.0);
	T.testCoerceSoftMin(-3.5, false, -3.5, -5.0, -1.0);

	// Test coerceSoftEllP
	T.testCoerceSoftEllP(0.7, false, 0.7, 1.9, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllP(-2.4, false, -2.4, 1.9, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllP(3.1843602357410350, true, 3.2, 1.9, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllP(3.9991252167894755, true, 7.0, 1.9, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllP(1.9975212478233337, true, 7.0, 0.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllP(4.9502129316321364, true, 7.0, M_PI_2, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllP(1.9975212478233337, true, 7.0, M_PI, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllP(4.9502129316321364, true, 7.0, 3.0*M_PI_2, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllP(1.9975212478233337, true, 7.0, 0.0, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllP(0.0, true, 7.0, 0.01, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllP(0.0, true, 7.0, -0.01, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllP(0.0, true, 7.0, 1.57, 0.0, 5.0, 1.0);
	T.testCoerceSoftEllP(0.0, true, 7.0, 1.571, 0.0, 5.0, 1.0);
	T.testCoerceSoftEllP(0.4998973901285135, true, 7.0, 1.2, 0.5, 0.5, 1.0);
	T.testCoerceSoftEllP(0.0, true, 7.0, 1.2, 0.0, 0.0, 1.0);

	// Test coerceSoftEllC
	T.testCoerceSoftEllC(0.7, 1.9, false, 0.7, 1.9, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC(0.8, -2.3, false, 0.8, -2.3, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC(-0.9980415876987310, 2.9941247630961927, true, -1.0, 3.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC(-1.2740195694222953, 3.8220587082668849, true, -2.2, 6.6, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC(1.9975212478233337, 0.0, true, 7.0, 0.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC(0.0, 4.9502129316321364, true, 0.0, 7.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC(-1.9975212478233337, 0.0, true, -7.0, 0.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC(0.0, -4.9502129316321364, true, 0.0, -7.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC(1.9975212478233337, 0.0, true, 7.0, 0.0, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllC(0.0, 0.0, true, 7.0, 0.01, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllC(0.0, 0.0, true, 7.0, -0.01, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllC(0.0, 4.9502129316321364, true, 0.0, 7.0, 0.0, 5.0, 1.0);
	T.testCoerceSoftEllC(0.0, 0.0, true, 0.01, 7.0, 0.0, 5.0, 1.0);
	T.testCoerceSoftEllC(0.0, 0.0, true, -0.01, 7.0, 0.0, 5.0, 1.0);
	T.testCoerceSoftEllC(0.2706844181193907, 0.4202731755011593, true, 3.8, 5.9, 0.5, 0.5, 1.0);
	T.testCoerceSoftEllC(0.0, 0.0, true, 3.8, 5.9, 0.0, 0.0, 1.0);
}

// Test: interpolate
TEST(MathFuncsTest, test_interpolate)
{
	// Check that we are getting the expected values for the x form
	EXPECT_DOUBLE_EQ(-0.3, interpolate(4.0, 5.0, 0.0, 1.0, 3.7));
	EXPECT_DOUBLE_EQ(0.0, interpolate(4.0, 5.0, 0.0, 1.0, 4.0));
	EXPECT_DOUBLE_EQ(0.3, interpolate(4.0, 5.0, 0.0, 1.0, 4.3));
	EXPECT_DOUBLE_EQ(0.6, interpolate(4.0, 5.0, 0.0, 1.0, 4.6));
	EXPECT_DOUBLE_EQ(1.0, interpolate(4.0, 5.0, 0.0, 1.0, 5.0));
	EXPECT_DOUBLE_EQ(1.3, interpolate(4.0, 5.0, 0.0, 1.0, 5.3));
	EXPECT_DOUBLE_EQ(-0.3, interpolate(5.0, 4.0, 1.0, 0.0, 3.7));
	EXPECT_DOUBLE_EQ(0.0, interpolate(5.0, 4.0, 1.0, 0.0, 4.0));
	EXPECT_DOUBLE_EQ(0.3, interpolate(5.0, 4.0, 1.0, 0.0, 4.3));
	EXPECT_DOUBLE_EQ(0.6, interpolate(5.0, 4.0, 1.0, 0.0, 4.6));
	EXPECT_DOUBLE_EQ(1.0, interpolate(5.0, 4.0, 1.0, 0.0, 5.0));
	EXPECT_DOUBLE_EQ(1.3, interpolate(5.0, 4.0, 1.0, 0.0, 5.3));
	EXPECT_DOUBLE_EQ(12.5606153846153852, interpolate(3.12, 1.82, 4.12, 7.93, 0.24));
	EXPECT_DOUBLE_EQ(7.6076153846153849, interpolate(3.12, 1.82, 4.12, 7.93, 1.93));
	EXPECT_DOUBLE_EQ(5.1457692307692309, interpolate(3.12, 1.82, 4.12, 7.93, 2.77));
	EXPECT_DOUBLE_EQ(4.12, interpolate(3.12, 1.82, 4.12, 7.93, 3.12));
	EXPECT_DOUBLE_EQ(-16.6884615384615387, interpolate(3.12, 1.82, 4.12, 7.93, 10.22));
	EXPECT_DOUBLE_EQ(2.0, interpolate(4.0, 4.0, 1.0, 3.0, 3.0));
	EXPECT_DOUBLE_EQ(2.0, interpolate(4.0, 4.0, 1.0, 3.0, 4.0));
	EXPECT_DOUBLE_EQ(2.0, interpolate(4.0, 4.0, 1.0, 3.0, 5.0));

	// Check that we are getting the expected values for the u form
	EXPECT_DOUBLE_EQ(0.4, interpolate(1.0, 3.0, -0.3));
	EXPECT_DOUBLE_EQ(1.0, interpolate(1.0, 3.0, 0.0));
	EXPECT_DOUBLE_EQ(1.6, interpolate(1.0, 3.0, 0.3));
	EXPECT_DOUBLE_EQ(2.2, interpolate(1.0, 3.0, 0.6));
	EXPECT_DOUBLE_EQ(3.0, interpolate(1.0, 3.0, 1.0));
	EXPECT_DOUBLE_EQ(3.6, interpolate(1.0, 3.0, 1.3));
	EXPECT_DOUBLE_EQ(3.6, interpolate(3.0, 1.0, -0.3));
	EXPECT_DOUBLE_EQ(3.0, interpolate(3.0, 1.0, 0.0));
	EXPECT_DOUBLE_EQ(2.4, interpolate(3.0, 1.0, 0.3));
	EXPECT_DOUBLE_EQ(1.8, interpolate(3.0, 1.0, 0.6));
	EXPECT_DOUBLE_EQ(1.0, interpolate(3.0, 1.0, 1.0));
	EXPECT_DOUBLE_EQ(0.4, interpolate(3.0, 1.0, 1.3));
}

// Test: interpolateCoerced
TEST(MathFuncsTest, test_interpolateCoerced)
{
	// Check that we are getting the expected values for the x form
	EXPECT_DOUBLE_EQ(0.0, interpolateCoerced(4.0, 5.0, 0.0, 1.0, 3.7));
	EXPECT_DOUBLE_EQ(0.0, interpolateCoerced(4.0, 5.0, 0.0, 1.0, 4.0));
	EXPECT_DOUBLE_EQ(0.3, interpolateCoerced(4.0, 5.0, 0.0, 1.0, 4.3));
	EXPECT_DOUBLE_EQ(0.6, interpolateCoerced(4.0, 5.0, 0.0, 1.0, 4.6));
	EXPECT_DOUBLE_EQ(1.0, interpolateCoerced(4.0, 5.0, 0.0, 1.0, 5.0));
	EXPECT_DOUBLE_EQ(1.0, interpolateCoerced(4.0, 5.0, 0.0, 1.0, 5.3));
	EXPECT_DOUBLE_EQ(0.0, interpolateCoerced(5.0, 4.0, 1.0, 0.0, 3.7));
	EXPECT_DOUBLE_EQ(0.0, interpolateCoerced(5.0, 4.0, 1.0, 0.0, 4.0));
	EXPECT_DOUBLE_EQ(0.3, interpolateCoerced(5.0, 4.0, 1.0, 0.0, 4.3));
	EXPECT_DOUBLE_EQ(0.6, interpolateCoerced(5.0, 4.0, 1.0, 0.0, 4.6));
	EXPECT_DOUBLE_EQ(1.0, interpolateCoerced(5.0, 4.0, 1.0, 0.0, 5.0));
	EXPECT_DOUBLE_EQ(1.0, interpolateCoerced(5.0, 4.0, 1.0, 0.0, 5.3));
	EXPECT_DOUBLE_EQ(7.93, interpolateCoerced(3.12, 1.82, 4.12, 7.93, 0.24));
	EXPECT_DOUBLE_EQ(7.6076153846153849, interpolateCoerced(3.12, 1.82, 4.12, 7.93, 1.93));
	EXPECT_DOUBLE_EQ(5.1457692307692309, interpolateCoerced(3.12, 1.82, 4.12, 7.93, 2.77));
	EXPECT_DOUBLE_EQ(4.12, interpolateCoerced(3.12, 1.82, 4.12, 7.93, 3.12));
	EXPECT_DOUBLE_EQ(4.12, interpolateCoerced(3.12, 1.82, 4.12, 7.93, 10.22));
	EXPECT_DOUBLE_EQ(2.0, interpolateCoerced(4.0, 4.0, 1.0, 3.0, 3.0));
	EXPECT_DOUBLE_EQ(2.0, interpolateCoerced(4.0, 4.0, 1.0, 3.0, 4.0));
	EXPECT_DOUBLE_EQ(2.0, interpolateCoerced(4.0, 4.0, 1.0, 3.0, 5.0));

	// Check that we are getting the expected values for the u form
	EXPECT_DOUBLE_EQ(1.0, interpolateCoerced(1.0, 3.0, -0.3));
	EXPECT_DOUBLE_EQ(1.0, interpolateCoerced(1.0, 3.0, 0.0));
	EXPECT_DOUBLE_EQ(1.6, interpolateCoerced(1.0, 3.0, 0.3));
	EXPECT_DOUBLE_EQ(2.2, interpolateCoerced(1.0, 3.0, 0.6));
	EXPECT_DOUBLE_EQ(3.0, interpolateCoerced(1.0, 3.0, 1.0));
	EXPECT_DOUBLE_EQ(3.0, interpolateCoerced(1.0, 3.0, 1.3));
	EXPECT_DOUBLE_EQ(3.0, interpolateCoerced(3.0, 1.0, -0.3));
	EXPECT_DOUBLE_EQ(3.0, interpolateCoerced(3.0, 1.0, 0.0));
	EXPECT_DOUBLE_EQ(2.4, interpolateCoerced(3.0, 1.0, 0.3));
	EXPECT_DOUBLE_EQ(1.8, interpolateCoerced(3.0, 1.0, 0.6));
	EXPECT_DOUBLE_EQ(1.0, interpolateCoerced(3.0, 1.0, 1.0));
	EXPECT_DOUBLE_EQ(1.0, interpolateCoerced(3.0, 1.0, 1.3));
}

//
// Math spline tests
//

// Test: LinearSpline
TEST(MathSplineTest, test_LinearSpline)
{
	// Test linear spline created using constructor overload
	LinearSpline spline(1.0, 2.0, 2.0);
	EXPECT_DOUBLE_EQ(1.00, spline(-0.5));
	EXPECT_DOUBLE_EQ(1.00, spline(0.0));
	EXPECT_DOUBLE_EQ(1.25, spline.x(0.5));
	EXPECT_DOUBLE_EQ(1.50, spline.x(1.0));
	EXPECT_DOUBLE_EQ(1.75, spline.x(1.5));
	EXPECT_DOUBLE_EQ(2.00, spline.x(2.0));
	EXPECT_DOUBLE_EQ(2.00, spline.x(2.5));
	EXPECT_DOUBLE_EQ(0.00, spline.v(-0.5));
	EXPECT_DOUBLE_EQ(0.50, spline.v(0.0));
	EXPECT_DOUBLE_EQ(0.50, spline.v(1.0));
	EXPECT_DOUBLE_EQ(0.50, spline.v(2.0));
	EXPECT_DOUBLE_EQ(0.00, spline.v(2.5));
	EXPECT_DOUBLE_EQ(0.50, spline.v());

	// Test linear spline with manual setParams() call
	spline.setParams(5.0, 3.0, 4.0);
	EXPECT_DOUBLE_EQ(5.00, spline(-1.0));
	EXPECT_DOUBLE_EQ(5.00, spline(0.0));
	EXPECT_DOUBLE_EQ(4.50, spline.x(1.0));
	EXPECT_DOUBLE_EQ(4.00, spline.x(2.0));
	EXPECT_DOUBLE_EQ(3.50, spline.x(3.0));
	EXPECT_DOUBLE_EQ(3.00, spline.x(4.0));
	EXPECT_DOUBLE_EQ(3.00, spline.x(5.0));
	EXPECT_DOUBLE_EQ(0.00, spline.v(-1.0));
	EXPECT_DOUBLE_EQ(-0.5, spline.v(0.0));
	EXPECT_DOUBLE_EQ(-0.5, spline.v(2.0));
	EXPECT_DOUBLE_EQ(-0.5, spline.v(4.0));
	EXPECT_DOUBLE_EQ(0.00, spline.v(5.0));
	EXPECT_DOUBLE_EQ(-0.5, spline.v());

	// Test static evaluation of linear spline
	EXPECT_DOUBLE_EQ(0.00, LinearSpline::eval(0.0, 1.0, 0.5, -0.25));
	EXPECT_DOUBLE_EQ(0.00, LinearSpline::eval(0.0, 1.0, 0.5,  0.00));
	EXPECT_DOUBLE_EQ(0.50, LinearSpline::eval(0.0, 1.0, 0.5,  0.25));
	EXPECT_DOUBLE_EQ(1.00, LinearSpline::eval(0.0, 1.0, 0.5,  0.50));
	EXPECT_DOUBLE_EQ(1.00, LinearSpline::eval(0.0, 1.0, 0.5,  0.75));
}

// Test: CubicSpline
TEST(MathSplineTest, test_CubicSpline)
{
	// Create a cubic spline object
	CubicSpline spline(2.0, 1.0, 4.0, 1.0, 0.8);

	// Test spline 1 (point symmetric cubic)
	EXPECT_DOUBLE_EQ(2.0, spline(0.0));
	EXPECT_DOUBLE_EQ(3.0, spline(0.4)); // Halfway point should be halfway due to symmetry
	EXPECT_DOUBLE_EQ(4.0, spline(0.8));
	EXPECT_DOUBLE_EQ(2.0, spline.x(0.0));
	EXPECT_DOUBLE_EQ(3.0, spline.x(0.4)); // Halfway point should be halfway due to symmetry
	EXPECT_DOUBLE_EQ(4.0, spline.x(0.8));
	EXPECT_DOUBLE_EQ(1.0, spline.v(0.0));
	EXPECT_DOUBLE_EQ(1.0, spline.v(0.8));
	EXPECT_DOUBLE_EQ(0.0, spline.a(0.4));
	EXPECT_NEAR(0.0, spline.a(0.2) + spline.a(0.6), 32*DBL_EPSILON); // Acceleration waveform should be negative symmetric

	// Test spline 1 again with setParams method
	spline.setParams(2.0, 1.0, 4.0, 1.0, 0.8);
	EXPECT_DOUBLE_EQ(2.0, spline(0.0));
	EXPECT_DOUBLE_EQ(3.0, spline(0.4)); // Halfway point should be halfway due to symmetry
	EXPECT_DOUBLE_EQ(4.0, spline(0.8));

	// Test spline 1 again with static method
	EXPECT_DOUBLE_EQ(2.0, CubicSpline::eval(2.0, 1.0, 4.0, 1.0, 0.8, 0.0));
	EXPECT_DOUBLE_EQ(3.0, CubicSpline::eval(2.0, 1.0, 4.0, 1.0, 0.8, 0.4)); // Halfway point should be halfway due to symmetry
	EXPECT_DOUBLE_EQ(4.0, CubicSpline::eval(2.0, 1.0, 4.0, 1.0, 0.8, 0.8));

	// Test spline 2 (linear)
	spline.setParams(1.0, 0.5, 2.0, 0.5, 2.0);
	EXPECT_DOUBLE_EQ(1.0, spline(0.0));
	EXPECT_DOUBLE_EQ(1.2, spline(0.4));
	EXPECT_DOUBLE_EQ(1.4, spline(0.8));
	EXPECT_DOUBLE_EQ(1.6, spline.x(1.2));
	EXPECT_DOUBLE_EQ(1.8, spline.x(1.6));
	EXPECT_DOUBLE_EQ(2.0, spline.x(2.0));
	EXPECT_DOUBLE_EQ(0.5, spline.v(0.4));
	EXPECT_DOUBLE_EQ(0.5, spline.v(1.0));
	EXPECT_DOUBLE_EQ(0.5, spline.v(1.6));
	EXPECT_DOUBLE_EQ(0.0, spline.a(0.4));
	EXPECT_DOUBLE_EQ(0.0, spline.a(1.0));
	EXPECT_DOUBLE_EQ(0.0, spline.a(1.6));
	EXPECT_DOUBLE_EQ(0.0, spline.jerk());

	// Test spline 3 (quadratic)
	spline.setParams(0.0, 1.0, 0.0, -1.0, 1.0);
	EXPECT_DOUBLE_EQ(0.00, spline(0.0));
	EXPECT_DOUBLE_EQ(0.16, spline(0.2));
	EXPECT_DOUBLE_EQ(0.24, spline(0.4));
	EXPECT_DOUBLE_EQ(0.25, spline(0.5));
	EXPECT_DOUBLE_EQ(0.24, spline.x(0.6));
	EXPECT_DOUBLE_EQ(0.16, spline.x(0.8));
	EXPECT_DOUBLE_EQ(0.00, spline.x(1.0));
	EXPECT_DOUBLE_EQ( 0.6, spline.v(0.2));
	EXPECT_DOUBLE_EQ( 0.0, spline.v(0.5));
	EXPECT_DOUBLE_EQ(-0.6, spline.v(0.8));
	EXPECT_DOUBLE_EQ(-2.0, spline.a(0.2));
	EXPECT_DOUBLE_EQ(-2.0, spline.a(0.5));
	EXPECT_DOUBLE_EQ(-2.0, spline.a(0.8));
	EXPECT_DOUBLE_EQ(0.0, spline.jerk());

	// Test that zero dT case doesn't make anything explode
	EXPECT_NO_THROW(CubicSpline::eval(2.0, 1.0, 4.0, 1.0, 0.0, 1.0)); // Zero dT!
}

// Test: TrapVelSpline (the splitting up into so many individual tests was necessary for a huge reduction in compilation time, for some unknown reason)
TEST(MathSplineTest, test_TrapVelSpline1)
{
	// Set the double comparison absolute tolerance
	double Tol = 8*DBL_EPSILON;

	// Test the static evaluation function
	EXPECT_DOUBLE_EQ(1.5, TrapVelSpline::eval(1, 0, 2, 0, 1, 5, 0.6));

	// Create a trapezoidal spline object
	TrapVelSpline spline(1, 0, 2, 0, 1, 5);

	// Test the ()-operator
	EXPECT_DOUBLE_EQ(spline.x(0.2066), spline(0.2066));
	EXPECT_DOUBLE_EQ(spline.x(0.5913), spline(0.5913));
	EXPECT_DOUBLE_EQ(spline.x(0.9377), spline(0.9377));

	// Test spline 1
	EXPECT_NEAR( 1.2000000000000000, spline.T(), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 1.3000000000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR( 1.7000000000000002, spline.x( 0.8), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.x( 1.2), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.x( 1.6), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 1.6), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR( 5.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR(-5.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.6), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline2)
{
	// Test spline 2
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(-1, -1, -2, 0, 2, 4);
	EXPECT_NEAR( 0.8125000000000000, spline.T(), Tol);
	EXPECT_NEAR(-0.6000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR(-1.6596875000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR(-1.9996875000000001, spline.x( 0.8), Tol);
	EXPECT_NEAR(-2.0000000000000000, spline.x( 1.2), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR(-1.6499999999999999, spline.v( 0.4), Tol);
	EXPECT_NEAR(-0.0499999999999998, spline.v( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR(-4.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 4.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 4.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR(-2.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline3)
{
	// Test spline 3
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(-1, 0, -2, 0, 2, 4);
	EXPECT_NEAR( 1.0000000000000000, spline.T(), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR(-1.3200000000000001, spline.x( 0.4), Tol);
	EXPECT_NEAR(-1.9199999999999999, spline.x( 0.8), Tol);
	EXPECT_NEAR(-2.0000000000000000, spline.x( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR(-1.6000000000000001, spline.v( 0.4), Tol);
	EXPECT_NEAR(-0.7999999999999998, spline.v( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR(-4.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR(-4.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 4.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR(-2.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline4)
{
	// Test spline 4
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 0, 1, 0, 2, 4);
	EXPECT_NEAR( 1.0000000000000000, spline.T(), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.3200000000000001, spline.x( 0.4), Tol);
	EXPECT_NEAR( 0.9200000000000000, spline.x( 0.8), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 1.6000000000000001, spline.v( 0.4), Tol);
	EXPECT_NEAR( 0.7999999999999998, spline.v( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR( 4.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 4.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR(-4.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline5)
{
	// Test spline 5
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 0, 1, 0, 2, 5);
	EXPECT_NEAR( 0.9000000000000000, spline.T(), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.4000000000000001, spline.x( 0.4), Tol);
	EXPECT_NEAR( 0.9750000000000000, spline.x( 0.8), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 0.4), Tol);
	EXPECT_NEAR( 0.4999999999999999, spline.v( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR( 5.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 5.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR(-5.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline6)
{
	// Test spline 6
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, -1, 1, -1, 2, 2);
	EXPECT_NEAR( 2.7320508075688772, spline.T(), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR(-0.2400000000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR(-0.1599999999999999, spline.x( 0.8), Tol);
	EXPECT_NEAR( 0.2400000000000000, spline.x( 1.2), Tol);
	EXPECT_NEAR( 0.8505117766515302, spline.x( 1.6), Tol);
	EXPECT_NEAR( 1.1961524227066320, spline.x( 2.0), Tol);
	EXPECT_NEAR( 1.2217930687617338, spline.x( 2.4), Tol);
	EXPECT_NEAR( 0.9320508075688774, spline.x( 2.8), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR(-0.2000000000000000, spline.v( 0.4), Tol);
	EXPECT_NEAR( 0.6000000000000001, spline.v( 0.8), Tol);
	EXPECT_NEAR( 1.3999999999999999, spline.v( 1.2), Tol);
	EXPECT_NEAR( 1.2641016151377542, spline.v( 1.6), Tol);
	EXPECT_NEAR( 0.4641016151377544, spline.v( 2.0), Tol);
	EXPECT_NEAR(-0.3358983848622454, spline.v( 2.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v( 2.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR(-2.0000000000000000, spline.a( 1.6), Tol);
	EXPECT_NEAR(-2.0000000000000000, spline.a( 2.0), Tol);
	EXPECT_NEAR(-2.0000000000000000, spline.a( 2.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 2.8), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline7)
{
	// Test spline 7
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(1, 0, 1, 3, 1, 5);
	EXPECT_NEAR( 1.7000000000000002, spline.T(), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.7000000000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR( 0.2999999999999999, spline.x( 0.8), Tol);
	EXPECT_NEAR( 0.1249999999999999, spline.x( 1.2), Tol);
	EXPECT_NEAR( 0.7249999999999998, spline.x( 1.6), Tol);
	EXPECT_NEAR( 1.8999999999999995, spline.x( 2.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v( 0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v( 0.8), Tol);
	EXPECT_NEAR( 0.4999999999999991, spline.v( 1.2), Tol);
	EXPECT_NEAR( 2.4999999999999996, spline.v( 1.6), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v( 2.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR(-5.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 5.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 5.0000000000000000, spline.a( 1.6), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 2.0), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline8)
{
	// Test spline 8
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 3, 2, 2, 2, 1);
	EXPECT_NEAR( 9.2500000000000000, spline.T(), Tol);
	EXPECT_NEAR(-1.2000000000000002, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 1.1200000000000001, spline.x( 0.4), Tol);
	EXPECT_NEAR( 2.0800000000000001, spline.x( 0.8), Tol);
	EXPECT_NEAR( 2.8799999999999999, spline.x( 1.2), Tol);
	EXPECT_NEAR( 3.5200000000000005, spline.x( 1.6), Tol);
	EXPECT_NEAR( 4.0000000000000000, spline.x( 2.0), Tol);
	EXPECT_NEAR( 4.3199999999999994, spline.x( 2.4), Tol);
	EXPECT_NEAR( 4.4799999999999986, spline.x( 2.8), Tol);
	EXPECT_NEAR( 4.4800000000000004, spline.x( 3.2), Tol);
	EXPECT_NEAR( 4.3200000000000003, spline.x( 3.6), Tol);
	EXPECT_NEAR( 4.0000000000000000, spline.x( 4.0), Tol);
	EXPECT_NEAR( 3.5199999999999996, spline.x( 4.4), Tol);
	EXPECT_NEAR( 2.8799999999999990, spline.x( 4.8), Tol);
	EXPECT_NEAR( 2.0999999999999996, spline.x( 5.2), Tol);
	EXPECT_NEAR( 1.3612500000000010, spline.x( 5.6), Tol);
	EXPECT_NEAR( 0.7812500000000000, spline.x( 6.0), Tol);
	EXPECT_NEAR( 0.3612500000000001, spline.x( 6.4), Tol);
	EXPECT_NEAR( 0.1012500000000003, spline.x( 6.8), Tol);
	EXPECT_NEAR( 0.0012500000000002, spline.x( 7.2), Tol);
	EXPECT_NEAR( 0.0612499999999998, spline.x( 7.6), Tol);
	EXPECT_NEAR( 0.2812500000000000, spline.x( 8.0), Tol);
	EXPECT_NEAR( 0.6612500000000003, spline.x( 8.4), Tol);
	EXPECT_NEAR( 1.2012500000000010, spline.x( 8.8), Tol);
	EXPECT_NEAR( 1.9012499999999986, spline.x( 9.2), Tol);
	EXPECT_NEAR( 2.6999999999999993, spline.x( 9.6), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 2.6000000000000001, spline.v( 0.4), Tol);
	EXPECT_NEAR( 2.2000000000000002, spline.v( 0.8), Tol);
	EXPECT_NEAR( 1.8000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 1.3999999999999999, spline.v( 1.6), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 2.0), Tol);
	EXPECT_NEAR( 0.6000000000000001, spline.v( 2.4), Tol);
	EXPECT_NEAR( 0.2000000000000002, spline.v( 2.8), Tol);
	EXPECT_NEAR(-0.2000000000000002, spline.v( 3.2), Tol);
	EXPECT_NEAR(-0.6000000000000001, spline.v( 3.6), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.v( 4.0), Tol);
	EXPECT_NEAR(-1.4000000000000004, spline.v( 4.4), Tol);
	EXPECT_NEAR(-1.7999999999999998, spline.v( 4.8), Tol);
	EXPECT_NEAR(-2.0000000000000000, spline.v( 5.2), Tol);
	EXPECT_NEAR(-1.6500000000000004, spline.v( 5.6), Tol);
	EXPECT_NEAR(-1.2500000000000000, spline.v( 6.0), Tol);
	EXPECT_NEAR(-0.8499999999999996, spline.v( 6.4), Tol);
	EXPECT_NEAR(-0.4500000000000002, spline.v( 6.8), Tol);
	EXPECT_NEAR(-0.0499999999999998, spline.v( 7.2), Tol);
	EXPECT_NEAR( 0.3499999999999996, spline.v( 7.6), Tol);
	EXPECT_NEAR( 0.7500000000000000, spline.v( 8.0), Tol);
	EXPECT_NEAR( 1.1500000000000004, spline.v( 8.4), Tol);
	EXPECT_NEAR( 1.5500000000000007, spline.v( 8.8), Tol);
	EXPECT_NEAR( 1.9499999999999993, spline.v( 9.2), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 9.6), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 1.6), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 2.0), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 2.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 2.8), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 3.2), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 3.6), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 4.0), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 4.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 4.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 5.2), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 5.6), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 6.0), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 6.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 6.8), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 7.2), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 7.6), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 8.0), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 8.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 8.8), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 9.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 9.6), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline9)
{
	// Test spline 9
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 3, 7, 2, 2, 1);
	EXPECT_NEAR( 3.2500000000000000, spline.T(), Tol);
	EXPECT_NEAR(-1.2000000000000002, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 1.1200000000000001, spline.x( 0.4), Tol);
	EXPECT_NEAR( 2.0800000000000001, spline.x( 0.8), Tol);
	EXPECT_NEAR( 2.8999999999999999, spline.x( 1.2), Tol);
	EXPECT_NEAR( 3.7000000000000002, spline.x( 1.6), Tol);
	EXPECT_NEAR( 4.5000000000000000, spline.x( 2.0), Tol);
	EXPECT_NEAR( 5.2999999999999998, spline.x( 2.4), Tol);
	EXPECT_NEAR( 6.0999999999999996, spline.x( 2.8), Tol);
	EXPECT_NEAR( 6.9000000000000004, spline.x( 3.2), Tol);
	EXPECT_NEAR( 7.7000000000000002, spline.x( 3.6), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 2.6000000000000001, spline.v( 0.4), Tol);
	EXPECT_NEAR( 2.2000000000000002, spline.v( 0.8), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 1.6), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 2.0), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 2.4), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 2.8), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 3.2), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 3.6), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.6), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 2.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 2.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 2.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 3.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 3.6), Tol);
	EXPECT_NEAR( 7.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline10)
{
	// Test spline 10
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 3, 3, 3, 1, 1);
	EXPECT_NEAR( 1.1010205144336442, spline.T(), Tol);
	EXPECT_NEAR(-1.2000000000000002, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 1.1200000000000001, spline.x( 0.4), Tol);
	EXPECT_NEAR( 2.1422451317540157, spline.x( 0.8), Tol);
	EXPECT_NEAR( 3.2969384566990669, spline.x( 1.2), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 2.6000000000000001, spline.v( 0.4), Tol);
	EXPECT_NEAR( 2.6989794855663556, spline.v( 0.8), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline11)
{
	// Test spline 11
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 1, 3, 1, 3, 1);
	EXPECT_NEAR( 2.0000000000000000, spline.T(), Tol);
	EXPECT_NEAR(-0.4000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.4800000000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR( 1.1200000000000001, spline.x( 0.8), Tol);
	EXPECT_NEAR( 1.8800000000000001, spline.x( 1.2), Tol);
	EXPECT_NEAR( 2.5200000000000000, spline.x( 1.6), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.x( 2.0), Tol);
	EXPECT_NEAR( 3.3999999999999999, spline.x( 2.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 1.3999999999999999, spline.v( 0.4), Tol);
	EXPECT_NEAR( 1.8000000000000000, spline.v( 0.8), Tol);
	EXPECT_NEAR( 1.8000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 1.3999999999999999, spline.v( 1.6), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 2.0), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 2.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 1.6), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 2.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 2.4), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline12)
{
	// Test spline 12
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 1, 0, 1, 0.5, 1);
	EXPECT_NEAR( 0.0000000000000000, spline.T(), Tol);
	EXPECT_NEAR(-0.4000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline13)
{
	// Test spline 13
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 2, 3, 2, 0.5, 1);
	EXPECT_NEAR( 2.0000000000000000, spline.T(), Tol);
	EXPECT_NEAR(-0.8000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.7200000000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR( 1.2800000000000000, spline.x( 0.8), Tol);
	EXPECT_NEAR( 1.7200000000000000, spline.x( 1.2), Tol);
	EXPECT_NEAR( 2.2800000000000002, spline.x( 1.6), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.x( 2.0), Tol);
	EXPECT_NEAR( 3.7999999999999998, spline.x( 2.4), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 1.6000000000000001, spline.v( 0.4), Tol);
	EXPECT_NEAR( 1.2000000000000000, spline.v( 0.8), Tol);
	EXPECT_NEAR( 1.2000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 1.6000000000000001, spline.v( 1.6), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 2.0), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v( 2.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 1.6), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 2.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 2.4), Tol);
	EXPECT_NEAR( 3.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 2.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline14)
{
	// Test spline 14
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0, 1, 1, 1, 1, 1);
	EXPECT_NEAR( 1.0000000000000000, spline.T(), Tol);
	EXPECT_NEAR(-0.4000000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR( 0.8000000000000000, spline.x( 0.8), Tol);
	EXPECT_NEAR( 1.2000000000000000, spline.x( 1.2), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 0.8), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v( 1.2), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline15)
{
	// Test spline 15
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0.5, 0.4, 0.5, 0.4, 0.5, 1);
	EXPECT_NEAR( 0.0000000000000000, spline.T(), Tol);
	EXPECT_NEAR( 0.3400000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.5000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.6600000000000000, spline.x( 0.4), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.v( 0.4), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR( 0.5000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.v(spline.T()), Tol);
}
TEST(MathSplineTest, test_TrapVelSpline16)
{
	// Test spline 16
	TrapVelSpline spline;
	double Tol = 8*DBL_EPSILON;
	spline.setParams(0.5, 0.4, 0.5, 0.41, 0.5, 1);
	EXPECT_NEAR( 1.6200617260431456, spline.T(), Tol);
	EXPECT_NEAR( 0.3400000000000000, spline.x(-0.4), Tol);
	EXPECT_NEAR( 0.5000000000000000, spline.x( 0.0), Tol);
	EXPECT_NEAR( 0.5800000000000001, spline.x( 0.4), Tol);
	EXPECT_NEAR( 0.5000000000000000, spline.x( 0.8), Tol);
	EXPECT_NEAR( 0.4160006191654836, spline.x( 1.2), Tol);
	EXPECT_NEAR( 0.4919759287482255, spline.x( 1.6), Tol);
	EXPECT_NEAR( 0.6557746923223103, spline.x( 2.0), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.v(-0.4), Tol);
	EXPECT_NEAR( 0.4000000000000000, spline.v( 0.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.v( 0.4), Tol);
	EXPECT_NEAR(-0.4000000000000000, spline.v( 0.8), Tol);
	EXPECT_NEAR(-0.0100617260431456, spline.v( 1.2), Tol);
	EXPECT_NEAR( 0.3899382739568545, spline.v( 1.6), Tol);
	EXPECT_NEAR( 0.4100000000000000, spline.v( 2.0), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a(-0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.0), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.4), Tol);
	EXPECT_NEAR(-1.0000000000000000, spline.a( 0.8), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 1.2), Tol);
	EXPECT_NEAR( 1.0000000000000000, spline.a( 1.6), Tol);
	EXPECT_NEAR( 0.0000000000000000, spline.a( 2.0), Tol);
	EXPECT_NEAR( 0.5000000000000000, spline.x(spline.T()), Tol);
	EXPECT_NEAR( 0.4100000000000000, spline.v(spline.T()), Tol);
}

//
// Math vector matrix tests
//

// Test: normalize, eigenNormalize, eigenNormalizeDefX, eigenNormalizeDefY, eigenNormalizeDefZ (double)
TEST(MathVecMatTest, test_eigenNormalizeDouble)
{
	// Test cases
	Eigen::Vector2d vec2d(0.3, 0.4), vec2dTmp;
	Eigen::Vector3d vec3d(1.2, 0.4, 0.3), vec3dTmp;
	Eigen::Vector2d vec2dZero(0.0, 0.0), vec2dZeroTmp;
	Eigen::Vector3d vec3dZero(0.0, 0.0, 0.0), vec3dZeroTmp;

	// Check that we are getting the expected values for normalize
	vec2dTmp = vec2d; vec2dTmp.normalize();
	vec3dTmp = vec3d; vec3dTmp.normalize();
	vec2dZeroTmp = vec2dZero; vec2dZeroTmp.normalize();
	vec3dZeroTmp = vec3dZero; vec3dZeroTmp.normalize();
	EXPECT_DOUBLE_EQ(0.6, vec2dTmp.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dTmp.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dTmp.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dTmp.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dTmp.z());
	EXPECT_TRUE(std::isnan(vec2dZeroTmp.x()));
	EXPECT_TRUE(std::isnan(vec2dZeroTmp.y()));
	EXPECT_TRUE(std::isnan(vec3dZeroTmp.x()));
	EXPECT_TRUE(std::isnan(vec3dZeroTmp.y()));
	EXPECT_TRUE(std::isnan(vec3dZeroTmp.z()));

	// Check that we are getting the expected values for eigenNormalize (zero)
	vec2dTmp = vec2d; eigenNormalize(vec2dTmp);
	vec3dTmp = vec3d; eigenNormalize(vec3dTmp);
	vec2dZeroTmp = vec2dZero; eigenNormalize(vec2dZeroTmp);
	vec3dZeroTmp = vec3dZero; eigenNormalize(vec3dZeroTmp);
	EXPECT_DOUBLE_EQ(0.6, vec2dTmp.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dTmp.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dTmp.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dTmp.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dTmp.z());
	EXPECT_DOUBLE_EQ(0.0, vec2dZeroTmp.x());
	EXPECT_DOUBLE_EQ(0.0, vec2dZeroTmp.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.x());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.z());

	// Check that we are getting the expected values for eigenNormalize (custom)
	Eigen::Vector2d vec2dCustom(0.7, 0.9);
	Eigen::Vector3d vec3dCustom(0.7, 0.9, 0.2);
	vec2dTmp = vec2d; eigenNormalize(vec2dTmp, vec2dCustom);
	vec3dTmp = vec3d; eigenNormalize(vec3dTmp, vec3dCustom);
	vec2dZeroTmp = vec2dZero; eigenNormalize(vec2dZeroTmp, vec2dCustom);
	vec3dZeroTmp = vec3dZero; eigenNormalize(vec3dZeroTmp, vec3dCustom);
	EXPECT_DOUBLE_EQ(0.6, vec2dTmp.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dTmp.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dTmp.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dTmp.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dTmp.z());
	EXPECT_DOUBLE_EQ(0.7, vec2dZeroTmp.x());
	EXPECT_DOUBLE_EQ(0.9, vec2dZeroTmp.y());
	EXPECT_DOUBLE_EQ(0.7, vec3dZeroTmp.x());
	EXPECT_DOUBLE_EQ(0.9, vec3dZeroTmp.y());
	EXPECT_DOUBLE_EQ(0.2, vec3dZeroTmp.z());

	// Check that we are getting the expected values for eigenNormalizeDefX
	vec2dTmp = vec2d; eigenNormalizeDefX(vec2dTmp);
	vec3dTmp = vec3d; eigenNormalizeDefX(vec3dTmp);
	vec2dZeroTmp = vec2dZero; eigenNormalizeDefX(vec2dZeroTmp);
	vec3dZeroTmp = vec3dZero; eigenNormalizeDefX(vec3dZeroTmp);
	EXPECT_DOUBLE_EQ(0.6, vec2dTmp.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dTmp.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dTmp.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dTmp.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dTmp.z());
	EXPECT_DOUBLE_EQ(1.0, vec2dZeroTmp.x());
	EXPECT_DOUBLE_EQ(0.0, vec2dZeroTmp.y());
	EXPECT_DOUBLE_EQ(1.0, vec3dZeroTmp.x());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.z());

	// Check that we are getting the expected values for eigenNormalizeDefY
	vec2dTmp = vec2d; eigenNormalizeDefY(vec2dTmp);
	vec3dTmp = vec3d; eigenNormalizeDefY(vec3dTmp);
	vec2dZeroTmp = vec2dZero; eigenNormalizeDefY(vec2dZeroTmp);
	vec3dZeroTmp = vec3dZero; eigenNormalizeDefY(vec3dZeroTmp);
	EXPECT_DOUBLE_EQ(0.6, vec2dTmp.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dTmp.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dTmp.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dTmp.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dTmp.z());
	EXPECT_DOUBLE_EQ(0.0, vec2dZeroTmp.x());
	EXPECT_DOUBLE_EQ(1.0, vec2dZeroTmp.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.x());
	EXPECT_DOUBLE_EQ(1.0, vec3dZeroTmp.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.z());

	// Check that we are getting the expected values for eigenNormalizeDefZ
	vec3dTmp = vec3d; eigenNormalizeDefZ(vec3dTmp);
	vec3dZeroTmp = vec3dZero; eigenNormalizeDefZ(vec3dZeroTmp);
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dTmp.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dTmp.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dTmp.z());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.x());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroTmp.y());
	EXPECT_DOUBLE_EQ(1.0, vec3dZeroTmp.z());
}

// Test: normalize, eigenNormalize, eigenNormalizeDefX, eigenNormalizeDefY, eigenNormalizeDefZ (float)
TEST(MathVecMatTest, test_eigenNormalizeFloat)
{
	// Test cases
	Eigen::Vector2f vec2f(0.3, 0.4), vec2fTmp;
	Eigen::Vector3f vec3f(1.2, 0.4, 0.3), vec3fTmp;
	Eigen::Vector2f vec2fZero(0.0, 0.0), vec2fZeroTmp;
	Eigen::Vector3f vec3fZero(0.0, 0.0, 0.0), vec3fZeroTmp;

	// Check that we are getting the expected values for normalize
	vec2fTmp = vec2f; vec2fTmp.normalize();
	vec3fTmp = vec3f; vec3fTmp.normalize();
	vec2fZeroTmp = vec2fZero; vec2fZeroTmp.normalize();
	vec3fZeroTmp = vec3fZero; vec3fZeroTmp.normalize();
	EXPECT_FLOAT_EQ(0.6, vec2fTmp.x());
	EXPECT_FLOAT_EQ(0.8, vec2fTmp.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fTmp.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fTmp.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fTmp.z());
	EXPECT_TRUE(std::isnan(vec2fZeroTmp.x()));
	EXPECT_TRUE(std::isnan(vec2fZeroTmp.y()));
	EXPECT_TRUE(std::isnan(vec3fZeroTmp.x()));
	EXPECT_TRUE(std::isnan(vec3fZeroTmp.y()));
	EXPECT_TRUE(std::isnan(vec3fZeroTmp.z()));

	// Check that we are getting the expected values for eigenNormalize (zero)
	vec2fTmp = vec2f; eigenNormalize(vec2fTmp);
	vec3fTmp = vec3f; eigenNormalize(vec3fTmp);
	vec2fZeroTmp = vec2fZero; eigenNormalize(vec2fZeroTmp);
	vec3fZeroTmp = vec3fZero; eigenNormalize(vec3fZeroTmp);
	EXPECT_FLOAT_EQ(0.6, vec2fTmp.x());
	EXPECT_FLOAT_EQ(0.8, vec2fTmp.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fTmp.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fTmp.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fTmp.z());
	EXPECT_FLOAT_EQ(0.0, vec2fZeroTmp.x());
	EXPECT_FLOAT_EQ(0.0, vec2fZeroTmp.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.x());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.z());

	// Check that we are getting the expected values for eigenNormalize (custom)
	Eigen::Vector2f vec2fCustom(0.7, 0.9);
	Eigen::Vector3f vec3fCustom(0.7, 0.9, 0.2);
	vec2fTmp = vec2f; eigenNormalize(vec2fTmp, vec2fCustom);
	vec3fTmp = vec3f; eigenNormalize(vec3fTmp, vec3fCustom);
	vec2fZeroTmp = vec2fZero; eigenNormalize(vec2fZeroTmp, vec2fCustom);
	vec3fZeroTmp = vec3fZero; eigenNormalize(vec3fZeroTmp, vec3fCustom);
	EXPECT_FLOAT_EQ(0.6, vec2fTmp.x());
	EXPECT_FLOAT_EQ(0.8, vec2fTmp.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fTmp.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fTmp.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fTmp.z());
	EXPECT_FLOAT_EQ(0.7, vec2fZeroTmp.x());
	EXPECT_FLOAT_EQ(0.9, vec2fZeroTmp.y());
	EXPECT_FLOAT_EQ(0.7, vec3fZeroTmp.x());
	EXPECT_FLOAT_EQ(0.9, vec3fZeroTmp.y());
	EXPECT_FLOAT_EQ(0.2, vec3fZeroTmp.z());

	// Check that we are getting the expected values for eigenNormalizeDefX
	vec2fTmp = vec2f; eigenNormalizeDefX(vec2fTmp);
	vec3fTmp = vec3f; eigenNormalizeDefX(vec3fTmp);
	vec2fZeroTmp = vec2fZero; eigenNormalizeDefX(vec2fZeroTmp);
	vec3fZeroTmp = vec3fZero; eigenNormalizeDefX(vec3fZeroTmp);
	EXPECT_FLOAT_EQ(0.6, vec2fTmp.x());
	EXPECT_FLOAT_EQ(0.8, vec2fTmp.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fTmp.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fTmp.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fTmp.z());
	EXPECT_FLOAT_EQ(1.0, vec2fZeroTmp.x());
	EXPECT_FLOAT_EQ(0.0, vec2fZeroTmp.y());
	EXPECT_FLOAT_EQ(1.0, vec3fZeroTmp.x());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.z());

	// Check that we are getting the expected values for eigenNormalizeDefY
	vec2fTmp = vec2f; eigenNormalizeDefY(vec2fTmp);
	vec3fTmp = vec3f; eigenNormalizeDefY(vec3fTmp);
	vec2fZeroTmp = vec2fZero; eigenNormalizeDefY(vec2fZeroTmp);
	vec3fZeroTmp = vec3fZero; eigenNormalizeDefY(vec3fZeroTmp);
	EXPECT_FLOAT_EQ(0.6, vec2fTmp.x());
	EXPECT_FLOAT_EQ(0.8, vec2fTmp.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fTmp.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fTmp.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fTmp.z());
	EXPECT_FLOAT_EQ(0.0, vec2fZeroTmp.x());
	EXPECT_FLOAT_EQ(1.0, vec2fZeroTmp.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.x());
	EXPECT_FLOAT_EQ(1.0, vec3fZeroTmp.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.z());

	// Check that we are getting the expected values for eigenNormalizeDefZ
	vec3fTmp = vec3f; eigenNormalizeDefZ(vec3fTmp);
	vec3fZeroTmp = vec3fZero; eigenNormalizeDefZ(vec3fZeroTmp);
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fTmp.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fTmp.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fTmp.z());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.x());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroTmp.y());
	EXPECT_FLOAT_EQ(1.0, vec3fZeroTmp.z());
}

// Test: normalized, eigenNormalized, eigenNormalizedDefX, eigenNormalizedDefY, eigenNormalizedDefZ (double)
TEST(MathVecMatTest, test_eigenNormalizedDouble)
{
	// Test cases
	Eigen::Vector2d vec2d(0.3, 0.4), vec2dUnit;
	Eigen::Vector3d vec3d(1.2, 0.4, 0.3), vec3dUnit;
	Eigen::Vector2d vec2dZero(0.0, 0.0), vec2dZeroUnit;
	Eigen::Vector3d vec3dZero(0.0, 0.0, 0.0), vec3dZeroUnit;

	// Check that we are getting the expected values for normalized
	vec2dUnit = vec2d.normalized();
	vec3dUnit = vec3d.normalized();
	vec2dZeroUnit = vec2dZero.normalized();
	vec3dZeroUnit = vec3dZero.normalized();
	EXPECT_DOUBLE_EQ(0.6, vec2dUnit.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dUnit.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dUnit.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dUnit.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dUnit.z());
	EXPECT_TRUE(std::isnan(vec2dZeroUnit.x()));
	EXPECT_TRUE(std::isnan(vec2dZeroUnit.y()));
	EXPECT_TRUE(std::isnan(vec3dZeroUnit.x()));
	EXPECT_TRUE(std::isnan(vec3dZeroUnit.y()));
	EXPECT_TRUE(std::isnan(vec3dZeroUnit.z()));

	// Check that we are getting the expected values for eigenNormalized (zero)
	vec2dUnit = eigenNormalized(vec2d);
	vec3dUnit = eigenNormalized(vec3d);
	vec2dZeroUnit = eigenNormalized(vec2dZero);
	vec3dZeroUnit = eigenNormalized(vec3dZero);
	EXPECT_DOUBLE_EQ(0.6, vec2dUnit.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dUnit.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dUnit.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dUnit.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dUnit.z());
	EXPECT_DOUBLE_EQ(0.0, vec2dZeroUnit.x());
	EXPECT_DOUBLE_EQ(0.0, vec2dZeroUnit.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.x());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.z());

	// Check that we are getting the expected values for eigenNormalized (custom)
	Eigen::Vector2d vec2dCustom(0.7, 0.9);
	Eigen::Vector3d vec3dCustom(0.7, 0.9, 0.2);
	vec2dUnit = eigenNormalized(vec2d, vec2dCustom);
	vec3dUnit = eigenNormalized(vec3d, vec3dCustom);
	vec2dZeroUnit = eigenNormalized(vec2dZero, vec2dCustom);
	vec3dZeroUnit = eigenNormalized(vec3dZero, vec3dCustom);
	EXPECT_DOUBLE_EQ(0.6, vec2dUnit.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dUnit.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dUnit.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dUnit.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dUnit.z());
	EXPECT_DOUBLE_EQ(0.7, vec2dZeroUnit.x());
	EXPECT_DOUBLE_EQ(0.9, vec2dZeroUnit.y());
	EXPECT_DOUBLE_EQ(0.7, vec3dZeroUnit.x());
	EXPECT_DOUBLE_EQ(0.9, vec3dZeroUnit.y());
	EXPECT_DOUBLE_EQ(0.2, vec3dZeroUnit.z());

	// Check that we are getting the expected values for eigenNormalizedDefX
	vec2dUnit = eigenNormalizedDefX(vec2d);
	vec3dUnit = eigenNormalizedDefX(vec3d);
	vec2dZeroUnit = eigenNormalizedDefX(vec2dZero);
	vec3dZeroUnit = eigenNormalizedDefX(vec3dZero);
	EXPECT_DOUBLE_EQ(0.6, vec2dUnit.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dUnit.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dUnit.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dUnit.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dUnit.z());
	EXPECT_DOUBLE_EQ(1.0, vec2dZeroUnit.x());
	EXPECT_DOUBLE_EQ(0.0, vec2dZeroUnit.y());
	EXPECT_DOUBLE_EQ(1.0, vec3dZeroUnit.x());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.z());

	// Check that we are getting the expected values for eigenNormalizedDefY
	vec2dUnit = eigenNormalizedDefY(vec2d);
	vec3dUnit = eigenNormalizedDefY(vec3d);
	vec2dZeroUnit = eigenNormalizedDefY(vec2dZero);
	vec3dZeroUnit = eigenNormalizedDefY(vec3dZero);
	EXPECT_DOUBLE_EQ(0.6, vec2dUnit.x());
	EXPECT_DOUBLE_EQ(0.8, vec2dUnit.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dUnit.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dUnit.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dUnit.z());
	EXPECT_DOUBLE_EQ(0.0, vec2dZeroUnit.x());
	EXPECT_DOUBLE_EQ(1.0, vec2dZeroUnit.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.x());
	EXPECT_DOUBLE_EQ(1.0, vec3dZeroUnit.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.z());

	// Check that we are getting the expected values for eigenNormalizedDefZ
	vec3dUnit = eigenNormalizedDefZ(vec3d);
	vec3dZeroUnit = eigenNormalizedDefZ(vec3dZero);
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3dUnit.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3dUnit.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3dUnit.z());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.x());
	EXPECT_DOUBLE_EQ(0.0, vec3dZeroUnit.y());
	EXPECT_DOUBLE_EQ(1.0, vec3dZeroUnit.z());
}

// Test: normalized, eigenNormalized, eigenNormalizedDefX, eigenNormalizedDefY, eigenNormalizedDefZ (float)
TEST(MathVecMatTest, test_eigenNormalizedFloat)
{
	// Test cases
	Eigen::Vector2f vec2f(0.3, 0.4), vec2fUnit;
	Eigen::Vector3f vec3f(1.2, 0.4, 0.3), vec3fUnit;
	Eigen::Vector2f vec2fZero(0.0, 0.0), vec2fZeroUnit;
	Eigen::Vector3f vec3fZero(0.0, 0.0, 0.0), vec3fZeroUnit;

	// Check that we are getting the expected values for normalized
	vec2fUnit = vec2f.normalized();
	vec3fUnit = vec3f.normalized();
	vec2fZeroUnit = vec2fZero.normalized();
	vec3fZeroUnit = vec3fZero.normalized();
	EXPECT_FLOAT_EQ(0.6, vec2fUnit.x());
	EXPECT_FLOAT_EQ(0.8, vec2fUnit.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fUnit.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fUnit.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fUnit.z());
	EXPECT_TRUE(std::isnan(vec2fZeroUnit.x()));
	EXPECT_TRUE(std::isnan(vec2fZeroUnit.y()));
	EXPECT_TRUE(std::isnan(vec3fZeroUnit.x()));
	EXPECT_TRUE(std::isnan(vec3fZeroUnit.y()));
	EXPECT_TRUE(std::isnan(vec3fZeroUnit.z()));

	// Check that we are getting the expected values for eigenNormalized (zero)
	vec2fUnit = eigenNormalized(vec2f);
	vec3fUnit = eigenNormalized(vec3f);
	vec2fZeroUnit = eigenNormalized(vec2fZero);
	vec3fZeroUnit = eigenNormalized(vec3fZero);
	EXPECT_FLOAT_EQ(0.6, vec2fUnit.x());
	EXPECT_FLOAT_EQ(0.8, vec2fUnit.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fUnit.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fUnit.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fUnit.z());
	EXPECT_FLOAT_EQ(0.0, vec2fZeroUnit.x());
	EXPECT_FLOAT_EQ(0.0, vec2fZeroUnit.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.x());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.z());

	// Check that we are getting the expected values for eigenNormalized (custom)
	Eigen::Vector2f vec2fCustom(0.7, 0.9);
	Eigen::Vector3f vec3fCustom(0.7, 0.9, 0.2);
	vec2fUnit = eigenNormalized(vec2f, vec2fCustom);
	vec3fUnit = eigenNormalized(vec3f, vec3fCustom);
	vec2fZeroUnit = eigenNormalized(vec2fZero, vec2fCustom);
	vec3fZeroUnit = eigenNormalized(vec3fZero, vec3fCustom);
	EXPECT_FLOAT_EQ(0.6, vec2fUnit.x());
	EXPECT_FLOAT_EQ(0.8, vec2fUnit.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fUnit.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fUnit.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fUnit.z());
	EXPECT_FLOAT_EQ(0.7, vec2fZeroUnit.x());
	EXPECT_FLOAT_EQ(0.9, vec2fZeroUnit.y());
	EXPECT_FLOAT_EQ(0.7, vec3fZeroUnit.x());
	EXPECT_FLOAT_EQ(0.9, vec3fZeroUnit.y());
	EXPECT_FLOAT_EQ(0.2, vec3fZeroUnit.z());

	// Check that we are getting the expected values for eigenNormalizedDefX
	vec2fUnit = eigenNormalizedDefX(vec2f);
	vec3fUnit = eigenNormalizedDefX(vec3f);
	vec2fZeroUnit = eigenNormalizedDefX(vec2fZero);
	vec3fZeroUnit = eigenNormalizedDefX(vec3fZero);
	EXPECT_FLOAT_EQ(0.6, vec2fUnit.x());
	EXPECT_FLOAT_EQ(0.8, vec2fUnit.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fUnit.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fUnit.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fUnit.z());
	EXPECT_FLOAT_EQ(1.0, vec2fZeroUnit.x());
	EXPECT_FLOAT_EQ(0.0, vec2fZeroUnit.y());
	EXPECT_FLOAT_EQ(1.0, vec3fZeroUnit.x());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.z());

	// Check that we are getting the expected values for eigenNormalizedDefY
	vec2fUnit = eigenNormalizedDefY(vec2f);
	vec3fUnit = eigenNormalizedDefY(vec3f);
	vec2fZeroUnit = eigenNormalizedDefY(vec2fZero);
	vec3fZeroUnit = eigenNormalizedDefY(vec3fZero);
	EXPECT_FLOAT_EQ(0.6, vec2fUnit.x());
	EXPECT_FLOAT_EQ(0.8, vec2fUnit.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fUnit.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fUnit.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fUnit.z());
	EXPECT_FLOAT_EQ(0.0, vec2fZeroUnit.x());
	EXPECT_FLOAT_EQ(1.0, vec2fZeroUnit.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.x());
	EXPECT_FLOAT_EQ(1.0, vec3fZeroUnit.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.z());

	// Check that we are getting the expected values for eigenNormalizedDefZ
	vec3fUnit = eigenNormalizedDefZ(vec3f);
	vec3fZeroUnit = eigenNormalizedDefZ(vec3fZero);
	EXPECT_FLOAT_EQ(1.2/1.3, vec3fUnit.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3fUnit.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3fUnit.z());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.x());
	EXPECT_FLOAT_EQ(0.0, vec3fZeroUnit.y());
	EXPECT_FLOAT_EQ(1.0, vec3fZeroUnit.z());
}

// Test: zeroZ, withZ
TEST(MathVecMatTest, test_zeroZ_withZ)
{
	//
	// Float
	//

	// Check that we are getting the expected values for zeroZ
	Eigen::Vector2f vec2f(1.41, 2.97);
	Eigen::Vector3f vec3fa = zeroZ(vec2f);
	EXPECT_EQ(vec2f.x(), vec3fa.x());
	EXPECT_EQ(vec2f.y(), vec3fa.y());
	EXPECT_EQ(0.0      , vec3fa.z());

	// Check that we are getting the expected values for withZ
	Eigen::Vector3f vec3fb = withZ(vec2f, 3.83f);
	EXPECT_EQ(vec2f.x(), vec3fb.x());
	EXPECT_EQ(vec2f.y(), vec3fb.y());
	EXPECT_EQ(3.83f    , vec3fb.z());

	//
	// Double
	//

	// Check that we are getting the expected values for zeroZ
	Eigen::Vector2d vec2d(1.41, 2.97);
	Eigen::Vector3d vec3da = zeroZ(vec2d);
	EXPECT_EQ(vec2d.x(), vec3da.x());
	EXPECT_EQ(vec2d.y(), vec3da.y());
	EXPECT_EQ(0.0      , vec3da.z());

	// Check that we are getting the expected values for withZ
	Eigen::Vector3d vec3db = withZ(vec2d, 3.83);
	EXPECT_EQ(vec2d.x(), vec3db.x());
	EXPECT_EQ(vec2d.y(), vec3db.y());
	EXPECT_EQ(3.83     , vec3db.z());
}

// Test: eigenToTF
TEST(MathVecMatTest, test_eigenToTF)
{
	// Check that we are getting the expected vector conversion
	Eigen::Vector3d eigenvec(1.68, 2.03, 3.25);
	tf::Vector3 tfvec = eigenToTF(eigenvec);
	EXPECT_EQ(eigenvec.x(), tfvec.x());
	EXPECT_EQ(eigenvec.y(), tfvec.y());
	EXPECT_EQ(eigenvec.z(), tfvec.z());
	EXPECT_EQ(0.0         , tfvec.w());

	// Check that we are getting the expected matrix conversion
	Eigen::Matrix3d eigenmat;
	eigenmat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
	tf::Matrix3x3 tfmat = eigenToTF(eigenmat);
	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
			EXPECT_EQ(eigenmat(i,j), tfmat[i][j]);
}

// Test: tfToEigen
TEST(MathVecMatTest, test_tfToEigen)
{
	// Check that we are getting the expected vector conversion
	tf::Vector3 tfvec(1.68, 2.03, 3.25);
	Eigen::Vector3d eigenvec = tfToEigen(tfvec);
	EXPECT_EQ(tfvec.x(), eigenvec.x());
	EXPECT_EQ(tfvec.y(), eigenvec.y());
	EXPECT_EQ(tfvec.z(), eigenvec.z());

	// Check that we are getting the expected matrix conversion
	tf::Matrix3x3 tfmat(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
	Eigen::Matrix3d eigenmat = tfToEigen(tfmat);
	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
			EXPECT_EQ(tfmat[i][j], eigenmat(i,j));
}

// Test: eigenAngleOf
TEST(MathVecMatTest, test_eigenAngleOf)
{
	// Test cases
	Eigen::Vector2d vd1(0.0, 3.0);
	Eigen::Vector2d vd2(4.0, 0.0);
	Eigen::Vector2d vd3(2.0, 2.0);
	Eigen::Vector2d vd4(-1.52, 0.78);
	Eigen::Vector2f vf1 = vd1.cast<float>();
	Eigen::Vector2f vf2 = vd2.cast<float>();
	Eigen::Vector2f vf3 = vd3.cast<float>();
	Eigen::Vector2f vf4 = vd4.cast<float>();

	// Check eigenAngleOf (double)
	EXPECT_DOUBLE_EQ(M_PI_2, eigenAngleOf(vd1));
	EXPECT_DOUBLE_EQ(0.0, eigenAngleOf(vd2));
	EXPECT_DOUBLE_EQ(M_PI_4, eigenAngleOf(vd3));
	EXPECT_DOUBLE_EQ(2.6674742230842905, eigenAngleOf(vd4));

	// Check eigenAngleOf (float)
	EXPECT_FLOAT_EQ(M_PI_2, eigenAngleOf(vf1));
	EXPECT_FLOAT_EQ(0.0, eigenAngleOf(vf2));
	EXPECT_FLOAT_EQ(M_PI_4, eigenAngleOf(vf3));
	EXPECT_FLOAT_EQ(2.6674742230842905, eigenAngleOf(vf4));
}

// Test: eigenRotateCW, eigenRotateCCW (double)
TEST(MathVecMatTest, test_eigenRotateDouble)
{
	// Declare variables
	Eigen::Vector2d vdtmp;
	Eigen::Vector2d vd1(0.0, 3.0);
	Eigen::Vector2d vd2(2.0, 2.0);
	Eigen::Vector2d vd3(-1.52, 0.78);
	double ang1 = 0.0;
	double ang2 = M_PI_2;
	double ang3 = 2.317;

	// Check eigenRotateCW
	vdtmp = vd1; eigenRotateCW(vdtmp, ang1);
	EXPECT_DOUBLE_EQ(0.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(3.0, vdtmp.y());
	vdtmp = vd2; eigenRotateCW(vdtmp, ang1);
	EXPECT_DOUBLE_EQ(2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(2.0, vdtmp.y());
	vdtmp = vd3; eigenRotateCW(vdtmp, ang1);
	EXPECT_DOUBLE_EQ(-1.52, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.78, vdtmp.y());
	vdtmp = vd1; eigenRotateCW(vdtmp, ang2);
	EXPECT_DOUBLE_EQ(3.0, vdtmp.x());
	EXPECT_NEAR(0.0, vdtmp.y(), 5e-16); // A value of zero doesn't compare well with ULP size
	vdtmp = vd2; eigenRotateCW(vdtmp, ang2);
	EXPECT_DOUBLE_EQ(2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.0, vdtmp.y());
	vdtmp = vd3; eigenRotateCW(vdtmp, ang2);
	EXPECT_DOUBLE_EQ(0.78, vdtmp.x());
	EXPECT_DOUBLE_EQ(1.52, vdtmp.y());
	vdtmp = vd1; eigenRotateCW(vdtmp, ang3);
	EXPECT_DOUBLE_EQ(2.2028139407039871, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.0365683741628149, vdtmp.y());
	vdtmp = vd2; eigenRotateCW(vdtmp, ang3);
	EXPECT_DOUBLE_EQ(0.1108303776941149, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.8262548765778677, vdtmp.y());
	vdtmp = vd3; eigenRotateCW(vdtmp, ang3);
	EXPECT_DOUBLE_EQ(1.6045929341588629, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.5865846193410215, vdtmp.y());

	// Check eigenRotateCCW
	vdtmp = vd1; eigenRotateCCW(vdtmp, ang1);
	EXPECT_DOUBLE_EQ(0.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(3.0, vdtmp.y());
	vdtmp = vd2; eigenRotateCCW(vdtmp, ang1);
	EXPECT_DOUBLE_EQ(2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(2.0, vdtmp.y());
	vdtmp = vd3; eigenRotateCCW(vdtmp, ang1);
	EXPECT_DOUBLE_EQ(-1.52, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.78, vdtmp.y());
	vdtmp = vd1; eigenRotateCCW(vdtmp, ang2);
	EXPECT_DOUBLE_EQ(-3.0, vdtmp.x());
	EXPECT_NEAR(0.0, vdtmp.y(), 5e-16); // A value of zero doesn't compare well with ULP size
	vdtmp = vd2; eigenRotateCCW(vdtmp, ang2);
	EXPECT_DOUBLE_EQ(-2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(2.0, vdtmp.y());
	vdtmp = vd3; eigenRotateCCW(vdtmp, ang2);
	EXPECT_DOUBLE_EQ(-0.78, vdtmp.x());
	EXPECT_DOUBLE_EQ(-1.52, vdtmp.y());
	vdtmp = vd1; eigenRotateCCW(vdtmp, ang3);
	EXPECT_DOUBLE_EQ(-2.2028139407039871, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.0365683741628149, vdtmp.y());
	vdtmp = vd2; eigenRotateCCW(vdtmp, ang3);
	EXPECT_DOUBLE_EQ(-2.8262548765778677, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.1108303776941149, vdtmp.y());
	vdtmp = vd3; eigenRotateCCW(vdtmp, ang3);
	EXPECT_DOUBLE_EQ(0.4591296849927896, vdtmp.x());
	EXPECT_DOUBLE_EQ(-1.6456001739056854, vdtmp.y());
}

// Test: eigenRotateCW, eigenRotateCCW (float)
TEST(MathVecMatTest, test_eigenRotateFloat)
{
	// Declare variables
	Eigen::Vector2f vftmp;
	Eigen::Vector2f vf1(0.0, 3.0);
	Eigen::Vector2f vf2(2.0, 2.0);
	Eigen::Vector2f vf3(-1.52, 0.78);
	double ang1 = 0.0;
	double ang2 = M_PI_2;
	double ang3 = 2.317;

	// Check eigenRotateCW
	vftmp = vf1; eigenRotateCW(vftmp, ang1);
	EXPECT_FLOAT_EQ(0.0, vftmp.x());
	EXPECT_FLOAT_EQ(3.0, vftmp.y());
	vftmp = vf2; eigenRotateCW(vftmp, ang1);
	EXPECT_FLOAT_EQ(2.0, vftmp.x());
	EXPECT_FLOAT_EQ(2.0, vftmp.y());
	vftmp = vf3; eigenRotateCW(vftmp, ang1);
	EXPECT_FLOAT_EQ(-1.52, vftmp.x());
	EXPECT_FLOAT_EQ(0.78, vftmp.y());
	vftmp = vf1; eigenRotateCW(vftmp, ang2);
	EXPECT_FLOAT_EQ(3.0, vftmp.x());
	EXPECT_NEAR(0.0, vftmp.y(), 5e-16); // A value of zero doesn't compare well with ULP size
	vftmp = vf2; eigenRotateCW(vftmp, ang2);
	EXPECT_FLOAT_EQ(2.0, vftmp.x());
	EXPECT_FLOAT_EQ(-2.0, vftmp.y());
	vftmp = vf3; eigenRotateCW(vftmp, ang2);
	EXPECT_FLOAT_EQ(0.78, vftmp.x());
	EXPECT_FLOAT_EQ(1.52, vftmp.y());
	vftmp = vf1; eigenRotateCW(vftmp, ang3);
	EXPECT_FLOAT_EQ(2.2028139407039871, vftmp.x());
	EXPECT_FLOAT_EQ(-2.0365683741628149, vftmp.y());
	vftmp = vf2; eigenRotateCW(vftmp, ang3);
	EXPECT_FLOAT_EQ(0.1108303776941149, vftmp.x());
	EXPECT_FLOAT_EQ(-2.8262548765778677, vftmp.y());
	vftmp = vf3; eigenRotateCW(vftmp, ang3);
	EXPECT_FLOAT_EQ(1.6045929341588629, vftmp.x());
	EXPECT_FLOAT_EQ(0.5865846193410215, vftmp.y());

	// Check eigenRotateCCW
	vftmp = vf1; eigenRotateCCW(vftmp, ang1);
	EXPECT_FLOAT_EQ(0.0, vftmp.x());
	EXPECT_FLOAT_EQ(3.0, vftmp.y());
	vftmp = vf2; eigenRotateCCW(vftmp, ang1);
	EXPECT_FLOAT_EQ(2.0, vftmp.x());
	EXPECT_FLOAT_EQ(2.0, vftmp.y());
	vftmp = vf3; eigenRotateCCW(vftmp, ang1);
	EXPECT_FLOAT_EQ(-1.52, vftmp.x());
	EXPECT_FLOAT_EQ(0.78, vftmp.y());
	vftmp = vf1; eigenRotateCCW(vftmp, ang2);
	EXPECT_FLOAT_EQ(-3.0, vftmp.x());
	EXPECT_NEAR(0.0, vftmp.y(), 5e-16); // A value of zero doesn't compare well with ULP size
	vftmp = vf2; eigenRotateCCW(vftmp, ang2);
	EXPECT_FLOAT_EQ(-2.0, vftmp.x());
	EXPECT_FLOAT_EQ(2.0, vftmp.y());
	vftmp = vf3; eigenRotateCCW(vftmp, ang2);
	EXPECT_FLOAT_EQ(-0.78, vftmp.x());
	EXPECT_FLOAT_EQ(-1.52, vftmp.y());
	vftmp = vf1; eigenRotateCCW(vftmp, ang3);
	EXPECT_FLOAT_EQ(-2.2028139407039871, vftmp.x());
	EXPECT_FLOAT_EQ(-2.0365683741628149, vftmp.y());
	vftmp = vf2; eigenRotateCCW(vftmp, ang3);
	EXPECT_FLOAT_EQ(-2.8262548765778677, vftmp.x());
	EXPECT_FLOAT_EQ(0.1108303776941149, vftmp.y());
	vftmp = vf3; eigenRotateCCW(vftmp, ang3);
	EXPECT_FLOAT_EQ(0.4591296849927896, vftmp.x());
	EXPECT_FLOAT_EQ(-1.6456001739056854, vftmp.y());
}

// Test: eigenRotatedCW, eigenRotatedCCW (double)
TEST(MathVecMatTest, test_eigenRotatedDouble)
{
	// Declare variables
	Eigen::Vector2d vdtmp;
	Eigen::Vector2d vd1(0.0, 3.0);
	Eigen::Vector2d vd2(2.0, 2.0);
	Eigen::Vector2d vd3(-1.52, 0.78);
	double ang1 = 0.0;
	double ang2 = M_PI_2;
	double ang3 = 2.317;

	// Check eigenRotatedCW
	vdtmp = eigenRotatedCW(vd1, ang1);
	EXPECT_DOUBLE_EQ(0.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(3.0, vdtmp.y());
	vdtmp = eigenRotatedCW(vd2, ang1);
	EXPECT_DOUBLE_EQ(2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(2.0, vdtmp.y());
	vdtmp = eigenRotatedCW(vd3, ang1);
	EXPECT_DOUBLE_EQ(-1.52, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.78, vdtmp.y());
	vdtmp = eigenRotatedCW(vd1, ang2);
	EXPECT_DOUBLE_EQ(3.0, vdtmp.x());
	EXPECT_NEAR(0.0, vdtmp.y(), 5e-16); // A value of zero doesn't compare well with ULP size
	vdtmp = eigenRotatedCW(vd2, ang2);
	EXPECT_DOUBLE_EQ(2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.0, vdtmp.y());
	vdtmp = eigenRotatedCW(vd3, ang2);
	EXPECT_DOUBLE_EQ(0.78, vdtmp.x());
	EXPECT_DOUBLE_EQ(1.52, vdtmp.y());
	vdtmp = eigenRotatedCW(vd1, ang3);
	EXPECT_DOUBLE_EQ(2.2028139407039871, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.0365683741628149, vdtmp.y());
	vdtmp = eigenRotatedCW(vd2, ang3);
	EXPECT_DOUBLE_EQ(0.1108303776941149, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.8262548765778677, vdtmp.y());
	vdtmp = eigenRotatedCW(vd3, ang3);
	EXPECT_DOUBLE_EQ(1.6045929341588629, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.5865846193410215, vdtmp.y());

	// Check eigenRotatedCCW
	vdtmp = eigenRotatedCCW(vd1, ang1);
	EXPECT_DOUBLE_EQ(0.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(3.0, vdtmp.y());
	vdtmp = eigenRotatedCCW(vd2, ang1);
	EXPECT_DOUBLE_EQ(2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(2.0, vdtmp.y());
	vdtmp = eigenRotatedCCW(vd3, ang1);
	EXPECT_DOUBLE_EQ(-1.52, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.78, vdtmp.y());
	vdtmp = eigenRotatedCCW(vd1, ang2);
	EXPECT_DOUBLE_EQ(-3.0, vdtmp.x());
	EXPECT_NEAR(0.0, vdtmp.y(), 5e-16); // A value of zero doesn't compare well with ULP size
	vdtmp = eigenRotatedCCW(vd2, ang2);
	EXPECT_DOUBLE_EQ(-2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(2.0, vdtmp.y());
	vdtmp = eigenRotatedCCW(vd3, ang2);
	EXPECT_DOUBLE_EQ(-0.78, vdtmp.x());
	EXPECT_DOUBLE_EQ(-1.52, vdtmp.y());
	vdtmp = eigenRotatedCCW(vd1, ang3);
	EXPECT_DOUBLE_EQ(-2.2028139407039871, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.0365683741628149, vdtmp.y());
	vdtmp = eigenRotatedCCW(vd2, ang3);
	EXPECT_DOUBLE_EQ(-2.8262548765778677, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.1108303776941149, vdtmp.y());
	vdtmp = eigenRotatedCCW(vd3, ang3);
	EXPECT_DOUBLE_EQ(0.4591296849927896, vdtmp.x());
	EXPECT_DOUBLE_EQ(-1.6456001739056854, vdtmp.y());
}

// Test: eigenRotatedCW, eigenRotatedCCW (float)
TEST(MathVecMatTest, test_eigenRotatedFloat)
{
	// Declare variables
	Eigen::Vector2f vftmp;
	Eigen::Vector2f vf1(0.0, 3.0);
	Eigen::Vector2f vf2(2.0, 2.0);
	Eigen::Vector2f vf3(-1.52, 0.78);
	double ang1 = 0.0;
	double ang2 = M_PI_2;
	double ang3 = 2.317;

	// Check eigenRotatedCW
	vftmp = eigenRotatedCW(vf1, ang1);
	EXPECT_FLOAT_EQ(0.0, vftmp.x());
	EXPECT_FLOAT_EQ(3.0, vftmp.y());
	vftmp = eigenRotatedCW(vf2, ang1);
	EXPECT_FLOAT_EQ(2.0, vftmp.x());
	EXPECT_FLOAT_EQ(2.0, vftmp.y());
	vftmp = eigenRotatedCW(vf3, ang1);
	EXPECT_FLOAT_EQ(-1.52, vftmp.x());
	EXPECT_FLOAT_EQ(0.78, vftmp.y());
	vftmp = eigenRotatedCW(vf1, ang2);
	EXPECT_FLOAT_EQ(3.0, vftmp.x());
	EXPECT_NEAR(0.0, vftmp.y(), 5e-16); // A value of zero doesn't compare well with ULP size
	vftmp = eigenRotatedCW(vf2, ang2);
	EXPECT_FLOAT_EQ(2.0, vftmp.x());
	EXPECT_FLOAT_EQ(-2.0, vftmp.y());
	vftmp = eigenRotatedCW(vf3, ang2);
	EXPECT_FLOAT_EQ(0.78, vftmp.x());
	EXPECT_FLOAT_EQ(1.52, vftmp.y());
	vftmp = eigenRotatedCW(vf1, ang3);
	EXPECT_FLOAT_EQ(2.2028139407039871, vftmp.x());
	EXPECT_FLOAT_EQ(-2.0365683741628149, vftmp.y());
	vftmp = eigenRotatedCW(vf2, ang3);
	EXPECT_FLOAT_EQ(0.1108303776941149, vftmp.x());
	EXPECT_FLOAT_EQ(-2.8262548765778677, vftmp.y());
	vftmp = eigenRotatedCW(vf3, ang3);
	EXPECT_FLOAT_EQ(1.6045929341588629, vftmp.x());
	EXPECT_FLOAT_EQ(0.5865846193410215, vftmp.y());

	// Check eigenRotatedCCW
	vftmp = eigenRotatedCCW(vf1, ang1);
	EXPECT_FLOAT_EQ(0.0, vftmp.x());
	EXPECT_FLOAT_EQ(3.0, vftmp.y());
	vftmp = eigenRotatedCCW(vf2, ang1);
	EXPECT_FLOAT_EQ(2.0, vftmp.x());
	EXPECT_FLOAT_EQ(2.0, vftmp.y());
	vftmp = eigenRotatedCCW(vf3, ang1);
	EXPECT_FLOAT_EQ(-1.52, vftmp.x());
	EXPECT_FLOAT_EQ(0.78, vftmp.y());
	vftmp = eigenRotatedCCW(vf1, ang2);
	EXPECT_FLOAT_EQ(-3.0, vftmp.x());
	EXPECT_NEAR(0.0, vftmp.y(), 5e-16); // A value of zero doesn't compare well with ULP size
	vftmp = eigenRotatedCCW(vf2, ang2);
	EXPECT_FLOAT_EQ(-2.0, vftmp.x());
	EXPECT_FLOAT_EQ(2.0, vftmp.y());
	vftmp = eigenRotatedCCW(vf3, ang2);
	EXPECT_FLOAT_EQ(-0.78, vftmp.x());
	EXPECT_FLOAT_EQ(-1.52, vftmp.y());
	vftmp = eigenRotatedCCW(vf1, ang3);
	EXPECT_FLOAT_EQ(-2.2028139407039871, vftmp.x());
	EXPECT_FLOAT_EQ(-2.0365683741628149, vftmp.y());
	vftmp = eigenRotatedCCW(vf2, ang3);
	EXPECT_FLOAT_EQ(-2.8262548765778677, vftmp.x());
	EXPECT_FLOAT_EQ(0.1108303776941149, vftmp.y());
	vftmp = eigenRotatedCCW(vf3, ang3);
	EXPECT_FLOAT_EQ(0.4591296849927896, vftmp.x());
	EXPECT_FLOAT_EQ(-1.6456001739056854, vftmp.y());
}

// Test: eigenRotateCW90, eigenRotateCCW90 (double)
TEST(MathVecMatTest, test_eigenRotate90Double)
{
	// Declare variables
	Eigen::Vector2d vdtmp;
	Eigen::Vector2d vd1(0.0, 3.0);
	Eigen::Vector2d vd2(2.0, 2.0);
	Eigen::Vector2d vd3(-1.52, 0.78);

	// Check eigenRotateCW90
	vdtmp = vd1; eigenRotateCW90(vdtmp);
	EXPECT_DOUBLE_EQ(3.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.0, vdtmp.y());
	vdtmp = vd2; eigenRotateCW90(vdtmp);
	EXPECT_DOUBLE_EQ(2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.0, vdtmp.y());
	vdtmp = vd3; eigenRotateCW90(vdtmp);
	EXPECT_DOUBLE_EQ(0.78, vdtmp.x());
	EXPECT_DOUBLE_EQ(1.52, vdtmp.y());

	// Check eigenRotateCCW90
	vdtmp = vd1; eigenRotateCCW90(vdtmp);
	EXPECT_DOUBLE_EQ(-3.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.0, vdtmp.y());
	vdtmp = vd2; eigenRotateCCW90(vdtmp);
	EXPECT_DOUBLE_EQ(-2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(2.0, vdtmp.y());
	vdtmp = vd3; eigenRotateCCW90(vdtmp);
	EXPECT_DOUBLE_EQ(-0.78, vdtmp.x());
	EXPECT_DOUBLE_EQ(-1.52, vdtmp.y());
}

// Test: eigenRotateCW90, eigenRotateCCW90 (float)
TEST(MathVecMatTest, test_eigenRotate90Float)
{
	// Declare variables
	Eigen::Vector2f vftmp;
	Eigen::Vector2f vf1(0.0, 3.0);
	Eigen::Vector2f vf2(2.0, 2.0);
	Eigen::Vector2f vf3(-1.52, 0.78);

	// Check eigenRotateCW90
	vftmp = vf1; eigenRotateCW90(vftmp);
	EXPECT_FLOAT_EQ(3.0, vftmp.x());
	EXPECT_FLOAT_EQ(0.0, vftmp.y());
	vftmp = vf2; eigenRotateCW90(vftmp);
	EXPECT_FLOAT_EQ(2.0, vftmp.x());
	EXPECT_FLOAT_EQ(-2.0, vftmp.y());
	vftmp = vf3; eigenRotateCW90(vftmp);
	EXPECT_FLOAT_EQ(0.78, vftmp.x());
	EXPECT_FLOAT_EQ(1.52, vftmp.y());

	// Check eigenRotateCCW90
	vftmp = vf1; eigenRotateCCW90(vftmp);
	EXPECT_FLOAT_EQ(-3.0, vftmp.x());
	EXPECT_FLOAT_EQ(0.0, vftmp.y());
	vftmp = vf2; eigenRotateCCW90(vftmp);
	EXPECT_FLOAT_EQ(-2.0, vftmp.x());
	EXPECT_FLOAT_EQ(2.0, vftmp.y());
	vftmp = vf3; eigenRotateCCW90(vftmp);
	EXPECT_FLOAT_EQ(-0.78, vftmp.x());
	EXPECT_FLOAT_EQ(-1.52, vftmp.y());
}

// Test: eigenRotatedCW90, eigenRotatedCCW90 (double)
TEST(MathVecMatTest, test_eigenRotated90Double)
{
	// Declare variables
	Eigen::Vector2d vdtmp;
	Eigen::Vector2d vd1(0.0, 3.0);
	Eigen::Vector2d vd2(2.0, 2.0);
	Eigen::Vector2d vd3(-1.52, 0.78);

	// Check eigenRotatedCW90
	vdtmp = eigenRotatedCW90(vd1);
	EXPECT_DOUBLE_EQ(3.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.0, vdtmp.y());
	vdtmp = eigenRotatedCW90(vd2);
	EXPECT_DOUBLE_EQ(2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(-2.0, vdtmp.y());
	vdtmp = eigenRotatedCW90(vd3);
	EXPECT_DOUBLE_EQ(0.78, vdtmp.x());
	EXPECT_DOUBLE_EQ(1.52, vdtmp.y());

	// Check eigenRotatedCCW90
	vdtmp = eigenRotatedCCW90(vd1);
	EXPECT_DOUBLE_EQ(-3.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(0.0, vdtmp.y());
	vdtmp = eigenRotatedCCW90(vd2);
	EXPECT_DOUBLE_EQ(-2.0, vdtmp.x());
	EXPECT_DOUBLE_EQ(2.0, vdtmp.y());
	vdtmp = eigenRotatedCCW90(vd3);
	EXPECT_DOUBLE_EQ(-0.78, vdtmp.x());
	EXPECT_DOUBLE_EQ(-1.52, vdtmp.y());
}

// Test: eigenRotatedCW90, eigenRotatedCCW90 (float)
TEST(MathVecMatTest, test_eigenRotated90Float)
{
	// Declare variables
	Eigen::Vector2f vftmp;
	Eigen::Vector2f vf1(0.0, 3.0);
	Eigen::Vector2f vf2(2.0, 2.0);
	Eigen::Vector2f vf3(-1.52, 0.78);

	// Check eigenRotatedCW90
	vftmp = eigenRotatedCW90(vf1);
	EXPECT_FLOAT_EQ(3.0, vftmp.x());
	EXPECT_FLOAT_EQ(0.0, vftmp.y());
	vftmp = eigenRotatedCW90(vf2);
	EXPECT_FLOAT_EQ(2.0, vftmp.x());
	EXPECT_FLOAT_EQ(-2.0, vftmp.y());
	vftmp = eigenRotatedCW90(vf3);
	EXPECT_FLOAT_EQ(0.78, vftmp.x());
	EXPECT_FLOAT_EQ(1.52, vftmp.y());

	// Check eigenRotatedCCW90
	vftmp = eigenRotatedCCW90(vf1);
	EXPECT_FLOAT_EQ(-3.0, vftmp.x());
	EXPECT_FLOAT_EQ(0.0, vftmp.y());
	vftmp = eigenRotatedCCW90(vf2);
	EXPECT_FLOAT_EQ(-2.0, vftmp.x());
	EXPECT_FLOAT_EQ(2.0, vftmp.y());
	vftmp = eigenRotatedCCW90(vf3);
	EXPECT_FLOAT_EQ(-0.78, vftmp.x());
	EXPECT_FLOAT_EQ(-1.52, vftmp.y());
}

// Test: collapseFlatEllipsoid
TEST(MathVecMatTest, test_collapseFlatEllipsoid)
{
	// Typedefs
	typedef Eigen::Matrix<double, 2, 1> Vec2;
	typedef Eigen::Matrix<double, 3, 1> Vec3;

	// Declare variables
	Vec2 tmp2;
	Vec3 tmp3;

	// Test a case
	tmp2 = collapseFlatEllipsoid(Vec2(3.0, 4.0));
	EXPECT_EQ(3.0, tmp2.x());
	EXPECT_EQ(4.0, tmp2.y());
	tmp2 = collapseFlatEllipsoid(Vec2(0.0, 4.0));
	EXPECT_EQ(0.0, tmp2.x());
	EXPECT_EQ(0.0, tmp2.y());
	tmp2 = collapseFlatEllipsoid(Vec2(3.0, 0.0));
	EXPECT_EQ(0.0, tmp2.x());
	EXPECT_EQ(0.0, tmp2.y());
	tmp2 = collapseFlatEllipsoid(Vec2(0.0, 0.0));
	EXPECT_EQ(0.0, tmp2.x());
	EXPECT_EQ(0.0, tmp2.y());

	// Test a case
	tmp2 = collapseFlatEllipsoid(3.0, 4.0);
	EXPECT_EQ(3.0, tmp2.x());
	EXPECT_EQ(4.0, tmp2.y());
	tmp2 = collapseFlatEllipsoid(0.0, 4.0);
	EXPECT_EQ(0.0, tmp2.x());
	EXPECT_EQ(0.0, tmp2.y());
	tmp2 = collapseFlatEllipsoid(3.0, 0.0);
	EXPECT_EQ(0.0, tmp2.x());
	EXPECT_EQ(0.0, tmp2.y());
	tmp2 = collapseFlatEllipsoid(0.0, 0.0);
	EXPECT_EQ(0.0, tmp2.x());
	EXPECT_EQ(0.0, tmp2.y());

	// Test a case
	tmp3 = collapseFlatEllipsoid(Vec3(3.0, 4.0, 5.0));
	EXPECT_EQ(3.0, tmp3.x());
	EXPECT_EQ(4.0, tmp3.y());
	EXPECT_EQ(5.0, tmp3.z());
	tmp3 = collapseFlatEllipsoid(Vec3(0.0, 4.0, 5.0));
	EXPECT_EQ(0.0, tmp3.x());
	EXPECT_EQ(0.0, tmp3.y());
	EXPECT_EQ(0.0, tmp3.z());
	tmp3 = collapseFlatEllipsoid(Vec3(3.0, 4.0, 0.0));
	EXPECT_EQ(0.0, tmp3.x());
	EXPECT_EQ(0.0, tmp3.y());
	EXPECT_EQ(0.0, tmp3.z());

	// Test a case
	tmp3 = collapseFlatEllipsoid(3.0, 4.0, 5.0);
	EXPECT_EQ(3.0, tmp3.x());
	EXPECT_EQ(4.0, tmp3.y());
	EXPECT_EQ(5.0, tmp3.z());
	tmp3 = collapseFlatEllipsoid(0.0, 4.0, 5.0);
	EXPECT_EQ(0.0, tmp3.x());
	EXPECT_EQ(0.0, tmp3.y());
	EXPECT_EQ(0.0, tmp3.z());
	tmp3 = collapseFlatEllipsoid(3.0, 4.0, 0.0);
	EXPECT_EQ(0.0, tmp3.x());
	EXPECT_EQ(0.0, tmp3.y());
	EXPECT_EQ(0.0, tmp3.z());
}

// Test: ellipsoidRadius (double)
TEST(MathVecMatTest, test_ellipsoidRadius)
{
	// Typedefs
	typedef Eigen::Matrix<double, 1, 1> Vec1;
	typedef Eigen::Matrix<double, 2, 1> Vec2;
	typedef Eigen::Matrix<double, 3, 1> Vec3;

	// Test 1D
	Vec1 semiaxes1d, v1d; // Note: There is no constructor by value for a 'Vector1d'...
	semiaxes1d << -0.7; v1d << -0.4; EXPECT_DOUBLE_EQ(0.7, ellipsoidRadius(semiaxes1d, v1d));
	semiaxes1d <<  0.8; v1d << -0.1; EXPECT_DOUBLE_EQ(0.8, ellipsoidRadius(semiaxes1d, v1d));
	semiaxes1d << -0.5; v1d <<  0.2; EXPECT_DOUBLE_EQ(0.5, ellipsoidRadius(semiaxes1d, v1d));
	semiaxes1d <<  0.3; v1d <<  0.6; EXPECT_DOUBLE_EQ(0.3, ellipsoidRadius(semiaxes1d, v1d));
	semiaxes1d <<  0.3; v1d <<  0.0; EXPECT_DOUBLE_EQ(0.3, ellipsoidRadius(semiaxes1d, v1d));
	semiaxes1d <<  0.0; v1d <<  0.3; EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(semiaxes1d, v1d));

	// Test 2D
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec2(4.0, 6.0), Vec2(0.0, 0.0)));
	EXPECT_DOUBLE_EQ(4.0, ellipsoidRadius(Vec2(4.0, 6.0), Vec2(2.0, 0.0)));
	EXPECT_DOUBLE_EQ(6.0, ellipseRadius(4.0, 6.0, 0.0, -0.5));
	EXPECT_DOUBLE_EQ(4.7067872433164171, ellipsoidRadius(Vec2(4.0, 6.0), Vec2(2.0, 2.0)));
	EXPECT_DOUBLE_EQ(5.6568542494923806, ellipseRadius(4.0, 6.0, -1.0, 3.0));
	EXPECT_DOUBLE_EQ(4.1159660434202126, ellipsoidRadius(Vec2(4.0, 6.0), Vec2(3.0, 1.0)));
	EXPECT_DOUBLE_EQ(4.2426406871192848, ellipseRadius(4.0, -6.0, -2.0, 1.0));
	EXPECT_DOUBLE_EQ(4.0, ellipsoidRadius(Vec2(4.0, 0.0), Vec2(5.0, 0.0)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec2(4.0, 0.0), Vec2(5.0, 0.01)));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(4.0, 0.0, 5.0, -0.01));
	EXPECT_DOUBLE_EQ(6.0, ellipseRadius(0.0, -6.0, 0.0, 0.1));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, -6.0, 0.001, 0.1));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, -6.0, -0.001, 0.1));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec2(0.0, 0.0), Vec2(1.4, 0.7)));
	EXPECT_DOUBLE_EQ(0.0, ellipseRadius(0.0, 0.0, -0.3, 0.0));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec2(0.0, 0.0), Vec2(0.0, 0.7)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec2(0.0, 0.0), Vec2(0.0, 0.0)));

	// Test 3D
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(3.0, 2.0, 1.0), Vec3(0.0, 0.0, 0.0)));
	EXPECT_DOUBLE_EQ(3.0, ellipsoidRadius(Vec3(3.0, 2.0, 1.0), Vec3(2.0, 0.0, 0.0)));
	EXPECT_DOUBLE_EQ(2.0, ellipsoidRadius(Vec3(3.0, -2.0, 1.0), Vec3(0.0, -1.5, 0.0)));
	EXPECT_DOUBLE_EQ(1.0, ellipsoidRadius(Vec3(3.0, 2.0, 1.0), Vec3(0.0, 0.0, 0.7)));
	EXPECT_DOUBLE_EQ(1.6619351998747047, ellipsoidRadius(Vec3(-3.0, 2.0, 1.0), Vec3(-0.2, 1.4, 0.6)));
	EXPECT_DOUBLE_EQ(1.1292776976001264, ellipsoidRadius(Vec3(3.0, 2.0, -1.0), Vec3(0.8, -0.9, 2.0)));
	EXPECT_DOUBLE_EQ(1.0060790833484312, ellipsoidRadius(Vec3(3.0, 2.0, 1.0), Vec3(0.3, 0.2, -3.0)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(3.0, 2.0, 0.0), Vec3(0.3, 0.2, -3.0)));
	EXPECT_DOUBLE_EQ(2.5495097567963922, ellipsoidRadius(Vec3(3.0, 2.0, 0.0), Vec3(0.3, 0.2, 0.0)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(0.0, 2.0, 0.0), Vec3(0.3, 0.2, 0.0)));
	EXPECT_DOUBLE_EQ(2.0, ellipsoidRadius(Vec3(0.0, 2.0, 0.0), Vec3(0.0, -0.2, 0.0)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(0.0, 2.0, 0.0), Vec3(7.0, -0.2, 1.0)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(0.0, 0.0, 0.0), Vec3(0.0, -0.2, 0.0)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(0.0, 0.0, 0.0), Vec3(0.3, -0.2, 5.0)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

	// Test edge cases
	double inf = std::numeric_limits<double>::infinity();
	EXPECT_DOUBLE_EQ(inf, ellipsoidRadius(Vec3(inf, inf, inf), Vec3(1.0, -2.0, 1.5)));
	EXPECT_DOUBLE_EQ(inf, ellipsoidRadius(Vec3(-inf, inf, -inf), Vec3(-3.0, 0.5, 0.2)));
	EXPECT_DOUBLE_EQ(1.0, ellipsoidRadius(Vec3(inf, 1.0, inf), Vec3(0.0, 0.5, 0.0)));
	EXPECT_DOUBLE_EQ(6.0827625302982193, ellipsoidRadius(Vec3(inf, 1.0, inf), Vec3(-3.0, 0.5, 0.0)));
	EXPECT_DOUBLE_EQ(6.0959002616512681, ellipsoidRadius(Vec3(inf, 1.0, inf), Vec3(-3.0, 0.5, 0.2)));
	EXPECT_DOUBLE_EQ(inf, ellipsoidRadius(Vec3(inf, 1.0, inf), Vec3(-3.0, 0.0, 0.2)));
	EXPECT_DOUBLE_EQ(1.0, ellipsoidRadius(Vec3(inf, 1.0, 2.0), Vec3(0.0, 0.5, 0.0)));
	EXPECT_DOUBLE_EQ(2.0, ellipsoidRadius(Vec3(inf, 1.0, -2.0), Vec3(0.0, 0.0, 0.8)));
	EXPECT_DOUBLE_EQ(inf, ellipsoidRadius(Vec3(inf, 1.0, 2.0), Vec3(2.0, 0.0, 0.0)));
	EXPECT_DOUBLE_EQ(8.0622577482985491, ellipsoidRadius(Vec3(inf, 1.0, 2.0), Vec3(2.0, 0.25, 0.0)));
	EXPECT_DOUBLE_EQ(2.3766885889086624, ellipsoidRadius(Vec3(inf, 1.0, 2.0), Vec3(2.0, 0.25, 3.0)));
	EXPECT_TRUE(std::isnan(ellipsoidRadius(Vec3(-3.0, 2.0, 1.0), Vec3(-0.2, NAN, 0.6))));
	EXPECT_TRUE(std::isnan(ellipsoidRadius(Vec3(-3.0, 2.0, NAN), Vec3(-0.2, 1.4, 0.6))));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(-3.0, 2.0, 0.0), Vec3(-0.2, 1.4, NAN)));
	EXPECT_DOUBLE_EQ(0.0, ellipsoidRadius(Vec3(NAN, NAN, 1.0), Vec3(0.0, 0.0, 0.0)));
}

// Test class
struct TestCoerceEllCND
{
	template<typename T> void testCoerceEllC1D(T exp, bool expcoerced, T x, T maxAbs)
	{
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, maxAbs);
		typedef Eigen::Matrix<T, 1, 1> Vec1;
		Vec1 semiaxes, v, ret;
		bool coerced;
		semiaxes << maxAbs;
		v << x;
		ret = v;
		coerceEllC(semiaxes, ret);
		EXPECT_THAT_UTEQ(T, exp, ret.x());
		ret = v; coerced = !expcoerced;
		coerceEllC(semiaxes, ret, coerced);
		EXPECT_THAT_UTEQ(T, exp, ret.x());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
		ret << NAN;
		coerceEllC(semiaxes, v, ret);
		EXPECT_THAT_UTEQ(T, exp, ret.x());
		ret << NAN; coerced = !expcoerced;
		coerceEllC(semiaxes, v, ret, coerced);
		EXPECT_THAT_UTEQ(T, exp, ret.x());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceEllC2D(T expX, T expY, bool expcoerced, T x, T y, T maxAbsX, T maxAbsY)
	{
		std::string args = argsAsString(typeid(T).name(), expX, expY, expcoerced, x, y, maxAbsX, maxAbsY);
		typedef Eigen::Matrix<T, 2, 1> Vec2;
		Vec2 semiaxes(maxAbsX, maxAbsY), v(x, y), ret;
		bool coerced;
		ret = v;
		coerceEllC(semiaxes, ret);
		EXPECT_THAT_UTEQ(T, expX, ret.x());
		EXPECT_THAT_UTEQ(T, expY, ret.y());
		ret = v; coerced = !expcoerced;
		coerceEllC(semiaxes, ret, coerced);
		EXPECT_THAT_UTEQ(T, expX, ret.x());
		EXPECT_THAT_UTEQ(T, expY, ret.y());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
		ret << NAN, NAN;
		coerceEllC(semiaxes, v, ret);
		EXPECT_THAT_UTEQ(T, expX, ret.x());
		EXPECT_THAT_UTEQ(T, expY, ret.y());
		ret << NAN, NAN; coerced = !expcoerced;
		coerceEllC(semiaxes, v, ret, coerced);
		EXPECT_THAT_UTEQ(T, expX, ret.x());
		EXPECT_THAT_UTEQ(T, expY, ret.y());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceEllC3D(const Eigen::Matrix<T, 3, 1>& exp, bool expcoerced, const Eigen::Matrix<T, 3, 1>& v, const Eigen::Matrix<T, 3, 1>& semiaxes)
	{
		std::string args = argsAsString(typeid(T).name(), exp.transpose(), expcoerced, v.transpose(), semiaxes.transpose());
		typedef Eigen::Matrix<T, 3, 1> Vec3;
		Vec3 ret;
		bool coerced;
		ret = v;
		coerceEllC(semiaxes, ret);
		EXPECT_THAT_UTEQ(T, exp.x(), ret.x());
		EXPECT_THAT_UTEQ(T, exp.y(), ret.y());
		EXPECT_THAT_UTEQ(T, exp.z(), ret.z());
		ret = v; coerced = !expcoerced;
		coerceEllC(semiaxes, ret, coerced);
		EXPECT_THAT_UTEQ(T, exp.x(), ret.x());
		EXPECT_THAT_UTEQ(T, exp.y(), ret.y());
		EXPECT_THAT_UTEQ(T, exp.z(), ret.z());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
		ret << NAN, NAN, NAN;
		coerceEllC(semiaxes, v, ret);
		EXPECT_THAT_UTEQ(T, exp.x(), ret.x());
		EXPECT_THAT_UTEQ(T, exp.y(), ret.y());
		EXPECT_THAT_UTEQ(T, exp.z(), ret.z());
		ret << NAN, NAN, NAN; coerced = !expcoerced;
		coerceEllC(semiaxes, v, ret, coerced);
		EXPECT_THAT_UTEQ(T, exp.x(), ret.x());
		EXPECT_THAT_UTEQ(T, exp.y(), ret.y());
		EXPECT_THAT_UTEQ(T, exp.z(), ret.z());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
};

// Test: coerceEllC (N-dimensional)
TEST(MathVecMatTest, test_coerceEllCND)
{
	// Test class
	TestCoerceEllCND T;

	// Test coerceEllC (1D)
	T.testCoerceEllC1D(-8.0, true, -9.0, 8.0);
	T.testCoerceEllC1D(-8.0, false, -8.0, 8.0);
	T.testCoerceEllC1D(1.0, false, 1.0, 8.0);
	T.testCoerceEllC1D(8.0, false, 8.0, 8.0);
	T.testCoerceEllC1D(8.0, true, 9.0, 8.0);
	T.testCoerceEllC1D(-8.0, true, -9.0, -8.0);
	T.testCoerceEllC1D(0.0, false, 0.0, -8.0);

	// Test coerceEllC (2D)
	T.testCoerceEllC2D(0.7, 1.1, false, 0.7, 1.1, 2.0, 5.0);
	T.testCoerceEllC2D(-1.2803687993289596, 3.8411063979868789, true, -2.2, 6.6, 2.0, 5.0);
	T.testCoerceEllC2D(-1.2803687993289596, -3.8411063979868789, true, -3.3, -9.9, 2.0, 5.0);
	T.testCoerceEllC2D(2.0, 0.0, true, 7.0, 0.0, 2.0, 5.0);
	T.testCoerceEllC2D(0.0, 5.0, true, 0.0, 7.0, 2.0, 5.0);
	T.testCoerceEllC2D(-2.0, 0.0, true, -7.0, 0.0, 2.0, 5.0);
	T.testCoerceEllC2D(0.0, -5.0, true, 0.0, -7.0, 2.0, 5.0);
	T.testCoerceEllC2D(2.0, 0.0, true, 7.0, 0.0, 2.0, 0.0);
	T.testCoerceEllC2D(0.0, 0.0, true, 7.0, 0.01, 2.0, 0.0);
	T.testCoerceEllC2D(0.0, 0.0, true, 7.0, -0.01, 2.0, 0.0);
	T.testCoerceEllC2D(0.0, 5.0, true, 0.0, 7.0, 0.0, 5.0);
	T.testCoerceEllC2D(0.0, 0.0, true, 0.01, 7.0, 0.0, 5.0);
	T.testCoerceEllC2D(0.0, 0.0, true, -0.01, 7.0, 0.0, 5.0);
	T.testCoerceEllC2D(0.0, 0.0, true, 1.3, 5.4, 0.0, 0.0);

	// Test coerceEllC (3D)
	typedef Eigen::Vector3d V3;
	T.testCoerceEllC3D(V3(1.0, 2.0, 0.5), false, V3(1.0, 2.0, 0.5), V3(2.0, 5.0, 3.0));
	T.testCoerceEllC3D(V3(2.0, 0.0, 0.0), false, V3(2.0, 0.0, 0.0), V3(2.0, 5.0, 3.0));
	T.testCoerceEllC3D(V3(0.0, 5.0, 0.0), false, V3(0.0, 5.0, 0.0), V3(2.0, 5.0, 3.0));
	T.testCoerceEllC3D(V3(0.0, 0.0, 3.0), false, V3(0.0, 0.0, 3.0), V3(2.0, 5.0, 3.0));
	T.testCoerceEllC3D(V3(-1.2601285396669033, 1.6801713862225378, -2.1002142327781721), true, V3(-3.0, 4.0, -5.0), V3(2.0, -5.0, -3.0));
	T.testCoerceEllC3D(V3( 1.2601285396669033, 1.6801713862225378,  2.1002142327781721), true, V3(3.0, 4.0, 5.0), V3(2.0, 5.0, 3.0));
	T.testCoerceEllC3D(V3(0.0, 0.0, 0.0), true, V3(3.0, 4.0, 5.0), V3(2.0, 5.0, 0.0));
	T.testCoerceEllC3D(V3(1.7647058823529413, 2.3529411764705883, 0.0), true, V3(3.0, 4.0, 0.0), V3(2.0, 5.0, 0.0));
	T.testCoerceEllC3D(V3(0.0, 0.0, 0.0), true, V3(3.0, 4.0, 0.0), V3(0.0, 5.0, 0.0));
	T.testCoerceEllC3D(V3(0.0, 4.0, 0.0), false, V3(0.0, 4.0, 0.0), V3(0.0, 5.0, 0.0));
	T.testCoerceEllC3D(V3(0.0, 0.0, 0.0), true, V3(0.0, 4.0, 0.0), V3(0.0, 0.0, 0.0));
	T.testCoerceEllC3D(V3(0.0, 0.0, 0.0), false, V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0));

	// Test edge cases
	double inf = std::numeric_limits<double>::infinity();
	T.testCoerceEllC3D(V3(1000.0, 6.0, -712.0), false, V3(1000.0, 6.0, -712.0), V3(inf, inf, inf));
	T.testCoerceEllC3D(V3(1000.0, 6.0, -712.0), false, V3(1000.0, 6.0, -712.0), V3(-inf, -inf, inf));
	T.testCoerceEllC3D(V3(600.0, 3.6, -427.2), true, V3(1000.0, 6.0, -712.0), V3(600.0, inf, -inf));
	T.testCoerceEllC3D(V3(477.2399890175149153, -2.8634399341050893, -339.7948721804706338), true, V3(1000.0, -6.0, -712.0), V3(1600.0, 3.0, inf));
	T.testCoerceEllC3D(V3(360.8671414595940519, 2.1652028487575641, -256.9374047192309263), true, V3(1000.0, 6.0, -712.0), V3(600.0, 3.0, 750.0));
}

// Test class
struct TestCoerceSoftEllCND
{
	template<typename T> void testCoerceSoftEllC1D(T exp, bool expcoerced, T x, T maxAbs, T buffer)
	{
		std::string args = argsAsString(typeid(T).name(), exp, expcoerced, x, maxAbs, buffer);
		typedef Eigen::Matrix<T, 1, 1> Vec1;
		Vec1 semiaxes, v, ret;
		bool coerced;
		semiaxes << maxAbs;
		v << x;
		ret = v;
		coerceSoftEllC(semiaxes, ret, buffer);
		EXPECT_THAT_UTEQ(T, exp, ret.x());
		ret = v; coerced = !expcoerced;
		coerceSoftEllC(semiaxes, ret, buffer, coerced);
		EXPECT_THAT_UTEQ(T, exp, ret.x());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
		ret << NAN;
		coerceSoftEllC(semiaxes, v, buffer, ret);
		EXPECT_THAT_UTEQ(T, exp, ret.x());
		ret << NAN; coerced = !expcoerced;
		coerceSoftEllC(semiaxes, v, buffer, ret, coerced);
		EXPECT_THAT_UTEQ(T, exp, ret.x());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceSoftEllC2D(T expX, T expY, bool expcoerced, T x, T y, T maxAbsX, T maxAbsY, T buffer)
	{
		std::string args = argsAsString(typeid(T).name(), expX, expY, expcoerced, x, y, maxAbsX, maxAbsY, buffer);
		typedef Eigen::Matrix<T, 2, 1> Vec2;
		Vec2 semiaxes(maxAbsX, maxAbsY), v(x, y), ret;
		bool coerced;
		ret = v;
		coerceSoftEllC(semiaxes, ret, buffer);
		EXPECT_THAT_UTEQ(T, expX, ret.x());
		EXPECT_THAT_UTEQ(T, expY, ret.y());
		ret = v; coerced = !expcoerced;
		coerceSoftEllC(semiaxes, ret, buffer, coerced);
		EXPECT_THAT_UTEQ(T, expX, ret.x());
		EXPECT_THAT_UTEQ(T, expY, ret.y());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
		ret << NAN, NAN;
		coerceSoftEllC(semiaxes, v, buffer, ret);
		EXPECT_THAT_UTEQ(T, expX, ret.x());
		EXPECT_THAT_UTEQ(T, expY, ret.y());
		ret << NAN, NAN; coerced = !expcoerced;
		coerceSoftEllC(semiaxes, v, buffer, ret, coerced);
		EXPECT_THAT_UTEQ(T, expX, ret.x());
		EXPECT_THAT_UTEQ(T, expY, ret.y());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
	template<typename T> void testCoerceSoftEllC3D(const Eigen::Matrix<T, 3, 1>& exp, bool expcoerced, const Eigen::Matrix<T, 3, 1>& v, const Eigen::Matrix<T, 3, 1>& semiaxes, T buffer)
	{
		std::string args = argsAsString(typeid(T).name(), exp.transpose(), expcoerced, v.transpose(), semiaxes.transpose(), buffer);
		typedef Eigen::Matrix<T, 3, 1> Vec3;
		Vec3 ret;
		bool coerced;
		ret = v;
		coerceSoftEllC(semiaxes, ret, buffer);
		EXPECT_THAT_UTEQ(T, exp.x(), ret.x());
		EXPECT_THAT_UTEQ(T, exp.y(), ret.y());
		EXPECT_THAT_UTEQ(T, exp.z(), ret.z());
		ret = v; coerced = !expcoerced;
		coerceSoftEllC(semiaxes, ret, buffer, coerced);
		EXPECT_THAT_UTEQ(T, exp.x(), ret.x());
		EXPECT_THAT_UTEQ(T, exp.y(), ret.y());
		EXPECT_THAT_UTEQ(T, exp.z(), ret.z());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
		ret << NAN, NAN, NAN;
		coerceSoftEllC(semiaxes, v, buffer, ret);
		EXPECT_THAT_UTEQ(T, exp.x(), ret.x());
		EXPECT_THAT_UTEQ(T, exp.y(), ret.y());
		EXPECT_THAT_UTEQ(T, exp.z(), ret.z());
		ret << NAN, NAN, NAN; coerced = !expcoerced;
		coerceSoftEllC(semiaxes, v, buffer, ret, coerced);
		EXPECT_THAT_UTEQ(T, exp.x(), ret.x());
		EXPECT_THAT_UTEQ(T, exp.y(), ret.y());
		EXPECT_THAT_UTEQ(T, exp.z(), ret.z());
		EXPECT_THAT_EQ(bool, expcoerced, coerced);
	}
};

// Test: coerceSoftEllC (N-dimensional)
TEST(MathVecMatTest, test_coerceSoftEllCND)
{
	// Test class
	TestCoerceSoftEllCND T;

	// Test coerceSoftEllC (1D)
	T.testCoerceSoftEllC1D(-7.8646647167633876, true, -9.0, 8.0, 1.0);
	T.testCoerceSoftEllC1D(-7.0, false, -7.0, 8.0, 1.0);
	T.testCoerceSoftEllC1D(1.0, false, 1.0, 8.0, 1.0);
	T.testCoerceSoftEllC1D(7.0, false, 7.0, 8.0, 1.0);
	T.testCoerceSoftEllC1D(7.8646647167633876, true, 9.0, 8.0, 1.0);
	T.testCoerceSoftEllC1D(-7.8646647167633876, true, -9.0, -8.0, 1.0);
	T.testCoerceSoftEllC1D(0.0, false, 0.0, -8.0, 1.0);

	// Test coerceSoftEllC (2D)
	T.testCoerceSoftEllC2D(0.7, 1.1, false, 0.7, 1.1, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(-1.2740195694222953, 3.8220587082668849, true, -2.2, 6.6, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(-1.2801729034312874, -3.8405187102938623, true, -3.3, -9.9, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(1.9975212478233337, 0.0, true, 7.0, 0.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(0.0, 4.9502129316321364, true, 0.0, 7.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(-1.9975212478233337, 0.0, true, -7.0, 0.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(0.0, -4.9502129316321364, true, 0.0, -7.0, 2.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(1.9975212478233337, 0.0, true, 7.0, 0.0, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllC2D(0.0, 0.0, true, 7.0, 0.01, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllC2D(0.0, 0.0, true, 7.0, -0.01, 2.0, 0.0, 1.0);
	T.testCoerceSoftEllC2D(0.0, 4.9502129316321364, true, 0.0, 7.0, 0.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(0.0, 0.0, true, 0.01, 7.0, 0.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(0.0, 0.0, true, -0.01, 7.0, 0.0, 5.0, 1.0);
	T.testCoerceSoftEllC2D(0.0, 0.0, true, 1.3, 5.4, 0.0, 0.0, 1.0);

	// Test coerceSoftEllC (3D)
	typedef Eigen::Vector3d V3;
	T.testCoerceSoftEllC3D(V3(1.0, 2.0, 0.5), false, V3(1.0, 2.0, 0.5), V3(2.0, 5.0, 3.0), 1.0);
	T.testCoerceSoftEllC3D(V3(1.0, 0.0, 0.0), false, V3(1.0, 0.0, 0.0), V3(2.0, 5.0, 3.0), 1.0);
	T.testCoerceSoftEllC3D(V3(0.0, 4.0, 0.0), false, V3(0.0, 4.0, 0.0), V3(2.0, 5.0, 3.0), 1.0);
	T.testCoerceSoftEllC3D(V3(0.0, 0.0, 2.0), false, V3(0.0, 0.0, 2.0), V3(2.0, 5.0, 3.0), 1.0);
	T.testCoerceSoftEllC3D(V3(-1.2575442783621078, 1.6767257044828101, -2.0959071306035124), true, V3(-3.0, 4.0, -5.0), V3(2.0, -5.0, -3.0), 1.0);
	T.testCoerceSoftEllC3D(V3( 1.2575442783621078, 1.6767257044828101,  2.0959071306035124), true, V3(3.0, 4.0, 5.0), V3(2.0, 5.0, 3.0), 1.0);
	T.testCoerceSoftEllC3D(V3(0.0, 0.0, 0.0), true, V3(3.0, 4.0, 5.0), V3(2.0, 5.0, 0.0), 1.0);
	T.testCoerceSoftEllC3D(V3(1.7365401485477459, 2.3153868647303280, 0.0), true, V3(3.0, 4.0, 0.0), V3(2.0, 5.0, 0.0), 1.0);
	T.testCoerceSoftEllC3D(V3(0.0, 0.0, 0.0), true, V3(3.0, 4.0, 0.0), V3(0.0, 5.0, 0.0), 1.0);
	T.testCoerceSoftEllC3D(V3(0.0, 4.0, 0.0), false, V3(0.0, 4.0, 0.0), V3(0.0, 5.0, 0.0), 1.0);
	T.testCoerceSoftEllC3D(V3(0.0, 0.0, 0.0), true, V3(0.0, 4.0, 0.0), V3(0.0, 0.0, 0.0), 1.0);
	T.testCoerceSoftEllC3D(V3(0.0, 0.0, 0.0), false, V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), 1.0);

	// Test edge cases
	double inf = std::numeric_limits<double>::infinity();
	T.testCoerceSoftEllC3D(V3(1000.0, 6.0, -712.0), false, V3(1000.0, 6.0, -712.0), V3(inf, inf, inf), 0.0);
	T.testCoerceSoftEllC3D(V3(1000.0, 6.0, -712.0), false, V3(1000.0, 6.0, -712.0), V3(inf, -inf, inf), 100000.0);
	T.testCoerceSoftEllC3D(V3(600.0, 3.6, -427.2), true, V3(1000.0, 6.0, -712.0), V3(600.0, inf, -inf), 0.0);
	T.testCoerceSoftEllC3D(V3(477.2399890175149153, -2.8634399341050893, -339.7948721804706338), true, V3(1000.0, -6.0, -712.0), V3(1600.0, 3.0, inf), 0.0);
	T.testCoerceSoftEllC3D(V3(360.8671414595940519, 2.1652028487575641, -256.9374047192309263), true, V3(1000.0, 6.0, -712.0), V3(600.0, 3.0, 750.0), 0.0);
	T.testCoerceSoftEllC3D(V3(360.8671414595940519, 2.1652028487575641, -256.9374047192309263), true, V3(1000.0, 6.0, -712.0), V3(600.0, 3.0, 750.0), -inf);
	T.testCoerceSoftEllC3D(V3(230.6663176544479086, 1.3839979059266874, -164.2344181699668866), true, V3(1000.0, 6.0, -712.0), V3(600.0, 3.0, 750.0), inf);
}

//
// Main function
//
int main(int argc, char **argv)
{
	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF