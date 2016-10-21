// Unit testing of the math robotcontrol utilities headers
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/math_funcs.h>
#include <rc_utils/math_spline.h>
#include <rc_utils/math_vec_mat.h>
#include <gtest/gtest.h>

// Namespaces
using namespace rc_utils;

//
// Math functions tests
//

// Test: picut
TEST(MathFuncsTest, test_picut)
{
	// Check that we are getting the expected values
	EXPECT_DOUBLE_EQ(0.0, picut(-M_2PI));
	EXPECT_DOUBLE_EQ(M_PI_2, picut(-3.0*M_PI_2));
	EXPECT_DOUBLE_EQ(M_PI, picut(-M_PI));
	EXPECT_DOUBLE_EQ(-M_PI_2, picut(-M_PI_2));
	EXPECT_DOUBLE_EQ(0.0, picut(0.0));
	EXPECT_DOUBLE_EQ(M_PI_2, picut(M_PI_2));
	EXPECT_DOUBLE_EQ(M_PI, picut(M_PI));
	EXPECT_DOUBLE_EQ(-M_PI_2, picut(3.0*M_PI_2));
	EXPECT_DOUBLE_EQ(0.0, picut(M_2PI));
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

// Test: coerce, coerceAbs, coerceMax, coerceMin
TEST(MathFuncsTest, test_coerce)
{
	// Check that we are getting the expected values (we want exact comparisons to be possible)
	EXPECT_EQ( 2.0, coerce(1.0, 2.0, 8.0));
	EXPECT_EQ( 5.0, coerce(5.0, 2.0, 8.0));
	EXPECT_EQ( 8.0, coerce(9.0, 2.0, 8.0));
	EXPECT_EQ(-8.0, coerceAbs(-9.0, 8.0));
	EXPECT_EQ( 1.0, coerceAbs( 1.0, 8.0));
	EXPECT_EQ( 8.0, coerceAbs( 9.0, 8.0));
	EXPECT_EQ(-5.0, coerceMax( 0.0,-5.0));
	EXPECT_EQ(-8.0, coerceMax(-8.0,-5.0));
	EXPECT_EQ( 0.0, coerceMin( 0.0,-5.0));
	EXPECT_EQ(-5.0, coerceMin(-8.0,-5.0));
}

// Test: coerceSoft, coerceSoftAbs, coerceSoftMax, coerceSoftMin
TEST(MathFuncsTest, test_coerceSoft)
{
	// Check that we are getting the expected values for coerceSoft
	EXPECT_DOUBLE_EQ(2.0248935341839318, coerceSoft(1.0, 2.0, 8.0, 0.5));
	EXPECT_DOUBLE_EQ(7.9751064658160677, coerceSoft(9.0, 2.0, 8.0, 0.5));
	EXPECT_EQ(5.0, coerceSoft(5.0, 2.0, 8.0, 0.5));
	EXPECT_EQ(2.0, coerceSoft(1.0, 2.0, 8.0, 0.0));
	EXPECT_EQ(5.0, coerceSoft(5.0, 2.0, 8.0, 0.0));
	EXPECT_EQ(8.0, coerceSoft(9.0, 2.0, 8.0, 0.0));
	EXPECT_DOUBLE_EQ(coerceSoft(1.0, 2.0, 8.0, 3.0), coerceSoft(1.0, 2.0, 8.0, 10.0));
	EXPECT_DOUBLE_EQ(coerceSoft(5.0, 2.0, 8.0, 3.0), coerceSoft(5.0, 2.0, 8.0, 10.0));
	EXPECT_DOUBLE_EQ(coerceSoft(9.0, 2.0, 8.0, 3.0), coerceSoft(9.0, 2.0, 8.0, 10.0));
	
	// Check that we are getting the expected values for coerceSoftAbs
	EXPECT_DOUBLE_EQ(-7.3835075901459835, coerceSoftAbs(-9.0, 8.0, 2.5));
	EXPECT_DOUBLE_EQ( 7.3835075901459835, coerceSoftAbs( 9.0, 8.0, 2.5));
	EXPECT_EQ( 1.0, coerceSoftAbs( 1.0, 8.0, 2.5));
	EXPECT_EQ(-8.0, coerceSoftAbs(-9.0, 8.0, 0.0));
	EXPECT_EQ( 1.0, coerceSoftAbs( 1.0, 8.0, 0.0));
	EXPECT_EQ( 8.0, coerceSoftAbs( 9.0, 8.0, 0.0));
	EXPECT_DOUBLE_EQ(coerceSoftAbs(1.0, 8.0, 8.0), coerceSoftAbs(1.0, 8.0, 10.0));
	EXPECT_DOUBLE_EQ(coerceSoftAbs(5.0, 8.0, 8.0), coerceSoftAbs(5.0, 8.0, 10.0));
	EXPECT_DOUBLE_EQ(coerceSoftAbs(9.0, 8.0, 8.0), coerceSoftAbs(9.0, 8.0, 10.0));
	
	// Check that we are getting the expected values for coerceSoftMax
	EXPECT_DOUBLE_EQ(-5.5518191617571633, coerceSoftMax(-5.0, -5.0, 1.5));
	EXPECT_EQ(-6.5, coerceSoftMax(-6.5, -5.0, 1.5));
	EXPECT_EQ(-5.0, coerceSoftMax(-4.5, -5.0, 0.0));
	EXPECT_EQ(-6.5, coerceSoftMax(-6.5, -5.0, 0.0));
	EXPECT_EQ(-5.0, coerceSoftMax(-4.5, -5.0, -1.0));
	EXPECT_EQ(-6.5, coerceSoftMax(-6.5, -5.0, -1.0));
	
	// Check that we are getting the expected values for coerceSoftMin
	EXPECT_DOUBLE_EQ(-4.4481808382428367, coerceSoftMin(-5.0, -5.0, 1.5));
	EXPECT_EQ(-3.5, coerceSoftMin(-3.5, -5.0, 1.5));
	EXPECT_EQ(-5.0, coerceSoftMin(-5.5, -5.0, 0.0));
	EXPECT_EQ(-3.5, coerceSoftMin(-3.5, -5.0, 0.0));
	EXPECT_EQ(-5.0, coerceSoftMin(-5.5, -5.0, -1.0));
	EXPECT_EQ(-3.5, coerceSoftMin(-3.5, -5.0, -1.0));
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

// Test: eigenNormalized, eigenNormalize
TEST(MathVecMatTest, test_eigenNormalization)
{
	//
	// Float
	//

	// Test cases
	Eigen::Vector2f vec2f(0.3, 0.4);
	Eigen::Vector3f vec3f(1.2, 0.4, 0.3);
	Eigen::Vector2f vec2fZero(0.0, 0.0);
	Eigen::Vector3f vec3fZero(0.0, 0.0, 0.0);

	// Check that we are getting the expected values for eigenNormalized
	Eigen::Vector2f vec2fUnit = eigenNormalized(vec2f);
	Eigen::Vector3f vec3fUnit = eigenNormalized(vec3f);
	Eigen::Vector2f vec2fZeroUnit = eigenNormalized(vec2fZero);
	Eigen::Vector3f vec3fZeroUnit = eigenNormalized(vec3fZero);
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

	// Check that we are getting the expected values for eigenNormalize
	eigenNormalize(vec2f);
	eigenNormalize(vec3f);
	eigenNormalize(vec2fZero);
	eigenNormalize(vec3fZero);
	EXPECT_FLOAT_EQ(0.6, vec2f.x());
	EXPECT_FLOAT_EQ(0.8, vec2f.y());
	EXPECT_FLOAT_EQ(1.2/1.3, vec3f.x());
	EXPECT_FLOAT_EQ(0.4/1.3, vec3f.y());
	EXPECT_FLOAT_EQ(0.3/1.3, vec3f.z());
	EXPECT_FLOAT_EQ(1.0, vec2fZero.x());
	EXPECT_FLOAT_EQ(0.0, vec2fZero.y());
	EXPECT_FLOAT_EQ(1.0, vec3fZero.x());
	EXPECT_FLOAT_EQ(0.0, vec3fZero.y());
	EXPECT_FLOAT_EQ(0.0, vec3fZero.z());

	//
	// Double
	//

	// Test cases
	Eigen::Vector2d vec2d(0.3, 0.4);
	Eigen::Vector3d vec3d(1.2, 0.4, 0.3);
	Eigen::Vector2d vec2dZero(0.0, 0.0);
	Eigen::Vector3d vec3dZero(0.0, 0.0, 0.0);

	// Check that we are getting the expected values for eigenNormalized
	Eigen::Vector2d vec2dUnit = eigenNormalized(vec2d);
	Eigen::Vector3d vec3dUnit = eigenNormalized(vec3d);
	Eigen::Vector2d vec2dZeroUnit = eigenNormalized(vec2dZero);
	Eigen::Vector3d vec3dZeroUnit = eigenNormalized(vec3dZero);
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

	// Check that we are getting the expected values for eigenNormalize
	eigenNormalize(vec2d);
	eigenNormalize(vec3d);
	eigenNormalize(vec2dZero);
	eigenNormalize(vec3dZero);
	EXPECT_DOUBLE_EQ(0.6, vec2d.x());
	EXPECT_DOUBLE_EQ(0.8, vec2d.y());
	EXPECT_DOUBLE_EQ(1.2/1.3, vec3d.x());
	EXPECT_DOUBLE_EQ(0.4/1.3, vec3d.y());
	EXPECT_DOUBLE_EQ(0.3/1.3, vec3d.z());
	EXPECT_DOUBLE_EQ(1.0, vec2dZero.x());
	EXPECT_DOUBLE_EQ(0.0, vec2dZero.y());
	EXPECT_DOUBLE_EQ(1.0, vec3dZero.x());
	EXPECT_DOUBLE_EQ(0.0, vec3dZero.y());
	EXPECT_DOUBLE_EQ(0.0, vec3dZero.z());
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