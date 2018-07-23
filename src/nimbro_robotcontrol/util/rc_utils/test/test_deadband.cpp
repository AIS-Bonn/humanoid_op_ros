// Unit test of deadband classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/smooth_deadband.h>
#include <rc_utils/smooth_deadband_ell.h>
#include <test_utilities/test_eigen.h>
#include <test_utilities/test_misc.h>
#include <gtest/gtest.h>

// Namespaces
using namespace rc_utils;
using namespace testutilities;

// Typedefs
typedef Eigen::Matrix<double, 2, 1> V2;
typedef Eigen::Matrix<double, 3, 1> V3;

// Test the SharpDeadband class
TEST(DeadbandTest, testSharpDeadband)
{
	// Test construction, set and reset
	SharpDeadband DB;
	EXPECT_EQ( 0.0, DB.centre());
	EXPECT_EQ( 0.0, DB.radius());
	EXPECT_EQ(-1.0, DB.eval(-1.0));
	EXPECT_EQ( 0.0, DB.eval( 0.0));
	EXPECT_EQ( 1.0, DB.eval( 1.0));
	DB.set(0.3, 0.7);
	EXPECT_EQ( 0.7, DB.centre());
	EXPECT_EQ( 0.3, DB.radius());
	EXPECT_NE(-1.0, DB.eval(-1.0));
	EXPECT_NE( 0.0, DB.eval( 0.0));
	EXPECT_NE( 1.0, DB.eval( 1.0));
	DB.reset();
	EXPECT_EQ( 0.0, DB.centre());
	EXPECT_EQ( 0.0, DB.radius());
	EXPECT_EQ(-1.0, DB.eval(-1.0));
	EXPECT_EQ( 0.0, DB.eval( 0.0));
	EXPECT_EQ( 1.0, DB.eval( 1.0));
	SharpDeadband DBtmp(0.2, 0.4);
	EXPECT_EQ( 0.4, DBtmp.centre());
	EXPECT_EQ( 0.2, DBtmp.radius());
	
	// Test a normal case
	DB.set(0.4, 0.6);
	EXPECT_DOUBLE_EQ(-0.7, DB.eval(-0.5));
	EXPECT_DOUBLE_EQ(-0.2, DB.eval( 0.0));
	EXPECT_DOUBLE_EQ( 0.0, DB.eval( 0.5));
	EXPECT_DOUBLE_EQ( 0.0, DB.eval( 1.0));
	EXPECT_DOUBLE_EQ( 0.5, DB.eval( 1.5));
	EXPECT_DOUBLE_EQ(-0.1, SharpDeadband::eval(-0.5, 0.4));
	EXPECT_DOUBLE_EQ( 0.0, SharpDeadband::eval( 0.0, 0.4));
	EXPECT_DOUBLE_EQ( 0.1, SharpDeadband::eval( 0.5, 0.4));
	EXPECT_DOUBLE_EQ( 0.6, SharpDeadband::eval( 1.0, 0.4));
	EXPECT_DOUBLE_EQ( 1.1, SharpDeadband::eval( 1.5, 0.4));
	EXPECT_DOUBLE_EQ(-0.7, SharpDeadband::eval(-0.5, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-0.2, SharpDeadband::eval( 0.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.0, SharpDeadband::eval( 0.5, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.0, SharpDeadband::eval( 1.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.5, SharpDeadband::eval( 1.5, 0.4, 0.6));
	
	// Test edge cases
	EXPECT_DOUBLE_EQ(-0.2, SharpDeadband::eval(0.4,  0.0, 0.6));
	EXPECT_DOUBLE_EQ(-0.1, SharpDeadband::eval(0.5,  0.0, 0.6));
	EXPECT_DOUBLE_EQ( 0.0, SharpDeadband::eval(0.6,  0.0, 0.6));
	EXPECT_DOUBLE_EQ( 0.1, SharpDeadband::eval(0.7,  0.0, 0.6));
	EXPECT_DOUBLE_EQ( 0.2, SharpDeadband::eval(0.8,  0.0, 0.6));
	EXPECT_DOUBLE_EQ(-0.3, SharpDeadband::eval(0.4, -0.1, 0.6));
	EXPECT_DOUBLE_EQ(-0.2, SharpDeadband::eval(0.5, -0.1, 0.6));
	EXPECT_DOUBLE_EQ( 0.0, SharpDeadband::eval(0.6, -0.1, 0.6));
	EXPECT_DOUBLE_EQ( 0.2, SharpDeadband::eval(0.7, -0.1, 0.6));
	EXPECT_DOUBLE_EQ( 0.3, SharpDeadband::eval(0.8, -0.1, 0.6));
}

// Test the SmoothDeadband class
TEST(DeadbandTest, testSmoothDeadband)
{
	// Test construction, set and reset
	SmoothDeadband DB;
	EXPECT_EQ( 0.0, DB.centre());
	EXPECT_EQ( 0.0, DB.radius());
	EXPECT_EQ(-1.0, DB.eval(-1.0));
	EXPECT_EQ( 0.0, DB.eval( 0.0));
	EXPECT_EQ( 1.0, DB.eval( 1.0));
	DB.set(0.3, 0.7);
	EXPECT_EQ( 0.7, DB.centre());
	EXPECT_EQ( 0.3, DB.radius());
	EXPECT_NE(-1.0, DB.eval(-1.0));
	EXPECT_NE( 0.0, DB.eval( 0.0));
	EXPECT_NE( 1.0, DB.eval( 1.0));
	DB.reset();
	EXPECT_EQ( 0.0, DB.centre());
	EXPECT_EQ( 0.0, DB.radius());
	EXPECT_EQ(-1.0, DB.eval(-1.0));
	EXPECT_EQ( 0.0, DB.eval( 0.0));
	EXPECT_EQ( 1.0, DB.eval( 1.0));
	SmoothDeadband DBtmp(0.2, 0.4);
	EXPECT_EQ( 0.4, DBtmp.centre());
	EXPECT_EQ( 0.2, DBtmp.radius());
	
	// Test a normal case
	DB.set(0.4, 0.6);
	EXPECT_DOUBLE_EQ(-0.70000, DB.eval(-0.5));
	EXPECT_DOUBLE_EQ(-0.22500, DB.eval( 0.0));
	EXPECT_DOUBLE_EQ(-0.00625, DB.eval( 0.5));
	EXPECT_DOUBLE_EQ( 0.10000, DB.eval( 1.0));
	EXPECT_DOUBLE_EQ( 0.50000, DB.eval( 1.5));
	EXPECT_DOUBLE_EQ(-0.15625, SmoothDeadband::eval(-0.5, 0.4));
	EXPECT_DOUBLE_EQ( 0.00000, SmoothDeadband::eval( 0.0, 0.4));
	EXPECT_DOUBLE_EQ( 0.15625, SmoothDeadband::eval( 0.5, 0.4));
	EXPECT_DOUBLE_EQ( 0.60000, SmoothDeadband::eval( 1.0, 0.4));
	EXPECT_DOUBLE_EQ( 1.10000, SmoothDeadband::eval( 1.5, 0.4));
	EXPECT_DOUBLE_EQ(-0.70000, SmoothDeadband::eval(-0.5, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-0.22500, SmoothDeadband::eval( 0.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-0.00625, SmoothDeadband::eval( 0.5, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.10000, SmoothDeadband::eval( 1.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.50000, SmoothDeadband::eval( 1.5, 0.4, 0.6));
	
	// Test edge cases
	EXPECT_DOUBLE_EQ(-0.2, SmoothDeadband::eval(0.4,  0.0, 0.6));
	EXPECT_DOUBLE_EQ(-0.1, SmoothDeadband::eval(0.5,  0.0, 0.6));
	EXPECT_DOUBLE_EQ( 0.0, SmoothDeadband::eval(0.6,  0.0, 0.6));
	EXPECT_DOUBLE_EQ( 0.1, SmoothDeadband::eval(0.7,  0.0, 0.6));
	EXPECT_DOUBLE_EQ( 0.2, SmoothDeadband::eval(0.8,  0.0, 0.6));
	EXPECT_DOUBLE_EQ(-0.3, SmoothDeadband::eval(0.4, -0.1, 0.6));
	EXPECT_DOUBLE_EQ(-0.2, SmoothDeadband::eval(0.5, -0.1, 0.6));
	EXPECT_DOUBLE_EQ( 0.0, SmoothDeadband::eval(0.6, -0.1, 0.6));
	EXPECT_DOUBLE_EQ( 0.2, SmoothDeadband::eval(0.7, -0.1, 0.6));
	EXPECT_DOUBLE_EQ( 0.3, SmoothDeadband::eval(0.8, -0.1, 0.6));
}

// Test the DeadSmoothDeadband class
TEST(DeadbandTest, testDeadSmoothDeadband)
{
	// Test construction, set and reset
	DeadSmoothDeadband DB;
	EXPECT_EQ( 0.0, DB.centre());
	EXPECT_EQ( 0.0, DB.zeroRadius());
	EXPECT_EQ( 0.0, DB.deadRadius());
	EXPECT_EQ(-1.0, DB.eval(-1.0));
	EXPECT_EQ( 0.0, DB.eval( 0.0));
	EXPECT_EQ( 1.0, DB.eval( 1.0));
	DB.set(0.3, 0.7, 0.5);
	EXPECT_EQ( 0.5, DB.centre());
	EXPECT_EQ( 0.3, DB.zeroRadius());
	EXPECT_EQ( 0.7, DB.deadRadius());
	EXPECT_NE(-1.0, DB.eval(-1.0));
	EXPECT_NE( 0.0, DB.eval( 0.0));
	EXPECT_NE( 1.0, DB.eval( 1.0));
	DB.reset();
	EXPECT_EQ( 0.0, DB.centre());
	EXPECT_EQ( 0.0, DB.zeroRadius());
	EXPECT_EQ( 0.0, DB.deadRadius());
	EXPECT_EQ(-1.0, DB.eval(-1.0));
	EXPECT_EQ( 0.0, DB.eval( 0.0));
	EXPECT_EQ( 1.0, DB.eval( 1.0));
	DeadSmoothDeadband DBtmp(0.2, 0.4);
	EXPECT_EQ( 0.0, DBtmp.centre());
	EXPECT_EQ( 0.2, DBtmp.zeroRadius());
	EXPECT_EQ( 0.4, DBtmp.deadRadius());
	
	// Test a normal case
	DB.set(0.4, 0.6, 3.0);
	EXPECT_DOUBLE_EQ(-1.00, DB.eval(1.0));
	EXPECT_DOUBLE_EQ(-0.60, DB.eval(1.4));
	EXPECT_DOUBLE_EQ(-0.15, DB.eval(2.0));
	EXPECT_DOUBLE_EQ( 0.00, DB.eval(2.6));
	EXPECT_DOUBLE_EQ( 0.00, DB.eval(3.0));
	EXPECT_DOUBLE_EQ( 0.00, DB.eval(3.4));
	EXPECT_DOUBLE_EQ( 0.15, DB.eval(4.0));
	EXPECT_DOUBLE_EQ( 0.60, DB.eval(4.6));
	EXPECT_DOUBLE_EQ( 1.20, DB.eval(5.2));
	EXPECT_DOUBLE_EQ(-1.00, DeadSmoothDeadband::eval(-2.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-0.60, DeadSmoothDeadband::eval(-1.6, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-0.15, DeadSmoothDeadband::eval(-1.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.00, DeadSmoothDeadband::eval(-0.4, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.00, DeadSmoothDeadband::eval( 0.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.00, DeadSmoothDeadband::eval( 0.4, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.15, DeadSmoothDeadband::eval( 1.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.60, DeadSmoothDeadband::eval( 1.6, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 1.20, DeadSmoothDeadband::eval( 2.2, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-1.00, DeadSmoothDeadband::eval(1.0, 0.4, 0.6, 3.0));
	EXPECT_DOUBLE_EQ(-0.60, DeadSmoothDeadband::eval(1.4, 0.4, 0.6, 3.0));
	EXPECT_DOUBLE_EQ(-0.15, DeadSmoothDeadband::eval(2.0, 0.4, 0.6, 3.0));
	EXPECT_DOUBLE_EQ( 0.00, DeadSmoothDeadband::eval(2.6, 0.4, 0.6, 3.0));
	EXPECT_DOUBLE_EQ( 0.00, DeadSmoothDeadband::eval(3.0, 0.4, 0.6, 3.0));
	EXPECT_DOUBLE_EQ( 0.00, DeadSmoothDeadband::eval(3.4, 0.4, 0.6, 3.0));
	EXPECT_DOUBLE_EQ( 0.15, DeadSmoothDeadband::eval(4.0, 0.4, 0.6, 3.0));
	EXPECT_DOUBLE_EQ( 0.60, DeadSmoothDeadband::eval(4.6, 0.4, 0.6, 3.0));
	EXPECT_DOUBLE_EQ( 1.20, DeadSmoothDeadband::eval(5.2, 0.4, 0.6, 3.0));
	
	// Test edge cases
	EXPECT_DOUBLE_EQ(-0.2, DeadSmoothDeadband::eval(2.4, 0.4, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(2.6, 0.4, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(2.8, 0.4, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.0, 0.4, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.2, 0.4, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.4, 0.4, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.2, DeadSmoothDeadband::eval(3.6, 0.4, 0.0, 3.0));
	EXPECT_DOUBLE_EQ(-0.3, DeadSmoothDeadband::eval(2.4, 0.4, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(2.6, 0.4, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(2.8, 0.4, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.0, 0.4, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.2, 0.4, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.4, 0.4, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.3, DeadSmoothDeadband::eval(3.6, 0.4, -0.1, 3.0));
	EXPECT_DOUBLE_EQ(-0.40, DeadSmoothDeadband::eval(2.4, 0.0, 0.2, 3.0));
	EXPECT_DOUBLE_EQ(-0.20, DeadSmoothDeadband::eval(2.6, 0.0, 0.2, 3.0));
	EXPECT_NEAR     (-0.05, DeadSmoothDeadband::eval(2.8, 0.0, 0.2, 3.0), 1e-15);
	EXPECT_DOUBLE_EQ( 0.00, DeadSmoothDeadband::eval(3.0, 0.0, 0.2, 3.0));
	EXPECT_NEAR     ( 0.05, DeadSmoothDeadband::eval(3.2, 0.0, 0.2, 3.0), 1e-15);
	EXPECT_DOUBLE_EQ( 0.20, DeadSmoothDeadband::eval(3.4, 0.0, 0.2, 3.0));
	EXPECT_DOUBLE_EQ( 0.40, DeadSmoothDeadband::eval(3.6, 0.0, 0.2, 3.0));
	EXPECT_DOUBLE_EQ(-0.6, DeadSmoothDeadband::eval(2.4, 0.0, 0.0, 3.0));
	EXPECT_DOUBLE_EQ(-0.4, DeadSmoothDeadband::eval(2.6, 0.0, 0.0, 3.0));
	EXPECT_NEAR     (-0.2, DeadSmoothDeadband::eval(2.8, 0.0, 0.0, 3.0), 1e-15);
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.0, 0.0, 0.0, 3.0));
	EXPECT_NEAR     ( 0.2, DeadSmoothDeadband::eval(3.2, 0.0, 0.0, 3.0), 1e-15);
	EXPECT_DOUBLE_EQ( 0.4, DeadSmoothDeadband::eval(3.4, 0.0, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.6, DeadSmoothDeadband::eval(3.6, 0.0, 0.0, 3.0));
	EXPECT_DOUBLE_EQ(-0.7, DeadSmoothDeadband::eval(2.4, 0.0, -0.1, 3.0));
	EXPECT_DOUBLE_EQ(-0.5, DeadSmoothDeadband::eval(2.6, 0.0, -0.1, 3.0));
	EXPECT_DOUBLE_EQ(-0.3, DeadSmoothDeadband::eval(2.8, 0.0, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.0, 0.0, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.3, DeadSmoothDeadband::eval(3.2, 0.0, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.5, DeadSmoothDeadband::eval(3.4, 0.0, -0.1, 3.0));
	EXPECT_DOUBLE_EQ( 0.7, DeadSmoothDeadband::eval(3.6, 0.0, -0.1, 3.0));
	EXPECT_DOUBLE_EQ(-0.5000, DeadSmoothDeadband::eval(2.4, -0.1, 0.2, 3.0));
	EXPECT_DOUBLE_EQ(-0.3000, DeadSmoothDeadband::eval(2.6, -0.1, 0.2, 3.0));
	EXPECT_NEAR     (-0.1125, DeadSmoothDeadband::eval(2.8, -0.1, 0.2, 3.0), 1e-15);
	EXPECT_DOUBLE_EQ( 0.0000, DeadSmoothDeadband::eval(3.0, -0.1, 0.2, 3.0));
	EXPECT_NEAR     ( 0.1125, DeadSmoothDeadband::eval(3.2, -0.1, 0.2, 3.0), 1e-15);
	EXPECT_DOUBLE_EQ( 0.3000, DeadSmoothDeadband::eval(3.4, -0.1, 0.2, 3.0));
	EXPECT_DOUBLE_EQ( 0.5000, DeadSmoothDeadband::eval(3.6, -0.1, 0.2, 3.0));
	EXPECT_DOUBLE_EQ(-0.7, DeadSmoothDeadband::eval(2.4, -0.1, 0.0, 3.0));
	EXPECT_DOUBLE_EQ(-0.5, DeadSmoothDeadband::eval(2.6, -0.1, 0.0, 3.0));
	EXPECT_DOUBLE_EQ(-0.3, DeadSmoothDeadband::eval(2.8, -0.1, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.0, -0.1, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.3, DeadSmoothDeadband::eval(3.2, -0.1, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.5, DeadSmoothDeadband::eval(3.4, -0.1, 0.0, 3.0));
	EXPECT_DOUBLE_EQ( 0.7, DeadSmoothDeadband::eval(3.6, -0.1, 0.0, 3.0));
	EXPECT_DOUBLE_EQ(-0.9, DeadSmoothDeadband::eval(2.4, -0.1, -0.2, 3.0));
	EXPECT_DOUBLE_EQ(-0.7, DeadSmoothDeadband::eval(2.6, -0.1, -0.2, 3.0));
	EXPECT_DOUBLE_EQ(-0.5, DeadSmoothDeadband::eval(2.8, -0.1, -0.2, 3.0));
	EXPECT_DOUBLE_EQ( 0.0, DeadSmoothDeadband::eval(3.0, -0.1, -0.2, 3.0));
	EXPECT_DOUBLE_EQ( 0.5, DeadSmoothDeadband::eval(3.2, -0.1, -0.2, 3.0));
	EXPECT_DOUBLE_EQ( 0.7, DeadSmoothDeadband::eval(3.4, -0.1, -0.2, 3.0));
	EXPECT_DOUBLE_EQ( 0.9, DeadSmoothDeadband::eval(3.6, -0.1, -0.2, 3.0));
}

// Test class
struct TestEllDeadband
{
	template<int N> void testSharpEllDeadband(const Eigen::Matrix<double, N, 1>& exp, const Eigen::Matrix<double, N, 1>& expC, const Eigen::Matrix<double, N, 1>& x, const Eigen::Matrix<double, N, 1>& semiaxes, const Eigen::Matrix<double, N, 1>& centre)
	{
		typedef Eigen::Matrix<double, N, 1> Vec;
		std::string args = argsAsString(exp, expC, x, semiaxes, centre);
		SharpEllDeadband<N> S(semiaxes), SC(semiaxes, centre), SS, SSC;
		SS.set(semiaxes);
		SSC.set(semiaxes, centre);
		EXPECT_THAT_UTEQ(Vec, semiaxes, S.semiaxes());
		EXPECT_THAT_UTEQ(Vec, semiaxes, SC.semiaxes());
		EXPECT_THAT_UTEQ(Vec, semiaxes, SS.semiaxes());
		EXPECT_THAT_UTEQ(Vec, semiaxes, SSC.semiaxes());
		EXPECT_THAT_UTEQ(Vec, Vec::Zero(), S.centre());
		EXPECT_THAT_UTEQ(Vec, centre, SC.centre());
		EXPECT_THAT_UTEQ(Vec, Vec::Zero(), SS.centre());
		EXPECT_THAT_UTEQ(Vec, centre, SSC.centre());
		EXPECT_THAT_UTEQ(Vec, exp, S.eval(x));
		EXPECT_THAT_UTEQ(Vec, exp, SS.eval(x));
		EXPECT_THAT_UTEQ(Vec, exp, SharpEllDeadband<N>::eval(x, semiaxes));
		EXPECT_THAT_UTEQ(Vec, expC, SC.eval(x));
		EXPECT_THAT_UTEQ(Vec, expC, SSC.eval(x));
		EXPECT_THAT_UTEQ(Vec, expC, SharpEllDeadband<N>::eval(x, semiaxes, centre));
		S.reset();
		SC.reset();
		SS.reset();
		SSC.reset();
		EXPECT_THAT_UTEQ(Vec, x, S.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SC.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SS.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SSC.eval(x));
	}
	template<int N> void testSmoothEllDeadband(const Eigen::Matrix<double, N, 1>& exp, const Eigen::Matrix<double, N, 1>& expC, const Eigen::Matrix<double, N, 1>& x, const Eigen::Matrix<double, N, 1>& semiaxes, const Eigen::Matrix<double, N, 1>& centre)
	{
		typedef Eigen::Matrix<double, N, 1> Vec;
		std::string args = argsAsString(exp, expC, x, semiaxes, centre);
		SmoothEllDeadband<N> S(semiaxes), SC(semiaxes, centre), SS, SSC;
		SS.set(semiaxes);
		SSC.set(semiaxes, centre);
		EXPECT_THAT_UTEQ(Vec, semiaxes, S.semiaxes());
		EXPECT_THAT_UTEQ(Vec, semiaxes, SC.semiaxes());
		EXPECT_THAT_UTEQ(Vec, semiaxes, SS.semiaxes());
		EXPECT_THAT_UTEQ(Vec, semiaxes, SSC.semiaxes());
		EXPECT_THAT_UTEQ(Vec, Vec::Zero(), S.centre());
		EXPECT_THAT_UTEQ(Vec, centre, SC.centre());
		EXPECT_THAT_UTEQ(Vec, Vec::Zero(), SS.centre());
		EXPECT_THAT_UTEQ(Vec, centre, SSC.centre());
		EXPECT_THAT_UTEQ(Vec, exp, S.eval(x));
		EXPECT_THAT_UTEQ(Vec, exp, SS.eval(x));
		EXPECT_THAT_UTEQ(Vec, exp, SmoothEllDeadband<N>::eval(x, semiaxes));
		EXPECT_THAT_UTEQ(Vec, expC, SC.eval(x));
		EXPECT_THAT_UTEQ(Vec, expC, SSC.eval(x));
		EXPECT_THAT_UTEQ(Vec, expC, SmoothEllDeadband<N>::eval(x, semiaxes, centre));
		S.reset();
		SC.reset();
		SS.reset();
		SSC.reset();
		EXPECT_THAT_UTEQ(Vec, x, S.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SC.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SS.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SSC.eval(x));
	}
	template<int N> void testDeadSmoothEllDeadband(const Eigen::Matrix<double, N, 1>& exp, const Eigen::Matrix<double, N, 1>& expC, const Eigen::Matrix<double, N, 1>& x, const Eigen::Matrix<double, N, 1>& semiaxesZ, const Eigen::Matrix<double, N, 1>& semiaxesD, const Eigen::Matrix<double, N, 1>& centre)
	{
		typedef Eigen::Matrix<double, N, 1> Vec;
		std::string args = argsAsString(exp, expC, x, semiaxesZ, semiaxesD, centre);
		DeadSmoothEllDeadband<N> S(semiaxesZ, semiaxesD), SC(semiaxesZ, semiaxesD, centre), SS, SSC;
		SS.set(semiaxesZ, semiaxesD);
		SSC.set(semiaxesZ, semiaxesD, centre);
		EXPECT_THAT_UTEQ(Vec, semiaxesZ, S.semiaxesZ());
		EXPECT_THAT_UTEQ(Vec, semiaxesZ, SC.semiaxesZ());
		EXPECT_THAT_UTEQ(Vec, semiaxesZ, SS.semiaxesZ());
		EXPECT_THAT_UTEQ(Vec, semiaxesZ, SSC.semiaxesZ());
		EXPECT_THAT_UTEQ(Vec, semiaxesD, S.semiaxesD());
		EXPECT_THAT_UTEQ(Vec, semiaxesD, SC.semiaxesD());
		EXPECT_THAT_UTEQ(Vec, semiaxesD, SS.semiaxesD());
		EXPECT_THAT_UTEQ(Vec, semiaxesD, SSC.semiaxesD());
		EXPECT_THAT_UTEQ(Vec, Vec::Zero(), S.centre());
		EXPECT_THAT_UTEQ(Vec, centre, SC.centre());
		EXPECT_THAT_UTEQ(Vec, Vec::Zero(), SS.centre());
		EXPECT_THAT_UTEQ(Vec, centre, SSC.centre());
		EXPECT_THAT_UTEQ(Vec, exp, S.eval(x));
		EXPECT_THAT_UTEQ(Vec, exp, SS.eval(x));
		EXPECT_THAT_UTEQ(Vec, exp, DeadSmoothEllDeadband<N>::eval(x, semiaxesZ, semiaxesD));
		EXPECT_THAT_UTEQ(Vec, expC, SC.eval(x));
		EXPECT_THAT_UTEQ(Vec, expC, SSC.eval(x));
		EXPECT_THAT_UTEQ(Vec, expC, DeadSmoothEllDeadband<N>::eval(x, semiaxesZ, semiaxesD, centre));
		S.reset();
		SC.reset();
		SS.reset();
		SSC.reset();
		EXPECT_THAT_UTEQ(Vec, x, S.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SC.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SS.eval(x));
		EXPECT_THAT_UTEQ(Vec, x, SSC.eval(x));
	}
};

// Test the SharpEllDeadband class
TEST(DeadbandTest, testSharpEllDeadband)
{
	// Test class
	TestEllDeadband T;
	
	// Test 1D
	T.testSharpEllDeadband(Vector1( 0.3), Vector1(-0.9), Vector1( 0.7), Vector1(0.4), Vector1(2.0));
	T.testSharpEllDeadband(Vector1(-0.1), Vector1(-0.7), Vector1(-0.5), Vector1(0.4), Vector1(0.6));
	T.testSharpEllDeadband(Vector1( 0.0), Vector1(-0.2), Vector1( 0.0), Vector1(0.4), Vector1(0.6));
	T.testSharpEllDeadband(Vector1( 0.1), Vector1( 0.0), Vector1( 0.5), Vector1(0.4), Vector1(0.6));
	T.testSharpEllDeadband(Vector1( 0.6), Vector1( 0.0), Vector1( 1.0), Vector1(0.4), Vector1(0.6));
	T.testSharpEllDeadband(Vector1( 1.1), Vector1( 0.5), Vector1( 1.5), Vector1(0.4), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.4), Vector1(-0.2), Vector1(0.4), Vector1(0.0), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.5), Vector1(-0.1), Vector1(0.5), Vector1(0.0), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.6), Vector1( 0.0), Vector1(0.6), Vector1(0.0), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.7), Vector1( 0.1), Vector1(0.7), Vector1(0.0), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.8), Vector1( 0.2), Vector1(0.8), Vector1(0.0), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.3), Vector1(-0.1), Vector1(0.4), Vector1(-0.1), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.4), Vector1( 0.0), Vector1(0.5), Vector1(-0.1), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.5), Vector1( 0.0), Vector1(0.6), Vector1(-0.1), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.6), Vector1( 0.0), Vector1(0.7), Vector1(-0.1), Vector1(0.6));
	T.testSharpEllDeadband(Vector1(0.7), Vector1( 0.1), Vector1(0.8), Vector1(-0.1), Vector1(0.6));
	
	// Test 2D
	T.testSharpEllDeadband(V2(0.0, 0.0), V2(0.0, 0.0), V2(0.0, 0.0), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSharpEllDeadband(V2(0.0, 0.0), V2(-0.0258297735348777, 0.1033190941395108), V2(0.0, 0.2), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSharpEllDeadband(V2(0.0, 0.1), V2(-0.0502481404895006, 0.3014888429370034), V2(0.0, 0.4), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSharpEllDeadband(V2(0.0, 0.0), V2(0.0, 0.0), V2(0.3, 0.0), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSharpEllDeadband(V2(0.1, 0.0), V2(0.0839748528310781, 0.0335899411324313), V2(0.6, 0.0), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSharpEllDeadband(V2(0.0, 0.0), V2(-0.0427521222862368, 0.0427521222862368), V2(-0.2, 0.1), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSharpEllDeadband(V2(-0.2711686747198338, -0.6101295181196261), V2(-0.3030403507104162, -0.4242564909945826), V2(-0.4, -0.9), V2(0.5, 0.3), V2(0.1, -0.2));
	
	// Test 3D
	T.testSharpEllDeadband(V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSharpEllDeadband(V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), V3(0.2, -0.1, 0.0), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSharpEllDeadband(V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), V3(0.1, 0.2, -0.1), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSharpEllDeadband(V3(0.0, 0.0, 0.0), V3(0.1073222641964480, -0.0858578113571584, 0.0429289056785792), V3(0.4, -0.2, 0.3), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSharpEllDeadband(V3(-0.0369853719805798, 0.0184926859902899, 0.1294488019320291), V3(-0.0039042487723588, -0.0039042487723588, 0.0234254926341526), V3(-0.2, 0.1, 0.7), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSharpEllDeadband(V3(0.5187987963022916, -0.9634834788471133, 0.8152552513321728), V3(0.6122003647672010, -1.1478756839385020, 0.7652504559590013), V3(0.7, -1.3, 1.1), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
}

// Test the SmoothEllDeadband class
TEST(DeadbandTest, testSmoothEllDeadband)
{
	// Test class
	TestEllDeadband T;
	
	// Test 1D
	T.testSmoothEllDeadband(Vector1(-0.15625), Vector1(-0.70000), Vector1(-0.5), Vector1(0.4), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1( 0.00000), Vector1(-0.22500), Vector1( 0.0), Vector1(0.4), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1( 0.15625), Vector1(-0.00625), Vector1( 0.5), Vector1(0.4), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1( 0.60000), Vector1( 0.10000), Vector1( 1.0), Vector1(0.4), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1( 1.10000), Vector1( 0.50000), Vector1( 1.5), Vector1(0.4), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.4), Vector1(-0.2), Vector1(0.4), Vector1(0.0), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.5), Vector1(-0.1), Vector1(0.5), Vector1(0.0), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.6), Vector1( 0.0), Vector1(0.6), Vector1(0.0), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.7), Vector1( 0.1), Vector1(0.7), Vector1(0.0), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.8), Vector1( 0.2), Vector1(0.8), Vector1(0.0), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.3), Vector1(-0.1), Vector1(0.4), Vector1(-0.1), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.4), Vector1(-0.025), Vector1(0.5), Vector1(-0.1), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.5), Vector1( 0.0), Vector1(0.6), Vector1(-0.1), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.6), Vector1( 0.025), Vector1(0.7), Vector1(-0.1), Vector1(0.6));
	T.testSmoothEllDeadband(Vector1(0.7), Vector1( 0.1), Vector1(0.8), Vector1(-0.1), Vector1(0.6));
	
	// Test 2D
	T.testSmoothEllDeadband(V2(0.0, 0.0), V2(-0.0174005108481843, 0.0348010216963685), V2(0.0, 0.0), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSmoothEllDeadband(V2(0.0, 0.0333333333333333), V2(-0.0337062473602611, 0.1348249894410446), V2(0.0, 0.2), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSmoothEllDeadband(V2(0.0, 0.1333333333333333), V2(-0.0502481404895006, 0.3014888429370034), V2(0.0, 0.4), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSmoothEllDeadband(V2(0.045, 0.0), V2(0.0388730126323020, 0.0388730126323020), V2(0.3, 0.0), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSmoothEllDeadband(V2(0.18, 0.0), V2(0.1502313031443329, 0.0600925212577332), V2(0.6, 0.0), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSmoothEllDeadband(V2(-0.0260341655863555, 0.0130170827931778), V2(-0.0874642784226796, 0.0874642784226796), V2(-0.2, 0.1), V2(0.5, 0.3), V2(0.1, -0.2));
	T.testSmoothEllDeadband(V2(-0.2711686747198338, -0.6101295181196261), V2(-0.3030403507104162, -0.4242564909945826), V2(-0.4, -0.9), V2(0.5, 0.3), V2(0.1, -0.2));
	
	// Test 3D
	T.testSmoothEllDeadband(V3(0.0, 0.0, 0.0), V3(0.0136516011204520, -0.0273032022409041, -0.0136516011204520), V3(0.0, 0.0, 0.0), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSmoothEllDeadband(V3(0.0189824032370262, -0.0094912016185131, 0.0), V3(0.0659808742387226, -0.0659808742387226, -0.0219936247462409), V3(0.2, -0.1, 0.0), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSmoothEllDeadband(V3(0.0136516011204520, 0.0273032022409041, -0.0136516011204520), V3(0.0219512963268878, 0.0, -0.0219512963268878), V3(0.1, 0.2, -0.1), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSmoothEllDeadband(V3(0.0909137290096990, -0.0454568645048495, 0.0681852967572742), V3(0.1591635947276302, -0.1273308757821042, 0.0636654378910521), V3(0.4, -0.2, 0.3), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSmoothEllDeadband(V3(-0.0613441880737763, 0.0306720940368882, 0.2147046582582171), V3(-0.0260157183648812, -0.0260157183648812, 0.1560943101892871), V3(-0.2, 0.1, 0.7), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
	T.testSmoothEllDeadband(V3(0.5187987963022916, -0.9634834788471133, 0.8152552513321728), V3(0.6122003647672010, -1.1478756839385020, 0.7652504559590013), V3(0.7, -1.3, 1.1), V3(0.7, 0.4, 0.6), V3(-0.1, 0.2, 0.1));
}

// Test the DeadSmoothEllDeadband class
TEST(DeadbandTest, testDeadSmoothEllDeadband)
{
	// Test class
	TestEllDeadband T;
	
	// Test 1D
	T.testDeadSmoothEllDeadband(Vector1(-1.00), Vector1(-1.00), Vector1(-2.0), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(-0.60), Vector1(-0.60), Vector1(-1.6), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(-0.15), Vector1(-0.15), Vector1(-1.0), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(0.00), Vector1(0.00), Vector1(-0.4), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(0.00), Vector1(0.00), Vector1(0.0), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(0.00), Vector1(0.00), Vector1(0.4), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(0.15), Vector1(0.15), Vector1(1.0), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(0.60), Vector1(0.60), Vector1(1.6), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(1.20), Vector1(1.20), Vector1(2.2), Vector1(0.4), Vector1(0.6), Vector1(0.0));
	T.testDeadSmoothEllDeadband(Vector1(0.15), Vector1(-1.00), Vector1(1.0), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(0.4166666666666667), Vector1(-0.60), Vector1(1.4), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(1.0), Vector1(-0.15), Vector1(2.0), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(1.6), Vector1(0.00), Vector1(2.6), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.0), Vector1(0.00), Vector1(3.0), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.4), Vector1(0.00), Vector1(3.4), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.0), Vector1(0.15), Vector1(4.0), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.6), Vector1(0.60), Vector1(4.6), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(4.2), Vector1(1.20), Vector1(5.2), Vector1(0.4), Vector1(0.6), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.0), Vector1(-0.2), Vector1(2.4), Vector1(0.4), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.2), Vector1(0.0), Vector1(2.6), Vector1(0.4), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.4), Vector1(0.0), Vector1(2.8), Vector1(0.4), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.6), Vector1(0.0), Vector1(3.0), Vector1(0.4), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.8), Vector1(0.0), Vector1(3.2), Vector1(0.4), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.0), Vector1(0.0), Vector1(3.4), Vector1(0.4), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.2), Vector1(0.2), Vector1(3.6), Vector1(0.4), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(1.9), Vector1(-0.1), Vector1(2.4), Vector1(0.4), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.1), Vector1(0.0), Vector1(2.6), Vector1(0.4), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.3), Vector1(0.0), Vector1(2.8), Vector1(0.4), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.5), Vector1(0.0), Vector1(3.0), Vector1(0.4), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.7), Vector1(0.0), Vector1(3.2), Vector1(0.4), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.9), Vector1(0.0), Vector1(3.4), Vector1(0.4), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.1), Vector1(0.1), Vector1(3.6), Vector1(0.4), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.2), Vector1(-0.40), Vector1(2.4), Vector1(0.0), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.4), Vector1(-0.20), Vector1(2.6), Vector1(0.0), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.6), Vector1(-0.05), Vector1(2.8), Vector1(0.0), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.8), Vector1(0.00), Vector1(3.0), Vector1(0.0), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.0), Vector1(0.05), Vector1(3.2), Vector1(0.0), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.2), Vector1(0.20), Vector1(3.4), Vector1(0.0), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.4), Vector1(0.40), Vector1(3.6), Vector1(0.0), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.4), Vector1(-0.6), Vector1(2.4), Vector1(0.0), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.6), Vector1(-0.4), Vector1(2.6), Vector1(0.0), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.8), Vector1(-0.2), Vector1(2.8), Vector1(0.0), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.0), Vector1(0.0), Vector1(3.0), Vector1(0.0), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.2), Vector1(0.2), Vector1(3.2), Vector1(0.0), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.4), Vector1(0.4), Vector1(3.4), Vector1(0.0), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.6), Vector1(0.6), Vector1(3.6), Vector1(0.0), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.3), Vector1(-0.5), Vector1(2.4), Vector1(0.0), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.5), Vector1(-0.3), Vector1(2.6), Vector1(0.0), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.7), Vector1(-0.1), Vector1(2.8), Vector1(0.0), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.9), Vector1(0.0), Vector1(3.0), Vector1(0.0), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.1), Vector1(0.1), Vector1(3.2), Vector1(0.0), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.3), Vector1(0.3), Vector1(3.4), Vector1(0.0), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.5), Vector1(0.5), Vector1(3.6), Vector1(0.0), Vector1(-0.1), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.1), Vector1(-0.3000), Vector1(2.4), Vector1(-0.1), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.3), Vector1(-0.1125), Vector1(2.6), Vector1(-0.1), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.5), Vector1(-0.0125), Vector1(2.8), Vector1(-0.1), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.7), Vector1(0.0000), Vector1(3.0), Vector1(-0.1), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.9), Vector1(0.0125), Vector1(3.2), Vector1(-0.1), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.1), Vector1(0.1125), Vector1(3.4), Vector1(-0.1), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.3), Vector1(0.3000), Vector1(3.6), Vector1(-0.1), Vector1(0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.3), Vector1(-0.5), Vector1(2.4), Vector1(-0.1), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.5), Vector1(-0.3), Vector1(2.6), Vector1(-0.1), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.7), Vector1(-0.1), Vector1(2.8), Vector1(-0.1), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.9), Vector1(0.0), Vector1(3.0), Vector1(-0.1), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.1), Vector1(0.1), Vector1(3.2), Vector1(-0.1), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.3), Vector1(0.3), Vector1(3.4), Vector1(-0.1), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.5), Vector1(0.5), Vector1(3.6), Vector1(-0.1), Vector1(0.0), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.1), Vector1(-0.3000), Vector1(2.4), Vector1(-0.1), Vector1(-0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.3), Vector1(-0.1125), Vector1(2.6), Vector1(-0.1), Vector1(-0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.5), Vector1(-0.0125), Vector1(2.8), Vector1(-0.1), Vector1(-0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.7), Vector1(0.0), Vector1(3.0), Vector1(-0.1), Vector1(-0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(2.9), Vector1(0.0125), Vector1(3.2), Vector1(-0.1), Vector1(-0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.1), Vector1(0.1125), Vector1(3.4), Vector1(-0.1), Vector1(-0.2), Vector1(3.0));
	T.testDeadSmoothEllDeadband(Vector1(3.3), Vector1(0.3000), Vector1(3.6), Vector1(-0.1), Vector1(-0.2), Vector1(3.0));
	
	// Test 2D
	T.testDeadSmoothEllDeadband(V2(0.0, 0.0), V2(-0.0000467224453791, 0.0000934448907582), V2(0.0, 0.0), V2(0.4, 0.2), V2(0.15, 0.1), V2(0.1, -0.2));
	T.testDeadSmoothEllDeadband(V2(0.0, 0.0), V2(-0.0256877373966776, 0.1027509495867103), V2(0.0, 0.2), V2(0.4, 0.2), V2(0.15, 0.1), V2(0.1, -0.2));
	T.testDeadSmoothEllDeadband(V2(0.0, 0.1), V2(-0.0502053396766148, 0.3012320380596888), V2(0.0, 0.4), V2(0.4, 0.2), V2(0.15, 0.1), V2(0.1, -0.2));
	T.testDeadSmoothEllDeadband(V2(0.0, 0.0), V2(0.0013192698143081, 0.0013192698143081), V2(0.3, 0.0), V2(0.4, 0.2), V2(0.15, 0.1), V2(0.1, -0.2));
	T.testDeadSmoothEllDeadband(V2(0.0666666666666666, 0.0), V2(0.0671114764929447, 0.0268445905971779), V2(0.6, 0.0), V2(0.4, 0.2), V2(0.15, 0.1), V2(0.1, -0.2));
	T.testDeadSmoothEllDeadband(V2(0.0, 0.0), V2(-0.0434072431950092, 0.0434072431950092), V2(-0.2, 0.1), V2(0.4, 0.2), V2(0.15, 0.1), V2(0.1, -0.2));
	T.testDeadSmoothEllDeadband(V2(-0.2704199859081727, -0.6084449682933886), V2(-0.3003334160031456, -0.4204667824044038), V2(-0.4, -0.9), V2(0.4, 0.2), V2(0.15, 0.1), V2(0.1, -0.2));
	
	// Test 3D
	T.testDeadSmoothEllDeadband(V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), V3(0.7, 0.4, 0.6), V3(0.1, 0.15, 0.05), V3(-0.1, 0.2, 0.1));
	T.testDeadSmoothEllDeadband(V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), V3(0.2, -0.1, 0.0), V3(0.7, 0.4, 0.6), V3(0.1, 0.15, 0.05), V3(-0.1, 0.2, 0.1));
	T.testDeadSmoothEllDeadband(V3(0.0, 0.0, 0.0), V3(0.0, 0.0, 0.0), V3(0.1, 0.2, -0.1), V3(0.7, 0.4, 0.6), V3(0.1, 0.15, 0.05), V3(-0.1, 0.2, 0.1));
	T.testDeadSmoothEllDeadband(V3(0.0, 0.0, 0.0), V3(0.0277400857992690, -0.0221920686394152, 0.0110960343197076), V3(0.4, -0.2, 0.3), V3(0.7, 0.4, 0.6), V3(0.1, 0.15, 0.05), V3(-0.1, 0.2, 0.1));
	T.testDeadSmoothEllDeadband(V3(-0.0214941020335875, 0.0107470510167938, 0.0752293571175563), V3(-0.0004111156721394, -0.0004111156721394, 0.0024666940328365), V3(-0.2, 0.1, 0.7), V3(0.7, 0.4, 0.6), V3(0.1, 0.15, 0.05), V3(-0.1, 0.2, 0.1));
	T.testDeadSmoothEllDeadband(V3(0.4694210967074589, -0.8717820367424237, 0.7376617233974355), V3(0.5566227361355259, -1.0436676302541112, 0.6957784201694075), V3(0.7, -1.3, 1.1), V3(0.7, 0.4, 0.6), V3(0.1, 0.15, 0.05), V3(-0.1, 0.2, 0.1));
}

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF