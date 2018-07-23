// Unit test for miscellaneous robotcontrol utilities
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <Eigen/Core> // Needs to be before low_pass_filter.h
#include <rc_utils/cyclicwarp.h>
#include <rc_utils/ew_integrator.h>
#include <rc_utils/ell_bnd_integrator.h>
#include <rc_utils/limited_low_pass.h>
#include <rc_utils/lin_sin_fillet.h>
#include <rc_utils/low_pass_filter.h>
#include <rc_utils/hold_filter.h>
#include <rc_utils/slope_limiter.h>
#include <rc_utils/spike_filter.h>
#include <test_utilities/test_eigen.h>
#include <gtest/gtest.h>

// Namespaces
using namespace rc_utils;

// Test the CyclicWarp class
TEST(RCUtilsMiscTest, testCyclicWarp)
{
	// Declare variables
	std::vector<double> raw, warped;
	
	// Test construction
	CyclicWarp CW1;
	CW1.getRefValues(raw, warped);
	EXPECT_EQ(M_2PI, CW1.getModulus());
	EXPECT_EQ(1, raw.size());
	EXPECT_EQ(1, warped.size());
	CyclicWarp CW2(CyclicWarp::UNIT);
	CW1.getRefValues(raw, warped);
	EXPECT_EQ(1.0, CW2.getModulus());
	EXPECT_EQ(1, raw.size());
	EXPECT_EQ(1, warped.size());
	CyclicWarp CW3(CyclicWarp::RADIANS);
	CW1.getRefValues(raw, warped);
	EXPECT_EQ(M_2PI, CW3.getModulus());
	EXPECT_EQ(1, raw.size());
	EXPECT_EQ(1, warped.size());
	CyclicWarp CW4(CyclicWarp::DEGREES);
	CW1.getRefValues(raw, warped);
	EXPECT_EQ(360.0, CW4.getModulus());
	EXPECT_EQ(1, raw.size());
	EXPECT_EQ(1, warped.size());
	CyclicWarp CW5(10.0);
	CW1.getRefValues(raw, warped);
	EXPECT_EQ(10.0, CW5.getModulus());
	EXPECT_EQ(1, raw.size());
	EXPECT_EQ(1, warped.size());
	
	// Test reset
	CW1.reset(CyclicWarp::UNIT);
	EXPECT_EQ(1.0, CW1.getModulus());
	CW1.reset(CyclicWarp::RADIANS);
	EXPECT_EQ(M_2PI, CW1.getModulus());
	CW1.reset(CyclicWarp::DEGREES);
	EXPECT_EQ(360.0, CW1.getModulus());
	CW1.reset(10.0);
	EXPECT_EQ(10.0, CW1.getModulus());
	
	// Test setting the modulus
	CW1.setModulus(6.0);
	EXPECT_EQ(6.0, CW1.getModulus());
	CW1.setModulus(10.0);
	EXPECT_EQ(10.0, CW1.getModulus());
	
	// Test setting the ref values
	std::vector<double> newRaw, newWarped;
	newRaw.push_back(13.0);
	newRaw.push_back(4.0);
	newRaw.push_back(6.0);
	newRaw.push_back(9.0);
	newRaw.push_back(9.5);
	newWarped.push_back(14.0);
	newWarped.push_back(4.5);
	newWarped.push_back(5.5);
	newWarped.push_back(1.0);
	newWarped.push_back(2.0);
	CW1.setModulus(10.0);
	EXPECT_EQ(10.0, CW1.getModulus());
	ASSERT_FALSE(CW1.setRefValues(newRaw, newWarped));
	newRaw[0] = 3.0;
	ASSERT_FALSE(CW1.setRefValues(newRaw, newWarped));
	newWarped[0] = 4.0;
	ASSERT_TRUE(CW1.setRefValues(newRaw, newWarped));
	newRaw[4] = 10.0;
	ASSERT_FALSE(CW1.setRefValues(newRaw, newWarped));
	newRaw[4] = 8.0;
	ASSERT_FALSE(CW1.setRefValues(newRaw, newWarped));
	newRaw[4] = 1.0;
	ASSERT_TRUE(CW1.setRefValues(newRaw, newWarped));
	CW1.getRefValues(raw, warped);
	ASSERT_EQ(raw.size(), newRaw.size());
	ASSERT_EQ(warped.size(), newWarped.size());
	ASSERT_EQ(raw.size(), warped.size());
	for(size_t i = 0; i < raw.size(); i++)
	{
		EXPECT_EQ(newRaw[i], raw[i]);
		EXPECT_EQ(newWarped[i], warped[i]);
	}
	CW1.clear();
	CW1.getRefValues(raw, warped);
	EXPECT_EQ(10.0, CW1.getModulus());
	EXPECT_EQ(1, raw.size());
	EXPECT_EQ(1, warped.size());
	
	// Test wrapping
	CyclicWarp CW(10.0);
	EXPECT_EQ(8.0, CW.wrap(-2.0));
	EXPECT_EQ(0.0, CW.wrap(0.0));
	EXPECT_EQ(5.0, CW.wrap(5.0));
	EXPECT_EQ(0.0, CW.wrap(10.0));
	EXPECT_EQ(2.0, CW.wrap(12.0));
	
	// Test warping (3 4 6 9 1 -> 4 4.5 5.5 1 2)
	ASSERT_TRUE(CW.setRefValues(newRaw, newWarped));
	EXPECT_DOUBLE_EQ(1.5, CW.warp(0.0));
	EXPECT_DOUBLE_EQ(2.0, CW.warp(1.0));
	EXPECT_DOUBLE_EQ(4.0, CW.warp(3.0));
	EXPECT_DOUBLE_EQ(4.75, CW.warp(4.5));
	EXPECT_DOUBLE_EQ(5.5, CW.warp(6.0));
	EXPECT_DOUBLE_EQ(3.0, CW.warp(22.0));
	EXPECT_DOUBLE_EQ(0.0, CW.unwarp(1.5));
	EXPECT_DOUBLE_EQ(1.0, CW.unwarp(2.0));
	EXPECT_DOUBLE_EQ(3.0, CW.unwarp(4.0));
	EXPECT_DOUBLE_EQ(4.5, CW.unwarp(4.75));
	EXPECT_DOUBLE_EQ(6.0, CW.unwarp(5.5));
	EXPECT_DOUBLE_EQ(2.0, CW.unwarp(23.0));
	ASSERT_TRUE(CW.setRefValues(newWarped, newRaw));
	EXPECT_DOUBLE_EQ(1.5, CW.unwarp(0.0));
	EXPECT_DOUBLE_EQ(2.0, CW.unwarp(1.0));
	EXPECT_DOUBLE_EQ(4.0, CW.unwarp(3.0));
	EXPECT_DOUBLE_EQ(4.75, CW.unwarp(4.5));
	EXPECT_DOUBLE_EQ(5.5, CW.unwarp(6.0));
	EXPECT_DOUBLE_EQ(3.0, CW.unwarp(22.0));
	EXPECT_DOUBLE_EQ(0.0, CW.warp(1.5));
	EXPECT_DOUBLE_EQ(1.0, CW.warp(2.0));
	EXPECT_DOUBLE_EQ(3.0, CW.warp(4.0));
	EXPECT_DOUBLE_EQ(4.5, CW.warp(4.75));
	EXPECT_DOUBLE_EQ(6.0, CW.warp(5.5));
	EXPECT_DOUBLE_EQ(2.0, CW.warp(23.0));
}

// Test the EWIntegrator class
TEST(RCUtilsMiscTest, testEWIntegrator)
{
	// Test construction 
	EWIntegrator EWI;
	EXPECT_EQ(1.0, EWI.alpha());
	EXPECT_EQ(0.0, EWI.integral());
	
	// Test setting parameters
	EWI.setAlpha(-0.1);
	EXPECT_EQ(0.0, EWI.alpha());
	EWI.setAlpha(1.4);
	EXPECT_EQ(1.0, EWI.alpha());
	EWI.setAlpha(0.9);
	EXPECT_EQ(0.9, EWI.alpha());
	EWI.setIntegral(6513.92);
	EXPECT_EQ(6513.92, EWI.integral());
	EWI.resetIntegral();
	EXPECT_EQ(0.0, EWI.integral());
	EWI.setIntegral(-4.7);
	EXPECT_EQ(-4.7, EWI.integral());
	EWI.reset();
	EXPECT_EQ(0.9, EWI.alpha());
	EXPECT_EQ(0.0, EWI.integral());
	EWI.setIntegral(3.2);
	EXPECT_EQ(3.2, EWI.integral());
	
	// Test reset 
	EWI.resetAll();
	EXPECT_EQ(1.0, EWI.alpha());
	EXPECT_EQ(0.0, EWI.integral());
	
	// Test setting half life
	EWI.setHalfLife(0.5);
	EXPECT_DOUBLE_EQ(0.25, EWI.alpha());
	EWI.setHalfLife(1);
	EXPECT_DOUBLE_EQ(0.5, EWI.alpha());
	EWI.setHalfLife(100);
	EXPECT_DOUBLE_EQ(0.9930924954370359, EWI.alpha());
	
	// Test integration
	EWI.setAlpha(0.8);
	EXPECT_EQ(0.8, EWI.alpha());
	EWI.setIntegral(2.5);
	EXPECT_EQ(2.5, EWI.integral());
	EWI.integrate(0.1);
	EXPECT_EQ(2.1, EWI.integral());
	EWI.integrate(0.4);
	EXPECT_EQ(2.08, EWI.integral());
	EWI.integrate(-0.3);
	EXPECT_EQ(1.364, EWI.integral());
}

// Test the EllBndIntegrator class
TEST(RCUtilsMiscTest, testEllBndIntegrator)
{
	// Typedefs
	typedef Eigen::Vector2d V;
	
	// Constants
	V zero(0.0, 0.0);
	double inf = std::numeric_limits<double>::infinity();
	
	// Test construction
	EllBndIntegrator2D I;
	EXPECT_EIGEQ(2, V(inf, inf), I.semiaxes());
	EXPECT_EQ(0.0, I.buffer());
	EXPECT_EIGEQ(2, zero, I.integral());
	
	// Test setting ellipsoid parameters
	I.setEllBound(V(2.0, 3.0), 0.3);
	EXPECT_EIGEQ(2, V(2.0, 3.0), I.semiaxes());
	EXPECT_EQ(0.3, I.buffer());
	I.setEllBound(V(4.0, -5.0));
	EXPECT_EIGEQ(2, V(4.0, -5.0), I.semiaxes());
	EXPECT_EQ(0.0, I.buffer());
	I.setBuffer(20.0);
	EXPECT_EQ(20.0, I.buffer());
	I.unsetEllBound();
	EXPECT_EIGEQ(2, V(inf, inf), I.semiaxes());
	EXPECT_EQ(0.0, I.buffer());
	
	// Test setting integral
	I.setIntegral(V(0.3, -0.5));
	EXPECT_EIGEQ(2, V(0.3, -0.5), I.integral());
	I.setIntegralZero();
	EXPECT_EIGEQ(2, zero, I.integral());
	I.setIntegral(V(0.3, -0.5));
	EXPECT_EIGEQ(2, V(0.3, -0.5), I.integral());
	I.setIntegralXZero();
	EXPECT_EIGEQ(2, V(0.0, -0.5), I.integral());
	I.setIntegral(V(0.3, -0.5));
	EXPECT_EIGEQ(2, V(0.3, -0.5), I.integral());
	I.setIntegralYZero();
	EXPECT_EIGEQ(2, V(0.3, 0.0), I.integral());
	
	// Test reset
	I.setEllBound(V(2.5, 6.0), 0.2);
	I.setIntegral(V(-2.0, 1.5));
	EXPECT_EIGEQ(2, V(2.5, 6.0), I.semiaxes());
	EXPECT_EQ(0.2, I.buffer());
	EXPECT_EIGEQ(2, V(-2.0, 1.5), I.integral());
	I.resetAll();
	EXPECT_EIGEQ(2, V(inf, inf), I.semiaxes());
	EXPECT_EQ(0.0, I.buffer());
	EXPECT_EIGEQ(2, zero, I.integral());
	I.setEllBound(V(2.5, 6.0), 0.2);
	I.setIntegral(V(-2.0, 1.5));
	EXPECT_EIGEQ(2, V(2.5, 6.0), I.semiaxes());
	EXPECT_EQ(0.2, I.buffer());
	EXPECT_EIGEQ(2, V(-2.0, 1.5), I.integral());
	I.reset();
	EXPECT_EIGEQ(2, V(2.5, 6.0), I.semiaxes());
	EXPECT_EQ(0.2, I.buffer());
	EXPECT_EIGEQ(2, zero, I.integral());
	
	// Test integration
	I.resetAll();
	I.setEllBound(V(1.0, 2.0), 0.1);
	EXPECT_EIGEQ(2, V(1.0, 2.0), I.semiaxes());
	EXPECT_EQ(0.1, I.buffer());
	EXPECT_EIGEQ(2, zero, I.integral());
	EXPECT_EIGEQ_UT(2, V(0.15, -0.10), I.integrate(V(3.0, -2.0), 0.1));
	EXPECT_EIGEQ_UT(2, V(0.55, -0.40), I.integrate(V(1.0, -1.0), 0.2));
	EXPECT_EIGEQ_UT(2, V(0.55, -0.40), I.integral());
	I.clearLastInput();
	EXPECT_EIGEQ_UT(2, V(0.50, -0.35), I.integrate(V(-1.0, 1.0), 0.1));
	EXPECT_EIGEQ_UT(2, V(0.30, -0.30), I.integrate(V(-3.0, 0.0), 0.1));
	EXPECT_EIGEQ_UT(2, V(0.30, -0.30), I.integral());
	I.setEllBound(V(0.1, 0.2), 0.05);
	EXPECT_EIGEQ_UT(2, V(0.30, -0.30), I.integral());
	EXPECT_EIGEQ_UT(2, V(0.0727972877604889, -0.1368589009897192), I.integrate(V(1.5, -0.7), 0.15));
	I.clearLastInput();
	EXPECT_EIGEQ_UT(2, V(0.0642005969430037, -0.1206971222528470), I.integrate(zero, 3.7));
}

// Test the LimitedLowPass class
TEST(RCUtilsMiscTest, testLimitedLowPass)
{
	// Test construction
	LimitedLowPass LLP1;
	EXPECT_EQ(0.0, LLP1.maxDelta());
	EXPECT_EQ(0.0, LLP1.value());
	EXPECT_EQ(100.0, LLP1.getTs());
	EXPECT_DOUBLE_EQ(0.02276277904418933, LLP1.getAlpha());
	LimitedLowPass LLP2(10.0);
	EXPECT_EQ(0.0, LLP2.maxDelta());
	EXPECT_EQ(0.0, LLP2.value());
	EXPECT_EQ(10.0, LLP2.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LLP2.getAlpha());
	LimitedLowPass LLP3(-10.0, 0.05);
	EXPECT_EQ(0.05, LLP3.maxDelta());
	EXPECT_EQ(0.0, LLP3.value());
	EXPECT_EQ(0.0, LLP3.getTs());
	EXPECT_EQ(1.0, LLP3.getAlpha());
	
	// Test reset
	LLP2.setValue(0.5);
	LLP2.resetAll();
	EXPECT_EQ(0.0, LLP1.maxDelta());
	EXPECT_EQ(0.0, LLP1.value());
	EXPECT_EQ(100.0, LLP1.getTs());
	EXPECT_DOUBLE_EQ(0.02276277904418933, LLP1.getAlpha());
	LLP2.resetAll(10.0);
	EXPECT_EQ(0.0, LLP2.maxDelta());
	EXPECT_EQ(0.0, LLP2.value());
	EXPECT_EQ(10.0, LLP2.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LLP2.getAlpha());
	LLP2.setValue(-23.3);
	LLP2.resetAll(-10.0, 0.05);
	EXPECT_EQ(0.05, LLP2.maxDelta());
	EXPECT_EQ(0.0, LLP2.value());
	EXPECT_EQ(0.0, LLP2.getTs());
	EXPECT_EQ(1.0, LLP2.getAlpha());
	LLP2.resetAll(10.0);
	LLP2.setValue(-23.3);
	LLP2.setMaxDelta(-0.3);
	EXPECT_EQ(0.3, LLP2.maxDelta());
	EXPECT_EQ(-23.3, LLP2.value());
	EXPECT_EQ(10.0, LLP2.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LLP2.getAlpha());
	LLP2.reset();
	EXPECT_EQ(0.3, LLP2.maxDelta());
	EXPECT_EQ(0.0, LLP2.value());
	EXPECT_EQ(10.0, LLP2.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LLP2.getAlpha());
	
	// Test setting max delta
	LLP1.resetAll();
	EXPECT_EQ(0.0, LLP1.maxDelta());
	LLP1.setMaxDelta(0.1);
	EXPECT_EQ(0.1, LLP1.maxDelta());
	
	// Test freezing of the filter
	LLP1.setTs(50.0);
	EXPECT_FALSE(LLP1.isFrozen());
	LLP1.setValue(1.0);
	EXPECT_EQ(1.0, LLP1.value());
	LLP1.freeze();
	EXPECT_TRUE(LLP1.isFrozen());
	LLP1.setValue(2.0);
	EXPECT_EQ(2.0, LLP1.value());
	LLP1.put(3.0);
	LLP1.put(3.0, 1.0);
	EXPECT_EQ(2.0, LLP1.value());
	LLP1.unfreeze();
	EXPECT_FALSE(LLP1.isFrozen());
	LLP1.put(3.0);
	EXPECT_NE(2.0, LLP1.value());
	LLP1.setValue(3.0);
	EXPECT_EQ(3.0, LLP1.value());
	LLP1.put(2.0, 1.0);
	EXPECT_NE(3.0, LLP1.value());
	LLP1.setValue(3.0);
	EXPECT_EQ(3.0, LLP1.value());
	LLP1.setFrozen(true);
	EXPECT_TRUE(LLP1.isFrozen());
	LLP1.put(4.0);
	LLP1.put(4.0, 2.0);
	EXPECT_EQ(3.0, LLP1.value());
	LLP1.setFrozen(false);
	EXPECT_FALSE(LLP1.isFrozen());
	LLP1.put(4.0);
	EXPECT_NE(3.0, LLP1.value());
	
	// Test the response of the filter
	LimitedLowPass LLP(50.0, 0.10);
	LLP.setValue(4.0);
	for(int i = 0; i < 50; i++)
		LLP.put(2.0);
	EXPECT_NEAR(2.2, LLP.value(), 1e-15); // 2.2 is 90% of the way from 4.0 to 2.0 and the generous max delta is not violated at any point
	LLP.setValue(4.0);
	LLP.setMaxDelta(0.02);
	for(int i = 0; i < 50; i++)
		LLP.put(2.0);
	EXPECT_NEAR(3.0, LLP.value(), 1e-15); // Every single update is slope limited
	LLP.setMaxDelta(0.0);
	LLP.setValue(4.0);
	for(int i = 0; i < 50; i++)
		LLP.put(2.0, 0.10);
	EXPECT_NEAR(2.2, LLP.value(), 1e-15); // 2.2 is 90% of the way from 4.0 to 2.0 and the generous max delta is not violated at any point
	LLP.setValue(4.0);
	for(int i = 0; i < 50; i++)
		LLP.put(2.0, 0.02);
	EXPECT_NEAR(3.0, LLP.value(), 1e-15); // Every single update is slope limited
}

// Test the LinSinFillet class
TEST(RCUtilsMiscTest, testLinSinFillet)
{
	// Test construction and reset
	LinSinFillet L1;
	EXPECT_EQ(0.0, L1.eval(-1.0));
	EXPECT_EQ(0.0, L1.eval( 0.0));
	EXPECT_EQ(0.0, L1.eval( 1.0));
	ASSERT_TRUE(L1.update(1.0, 0.1, 2.0));
	EXPECT_NE(0.0, L1.eval(-1.0));
	EXPECT_NE(0.0, L1.eval( 0.0));
	EXPECT_NE(0.0, L1.eval( 1.0));
	L1.reset();
	EXPECT_EQ(0.0, L1.eval(-1.0));
	EXPECT_EQ(0.0, L1.eval( 0.0));
	EXPECT_EQ(0.0, L1.eval( 1.0));
	
	// Test bad update
	ASSERT_TRUE(L1.update(1.0, 0.1, 2.0));
	EXPECT_NE(0.0, L1.eval(-1.0));
	EXPECT_NE(0.0, L1.eval( 0.0));
	EXPECT_NE(0.0, L1.eval( 1.0));
	ASSERT_FALSE(L1.update(1.0, 0.0, 2.0));
	EXPECT_EQ(0.0, L1.eval(-1.0));
	EXPECT_EQ(0.0, L1.eval( 0.0));
	EXPECT_EQ(0.0, L1.eval( 1.0));
	
	// Test zero amplitude
	ASSERT_TRUE(L1.update(0.0, 0.1, 2.0));
	EXPECT_EQ(0.0, L1.eval(-1.0));
	EXPECT_EQ(0.0, L1.eval( 0.0));
	EXPECT_EQ(0.0, L1.eval( 1.0));
	
	// Test a normal case
	ASSERT_TRUE(L1.update(1.0, 1.5, 0.6));
	EXPECT_DOUBLE_EQ(1.0, L1.getA());
	EXPECT_DOUBLE_EQ(1.5, L1.getB());
	EXPECT_DOUBLE_EQ(-1.0802109567337852, L1.startTime());
	EXPECT_DOUBLE_EQ(0.6, L1.endTime());
	EXPECT_DOUBLE_EQ(0.0, L1.eval(-1.5));
	EXPECT_NEAR(0.0017851832951220, L1.eval(-1.0), 1e-16);
	EXPECT_NEAR(0.0934086881511417, L1.eval(-0.5), 1e-16);
	EXPECT_NEAR(0.3237669972097462, L1.eval( 0.0), 1e-16);
	EXPECT_NEAR(0.0112213504476012, L1.eval( 0.5), 1e-16);
	EXPECT_DOUBLE_EQ(0.0, L1.eval( 1.0));
	EXPECT_DOUBLE_EQ(0.0, L1.eval( 1.5));
	
	// Test a limited case that doesn't need to limit
	ASSERT_TRUE(L1.updateExact(1.0, 1.5, 0.6, 1.1));
	EXPECT_DOUBLE_EQ(1.0, L1.getA());
	EXPECT_DOUBLE_EQ(1.5, L1.getB());
	EXPECT_DOUBLE_EQ(-1.0802109567337852, L1.startTime());
	EXPECT_DOUBLE_EQ(0.6, L1.endTime());
	EXPECT_DOUBLE_EQ(0.0, L1.eval(-1.5));
	EXPECT_NEAR(0.0017851832951220, L1.eval(-1.0), 1e-16);
	EXPECT_NEAR(0.0934086881511417, L1.eval(-0.5), 1e-16);
	EXPECT_NEAR(0.3237669972097462, L1.eval( 0.0), 1e-16);
	EXPECT_NEAR(0.0112213504476012, L1.eval( 0.5), 1e-16);
	EXPECT_DOUBLE_EQ(0.0, L1.eval( 1.0));
	EXPECT_DOUBLE_EQ(0.0, L1.eval( 1.5));
	
	// Test a limited case that needs to limit
	ASSERT_TRUE(L1.update(-2.0, 1.5, 0.6, 0.5));
	EXPECT_DOUBLE_EQ(-2.0, L1.getA());
	EXPECT_DOUBLE_EQ(1.5, L1.getB());
	EXPECT_DOUBLE_EQ(-0.3122539058962907, L1.startTime());
	EXPECT_DOUBLE_EQ(0.2777235299548383, L1.endTime());
	EXPECT_DOUBLE_EQ(0.0, L1.eval(-0.5));
	EXPECT_NEAR(-0.0003491220257582, L1.eval(-0.3), 1e-16);
	EXPECT_NEAR(-0.1047465068910521, L1.eval(-0.1), 1e-16);
	EXPECT_NEAR(-0.0962698581466380, L1.eval( 0.1), 1e-16);
	EXPECT_DOUBLE_EQ(0.0, L1.eval( 0.3));
	EXPECT_DOUBLE_EQ(0.0, L1.eval( 0.5));
	
	// Test the success of various exactly limited cases
	ASSERT_TRUE(L1.updateExact(-2.0, 1.5, 0.6, 0.5));
	EXPECT_NEAR(-0.5, L1.startTime(), 1e-6);
	ASSERT_TRUE(L1.updateExact(1.0, 2.5, 0.6, 0.3));
	EXPECT_NEAR(-0.3, L1.startTime(), 1e-6);
	ASSERT_FALSE(L1.updateExact(1.0, 2.5, 0.8, 0.3));
	ASSERT_TRUE(L1.updateExact(4.0, -0.3, 0.6, 0.01));
	EXPECT_NEAR(-0.01, L1.startTime(), 1e-6);
}

// Test the LowPassFilter class
TEST(RCUtilsMiscTest, testLowPassFilter)
{
	// Constants
	double inf = std::numeric_limits<double>::infinity();

	// Test construction
	LowPassFilter LPF1;
	EXPECT_EQ(0.0, LPF1.value());
	EXPECT_FALSE(LPF1.isFrozen());
	EXPECT_EQ(100.0, LPF1.getTs());
	EXPECT_DOUBLE_EQ(0.02276277904418933, LPF1.getAlpha());
	LowPassFilter LPF2(10.0);
	EXPECT_EQ(0.0, LPF2.value());
	EXPECT_FALSE(LPF1.isFrozen());
	EXPECT_EQ(10.0, LPF2.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LPF2.getAlpha());
	LowPassFilter LPF3(-10.0);
	EXPECT_EQ(0.0, LPF3.value());
	EXPECT_FALSE(LPF1.isFrozen());
	EXPECT_EQ(0.0, LPF3.getTs());
	EXPECT_EQ(1.0, LPF3.getAlpha());
	
	// Test reset
	LPF3.setValue(0.5);
	LPF3.resetAll();
	EXPECT_EQ(0.0, LPF3.value());
	EXPECT_FALSE(LPF3.isFrozen());
	EXPECT_EQ(100.0, LPF3.getTs());
	EXPECT_DOUBLE_EQ(0.02276277904418933, LPF3.getAlpha());
	LPF3.resetAll(10.0);
	LPF3.freeze();
	EXPECT_EQ(0.0, LPF3.value());
	EXPECT_TRUE(LPF3.isFrozen());
	EXPECT_EQ(10.0, LPF3.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LPF3.getAlpha());
	LPF3.resetAll(-10.0);
	EXPECT_EQ(0.0, LPF3.value());
	EXPECT_FALSE(LPF3.isFrozen());
	EXPECT_EQ(0.0, LPF3.getTs());
	EXPECT_EQ(1.0, LPF3.getAlpha());
	LPF3.setValue(1.52);
	LPF3.freeze();
	LPF3.setTs(10.0);
	EXPECT_EQ(1.52, LPF3.value());
	EXPECT_TRUE(LPF3.isFrozen());
	EXPECT_EQ(10.0, LPF3.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LPF3.getAlpha());
	LPF3.reset();
	EXPECT_EQ(0.0, LPF3.value());
	EXPECT_FALSE(LPF3.isFrozen());
	EXPECT_EQ(10.0, LPF3.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LPF3.getAlpha());
	
	// Test setting the settling time
	LPF3.setValue(1.0);
	EXPECT_EQ(1.0, LPF3.value());
	LPF3.setTs(1.0/0.0); // +Inf
	EXPECT_EQ(1.0, LPF3.value());
	EXPECT_EQ(inf, LPF3.getTs());
	EXPECT_EQ(0.0, LPF3.getAlpha());
	LPF3.setTs(-1.0/0.0); // -Inf
	EXPECT_EQ(1.0, LPF3.value());
	EXPECT_EQ(inf, LPF3.getTs());
	EXPECT_EQ(0.0, LPF3.getAlpha());
	LPF3.setTs(0.0/0.0); // NaN
	EXPECT_EQ(1.0, LPF3.value());
	EXPECT_EQ(inf, LPF3.getTs());
	EXPECT_EQ(0.0, LPF3.getAlpha());
	LPF3.setTs(0.0);
	EXPECT_EQ(1.0, LPF3.value());
	EXPECT_EQ(0.0, LPF3.getTs());
	EXPECT_EQ(1.0, LPF3.getAlpha());
	LPF3.setTs(-5.0);
	EXPECT_EQ(1.0, LPF3.value());
	EXPECT_EQ(0.0, LPF3.getTs());
	EXPECT_EQ(1.0, LPF3.getAlpha());
	LPF3.setTs(30.0);
	EXPECT_EQ(1.0, LPF3.value());
	EXPECT_EQ(30.0, LPF3.getTs());
	EXPECT_DOUBLE_EQ(0.07388127187120652, LPF3.getAlpha());
	
	// Test setting the value
	LPF3.setValue(2.0);
	EXPECT_EQ(2.0, LPF3.value());
	EXPECT_EQ(30.0, LPF3.getTs());
	EXPECT_DOUBLE_EQ(0.07388127187120652, LPF3.getAlpha());
	
	// Test the response of the filter
	LowPassFilter LPF(50.0);
	LPF.setValue(2.0);
	for(int i = 0; i < 50; i++)
		LPF.put(4.0);
	EXPECT_NEAR(3.8, LPF.value(), 1e-15); // 3.8 is 90% of the way from 2.0 to 4.0
	LPF.setTs(1.0/0.0);
	LPF.setValue(2.0);
	for(int i = 0; i < 50; i++)
		LPF.put(4.0);
	EXPECT_EQ(2.0, LPF.value());
	LPF.setTs(0.0);
	LPF.setValue(2.0);
	EXPECT_EQ(2.0, LPF.value());
	LPF.put(4.0);
	EXPECT_EQ(4.0, LPF.value());
	
	// Test freezing of the filter
	LPF.setTs(50.0);
	EXPECT_FALSE(LPF.isFrozen());
	LPF.setValue(1.0);
	EXPECT_EQ(1.0, LPF.value());
	LPF.freeze();
	EXPECT_TRUE(LPF.isFrozen());
	LPF.setValue(2.0);
	EXPECT_EQ(2.0, LPF.value());
	LPF.put(3.0);
	EXPECT_EQ(2.0, LPF.value());
	LPF.unfreeze();
	EXPECT_FALSE(LPF.isFrozen());
	LPF.put(3.0);
	EXPECT_NE(2.0, LPF.value());
	LPF.setValue(3.0);
	EXPECT_EQ(3.0, LPF.value());
	LPF.setFrozen(true);
	EXPECT_TRUE(LPF.isFrozen());
	LPF.put(4.0);
	EXPECT_EQ(3.0, LPF.value());
	LPF.setFrozen(false);
	EXPECT_FALSE(LPF.isFrozen());
	LPF.put(4.0);
	EXPECT_NE(3.0, LPF.value());
	
	// Test the basics of a filter with Eigen
	LowPassFilterT<Eigen::Vector3d> LPF3d;
	Eigen::Vector3d value = LPF3d.value();
	EXPECT_EQ(0.0, value.x());
	EXPECT_EQ(0.0, value.y());
	EXPECT_EQ(0.0, value.z());
	LPF3d.setValue(Eigen::Vector3d(1.3, -3.2, 9.6));
	value = LPF3d.value();
	EXPECT_EQ(1.3, value.x());
	EXPECT_EQ(-3.2, value.y());
	EXPECT_EQ(9.6, value.z());
	LPF3d.setTs(1);
	LPF3d.put(Eigen::Vector3d::Zero());
	value = LPF3d.value();
	EXPECT_NEAR(0.13, value.x(), 1e-15);
	EXPECT_NEAR(-0.32, value.y(), 1e-15);
	EXPECT_NEAR(0.96, value.z(), 1e-15);
}

// Test the HoldFilter class
TEST(RCUtilsMiscTest, testHoldFilter)
{
	// Declare variables
	const HoldFilter<true>::Buffer* buf;

	// Test construction
	HoldMaxFilter HMF1;
	buf = &HMF1.buf();
	EXPECT_TRUE(buf->empty());
	EXPECT_EQ(0, HMF1.numPoints());
	EXPECT_EQ(0.0, HMF1.value());
	EXPECT_FALSE(HMF1.haveData());
	HoldMaxFilter HMF2(3);
	buf = &HMF2.buf();
	EXPECT_TRUE(buf->empty());
	EXPECT_EQ(3, HMF2.numPoints());
	EXPECT_EQ(0.0, HMF2.value());
	EXPECT_FALSE(HMF2.haveData());

	// Test putting and resetting data
	HMF2.put(-3.0);
	EXPECT_FALSE(buf->empty());
	EXPECT_EQ(1, HMF2.numData());
	EXPECT_EQ(3, HMF2.numPoints());
	EXPECT_EQ(-3.0, HMF2.value());
	EXPECT_TRUE(HMF2.haveData());
	HMF2.put(-5.0);
	EXPECT_FALSE(buf->empty());
	EXPECT_EQ(2, buf->size());
	EXPECT_EQ(2, HMF2.numData());
	EXPECT_EQ(3, HMF2.numPoints());
	EXPECT_EQ(-3.0, HMF2.value());
	EXPECT_TRUE(HMF2.haveData());
	HMF2.put(2.0);
	EXPECT_TRUE(buf->full());
	EXPECT_EQ(3, HMF2.numData());
	EXPECT_EQ(3, HMF2.numPoints());
	EXPECT_EQ(2.0, HMF2.value());
	EXPECT_TRUE(HMF2.haveData());
	HMF2.put(1.0);
	EXPECT_EQ(3, HMF2.numData());
	EXPECT_EQ(2.0, HMF2.value());
	HMF2.put(0.5);
	EXPECT_EQ(3, HMF2.numData());
	EXPECT_EQ(2.0, HMF2.value());
	HMF2.put(0.8);
	EXPECT_EQ(3, HMF2.numData());
	EXPECT_EQ(1.0, HMF2.value());
	HMF2.put(0.4);
	EXPECT_EQ(3, HMF2.numData());
	EXPECT_EQ(0.8, HMF2.value());
	HMF2.reset();
	buf = &HMF2.buf();
	EXPECT_TRUE(buf->empty());
	EXPECT_TRUE(buf->capacity());
	EXPECT_EQ(3, HMF2.numPoints());
	EXPECT_EQ(0.0, HMF2.value());
	EXPECT_FALSE(HMF2.haveData());
	EXPECT_EQ(3.0, HMF2.update(3.0));
	EXPECT_EQ(1, HMF2.numData());
	EXPECT_EQ(3.0, HMF2.value());
	HMF2.resetAll();
	buf = &HMF2.buf();
	EXPECT_TRUE(buf->empty());
	EXPECT_EQ(0, HMF2.numPoints());
	EXPECT_EQ(0.0, HMF2.value());
	EXPECT_FALSE(HMF2.haveData());
	HMF2.resetAll(5);
	buf = &HMF2.buf();
	EXPECT_TRUE(buf->empty());
	EXPECT_EQ(5, HMF2.numPoints());
	EXPECT_EQ(0.0, HMF2.value());
	EXPECT_FALSE(HMF2.haveData());

	// Test resizing data
	HMF2.resetAll(2);
	HMF2.put(0.5);
	HMF2.put(0.9);
	HMF2.put(0.7);
	EXPECT_EQ(2, HMF2.numPoints());
	EXPECT_EQ(2, HMF2.numData());
	EXPECT_EQ(0.9, HMF2.value());
	HMF2.resize(5);
	EXPECT_EQ(5, HMF2.numPoints());
	EXPECT_EQ(2, HMF2.numData());
	EXPECT_EQ(0.9, HMF2.value());
	EXPECT_EQ(0.9, HMF2.update(0.1));
	EXPECT_EQ(0.9, HMF2.update(0.6));
	EXPECT_EQ(0.9, HMF2.update(0.3));
	EXPECT_EQ(0.7, HMF2.update(0.4));
	EXPECT_EQ(0.6, HMF2.update(0.2));
	HMF2.resize(3);
	EXPECT_EQ(3, HMF2.numPoints());
	EXPECT_EQ(3, HMF2.numData());
	EXPECT_EQ(0.4, HMF2.value());
	EXPECT_EQ(0.4, HMF2.update(-0.2));
	EXPECT_EQ(0.2, HMF2.update(-0.7));
}

// Test the SlopeLimiter class
TEST(RCUtilsMiscTest, testSlopeLimiter)
{
	// Test construction 
	SlopeLimiter SL;
	EXPECT_EQ( 0.0, SL.maxDelta());
	EXPECT_EQ( 0.0, SL.value());
	EXPECT_FALSE(SL.valueSet());
	SlopeLimiter SL2(0.4, -0.6);
	EXPECT_EQ( 0.4, SL2.maxDelta());
	EXPECT_EQ(-0.6, SL2.value());
	EXPECT_TRUE(SL2.valueSet());
	SlopeLimiter SL3(-0.4);
	EXPECT_EQ( 0.4, SL3.maxDelta());
	EXPECT_EQ( 0.0, SL3.value());
	EXPECT_FALSE(SL3.valueSet());
	
	// Test reset and set
	SL2.reset();
	EXPECT_EQ( 0.0, SL2.maxDelta());
	EXPECT_EQ( 0.0, SL2.value());
	EXPECT_FALSE(SL2.valueSet());
	SL.set(0.9, 1.7);
	EXPECT_EQ( 0.9, SL.maxDelta());
	EXPECT_EQ( 1.7, SL.value());
	EXPECT_TRUE(SL.valueSet());
	SL.setMaxDelta(-1.9);
	EXPECT_EQ( 1.9, SL.maxDelta());
	EXPECT_EQ( 1.7, SL.value());
	EXPECT_TRUE(SL.valueSet());
	SL.setValue(51311);
	EXPECT_EQ( 1.9, SL.maxDelta());
	EXPECT_EQ(51311, SL.value());
	EXPECT_TRUE(SL.valueSet());
	
	// Test put
	SL3.put(1.0);
	EXPECT_EQ(0.4, SL3.value());
	EXPECT_TRUE(SL3.valueSet());
	SL.set(0.1, 3.0);
	EXPECT_EQ(0.1, SL.maxDelta());
	EXPECT_EQ(3.0, SL.value());
	EXPECT_TRUE(SL.valueSet());
	SL.put(2.8);
	EXPECT_DOUBLE_EQ(2.9, SL.value());
	SL.put(2.85);
	EXPECT_DOUBLE_EQ(2.85, SL.value());
	SL.put(3.0);
	EXPECT_DOUBLE_EQ(2.95, SL.value());
	SL.put(3.0);
	EXPECT_DOUBLE_EQ(3.0, SL.value());
	
	// Test eval
	EXPECT_DOUBLE_EQ(0.5, SlopeLimiter::eval(0.5, 0.2, 0.6));
	EXPECT_DOUBLE_EQ(-0.4, SlopeLimiter::eval(-0.4, 0.2, 0.6));
	EXPECT_DOUBLE_EQ(0.49, SlopeLimiter::eval(0.5, 0.2, 0.29));
	EXPECT_DOUBLE_EQ(-0.09, SlopeLimiter::eval(-0.5, 0.2, 0.29));
	EXPECT_DOUBLE_EQ(-0.05, SlopeLimiter::eval(-0.5, 0.2, -0.25));
}

// Test the SpikeFilter class
TEST(RCUtilsMiscTest, testSpikeFilter)
{
	// Test construction 
	SpikeFilter SF;
	EXPECT_EQ( 0.0, SF.maxDelta());
	EXPECT_EQ( 0.0, SF.value());
	EXPECT_EQ( 0, SF.holdCount());
	SpikeFilter SF2(0.4, -0.6);
	EXPECT_EQ( 0.4, SF2.maxDelta());
	EXPECT_EQ(-0.6, SF2.value());
	EXPECT_EQ( 0, SF2.holdCount());
	SpikeFilter SF3(-0.4);
	EXPECT_EQ( 0.4, SF3.maxDelta());
	EXPECT_EQ( 0.0, SF3.value());
	EXPECT_EQ( 0, SF3.holdCount());
	
	// Test reset and set
	SF2.reset();
	EXPECT_EQ( 0.0, SF2.maxDelta());
	EXPECT_EQ( 0.0, SF2.value());
	EXPECT_EQ( 0, SF2.holdCount());
	SF.set(0.9, 1.7);
	EXPECT_EQ( 0.9, SF.maxDelta());
	EXPECT_EQ( 1.7, SF.value());
	EXPECT_EQ( 0, SF2.holdCount());
	SF.set(-1.9);
	EXPECT_EQ( 1.9, SF.maxDelta());
	EXPECT_EQ( 0.0, SF.value());
	EXPECT_EQ( 0, SF2.holdCount());
	SF.setValue(51311);
	EXPECT_EQ( 1.9, SF.maxDelta());
	EXPECT_EQ(51311, SF.value());
	EXPECT_EQ( 0, SF2.holdCount());
	
	// Test put
	SF.set(0.5, 3.0);
	EXPECT_EQ( 0.5, SF.maxDelta());
	EXPECT_EQ( 3.0, SF.value());
	EXPECT_EQ( 0, SF.holdCount());
	SF.put(2.9);
	EXPECT_EQ( 2.9, SF.value());
	EXPECT_EQ( 0, SF.holdCount());
	SF.put(3.4);
	EXPECT_EQ( 3.4, SF.value());
	EXPECT_EQ( 0, SF.holdCount());
	SF.put(2.4);
	EXPECT_EQ( 3.4, SF.value());
	EXPECT_EQ( 1, SF.holdCount());
	SF.put(2.2);
	EXPECT_EQ( 3.4, SF.value());
	EXPECT_EQ( 2, SF.holdCount());
	SF.put(2.0);
	EXPECT_EQ( 2.0, SF.value());
	EXPECT_EQ( 0, SF.holdCount());
	SF.put(1.8);
	EXPECT_EQ( 1.8, SF.value());
	EXPECT_EQ( 0, SF.holdCount());
}

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF