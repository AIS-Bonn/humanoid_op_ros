// Unit test for miscellaneous robotcontrol utilities
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>
#include <rc_utils/cyclicwarp.h>
#include <rc_utils/ew_integrator.h>
#include <rc_utils/limited_low_pass.h>
#include <rc_utils/lin_sin_fillet.h>
#include <rc_utils/low_pass_filter.h>
#include <rc_utils/mean_filter.h>
#include <rc_utils/slope_limiter.h>
#include <rc_utils/smooth_deadband.h>
#include <rc_utils/spike_filter.h>
#include <rc_utils/wlbf_filter.h>
#include <gtest/gtest.h>

// Defines
#define PRINT_TESTING 0

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
	EXPECT_EQ(0.0, LLP2.maxDelta());
	EXPECT_EQ(-23.3, LLP2.value());
	EXPECT_EQ(10.0, LLP2.getTs());
	EXPECT_DOUBLE_EQ(0.20567176527571851, LLP2.getAlpha());
	LLP2.reset();
	EXPECT_EQ(0.0, LLP2.maxDelta());
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
	EXPECT_EQ(INFINITY, LPF3.getTs());
	EXPECT_EQ(0.0, LPF3.getAlpha());
	LPF3.setTs(-1.0/0.0); // -Inf
	EXPECT_EQ(1.0, LPF3.value());
	EXPECT_EQ(INFINITY, LPF3.getTs());
	EXPECT_EQ(0.0, LPF3.getAlpha());
	LPF3.setTs(0.0/0.0); // NaN
	EXPECT_EQ(1.0, LPF3.value());
	EXPECT_EQ(INFINITY, LPF3.getTs());
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

// Test the MeanFilter class
TEST(RCUtilsMiscTest, testMeanFilter)
{
	// Declare variables
	const MeanFilter::Buffer* buf;
	
	// Test construction
	MeanFilter MF1;
	buf = &MF1.buf();
	EXPECT_EQ(0, MF1.numPoints());
	EXPECT_EQ(0, buf->capacity());
	EXPECT_EQ(0, buf->size());
	EXPECT_EQ(0.0, MF1.value());
	MF1.put(5.0);
	EXPECT_EQ(0, MF1.numPoints());
	EXPECT_EQ(0, buf->capacity());
	EXPECT_EQ(0, buf->size());
	EXPECT_EQ(0.0, MF1.value());
	MeanFilter MF2(5);
	buf = &MF2.buf();
	EXPECT_EQ(5, MF2.numPoints());
	EXPECT_EQ(5, buf->capacity());
	EXPECT_EQ(5, buf->size());
	EXPECT_EQ(0.0, MF2.value());
	for(size_t i = 0; i < buf->size(); i++)
		EXPECT_EQ(0.0, buf->at(i));
	
	// Test reset all
	MF2.resetAll();
	EXPECT_EQ(0, MF2.numPoints());
	EXPECT_EQ(0, buf->capacity());
	EXPECT_EQ(0, buf->size());
	EXPECT_EQ(0.0, MF2.value());
	
	// Test resizing and putting
	MF2.resize(5, 3.0);            // 3 3 3 3 3 NOW
	EXPECT_EQ(5, MF2.numPoints());
	EXPECT_EQ(5, buf->capacity());
	EXPECT_EQ(5, buf->size());
	EXPECT_EQ(3.0, MF2.value());
	for(size_t i = 0; i < buf->size(); i++)
		EXPECT_EQ(3.0, buf->at(i));
	MF2.resize(8);                 // 0 0 0 3 3 3 3 3 NOW
	EXPECT_EQ(8, MF2.numPoints());
	EXPECT_EQ(8, buf->capacity());
	EXPECT_EQ(8, buf->size());
	EXPECT_EQ(1.875, MF2.value());
	for(size_t i = 0; i < 5; i++)
		EXPECT_EQ(3.0, buf->at(i));
	for(size_t i = 5; i < buf->size(); i++)
		EXPECT_EQ(0.0, buf->at(i));
	MF2.put(5.0);
	EXPECT_EQ(2.5, MF2.value());   // 0 0 3 3 3 3 3 5 NOW
	MF2.put(5.0);
	EXPECT_EQ(3.125, MF2.value()); // 0 3 3 3 3 3 5 5 NOW
	MF2.put(5.0);
	EXPECT_EQ(3.75, MF2.value());  // 3 3 3 3 3 5 5 5 NOW
	MF2.put(5.0);
	EXPECT_EQ(4.0, MF2.value());   // 3 3 3 3 5 5 5 5 NOW
	MF2.resize(2);                 // 5 5 NOW
	EXPECT_EQ(2, MF2.numPoints());
	EXPECT_EQ(2, buf->capacity());
	EXPECT_EQ(2, buf->size());
	EXPECT_EQ(5.0, MF2.value());
	EXPECT_EQ(5.0, buf->at(0));
	EXPECT_EQ(5.0, buf->at(1));
	
	// Test reset
	MF2.reset();
	EXPECT_EQ(2, MF2.numPoints());
	EXPECT_EQ(2, buf->capacity());
	EXPECT_EQ(2, buf->size());
	EXPECT_EQ(0.0, MF2.value());
	
	// Test setting the buffer
	std::vector<double> data;
	data.push_back(1.0);
	data.push_back(3.0);
	data.push_back(2.0);
	data.push_back(5.0);
	data.push_back(4.0);
	MF2.setBuf(data.begin(), data.end()); // 3 1 NOW
	EXPECT_EQ(2, MF2.numPoints());
	EXPECT_EQ(2, buf->capacity());
	EXPECT_EQ(2, buf->size());
	EXPECT_EQ(2.0, MF2.value());
	MF2.resize(8);
	MF2.setBuf(data.begin(), data.end()); // 0 0 0 4 5 2 3 1 NOW
	EXPECT_EQ(8, MF2.numPoints());
	EXPECT_EQ(8, buf->capacity());
	EXPECT_EQ(8, buf->size());
	EXPECT_EQ(1.875, MF2.value());
	MF2.put(9.0);                         // 0 0 4 5 2 3 1 9 NOW
	EXPECT_EQ(3.0, MF2.value());
	MF2.resize(5);
	MF2.setBuf(data.begin(), data.end()); // 4 5 2 3 1 NOW
	EXPECT_EQ(5, MF2.numPoints());
	EXPECT_EQ(5, buf->capacity());
	EXPECT_EQ(5, buf->size());
	EXPECT_EQ(3.0, MF2.value());
	MF2.put(9.0);                         // 5 2 3 1 9 NOW
	EXPECT_EQ(4.0, MF2.value());
	MF2.setZero();
	EXPECT_EQ(5, MF2.numPoints());
	EXPECT_EQ(5, buf->capacity());
	EXPECT_EQ(5, buf->size());
	EXPECT_EQ(0.0, MF2.value());
	for(size_t i = 0; i < buf->size(); i++)
		EXPECT_EQ(0.0, buf->at(i));
	MF2.setValue(-1.0);
	EXPECT_EQ(5, MF2.numPoints());
	EXPECT_EQ(5, buf->capacity());
	EXPECT_EQ(5, buf->size());
	EXPECT_EQ(-1.0, MF2.value());
	for(size_t i = 0; i < buf->size(); i++)
		EXPECT_EQ(-1.0, buf->at(i));
}

// Test the SharpDeadband class
TEST(RCUtilsMiscTest, testSharpDeadband)
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
	EXPECT_DOUBLE_EQ(-0.7, SharpDeadband::eval(-0.5, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-0.2, SharpDeadband::eval( 0.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.0, SharpDeadband::eval( 0.5, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.0, SharpDeadband::eval( 1.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.5, SharpDeadband::eval( 1.5, 0.4, 0.6));
}

// Test the SmoothDeadband class
TEST(RCUtilsMiscTest, testSmoothDeadband)
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
	EXPECT_DOUBLE_EQ(-0.70000, SmoothDeadband::eval(-0.5, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-0.22500, SmoothDeadband::eval( 0.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ(-0.00625, SmoothDeadband::eval( 0.5, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.10000, SmoothDeadband::eval( 1.0, 0.4, 0.6));
	EXPECT_DOUBLE_EQ( 0.50000, SmoothDeadband::eval( 1.5, 0.4, 0.6));
}

// Test the SlopeLimiter class
TEST(RCUtilsMiscTest, testSlopeLimiter)
{
	// Test construction 
	SlopeLimiter SL;
	EXPECT_EQ( 0.0, SL.maxDelta());
	EXPECT_EQ( 0.0, SL.value());
	SlopeLimiter SL2(0.4, -0.6);
	EXPECT_EQ( 0.4, SL2.maxDelta());
	EXPECT_EQ(-0.6, SL2.value());
	SlopeLimiter SL3(-0.4);
	EXPECT_EQ( 0.4, SL3.maxDelta());
	EXPECT_EQ( 0.0, SL3.value());
	
	// Test reset and set
	SL2.reset();
	EXPECT_EQ( 0.0, SL2.maxDelta());
	EXPECT_EQ( 0.0, SL2.value());
	SL.set(0.9, 1.7);
	EXPECT_EQ( 0.9, SL.maxDelta());
	EXPECT_EQ( 1.7, SL.value());
	SL.set(-1.9);
	EXPECT_EQ( 1.9, SL.maxDelta());
	EXPECT_EQ( 0.0, SL.value());
	SL.setValue(51311);
	EXPECT_EQ(51311, SL.value());
	
	// Test put
	SL.set(0.1, 3.0);
	EXPECT_EQ( 0.1, SL.maxDelta());
	EXPECT_EQ( 3.0, SL.value());
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

// Test the WLBFFilter class
TEST(RCUtilsMiscTest, testWLBFFilter)
{
	// Create WLBF filter
	WLBFFilter WF(5);
	
	// Add some data points
	WF.addXYW(1.0, 2.0, 0.0);
	WF.addXYW(-1.0, 2.0, 0.5);
	WF.addXYW(-3.0, 2.0, 1.0);
	WF.addXYW(1.0, 4.0, 1.5);
	WF.addXYW(2.0, 2.0, 2.0);
	WF.addXYW(1.0, 7.0, 2.5);
	WF.addXYW(-4.0, 2.0, 3.0);
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(5, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ(-4.0, WF.xBuf()[0]);
	EXPECT_EQ( 2.0, WF.xBuf()[2]);
	EXPECT_EQ(-3.0, WF.xBuf()[4]);
	EXPECT_EQ( 7.0, WF.yBuf()[1]);
	EXPECT_EQ( 4.0, WF.yBuf()[3]);
	EXPECT_EQ( 9.0, WF.wBuf()[0]);
	EXPECT_EQ( 6.25, WF.wBuf()[1]);
	EXPECT_EQ( 1.0, WF.wBuf()[4]);
	EXPECT_DOUBLE_EQ(4.0501792114695343, WF.getA());
	EXPECT_DOUBLE_EQ(0.4612903225806452, WF.getB());
	EXPECT_DOUBLE_EQ(2.2050179211469536, WF.value());
	EXPECT_DOUBLE_EQ(0.4612903225806452, WF.deriv());
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Truncate some elements
	WF.resize(3);
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(3, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ(-4.0, WF.xBuf()[0]);
	EXPECT_EQ( 2.0, WF.xBuf()[2]);
	EXPECT_EQ( 7.0, WF.yBuf()[1]);
	EXPECT_EQ( 9.0, WF.wBuf()[0]);
	EXPECT_EQ( 6.25, WF.wBuf()[1]);
	EXPECT_DOUBLE_EQ(4.1541846182051518, WF.getA());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.getB());
	EXPECT_DOUBLE_EQ(2.2750022916857642, WF.value());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.deriv());
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Pad out some elements
	WF.resize(6);
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(6, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ(-4.0, WF.xBuf()[0]);
	EXPECT_EQ( 2.0, WF.xBuf()[2]);
	EXPECT_EQ( 0.0, WF.xBuf()[4]);
	EXPECT_EQ( 7.0, WF.yBuf()[1]);
	EXPECT_EQ( 0.0, WF.yBuf()[3]);
	EXPECT_EQ( 9.0, WF.wBuf()[0]);
	EXPECT_EQ( 6.25, WF.wBuf()[1]);
	EXPECT_EQ( 0.0, WF.wBuf()[5]);
	EXPECT_DOUBLE_EQ(4.1541846182051518, WF.getA());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.getB());
	EXPECT_DOUBLE_EQ(2.2750022916857642, WF.value());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.deriv());
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Add some data back in again
	WF.addXYW(1.0, 2.0, 0.0);
	WF.addXYW(-1.0, 2.0, 0.5);
	WF.addXYW(-3.0, 2.0, 1.0);
	WF.setZeroY();
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(6, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ(-3.0, WF.xBuf()[0]);
	EXPECT_EQ( 1.0, WF.xBuf()[2]);
	EXPECT_EQ( 1.0, WF.xBuf()[4]);
	EXPECT_EQ( 0.0, WF.yBuf()[1]);
	EXPECT_EQ( 0.0, WF.yBuf()[2]);
	EXPECT_EQ( 0.0, WF.yBuf()[3]);
	EXPECT_EQ( 1.0, WF.wBuf()[0]);
	EXPECT_EQ( 0.25, WF.wBuf()[1]);
	EXPECT_EQ( 4.0, WF.wBuf()[5]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Set the buffers manually
	double tmpx[3] = {0.2, 0.7, 0.3};
	double tmpy[3] = {100, -42, 17};
	double tmpw[3] = {0.666, 0.777, 0.333};
	WF.setXBuf(tmpx, tmpx + 3);
	WF.setYBuf(tmpy, tmpy + 2);
	WF.setWBuf(tmpw, tmpw + 3);
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(6, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ( 0.2, WF.xBuf()[0]);
	EXPECT_EQ( 0.3, WF.xBuf()[2]);
	EXPECT_EQ( 0.0, WF.xBuf()[4]);
	EXPECT_EQ( -42, WF.yBuf()[1]);
	EXPECT_EQ( 0.0, WF.yBuf()[2]);
	EXPECT_EQ( 0.0, WF.yBuf()[3]);
	EXPECT_EQ( 0.666*0.666, WF.wBuf()[0]);
	EXPECT_EQ( 0.777*0.777, WF.wBuf()[1]);
	EXPECT_EQ( 0.0, WF.wBuf()[5]);
	EXPECT_NEAR( 140.5272727272729, WF.getA(), 1e-12);
	EXPECT_NEAR(-263.9720279720282, WF.getB(), 1e-12);
	EXPECT_NEAR(  87.7328671328672, WF.value(), 1e-12);
	EXPECT_NEAR(-263.9720279720282, WF.deriv(), 1e-12);
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Set the x and w buffers
	WF.setZeroX();
	WF.setEqualW();
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(6, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ( 0.0, WF.xBuf()[0]);
	EXPECT_EQ( 0.0, WF.xBuf()[2]);
	EXPECT_EQ( 0.0, WF.xBuf()[4]);
	EXPECT_EQ( -42, WF.yBuf()[1]);
	EXPECT_EQ( 0.0, WF.yBuf()[2]);
	EXPECT_EQ( 0.0, WF.yBuf()[3]);
	EXPECT_EQ( 1.0, WF.wBuf()[0]);
	EXPECT_EQ( 1.0, WF.wBuf()[1]);
	EXPECT_EQ( 1.0, WF.wBuf()[5]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Reduce the size and push back elements
	WF.resize(4);
	WF.addXYW(1.0, 2.0, 0.0);
	WF.addXYW(-1.0, 7.0, 0.5);
	WF.addXYW(-3.0, 1.0, 1.0);
	WF.addXYW(2.0, 3.0, 0.0);
	WF.addXYW(-2.0, 1.0, 0.5);
	WF.addXYW(-4.0, 4.0, 1.0);
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(4, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ(-4.0, WF.xBuf()[0]);
	EXPECT_EQ( 2.0, WF.xBuf()[2]);
	EXPECT_EQ(-3.0, WF.xBuf()[3]);
	EXPECT_EQ( 1.0, WF.yBuf()[1]);
	EXPECT_EQ( 3.0, WF.yBuf()[2]);
	EXPECT_EQ( 1.0, WF.yBuf()[3]);
	EXPECT_EQ( 1.0, WF.wBuf()[0]);
	EXPECT_EQ(0.25, WF.wBuf()[1]);
	EXPECT_EQ( 1.0, WF.wBuf()[3]);
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Set the init XY
	WF.resize(5);
	WF.setInitXY(0.3, 0.7);
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(5, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ( 0.3, WF.xBuf()[0]);
	EXPECT_EQ( 0.3, WF.xBuf()[2]);
	EXPECT_EQ( 0.3, WF.xBuf()[3]);
	EXPECT_EQ( 0.7, WF.yBuf()[1]);
	EXPECT_EQ( 0.7, WF.yBuf()[2]);
	EXPECT_EQ( 0.7, WF.yBuf()[4]);
	EXPECT_EQ( 1.0, WF.wBuf()[0]);
	EXPECT_EQ(0.25, WF.wBuf()[1]);
	EXPECT_EQ( 1.0, WF.wBuf()[3]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Set the init XYW
	WF.resize(5);
	WF.setInitXYW(0.5, 0.1, 2.0);
	WF.updateAB();
	
	// Testing
	EXPECT_EQ(5, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ( 0.5, WF.xBuf()[0]);
	EXPECT_EQ( 0.5, WF.xBuf()[2]);
	EXPECT_EQ( 0.5, WF.xBuf()[3]);
	EXPECT_EQ( 0.1, WF.yBuf()[1]);
	EXPECT_EQ( 0.1, WF.yBuf()[2]);
	EXPECT_EQ( 0.1, WF.yBuf()[4]);
	EXPECT_EQ( 4.0, WF.wBuf()[0]);
	EXPECT_EQ( 4.0, WF.wBuf()[1]);
	EXPECT_EQ( 4.0, WF.wBuf()[4]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
	
	// Reset the filter
	WF.reset();
	
	// Testing
	EXPECT_EQ(5, WF.size());
	EXPECT_EQ(WF.xBuf().capacity(), WF.xBuf().size());
	EXPECT_EQ(WF.yBuf().capacity(), WF.yBuf().size());
	EXPECT_EQ(WF.wBuf().capacity(), WF.wBuf().size());
	EXPECT_EQ( 0.0, WF.xBuf()[0]);
	EXPECT_EQ( 0.0, WF.xBuf()[2]);
	EXPECT_EQ( 0.0, WF.xBuf()[3]);
	EXPECT_EQ( 0.0, WF.yBuf()[1]);
	EXPECT_EQ( 0.0, WF.yBuf()[2]);
	EXPECT_EQ( 0.0, WF.yBuf()[4]);
	EXPECT_EQ( 0.0, WF.wBuf()[0]);
	EXPECT_EQ( 0.0, WF.wBuf()[1]);
	EXPECT_EQ( 0.0, WF.wBuf()[4]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());
	
	// Print testing
#if PRINT_TESTING
	std::cout << "PRINTING" << std::endl;
	for(size_t i = 0; i < WF.size(); i++)
		std::cout << "[" << i << "] = (" << WF.xBuf()[i] << ", " << WF.yBuf()[i] << ", " << WF.wBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
}

// Test the Boost circular buffer class for an important property
TEST(RCUtilsMiscTest, testBoostCircularBuffer)
{
	// Declare variables
	boost::circular_buffer<int> buf;
	
	// Test push_back/rset_capacity behaviour
	buf.clear();
	buf.rset_capacity(5);
	EXPECT_EQ(0, buf.size());
	EXPECT_EQ(5, buf.capacity());
	buf.push_back(1);
	EXPECT_EQ(1, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(1, buf[0]);
	buf.push_back(2);
	buf.push_back(3);
	buf.push_back(4);
	buf.push_back(5);
	EXPECT_EQ(5, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(1, buf[0]);
	EXPECT_EQ(2, buf[1]);
	EXPECT_EQ(3, buf[2]);
	EXPECT_EQ(4, buf[3]);
	EXPECT_EQ(5, buf[4]);
	buf.push_back(6);
	buf.push_back(7);
	EXPECT_EQ(5, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(3, buf[0]);
	EXPECT_EQ(4, buf[1]);
	EXPECT_EQ(5, buf[2]);
	EXPECT_EQ(6, buf[3]);
	EXPECT_EQ(7, buf[4]);
	buf.rset_capacity(3); // Note: This should retain the three most recent values pushed back
	EXPECT_EQ(3, buf.size());
	EXPECT_EQ(3, buf.capacity());
	EXPECT_EQ(5, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(7, buf[2]);
	buf.rset_capacity(5);
	EXPECT_EQ(3, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(5, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(7, buf[2]);
	
	// Test push_front/set_capacity behaviour
	buf.clear();
	buf.set_capacity(5);
	EXPECT_EQ(0, buf.size());
	EXPECT_EQ(5, buf.capacity());
	buf.push_front(1);
	EXPECT_EQ(1, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(1, buf[0]);
	buf.push_front(2);
	buf.push_front(3);
	buf.push_front(4);
	buf.push_front(5);
	EXPECT_EQ(5, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(5, buf[0]);
	EXPECT_EQ(4, buf[1]);
	EXPECT_EQ(3, buf[2]);
	EXPECT_EQ(2, buf[3]);
	EXPECT_EQ(1, buf[4]);
	buf.push_front(6);
	buf.push_front(7);
	EXPECT_EQ(5, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(7, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(5, buf[2]);
	EXPECT_EQ(4, buf[3]);
	EXPECT_EQ(3, buf[4]);
	buf.set_capacity(3); // Note: This should retain the three most recent values pushed front
	EXPECT_EQ(3, buf.size());
	EXPECT_EQ(3, buf.capacity());
	EXPECT_EQ(7, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(5, buf[2]);
	buf.set_capacity(5);
	EXPECT_EQ(3, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(7, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(5, buf[2]);
}

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF