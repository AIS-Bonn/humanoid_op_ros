// Unit test for miscellaneous NimbRo utilities
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <nimbro_utils/ew_integrator.h>
#include <nimbro_utils/lin_sin_fillet.h>
#include <nimbro_utils/slope_limiter.h>
#include <nimbro_utils/smooth_deadband.h>
#include <nimbro_utils/spike_filter.h>
#include <nimbro_utils/wlbf_filter.h>
#include <gtest/gtest.h>

// Defines
#define PRINT_TESTING 0

// Namespaces
using namespace nimbro_utils;

// Test the EWIntegrator class
TEST(NimbroUtilsMiscTest, testEWIntegrator)
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

// Test the LinSinFillet class
TEST(NimbroUtilsMiscTest, testLinSinFillet)
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

// Test the SharpDeadband class
TEST(NimbroUtilsMiscTest, testSharpDeadband)
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
TEST(NimbroUtilsMiscTest, testSmoothDeadband)
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
TEST(NimbroUtilsMiscTest, testSlopeLimiter)
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
TEST(NimbroUtilsMiscTest, testSpikeFilter)
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
TEST(NimbroUtilsMiscTest, testWLBFFilter)
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

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF