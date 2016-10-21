// Unit test for the GolayDerivative and GolayFilter classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Max Schwarz <max.schwarz@uni-bonn.de>

// Includes
#include <rc_utils/golay.h>
#include <rc_utils/golayfilter.h>
#include <gtest/gtest.h>

// Namespaces
using namespace rc_utils;

//
// GolayDerivative class
//

// GolayDerivative: Test a buffer that is not full
TEST(GolayDerivativeTest, testEmpty)
{
	// Create GolayDerivative object
	GolayDerivative<double, 1, 5> d;

	// Test that the values only start changing when the buffer is full
	ASSERT_DOUBLE_EQ(0.0, d.value());
	d.put(1.0);
	ASSERT_DOUBLE_EQ(0.0, d.value());
	d.put(2.0);
	ASSERT_DOUBLE_EQ(0.0, d.value());
	d.put(2.0);
	ASSERT_DOUBLE_EQ(0.0, d.value());
	d.put(1.0);
	ASSERT_DOUBLE_EQ(0.0, d.value());
	d.put(0.0); // Buffer is now full (5th value)
	if(d.value() == 0.0) FAIL();
}

// GolayDerivative: Test a window size of 1 (no filtering at all)
TEST(GolayDerivativeTest, testWindowSizeOne)
{
	// Create GolayDerivative object
	GolayDerivative<double, 0, 1> d;

	// Test that all values get passed directly through
	ASSERT_DOUBLE_EQ(0.0, d.value());
	d.put(1.0);
	ASSERT_DOUBLE_EQ(1.0, d.value());
	d.put(2.0);
	ASSERT_DOUBLE_EQ(2.0, d.value());
	d.put(3.1415);
	ASSERT_DOUBLE_EQ(3.1415, d.value());
	d.put(1.60e-19);
	ASSERT_DOUBLE_EQ(1.60e-19, d.value());
	d.put(0.0);
	ASSERT_DOUBLE_EQ(0.0, d.value());
}

// GolayDerivative: Test the first derivative
TEST(GolayDerivativeTest, testFirstDerivative)
{
	// Create GolayDerivative object
	GolayDerivative<double, 1, 9> d;

	// Test the first derivative of a linear ramp
	for(int i = 0; i < 9; i++) d.put(i);
	ASSERT_DOUBLE_EQ(1.0, d.value());
}

// GolayDerivative: Test the second derivative
TEST(GolayDerivativeTest, testSecondDerivative)
{
	// Create GolayDerivative object
	GolayDerivative<double, 2, 9> d;

	// Test the second derivative of a linear ramp
	for(int i = 0; i < 9; i++) d.put(i);
	ASSERT_DOUBLE_EQ(0.0, d.value());

	// Test the second derivative of a quadratic
	for(int i = 0; i < 9; i++) d.put(i*i);
	ASSERT_DOUBLE_EQ(2.0, d.value());
}

//
// GolayFilter class
//

// GolayFilter: Test the resetting of the filter
TEST(GolayFilterTest, testReset)
{
	// Create GolayFilter object
	GolayFilter<9> filter;

	// Test default conditions
	EXPECT_DOUBLE_EQ(1.0, filter.dt());
	EXPECT_DOUBLE_EQ(0.0, filter.x());
	EXPECT_DOUBLE_EQ(0.0, filter.v());

	// Test custom conditions
	filter.reset(0.2, 1.0, -2.0);
	EXPECT_DOUBLE_EQ(0.2, filter.dt());
	EXPECT_DOUBLE_EQ(1.0 - 4*(-2.0)*0.2, filter.x());
	EXPECT_DOUBLE_EQ(-2.0, filter.v());
	filter.reset(0.5, -0.4, 1.7);
	EXPECT_DOUBLE_EQ(0.5, filter.dt());
	EXPECT_DOUBLE_EQ(-0.4 - 4*(1.7)*0.5, filter.x());
	EXPECT_DOUBLE_EQ(1.7, filter.v());

	// Test invalid time increment
	filter.reset(0.7);
	EXPECT_DOUBLE_EQ(0.7, filter.dt());
	filter.reset(0.0);
	EXPECT_DOUBLE_EQ(1.0, filter.dt());
	filter.reset(0.3);
	EXPECT_DOUBLE_EQ(0.3, filter.dt());
	filter.reset(-3.0);
	EXPECT_DOUBLE_EQ(1.0, filter.dt());
}

// GolayFilter: Test updating of the filter
TEST(GolayFilterTest, testUpdate)
{
	// Create GolayFilter object
	GolayFilter<9> filter;

	// Test updating with a linear ramp
	filter.reset(0.1, 0.0, 10.0);
	EXPECT_DOUBLE_EQ(0.1, filter.dt());
	EXPECT_DOUBLE_EQ(0.0 - 4*10.0*0.1, filter.x());
	EXPECT_DOUBLE_EQ(10.0, filter.v());
	for(int i = 1; i <= 9; i++)
	{
		filter.update(i);
		EXPECT_DOUBLE_EQ(i - 4, filter.x());
		EXPECT_DOUBLE_EQ(10.0, filter.v());
	}
}

// GolayFilter: Test shifting of the filter
TEST(GolayFilterTest, testShift)
{
	// Create GolayFilter object
	GolayFilter<9> filtera, filterb, filterc;

	// Reset the filters
	filtera.reset(0.1, 0.4, 2.0);
	filterb.reset(0.1, 0.4, 2.0);
	filterc.reset(0.1, 0.4, 2.0);
	EXPECT_DOUBLE_EQ(0.1, filtera.dt());
	EXPECT_DOUBLE_EQ(0.4 - 4*2.0*0.1, filtera.x());
	EXPECT_DOUBLE_EQ(2.0, filtera.v());

	// Update with a few data points
	filtera.update(0.55);filterb.update(0.55);filterc.update(0.55);
	filtera.update(0.75);filterb.update(0.75);filterc.update(0.75);
	filtera.update(0.85);filterb.update(0.85);filterc.update(0.85);
	filtera.update(1.00);filterb.update(1.00);filterc.update(1.00);
	EXPECT_DOUBLE_EQ(filtera.x(), filterb.x());
	EXPECT_DOUBLE_EQ(filtera.x(), filterc.x());
	EXPECT_DOUBLE_EQ(filtera.v(), filterb.v());
	EXPECT_DOUBLE_EQ(filtera.v(), filterc.v());

	// Shift filters b and c
	filterb.shiftPosBy(2.0);
	filterc.shiftPosTo(2.0);
	EXPECT_DOUBLE_EQ(filtera.x() + 2.0, filterb.x());
	EXPECT_DOUBLE_EQ(filtera.x() + 1.0, filterc.x());
	EXPECT_DOUBLE_EQ(filtera.v(), filterb.v());
	EXPECT_DOUBLE_EQ(filtera.v(), filterc.v());

	// Update with a few data points
	filtera.update(1.00);filterb.update(1.00 + 2.0);filterc.update(1.00 + 1.0);
	filtera.update(0.85);filterb.update(0.85 + 2.0);filterc.update(0.85 + 1.0);
	filtera.update(0.75);filterb.update(0.75 + 2.0);filterc.update(0.75 + 1.0);
	filtera.update(0.55);filterb.update(0.55 + 2.0);filterc.update(0.55 + 1.0);
	EXPECT_DOUBLE_EQ(filtera.x() + 2.0, filterb.x());
	EXPECT_DOUBLE_EQ(filtera.x() + 1.0, filterc.x());
	EXPECT_NEAR(filtera.v(), filterb.v(), 1e-15);
	EXPECT_NEAR(filtera.v(), filterc.v(), 1e-15);

	// Shift filters b and c back to being aligned with a
	filterb.shiftPosTo(0.55);
	filterc.shiftPosBy(-1.0);
	EXPECT_DOUBLE_EQ(filtera.x(), filterb.x());
	EXPECT_DOUBLE_EQ(filtera.x(), filterc.x());
	EXPECT_NEAR(filtera.v(), filterb.v(), 1e-15);
	EXPECT_NEAR(filtera.v(), filterc.v(), 1e-15);

	// Update with a few data points
	filtera.update(0.55);filterb.update(0.55);filterc.update(0.55);
	filtera.update(0.75);filterb.update(0.75);filterc.update(0.75);
	filtera.update(0.85);filterb.update(0.85);filterc.update(0.85);
	filtera.update(1.00);filterb.update(1.00);filterc.update(1.00);
	EXPECT_DOUBLE_EQ(filtera.x(), filterb.x());
	EXPECT_DOUBLE_EQ(filtera.x(), filterc.x());
	EXPECT_NEAR(filtera.v(), filterb.v(), 1e-15);
	EXPECT_NEAR(filtera.v(), filterc.v(), 1e-15);
}

//
// Main
//

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF