// Unit test for the LinearInterpolator class in vision module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

// Includes
#include <vision_module/Tools/LinearInterpolator.hpp>
#include <gtest/gtest.h>

#define TOL 0.000001


TEST(LinearInterpolatorTest, Interpolate)
{
	LinearInterpolator input(Point2d(10, 10), Point2d(20, 20));

	//Sanity check
	EXPECT_NEAR(input.Interpolate(10),10,TOL);
	EXPECT_NEAR(input.Interpolate(20),20,TOL);

	//Inside range check
	EXPECT_NEAR(input.Interpolate(15),15,TOL);
	EXPECT_NEAR(input.Interpolate(16),16,TOL);

	//Outside range check
	EXPECT_NEAR(input.Interpolate(0),0,TOL);
	EXPECT_NEAR(input.Interpolate(-10),-10,TOL);
	EXPECT_NEAR(input.Interpolate(40),40,TOL);

	input=LinearInterpolator(Point2d(10, 5), Point2d(20, 10));

	//Sanity check
	EXPECT_NEAR(input.Interpolate(10),5,TOL);
	EXPECT_NEAR(input.Interpolate(20),10,TOL);

	//Inside range check
	EXPECT_NEAR(input.Interpolate(15),7.5,TOL);

	input=LinearInterpolator( Point2d(30, 0.9),Point2d(1, 0.5));

	//Sanity check
	EXPECT_NEAR(input.Interpolate(30),0.9,TOL);
	EXPECT_NEAR(input.Interpolate(1),0.5,TOL);

	//Inside range check
	EXPECT_NEAR(input.Interpolate(15.5),0.7,TOL);
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
