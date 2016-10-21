// Unit test for the general functions in vision module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

// Includes
#include <vision_module/Tools/General.hpp>
#include <gtest/gtest.h>

#define TOL 0.000001

TEST(GeneralVisionTest, clipRect)
{
	cv::Rect input(-100, -100, 1000, 1000);

	//Sanity check
	cv::Rect tmpOut = clipRect(input, IMGBOX);

	EXPECT_EQ(tmpOut.x, 0);
	EXPECT_EQ(tmpOut.y, 0);
	EXPECT_EQ(tmpOut.width, IMGWIDTH-1);
	EXPECT_EQ(tmpOut.height, IMGHEIGHT-1);
}

TEST(GeneralVisionTest, CircleC)
{
	CircleC input;

	input.Center.x = 0;
	input.Center.y = 0;
	input.Radius = 10;

	CircleC input2;

	input2.Center.x = 0;
	input2.Center.y = 0;
	input2.Radius = 5;

	//Sanity check
	CircleC tmpOut = mergeCircleC(input, input2);

	EXPECT_NEAR(tmpOut.Center.x, 0, TOL);
	EXPECT_NEAR(tmpOut.Center.y, 0, TOL);
	EXPECT_NEAR(tmpOut.Radius, 10, TOL);
	EXPECT_TRUE(hasIntersection(input, input2));

	tmpOut = mergeCircleC(input2, input);

	EXPECT_NEAR(tmpOut.Center.x, 0, TOL);
	EXPECT_NEAR(tmpOut.Center.y, 0, TOL);
	EXPECT_NEAR(tmpOut.Radius, 10, TOL);
	EXPECT_TRUE(hasIntersection(input2, input));

	input2.Center.x = 0;
	input2.Center.y = 0;
	input2.Radius = 10;
	tmpOut = mergeCircleC(input2, input);

	EXPECT_NEAR(tmpOut.Center.x, 0, TOL);
	EXPECT_NEAR(tmpOut.Center.y, 0, TOL);
	EXPECT_NEAR(tmpOut.Radius, 10, TOL);
	EXPECT_TRUE(hasIntersection(input, input2));

	//Adjacent MyCircles
	input.Center.x = 0;
	input.Center.y = 0;
	input.Radius = 10;
	input2.Center.x = 15;
	input2.Center.y = 0;
	input2.Radius = 5;
	tmpOut = mergeCircleC(input2, input);

	EXPECT_NEAR(tmpOut.Center.x, 5, TOL);
	EXPECT_NEAR(tmpOut.Center.y, 0, TOL);
	EXPECT_NEAR(tmpOut.Radius, 15, TOL);
	EXPECT_TRUE(hasIntersection(input, input2));

	//Inside MyCircles
	input.Center.x = 0;
	input.Center.y = 0;
	input.Radius = 10;
	input2.Center.x = 1;
	input2.Center.y = 1;
	input2.Radius = 5;
	tmpOut = mergeCircleC(input2, input);

	EXPECT_NEAR(tmpOut.Center.x, 0, TOL);
	EXPECT_NEAR(tmpOut.Center.y, 0, TOL);
	EXPECT_NEAR(tmpOut.Radius, 10, TOL);
	EXPECT_TRUE(hasIntersection(input, input2));

	//Have Common Part MyCircles
	input.Center.x = 0;
	input.Center.y = 0;
	input.Radius = 10;
	input2.Center.x = 12;
	input2.Center.y = 0;
	input2.Radius = 3;
	tmpOut = mergeCircleC(input2, input);

	EXPECT_NEAR(tmpOut.Center.x, 2.5, TOL);
	EXPECT_NEAR(tmpOut.Center.y, 0, TOL);
	EXPECT_NEAR(tmpOut.Radius, 12.5, TOL);
	EXPECT_TRUE(hasIntersection(input, input2));

	input.Center.x = 0;
	input.Center.y = 5;
	input.Radius = 10;
	input2.Center.x = 12;
	input2.Center.y = 5;
	input2.Radius = 3;
	tmpOut = mergeCircleC(input2, input);

	EXPECT_NEAR(tmpOut.Center.x, 2.5, TOL);
	EXPECT_NEAR(tmpOut.Center.y, 5, TOL);
	EXPECT_NEAR(tmpOut.Radius, 12.5, TOL);
	EXPECT_TRUE(hasIntersection(input, input2));

	//Far MyCircles
	input.Center.x = 0;
	input.Center.y = 0;
	input.Radius = 10;
	input2.Center.x = 20;
	input2.Center.y = 0;
	input2.Radius = 5;
	tmpOut = mergeCircleC(input2, input);

	EXPECT_NEAR(tmpOut.Center.x, 7.5, TOL);
	EXPECT_NEAR(tmpOut.Center.y, 0, TOL);
	EXPECT_NEAR(tmpOut.Radius, 17.5, TOL);
	EXPECT_FALSE(hasIntersection(input, input2));

}

TEST(GeneralVisionTest, lineFormIntersectionWithCircleC)
{
	CircleC inputC;

	inputC.Center.x = 0;
	inputC.Center.y = 0;
	inputC.Radius = 10;

	Point2f pr1, pr2;

	LineSegment inputL(Point2d(0, 0), Point2d(5, 0));

	EXPECT_TRUE(lineFormIntersectionWithCircleC(inputL, inputC, pr1, pr2));
	EXPECT_NEAR(abs(pr1.x), 10, TOL);
	EXPECT_NEAR(pr1.y, 0, TOL);
	EXPECT_NEAR(abs(pr2.x), 10, TOL);
	EXPECT_NEAR(pr2.y, 0, TOL);
	EXPECT_NEAR(-pr2.y, pr1.y, TOL);

	inputL=LineSegment(Point2d(0, 0), Point2d(0, 10));

	EXPECT_TRUE(lineFormIntersectionWithCircleC(inputL, inputC, pr1, pr2));
	EXPECT_NEAR(pr1.x, 0, TOL);
	EXPECT_NEAR(abs(pr1.y), 10, TOL);
	EXPECT_NEAR(pr2.x, 0, TOL);
	EXPECT_NEAR(abs(pr2.y), 10, TOL);
	EXPECT_NEAR(-pr2.y, pr1.y, TOL);

	inputL=LineSegment(Point2d(10, 0), Point2d(10, 10));

	EXPECT_TRUE(lineFormIntersectionWithCircleC(inputL, inputC, pr1, pr2));
	EXPECT_NEAR(pr1.x,10, TOL);
	EXPECT_NEAR(pr1.y, 0, TOL);
	EXPECT_NEAR(pr2.x, 10, TOL);
	EXPECT_NEAR(pr2.y, 0, TOL);

	inputL=LineSegment(Point2d(11, 0), Point2d(11, 10));

	EXPECT_FALSE(lineFormIntersectionWithCircleC(inputL, inputC, pr1, pr2));


	inputL=LineSegment(Point2d(-3,7.0710678100585938), Point2d(3,7.0710678100585938));

	EXPECT_TRUE(lineFormIntersectionWithCircleC(inputL, inputC, pr1, pr2));
	EXPECT_NEAR(abs(pr1.x),7.0710678100585938, TOL);
	EXPECT_NEAR(pr1.y, 7.0710678100585938, TOL);
	EXPECT_NEAR(abs(pr2.x),7.0710678100585938, TOL);
	EXPECT_NEAR(pr2.y, 7.0710678100585938, TOL);
	EXPECT_NEAR(-pr2.x, pr1.x, TOL);

}

TEST(GeneralVisionTest, minDimentionRect)
{
	cv::Rect input(10, 10, 1, 1);
	EXPECT_TRUE(minDimentionRect(3, input));

	EXPECT_EQ(input.x, 9);
	EXPECT_EQ(input.y, 9);
	EXPECT_EQ(input.width, 3);
	EXPECT_EQ(input.height, 3);

	input = cv::Rect(10, 10, 10, 10);
	EXPECT_FALSE(minDimentionRect(3, input));

	EXPECT_EQ(input.x, 10);
	EXPECT_EQ(input.y, 10);
	EXPECT_EQ(input.width, 10);
	EXPECT_EQ(input.height, 10);
}

TEST(GeneralVisionTest, maxDimentionRect)
{
	cv::Rect input(10, 10, 1, 1);
	EXPECT_FALSE(maxDimentionRect(3, input));

	EXPECT_EQ(input.x, 10);
	EXPECT_EQ(input.y, 10);
	EXPECT_EQ(input.width, 1);
	EXPECT_EQ(input.height, 1);

	input = cv::Rect(10, 10, 10, 10);
	EXPECT_TRUE(maxDimentionRect(5, input));

	EXPECT_EQ(input.x, 12);
	EXPECT_EQ(input.y, 12);
	EXPECT_EQ(input.width, 5);
	EXPECT_EQ(input.height, 5);
}

TEST(GeneralVisionTest, mergeRect)
{
	cv::Rect input(10, 10, 10, 10);
	cv::Rect input2(12, 12, 10, 10);

	EXPECT_TRUE(mergeRect(input, input2));

	EXPECT_EQ(input.x, 10);
	EXPECT_EQ(input.y, 10);
	EXPECT_EQ(input.width, 12);
	EXPECT_EQ(input.height, 12);

	input = cv::Rect(10, 10, 10, 10);
	input2 = cv::Rect(20, 20, 10, 10);

	EXPECT_FALSE(mergeRect(input, input2));

	EXPECT_EQ(input.x, 10);
	EXPECT_EQ(input.y, 10);
	EXPECT_EQ(input.width, 10);
	EXPECT_EQ(input.height, 10);
}

TEST(GeneralVisionTest, scaleRec)
{
	cv::Rect input(10, 10, 10, 10);

	//Sanity check
	cv::Rect tmpOut = scaleRec(1, 1, input);

	EXPECT_EQ(tmpOut.x, 10);
	EXPECT_EQ(tmpOut.y, 10);
	EXPECT_EQ(tmpOut.width, 10);
	EXPECT_EQ(tmpOut.height, 10);

	//check scale
	tmpOut = scaleRec(1.2, 1.2, input);

	EXPECT_EQ(tmpOut.x, 9);
	EXPECT_EQ(tmpOut.y, 9);
	EXPECT_EQ(tmpOut.width, 12);
	EXPECT_EQ(tmpOut.height, 12);

	//check shrink
	tmpOut = scaleRec(0.5, 0.5, input);

	EXPECT_EQ(tmpOut.x, 12);
	EXPECT_EQ(tmpOut.y, 12);
	EXPECT_EQ(tmpOut.width, 5);
	EXPECT_EQ(tmpOut.height, 5);

	//check box
	tmpOut = scaleRec(1.2, 1.2, input, cv::Rect(10, 10, 50, 10));

	EXPECT_EQ(tmpOut.x, 10);
	EXPECT_EQ(tmpOut.y, 10);
	EXPECT_EQ(tmpOut.width, 11);
	EXPECT_EQ(tmpOut.height, 9);

}

TEST(GeneralVisionTest, asymetricScaleRec)
{
	cv::Rect input(10, 10, 10, 10);

	//Sanity check
	cv::Rect tmpOut = asymetricScaleRec(1, 1, 1, 1, input);

	EXPECT_EQ(tmpOut.x, 10);
	EXPECT_EQ(tmpOut.y, 10);
	EXPECT_EQ(tmpOut.width, 10);
	EXPECT_EQ(tmpOut.height, 10);

	//check scale
	tmpOut = asymetricScaleRec(1.2, 1.4, 1.4, 1, input);

	EXPECT_EQ(tmpOut.x, 9);
	EXPECT_EQ(tmpOut.y, 8);
	EXPECT_EQ(tmpOut.width, 13);
	EXPECT_EQ(tmpOut.height, 12);

	//check box
	tmpOut = asymetricScaleRec(1.2, 1.4, 1.4, 1.2, input,
			cv::Rect(10, 10, 50, 10));

	EXPECT_EQ(tmpOut.x, 10);
	EXPECT_EQ(tmpOut.y, 10);
	EXPECT_EQ(tmpOut.width, 12);
	EXPECT_EQ(tmpOut.height, 9);

}

// Test the output of LineSegment.clip
TEST(GeneralVisionTest, clipLineSegment)
{
	LineSegment input(Point2d(-1, -1), Point2d(101, 101));

	//Sanity check
	input.Clip(cv::Rect(0, 0, 100, 100));

	EXPECT_EQ(input.P1.x, 0);
	EXPECT_EQ(input.P1.y, 0);
	EXPECT_EQ(input.P2.x, 99);
	EXPECT_EQ(input.P2.y, 99);

	//check clip
	input = LineSegment(Point2d(-1, -1), Point2d(101, 101));
	input.Clip(cv::Rect(10, 10, 5, 5));

	EXPECT_EQ(input.P1.x, 10);
	EXPECT_EQ(input.P1.y, 10);
	EXPECT_EQ(input.P2.x, 14);
	EXPECT_EQ(input.P2.y, 14);

	//check clip2
	input = LineSegment(Point2d(10, -10), Point2d(700, 800));
	input.Clip(IMGBOX);

	EXPECT_EQ(input.P1.x, 10);
	EXPECT_EQ(input.P1.y, 0);
	EXPECT_EQ(input.P2.x, IMGWIDTH-1);
	EXPECT_EQ(input.P2.y, IMGHEIGHT-1);

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
