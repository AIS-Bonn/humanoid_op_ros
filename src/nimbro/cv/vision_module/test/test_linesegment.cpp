// Unit test for the LineSegment class in vision module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

// Includes
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <gtest/gtest.h>

#define TOL 0.0001


TEST(LineSegmentTest, scale)
{
	LineSegment input(Point2d(10, 10), Point2d(20, 20));

	//Sanity check
	LineSegment tmpOut = input.scale(1);

	EXPECT_NEAR(tmpOut.P1.x, 10, TOL);
	EXPECT_NEAR(tmpOut.P1.y, 10, TOL);
	EXPECT_NEAR(tmpOut.P2.x, 20, TOL);
	EXPECT_NEAR(tmpOut.P2.y, 20, TOL);

	//Pos scale check
	tmpOut = input.scale(1.2);

	EXPECT_NEAR(tmpOut.P1.x, 9, TOL);
	EXPECT_NEAR(tmpOut.P1.y, 9, TOL);
	EXPECT_NEAR(tmpOut.P2.x, 21, TOL);
	EXPECT_NEAR(tmpOut.P2.y, 21, TOL);

	//Neg scale check
	tmpOut = input.scale(0.8);

	EXPECT_NEAR(tmpOut.P1.x, 11, TOL);
	EXPECT_NEAR(tmpOut.P1.y, 11, TOL);
	EXPECT_NEAR(tmpOut.P2.x, 19, TOL);
	EXPECT_NEAR(tmpOut.P2.y, 19, TOL);
}

TEST(LineSegmentTest, constructor)
{
	LineSegment input(Point2d(10, 10), Degree2Radian(0), 10);

	EXPECT_NEAR(input.P1.x, 15, TOL);
	EXPECT_NEAR(input.P1.y, 10, TOL);
	EXPECT_NEAR(input.P2.x, 5, TOL);
	EXPECT_NEAR(input.P2.y, 10, TOL);
	EXPECT_NEAR(input.GetLength(), 10, TOL);

	input = LineSegment(Point2d(10, 10), Degree2Radian(90), 10);

	EXPECT_NEAR(input.P1.x, 10, TOL);
	EXPECT_NEAR(input.P1.y, 15, TOL);
	EXPECT_NEAR(input.P2.x, 10, TOL);
	EXPECT_NEAR(input.P2.y, 5, TOL);
	EXPECT_NEAR(input.GetLength(), 10, TOL);

	input = LineSegment(Point2d(0,0), Degree2Radian(45), 10);

	EXPECT_NEAR(input.P1.x, input.P1.y, TOL);
	EXPECT_NEAR(input.P2.x, input.P2.y, TOL);
	EXPECT_NEAR(input.GetLength(), 10, TOL);

	input = LineSegment(Point2d(0,0), Degree2Radian(-45), 10);

	EXPECT_NEAR(input.P1.x, -input.P1.y, TOL);
	EXPECT_NEAR(input.P2.x, -input.P2.y, TOL);
	EXPECT_NEAR(input.GetLength(), 10, TOL);
}

TEST(LineSegmentTest, intersectLineForm)
{

	LineSegment inputVer(Point2d(0, 0), Point2d(0,0));
	LineSegment input(Point2d(10, 10), Point2d(20, 20));

	//Sanity check
	double inputX=10;
	double predictedY=10;
	Point2d outRes;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	inputX=20;
	predictedY=20;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	//Inside range check
	inputX=15;
	predictedY=15;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	inputX=16;
	predictedY=16;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	//Outside range check
	inputX=0;
	predictedY=0;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	inputX=-10;
	predictedY=-10;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	inputX=40;
	predictedY=40;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	input=LineSegment(Point2d(10, 5), Point2d(20, 10));

	//Sanity check
	inputX=10;
	predictedY=5;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	inputX=20;
	predictedY=10;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,5));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);


	//Inside range check
	inputX=15;
	predictedY=7.5;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,5));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);


	input=LineSegment( Point2d(1, 0.5),Point2d(30, 0.9));

	//Sanity check
	inputX=30;
	predictedY=0.9;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	inputX=1;
	predictedY=0.5;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);

	//Inside range check
	inputX=15.5;
	predictedY=0.7;
	inputVer=LineSegment(Point2d(inputX, 0), Point2d(inputX,20));
	EXPECT_TRUE(input.IntersectLineForm(inputVer,outRes));
	EXPECT_NEAR(outRes.x,inputX,TOL);
	EXPECT_NEAR(outRes.y,predictedY,TOL);//TODO: check why 0.000001 to fails
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
