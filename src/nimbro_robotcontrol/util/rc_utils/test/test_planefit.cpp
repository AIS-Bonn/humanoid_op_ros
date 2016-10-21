// Unit test for the PlaneFit class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/planefit.h>
#include <gtest/gtest.h>

// Namespaces
using namespace rc_utils;

// Defines
#define TOL_HIGH 1e-14
#define TOL_MED  1e-12
#define TOL_LOW  1e-10

//
// Helper functions
//

// Generate 3D plane data
void genPlaneData(PlaneFit::Points3D& P, Eigen::Vector3d& mean, size_t N, const Eigen::Vector3d& normal, const Eigen::Vector3d& point, double radius)
{
	// Resize the data point vector
	P.resize(N);

	// Generate the required data
	mean = Eigen::Vector3d::Zero();
	for(size_t i = 0; i < N; i++)
	{
		double r = radius*drand48();
		double ang = 2.0*M_PI*drand48();
		double u = r*cos(ang);
		double v = r*sin(ang);
		if(normal.z() != 0.0)
			P[i] = point + Eigen::Vector3d(u, v, -(u*normal.x() + v*normal.y())/normal.z());
		else if(normal.y() != 0.0)
			P[i] = point + Eigen::Vector3d(u, -(u*normal.x() + v*normal.z())/normal.y(), v);
		else if(normal.x() != 0.0)
			P[i] = point + Eigen::Vector3d(-(u*normal.y() + v*normal.z())/normal.x(), u, v);
		else
			P[i].setZero();
		mean += P[i];
	}
	if(N >= 1)
		mean /= N;
}

//
// Test functions
//

// Test plane fitting
TEST(PlaneFitTest, testFitPlane)
{
	// Declare variables
	Eigen::Vector3d FN1, FN2, FN3, FN4, FN5;
	Eigen::Vector3d FM1, FM2, FM3, FM4, FM5;
	Eigen::Vector4d FC1, FC2, FC3, FC4, FC5;

	// Generate test data
	Eigen::Vector3d N1(1.0, -2.0, 1.5), N2(-74.3, 105.3, 33.0), N3(-1.0, 0.0, 0.0), N4(0.0, 2.0, 0.0), N5(0.0, 0.0, 3.0);
	N1.normalize(); N2.normalize(); N3.normalize(); N4.normalize(); N5.normalize();
	PlaneFit::Points3D P1, P2, P3, P4, P5;
	Eigen::Vector3d M1, M2, M3, M4, M5;
	Eigen::Vector3d Q1(1.0+2.0*drand48(), 1.0+2.0*drand48(), 1.0+2.0*drand48());
	Eigen::Vector3d Q2(1.0+2.0*drand48(), 1.0+2.0*drand48(), 1.0+2.0*drand48());
	Eigen::Vector3d Q3(1.0+2.0*drand48(), 1.0+2.0*drand48(), 1.0+2.0*drand48());
	Eigen::Vector3d Q4(1.0+2.0*drand48(), 1.0+2.0*drand48(), 1.0+2.0*drand48());
	Eigen::Vector3d Q5(1.0+2.0*drand48(), 1.0+2.0*drand48(), 1.0+2.0*drand48());
	genPlaneData(P1, M1, 100, N1, Q1, 3.0*drand48());
	genPlaneData(P2, M2, 150, N2, Q2, 3.0*drand48());
	genPlaneData(P3, M3, 50, N3, Q3, 3.0*drand48());
	genPlaneData(P4, M4, 200, N4, Q4, 3.0*drand48());
	genPlaneData(P5, M5, 1000, N5, Q5, 3.0*drand48());

	// Sanity check the fitting errors of the generated data
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P1, N1, Q1), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P2, N2, Q2), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P3, N3, Q3), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P4, N4, Q4), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P5, N5, Q5), TOL_HIGH);

	// Test the plane fitting (C)
	FC1.setZero(); FC2.setZero(); FC3.setZero(); FC4.setZero(); FC5.setZero();
	PlaneFit::fitPlane(P1, FC1);
	PlaneFit::fitPlane(P2, FC2);
	PlaneFit::fitPlane(P3, FC3);
	PlaneFit::fitPlane(P4, FC4);
	PlaneFit::fitPlane(P5, FC5);
	EXPECT_NEAR(1.0, fabs(N1.dot(FC1.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N2.dot(FC2.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N3.dot(FC3.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N4.dot(FC4.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N5.dot(FC5.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P1, FC1), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P2, FC2), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P3, FC3), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P4, FC4), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P5, FC5), TOL_MED);

	// Test the plane fitting (CM)
	FC1.setZero(); FC2.setZero(); FC3.setZero(); FC4.setZero(); FC5.setZero();
	FM1.setZero(); FM2.setZero(); FM3.setZero(); FM4.setZero(); FM5.setZero();
	PlaneFit::fitPlane(P1, FC1, FM1);
	PlaneFit::fitPlane(P2, FC2, FM2);
	PlaneFit::fitPlane(P3, FC3, FM3);
	PlaneFit::fitPlane(P4, FC4, FM4);
	PlaneFit::fitPlane(P5, FC5, FM5);
	EXPECT_NEAR(1.0, fabs(N1.dot(FC1.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N2.dot(FC2.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N3.dot(FC3.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N4.dot(FC4.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N5.dot(FC5.head<3>().normalized())), TOL_HIGH);
	EXPECT_DOUBLE_EQ(M1.x(), FM1.x());
	EXPECT_DOUBLE_EQ(M1.y(), FM1.y());
	EXPECT_DOUBLE_EQ(M1.z(), FM1.z());
	EXPECT_DOUBLE_EQ(M2.x(), FM2.x());
	EXPECT_DOUBLE_EQ(M2.y(), FM2.y());
	EXPECT_DOUBLE_EQ(M2.z(), FM2.z());
	EXPECT_DOUBLE_EQ(M3.x(), FM3.x());
	EXPECT_DOUBLE_EQ(M3.y(), FM3.y());
	EXPECT_DOUBLE_EQ(M3.z(), FM3.z());
	EXPECT_DOUBLE_EQ(M4.x(), FM4.x());
	EXPECT_DOUBLE_EQ(M4.y(), FM4.y());
	EXPECT_DOUBLE_EQ(M4.z(), FM4.z());
	EXPECT_DOUBLE_EQ(M5.x(), FM5.x());
	EXPECT_DOUBLE_EQ(M5.y(), FM5.y());
	EXPECT_DOUBLE_EQ(M5.z(), FM5.z());
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P1, FC1), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P2, FC2), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P3, FC3), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P4, FC4), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P5, FC5), TOL_MED);

	// Test the plane fitting (N)
	FN1.setZero(); FN2.setZero(); FN3.setZero(); FN4.setZero(); FN5.setZero();
	PlaneFit::fitPlane(P1, FN1);
	PlaneFit::fitPlane(P2, FN2);
	PlaneFit::fitPlane(P3, FN3);
	PlaneFit::fitPlane(P4, FN4);
	PlaneFit::fitPlane(P5, FN5);
	EXPECT_NEAR(1.0, fabs(N1.dot(FN1.normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N2.dot(FN2.normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N3.dot(FN3.normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N4.dot(FN4.normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N5.dot(FN5.normalized())), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P1, FN1, Q1), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P2, FN2, Q2), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P3, FN3, Q3), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P4, FN4, Q4), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P5, FN5, Q5), TOL_HIGH);

	// Test the plane fitting (NM)
	FN1.setZero(); FN2.setZero(); FN3.setZero(); FN4.setZero(); FN5.setZero();
	FM1.setZero(); FM2.setZero(); FM3.setZero(); FM4.setZero(); FM5.setZero();
	PlaneFit::fitPlane(P1, FN1, FM1);
	PlaneFit::fitPlane(P2, FN2, FM2);
	PlaneFit::fitPlane(P3, FN3, FM3);
	PlaneFit::fitPlane(P4, FN4, FM4);
	PlaneFit::fitPlane(P5, FN5, FM5);
	EXPECT_NEAR(1.0, fabs(N1.dot(FN1.normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N2.dot(FN2.normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N3.dot(FN3.normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N4.dot(FN4.normalized())), TOL_HIGH);
	EXPECT_NEAR(1.0, fabs(N5.dot(FN5.normalized())), TOL_HIGH);
	EXPECT_DOUBLE_EQ(M1.x(), FM1.x());
	EXPECT_DOUBLE_EQ(M1.y(), FM1.y());
	EXPECT_DOUBLE_EQ(M1.z(), FM1.z());
	EXPECT_DOUBLE_EQ(M2.x(), FM2.x());
	EXPECT_DOUBLE_EQ(M2.y(), FM2.y());
	EXPECT_DOUBLE_EQ(M2.z(), FM2.z());
	EXPECT_DOUBLE_EQ(M3.x(), FM3.x());
	EXPECT_DOUBLE_EQ(M3.y(), FM3.y());
	EXPECT_DOUBLE_EQ(M3.z(), FM3.z());
	EXPECT_DOUBLE_EQ(M4.x(), FM4.x());
	EXPECT_DOUBLE_EQ(M4.y(), FM4.y());
	EXPECT_DOUBLE_EQ(M4.z(), FM4.z());
	EXPECT_DOUBLE_EQ(M5.x(), FM5.x());
	EXPECT_DOUBLE_EQ(M5.y(), FM5.y());
	EXPECT_DOUBLE_EQ(M5.z(), FM5.z());
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P1, FN1, FM1), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P2, FN2, FM2), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P3, FN3, FM3), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P4, FN4, FM4), TOL_HIGH);
	EXPECT_NEAR(0.0, PlaneFit::fitPlaneError(P5, FN5, FM5), TOL_MED);
}

// Test special cases of plane fitting
TEST(PlaneFitTest, testFitPlaneSpecial)
{
	// Declare variables
	Eigen::Vector4d FC1, FC2;
	Eigen::Vector3d FN1, FN2, FM1, FM2;

	// Test N = 0 case
	FC1.setZero(); FC2.setZero(); FN1.setZero(); FN2.setZero(); FM1.setZero(); FM2.setZero();
	PlaneFit::Points3D P;
	PlaneFit::fitPlane(P, FC1);
	PlaneFit::fitPlane(P, FC2, FM1);
	PlaneFit::fitPlane(P, FN1);
	PlaneFit::fitPlane(P, FN2, FM2);
	EXPECT_DOUBLE_EQ(0.0, FC1.x());
	EXPECT_DOUBLE_EQ(0.0, FC1.y());
	EXPECT_DOUBLE_EQ(1.0, FC1.z());
	EXPECT_DOUBLE_EQ(0.0, FC1.w());
	EXPECT_DOUBLE_EQ(0.0, FC2.x());
	EXPECT_DOUBLE_EQ(0.0, FC2.y());
	EXPECT_DOUBLE_EQ(1.0, FC2.z());
	EXPECT_DOUBLE_EQ(0.0, FC2.w());
	EXPECT_DOUBLE_EQ(0.0, FM1.x());
	EXPECT_DOUBLE_EQ(0.0, FM1.y());
	EXPECT_DOUBLE_EQ(0.0, FM1.z());
	EXPECT_DOUBLE_EQ(0.0, FM2.x());
	EXPECT_DOUBLE_EQ(0.0, FM2.y());
	EXPECT_DOUBLE_EQ(0.0, FM2.z());
	EXPECT_DOUBLE_EQ(0.0, FN1.x());
	EXPECT_DOUBLE_EQ(0.0, FN1.y());
	EXPECT_DOUBLE_EQ(1.0, FN1.z());
	EXPECT_DOUBLE_EQ(0.0, FN2.x());
	EXPECT_DOUBLE_EQ(0.0, FN2.y());
	EXPECT_DOUBLE_EQ(1.0, FN2.z());

	// Test N = 1 case
	FC1.setZero(); FC2.setZero(); FN1.setZero(); FN2.setZero(); FM1.setZero(); FM2.setZero();
	Eigen::Vector3d p1(1.0, 2.0, 3.0);
	P.push_back(p1);
	PlaneFit::fitPlane(P, FC1);
	PlaneFit::fitPlane(P, FC2, FM1);
	PlaneFit::fitPlane(P, FN1);
	PlaneFit::fitPlane(P, FN2, FM2);
	EXPECT_DOUBLE_EQ(0.0, FC1.x());
	EXPECT_DOUBLE_EQ(0.0, FC1.y());
	EXPECT_DOUBLE_EQ(1.0, FC1.z());
	EXPECT_DOUBLE_EQ(-p1.z(), FC1.w());
	EXPECT_DOUBLE_EQ(0.0, FC2.x());
	EXPECT_DOUBLE_EQ(0.0, FC2.y());
	EXPECT_DOUBLE_EQ(1.0, FC2.z());
	EXPECT_DOUBLE_EQ(-p1.z(), FC2.w());
	EXPECT_DOUBLE_EQ(p1.x(), FM1.x());
	EXPECT_DOUBLE_EQ(p1.y(), FM1.y());
	EXPECT_DOUBLE_EQ(p1.z(), FM1.z());
	EXPECT_DOUBLE_EQ(p1.x(), FM2.x());
	EXPECT_DOUBLE_EQ(p1.y(), FM2.y());
	EXPECT_DOUBLE_EQ(p1.z(), FM2.z());
	EXPECT_DOUBLE_EQ(0.0, FN1.x());
	EXPECT_DOUBLE_EQ(0.0, FN1.y());
	EXPECT_DOUBLE_EQ(1.0, FN1.z());
	EXPECT_DOUBLE_EQ(0.0, FN2.x());
	EXPECT_DOUBLE_EQ(0.0, FN2.y());
	EXPECT_DOUBLE_EQ(1.0, FN2.z());

	// Test N = 2 case
	FC1.setZero(); FC2.setZero(); FN1.setZero(); FN2.setZero(); FM1.setZero(); FM2.setZero();
	Eigen::Vector3d p2(-3.0, 4.0, -1.0);
	Eigen::Vector3d d2 = (p1 - p2).normalized();
	Eigen::Vector3d m2 = 0.5*(p1 + p2);
	P.push_back(p2);
	PlaneFit::fitPlane(P, FC1);
	PlaneFit::fitPlane(P, FC2, FM1);
	PlaneFit::fitPlane(P, FN1);
	PlaneFit::fitPlane(P, FN2, FM2);
	EXPECT_NEAR(0.0, fabs(d2.dot(FC1.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(0.0, FC1.head<3>().dot(p1) + FC1.w(), TOL_HIGH);
	EXPECT_NEAR(0.0, FC1.head<3>().dot(p2) + FC1.w(), TOL_HIGH);
	EXPECT_NEAR(0.0, fabs(d2.dot(FC2.head<3>().normalized())), TOL_HIGH);
	EXPECT_NEAR(0.0, FC2.head<3>().dot(p1) + FC2.w(), TOL_HIGH);
	EXPECT_NEAR(0.0, FC2.head<3>().dot(p2) + FC2.w(), TOL_HIGH);
	EXPECT_DOUBLE_EQ(m2.x(), FM1.x());
	EXPECT_DOUBLE_EQ(m2.y(), FM1.y());
	EXPECT_DOUBLE_EQ(m2.z(), FM1.z());
	EXPECT_DOUBLE_EQ(m2.x(), FM2.x());
	EXPECT_DOUBLE_EQ(m2.y(), FM2.y());
	EXPECT_DOUBLE_EQ(m2.z(), FM2.z());
	EXPECT_NEAR(0.0, fabs(d2.dot(FN1.normalized())), TOL_HIGH);
	EXPECT_NEAR(0.0, fabs(d2.dot(FN2.normalized())), TOL_HIGH);

	// Test straight line case
	P.resize(100);
	for(size_t i = 0; i < P.size(); i++)
		P[i] << 1.0, -2.0, 3.0*(1.0 + 2.0*drand48());
	PlaneFit::fitPlane(P, FC1);
	PlaneFit::fitPlane(P, FC2, FM1);
	PlaneFit::fitPlane(P, FN1);
	PlaneFit::fitPlane(P, FN2, FM2);
	EXPECT_DOUBLE_EQ(1.0, FC1.head<2>().norm());
	EXPECT_DOUBLE_EQ(0.0, FC1.z());
	EXPECT_DOUBLE_EQ(-1.0*FC1.x() + 2.0*FC1.y(), FC1.w());
	EXPECT_DOUBLE_EQ(1.0, FC2.head<2>().norm());
	EXPECT_DOUBLE_EQ(0.0, FC2.z());
	EXPECT_DOUBLE_EQ(-1.0*FC2.x() + 2.0*FC2.y(), FC2.w());
	EXPECT_DOUBLE_EQ(1.0, FM1.x());
	EXPECT_DOUBLE_EQ(-2.0, FM1.y());
	EXPECT_DOUBLE_EQ(1.0, FM2.x());
	EXPECT_DOUBLE_EQ(-2.0, FM2.y());
	EXPECT_DOUBLE_EQ(1.0, FN1.head<2>().norm());
	EXPECT_DOUBLE_EQ(0.0, FN1.z());
	EXPECT_DOUBLE_EQ(1.0, FN2.head<2>().norm());
	EXPECT_DOUBLE_EQ(0.0, FN2.z());
}

// Test plane fitting errors
TEST(PlaneFitTest, testFitPlaneError)
{
	// Generate test data
	Eigen::Vector3d N1(1.0, -2.0, 1.5), N2(-74.3, 105.3, 33.0);
	N1.normalize(); N2.normalize();
	PlaneFit::Points3D P1, P2;
	Eigen::Vector3d M1, M2;
	Eigen::Vector3d Q1(1.0+2.0*drand48(), 1.0+2.0*drand48(), 1.0+2.0*drand48());
	Eigen::Vector3d Q2(1.0+2.0*drand48(), 1.0+2.0*drand48(), 1.0+2.0*drand48());
	genPlaneData(P1, M1, 100, N1, Q1, 3.0*drand48());
	genPlaneData(P2, M2, 150, N2, Q2, 3.0*drand48());
	Eigen::Vector4d C1, C2;
	C1 << N1, -N1.dot(Q1);
	C2 << N2, -N2.dot(Q2);
	for(size_t i = 0; i < 50; i++)
	{
		P1[i] += 0.4*N1;
		P2[i] -= 60.0*N2;
	}

	// Sanity check the plane fitting error
	EXPECT_DOUBLE_EQ(0.2, PlaneFit::fitPlaneError(P1, C1));
	EXPECT_DOUBLE_EQ(0.2, PlaneFit::fitPlaneError(P1, N1, Q1));
	EXPECT_DOUBLE_EQ(20.0, PlaneFit::fitPlaneError(P2, C2));
	EXPECT_DOUBLE_EQ(20.0, PlaneFit::fitPlaneError(P2, N2, Q2));
}

// Main function
int main(int argc, char **argv)
{
	// Ensure repeatability of random number generation
	srand48(0x593BEA4A);
	
	// Run all the tests
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF