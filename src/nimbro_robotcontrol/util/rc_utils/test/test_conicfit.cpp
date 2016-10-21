// Unit test for the ConicFit class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/conicfit.h>
#include <Eigen/Geometry>
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

// Generate 3D data from 2D data by adding z values of a given mean
void gen3DFrom2D(ConicFit::Points3D& PP, const ConicFit::Points2D& P, double Z)
{
	// Retrieve the size
	size_t N = P.size();
	
	// Clear the output vector
	PP.clear();
	
	// If size is small then just write the constant z value in
	if(N < 3)
	{
		for(size_t i = 0; i < N; i++)
			PP.push_back(Eigen::Vector3d(P[i].x(), P[i].y(), Z));
		return;
	}
	
	// If larger then generate some more interesting data
	double Nm = 0.5*(N - 1);
	for(size_t i = 0; i < N; i++)
		PP.push_back(Eigen::Vector3d(P[i].x(), P[i].y(), Z + (0.5*Z + 1.0)*(i - Nm)));
}

// Generate random weights within a particular interval
void genWeights(ConicFit::Weights& W, size_t N, double minW, double maxW)
{
	// Generate the required weights
	W.resize(N);
	for(size_t i = 0; i < N; i++)
		W[i] = minW + drand48()*(maxW - minW);
}

// Generate random weights within a particular interval
void genWeights(ConicFit::WeightedPoints2D& WP, double minW, double maxW)
{
	// Generate the required weights
	size_t N = WP.size();
	for(size_t i = 0; i < N; i++)
		WP[i].z() = minW + drand48()*(maxW - minW);
}

// Merge a set of weighted points from a set of weights and a set of points
void mergeWeights(ConicFit::WeightedPoints2D& WP, const ConicFit::Points2D& P, const ConicFit::Weights& W)
{
	// Merge the two sets together
	size_t N = std::min(P.size(), W.size());
	WP.resize(N);
	for(size_t i = 0; i < N; i++)
	{
		WP[i].head<2>() = P[i];
		WP[i].z() = W[i];
	}
}

//
// Test functions
//

// Test circle fitting
TEST(ConicFitTest, testFitCircle)
{
	// Declare variables
	Eigen::Vector2d FC1, FC2, FC3, FC4;
	Eigen::Vector3d FCC1, FCC2, FCC3, FCC4;
	double FR1, FR2, FR3, FR4;
	
	// Generate circular test data
	Eigen::Vector2d C1(0.6, -0.8), C2(21312.0, -934511.0), C3(0.0, 0.0), C4(-5.6, 6123.0);
	double R1 = 1.0, R2 = 4.2, R3 = 0.0, R4 = 17.0;
	ConicFit::Points2D P1, P2, P3, P4;
	ConicFit::genCircleData(P1, 130, C1, R1);
	ConicFit::genCircleData(P2, 50, C2, R2);
	ConicFit::genCircleData(P3, 90, C3, R3);
	ConicFit::genCircleData(P4, 2000, C4, R4);
	double Z1 = 0.3, Z2 = 312.0, Z3 = 0.0, Z4 = -93.0;
	Eigen::Vector3d CC1(C1.x(), C1.y(), Z1), CC2(C2.x(), C2.y(), Z2), CC3(C3.x(), C3.y(), Z3), CC4(C4.x(), C4.y(), Z4);
	ConicFit::Points3D PP1, PP2, PP3, PP4;
	gen3DFrom2D(PP1, P1, Z1);
	gen3DFrom2D(PP2, P2, Z2);
	gen3DFrom2D(PP3, P3, Z3);
	gen3DFrom2D(PP4, P4, Z4);
	
	// Sanity check the fitting errors of the generated data
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P1, C1, R1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P2, C2, R2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P3, C3, R3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P4, C4, R4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP1, CC1, R1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP2, CC2, R2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP3, CC3, R3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP4, CC4, R4), TOL_HIGH);
	
	// Perform circle fitting 2D
	FC1.setZero(); FC2.setZero(); FC3.setZero(); FC4.setZero();
	FR1 = FR2 = FR3 = FR4 = 0.0;
	ConicFit::fitCircle(P1, FC1, FR1);
	ConicFit::fitCircle(P2, FC2, FR2);
	ConicFit::fitCircle(P3, FC3, FR3);
	ConicFit::fitCircle(P4, FC4, FR4);
	
	// Check circle fitting 2D
	EXPECT_NEAR(C1.x(), FC1.x(), TOL_HIGH);
	EXPECT_NEAR(C1.y(), FC1.y(), TOL_HIGH);
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(C2.x(), FC2.x(), TOL_HIGH);
	EXPECT_NEAR(C2.y(), FC2.y(), TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW); // ULP size of C2
	EXPECT_NEAR(C3.x(), FC3.x(), TOL_HIGH);
	EXPECT_NEAR(C3.y(), FC3.y(), TOL_HIGH);
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(C4.x(), FC4.x(), TOL_HIGH);
	EXPECT_NEAR(C4.y(), FC4.y(), TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_HIGH);
	
	// Check fitting errors 2D
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P1, FC1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P2, FC2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P3, FC3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P4, FC4, FR4), TOL_HIGH);
	
	// Perform circle fitting 3D
	FCC1.setZero(); FCC2.setZero(); FCC3.setZero(); FCC4.setZero();
	FR1 = FR2 = FR3 = FR4 = 0.0;
	ConicFit::fitCircle(PP1, FCC1, FR1);
	ConicFit::fitCircle(PP2, FCC2, FR2);
	ConicFit::fitCircle(PP3, FCC3, FR3);
	ConicFit::fitCircle(PP4, FCC4, FR4);
	
	// Check circle fitting 3D
	EXPECT_NEAR(CC1.x(), FCC1.x(), TOL_HIGH);
	EXPECT_NEAR(CC1.y(), FCC1.y(), TOL_HIGH);
	EXPECT_NEAR(CC1.z(), FCC1.z(), TOL_HIGH);
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(CC2.x(), FCC2.x(), TOL_HIGH);
	EXPECT_NEAR(CC2.y(), FCC2.y(), TOL_HIGH);
	EXPECT_NEAR(CC2.z(), FCC2.z(), TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW); // ULP size of CC2
	EXPECT_NEAR(CC3.x(), FCC3.x(), TOL_HIGH);
	EXPECT_NEAR(CC3.y(), FCC3.y(), TOL_HIGH);
	EXPECT_NEAR(CC3.z(), FCC3.z(), TOL_HIGH);
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(CC4.x(), FCC4.x(), TOL_HIGH);
	EXPECT_NEAR(CC4.y(), FCC4.y(), TOL_HIGH);
	EXPECT_NEAR(CC4.z(), FCC4.z(), TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_HIGH);
	
	// Check fitting errors 3D
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP1, FCC1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP2, FCC2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP3, FCC3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP4, FCC4, FR4), TOL_HIGH);
	
	// Perform circle fitting 2D with a known centre
	FR1 = FR2 = FR3 = FR4 = 0.0;
	ConicFit::fitCircleCentred(P1, C1, FR1);
	ConicFit::fitCircleCentred(P2, C2, FR2);
	ConicFit::fitCircleCentred(P3, C3, FR3);
	ConicFit::fitCircleCentred(P4, C4, FR4);
	
	// Check circle fitting 2D with a known centre
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW); // ULP size of C2
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_MED); // ULP size of C4
	
	// Check fitting errors 2D with a known centre
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P1, C1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P2, C2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P3, C3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(P4, C4, FR4), TOL_MED);
	
	// Perform circle fitting 3D with a known centre
	FR1 = FR2 = FR3 = FR4 = 0.0;
	ConicFit::fitCircleCentred(PP1, CC1, FR1);
	ConicFit::fitCircleCentred(PP2, CC2, FR2);
	ConicFit::fitCircleCentred(PP3, CC3, FR3);
	ConicFit::fitCircleCentred(PP4, CC4, FR4);
	
	// Check circle fitting 3D with a known centre
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW); // ULP size of C2
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_MED); // ULP size of C4
	
	// Check fitting errors 3D
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP1, CC1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP2, CC2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP3, CC3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleError(PP4, CC4, FR4), TOL_MED);
}

// Test circle fitting
TEST(ConicFitTest, testFitCircleWeighted)
{
	// Declare variables
	Eigen::Vector2d FC1, FC2, FC3, FC4, FC5;
	double FR1, FR2, FR3, FR4, FR5;
	
	// Generate circular test data
	Eigen::Vector2d C1(0.6, -0.8), C2(21312.0, -934511.0), C3(0.0, 0.0), C4(-5.6, 6123.0), C5(1.7, 0.5);
	double R1 = 1.0, R2 = 4.2, R3 = 0.0, R4 = 17.0, R5 = 4.0;
	ConicFit::Points2D P1, P2, P3, P4, P5;
	ConicFit::genCircleData(P1, 130, C1, R1);
	ConicFit::genCircleData(P2, 50, C2, R2);
	ConicFit::genCircleData(P3, 90, C3, R3);
	ConicFit::genCircleData(P4, 2000, C4, R4);
	ConicFit::genCircleData(P5, 110, C5, R5);
	ConicFit::Weights W1, W2, W3, W4, W5;
	genWeights(W1, P1.size(), 0.0, 1.0);
	genWeights(W2, P2.size(), 0.5, 3.0);
	genWeights(W3, P3.size(), -1.0, 1.0);
	genWeights(W4, P4.size(), 100.0, 150.0);
	genWeights(W5, P5.size(), 0.0, 1.0);
	for(size_t i = 0; i < 10; i++)
	{
		P5.push_back(Eigen::Vector2d(100.0, -1000.0));
		P5.push_back(Eigen::Vector2d(-8293.2, 12949.3));
		P5.push_back(Eigen::Vector2d(72.2, 96.3));
		W5.push_back(0.0);
		W5.push_back(0.0);
		W5.push_back(0.0);
	}
	ConicFit::WeightedPoints2D WP1, WP2, WP3, WP4, WP5;
	mergeWeights(WP1, P1, W1);
	mergeWeights(WP2, P2, W2);
	mergeWeights(WP3, P3, W3);
	mergeWeights(WP4, P4, W4);
	mergeWeights(WP5, P5, W5);
	
	// Sanity check the fitting errors of the generated data
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP1, C1, R1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP2, C2, R2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP3, C3, R3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP4, C4, R4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP5, C5, R5), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P1, W1, C1, R1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P2, W2, C2, R2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P3, W3, C3, R3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P4, W4, C4, R4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P5, W5, C5, R5), TOL_HIGH);
	
	// Perform weighted circle fitting 2D (WP form)
	FC1.setZero(); FC2.setZero(); FC3.setZero(); FC4.setZero(); FC5.setZero();
	FR1 = FR2 = FR3 = FR4 = FR5 = 0.0;
	ConicFit::fitCircleWeighted(WP1, FC1, FR1);
	ConicFit::fitCircleWeighted(WP2, FC2, FR2);
	ConicFit::fitCircleWeighted(WP3, FC3, FR3);
	ConicFit::fitCircleWeighted(WP4, FC4, FR4);
	ConicFit::fitCircleWeighted(WP5, FC5, FR5);
	
	// Check weighted circle fitting 2D (WP form)
	EXPECT_NEAR(C1.x(), FC1.x(), TOL_HIGH);
	EXPECT_NEAR(C1.y(), FC1.y(), TOL_HIGH);
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(C2.x(), FC2.x(), TOL_HIGH);
	EXPECT_NEAR(C2.y(), FC2.y(), TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW);
	EXPECT_NEAR(C3.x(), FC3.x(), TOL_HIGH);
	EXPECT_NEAR(C3.y(), FC3.y(), TOL_HIGH);
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(C4.x(), FC4.x(), TOL_HIGH);
	EXPECT_NEAR(C4.y(), FC4.y(), TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_HIGH);
	EXPECT_NEAR(C5.x(), FC5.x(), TOL_LOW);
	EXPECT_NEAR(C5.y(), FC5.y(), TOL_LOW);
	EXPECT_NEAR(R5, FR5, TOL_LOW);
	
	// Check fitting errors 2D (WP form)
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP1, FC1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP2, FC2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP3, FC3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP4, FC4, FR4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP5, FC5, FR5), TOL_LOW);
	
	// Perform weighted circle fitting 2D (P+W form)
	FC1.setZero(); FC2.setZero(); FC3.setZero(); FC4.setZero(); FC5.setZero();
	FR1 = FR2 = FR3 = FR4 = FR5 = 0.0;
	ConicFit::fitCircleWeighted(P1, W1, FC1, FR1);
	ConicFit::fitCircleWeighted(P2, W2, FC2, FR2);
	ConicFit::fitCircleWeighted(P3, W3, FC3, FR3);
	ConicFit::fitCircleWeighted(P4, W4, FC4, FR4);
	ConicFit::fitCircleWeighted(P5, W5, FC5, FR5);
	
	// Check weighted circle fitting 2D (P+W form)
	EXPECT_NEAR(C1.x(), FC1.x(), TOL_HIGH);
	EXPECT_NEAR(C1.y(), FC1.y(), TOL_HIGH);
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(C2.x(), FC2.x(), TOL_HIGH);
	EXPECT_NEAR(C2.y(), FC2.y(), TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW);
	EXPECT_NEAR(C3.x(), FC3.x(), TOL_HIGH);
	EXPECT_NEAR(C3.y(), FC3.y(), TOL_HIGH);
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(C4.x(), FC4.x(), TOL_HIGH);
	EXPECT_NEAR(C4.y(), FC4.y(), TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_HIGH);
	EXPECT_NEAR(C5.x(), FC5.x(), TOL_LOW);
	EXPECT_NEAR(C5.y(), FC5.y(), TOL_LOW);
	EXPECT_NEAR(R5, FR5, TOL_LOW);
	
	// Check fitting errors 2D (P+W form)
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P1, W1, FC1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P2, W2, FC2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P3, W3, FC3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P4, W4, FC4, FR4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P5, W5, FC5, FR5), TOL_LOW);
	
	// Perform weighted circle fitting 2D with a known centre (WP form)
	FR1 = FR2 = FR3 = FR4 = FR5 = 0.0;
	ConicFit::fitCircleCentredWeighted(WP1, C1, FR1);
	ConicFit::fitCircleCentredWeighted(WP2, C2, FR2);
	ConicFit::fitCircleCentredWeighted(WP3, C3, FR3);
	ConicFit::fitCircleCentredWeighted(WP4, C4, FR4);
	ConicFit::fitCircleCentredWeighted(WP5, C5, FR5);
	
	// Check weighted circle fitting 2D with a known centre (WP form)
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW);
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_HIGH);
	EXPECT_NEAR(R5, FR5, TOL_HIGH);
	
	// Check fitting errors 2D with a known centre (WP form)
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP1, C1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP2, C2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP3, C3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP4, C4, FR4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(WP5, C5, FR5), TOL_HIGH);
	
	// Perform weighted circle fitting 2D with a known centre (P+W form)
	FR1 = FR2 = FR3 = FR4 = FR5 = 0.0;
	ConicFit::fitCircleCentredWeighted(P1, W1, C1, FR1);
	ConicFit::fitCircleCentredWeighted(P2, W2, C2, FR2);
	ConicFit::fitCircleCentredWeighted(P3, W3, C3, FR3);
	ConicFit::fitCircleCentredWeighted(P4, W4, C4, FR4);
	ConicFit::fitCircleCentredWeighted(P5, W5, C5, FR5);
	
	// Check weighted circle fitting 2D with a known centre (P+W form)
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW);
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_HIGH);
	EXPECT_NEAR(R5, FR5, TOL_HIGH);
	
	// Check fitting errors 2D with a known centre (P+W form)
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P1, W1, C1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P2, W2, C2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P3, W3, C3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P4, W4, C4, FR4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitCircleErrorWeighted(P5, W5, C5, FR5), TOL_HIGH);
}

// Test ellipse fitting
TEST(ConicFitTest, testFitEllipse)
{
	// Declare variables
	Eigen::Vector2d FC1, FC2, FC3, FC4;
	Eigen::Vector3d FCC1, FCC2, FCC3, FCC4;
	Eigen::Matrix2d FA1, FA2, FA3, FA4;
	
	// Generate elliptical test data
	Eigen::Vector2d C1(0.6, -0.8), C2(21312.0, -934511.0), C3(0.0, 0.0), C4(-5.6, 6123.0);
	Eigen::Vector2d R1(1.0, 1.0), R2(4.2, 2.5), R3(0.005, 0.4), R4(17.0, 31.2);
	double Ang1 = 0.91, Ang2 = 2.12, Ang3 = 1.41, Ang4 = 5.23;
	ConicFit::Points2D P1, P2, P3, P4;
	ConicFit::genEllipseData(P1, 130, C1, R1, Ang1);
	ConicFit::genEllipseData(P2, 50, C2, R2, Ang2);
	ConicFit::genEllipseData(P3, 90, C3, R3, Ang3);
	ConicFit::genEllipseData(P4, 2000, C4, R4, Ang4);
	double Z1 = 0.3, Z2 = 312.0, Z3 = 0.0, Z4 = -93.0;
	Eigen::Vector3d CC1(C1.x(), C1.y(), Z1), CC2(C2.x(), C2.y(), Z2), CC3(C3.x(), C3.y(), Z3), CC4(C4.x(), C4.y(), Z4);
	ConicFit::Points3D PP1, PP2, PP3, PP4;
	gen3DFrom2D(PP1, P1, Z1);
	gen3DFrom2D(PP2, P2, Z2);
	gen3DFrom2D(PP3, P3, Z3);
	gen3DFrom2D(PP4, P4, Z4);
	
	// Perform ellipse fitting 2D
	FC1.setZero(); FC2.setZero(); FC3.setZero(); FC4.setZero();
	FA1.setIdentity(); FA2.setIdentity(); FA3.setIdentity(); FA4.setIdentity();
	ConicFit::fitEllipse(P1, FC1, FA1);
	ConicFit::fitEllipse(P2, FC2, FA2);
	ConicFit::fitEllipse(P3, FC3, FA3);
	ConicFit::fitEllipse(P4, FC4, FA4);
	
	// Check ellipse fitting 2D
	EXPECT_NEAR(0.0, (FC1 - C1).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FC2 - C2).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FC3 - C3).norm(), TOL_MED);
	EXPECT_NEAR(0.0, (FC4 - C4).norm(), TOL_MED);
	for(size_t i = 0; i < P1.size(); i++)
	{
		Eigen::Vector2d v = P1[i] - FC1;
		double err = v.transpose() * FA1 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_HIGH);
	}
	for(size_t i = 0; i < P2.size(); i++)
	{
		Eigen::Vector2d v = P2[i] - FC2;
		double err = v.transpose() * FA2 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < P3.size(); i++)
	{
		Eigen::Vector2d v = P3[i] - FC3;
		double err = v.transpose() * FA3 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < P4.size(); i++)
	{
		Eigen::Vector2d v = P4[i] - FC4;
		double err = v.transpose() * FA4 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_MED);
	}
	
	// Check fitting errors 2D
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P1, C1, FA1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P2, C2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P3, C3, FA3), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P4, C4, FA4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P1, FC1, FA1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P2, FC2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P3, FC3, FA3), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P4, FC4, FA4), TOL_HIGH);
	
	// Perform ellipse fitting 2D with a known centre
	FA1.setIdentity(); FA2.setIdentity(); FA3.setIdentity(); FA4.setIdentity();
	ConicFit::fitEllipseCentred(P1, C1, FA1);
	ConicFit::fitEllipseCentred(P2, C2, FA2);
	ConicFit::fitEllipseCentred(P3, C3, FA3);
	ConicFit::fitEllipseCentred(P4, C4, FA4);
	
	// Check ellipse fitting 2D with a known centre
	for(size_t i = 0; i < P1.size(); i++)
	{
		Eigen::Vector2d v = P1[i] - C1;
		double err = v.transpose() * FA1 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_HIGH);
	}
	for(size_t i = 0; i < P2.size(); i++)
	{
		Eigen::Vector2d v = P2[i] - C2;
		double err = v.transpose() * FA2 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < P3.size(); i++)
	{
		Eigen::Vector2d v = P3[i] - C3;
		double err = v.transpose() * FA3 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < P4.size(); i++)
	{
		Eigen::Vector2d v = P4[i] - C4;
		double err = v.transpose() * FA4 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_MED);
	}
	
	// Check fitting errors 2D with a known centre
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P1, C1, FA1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P2, C2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P3, C3, FA3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(P4, C4, FA4), TOL_HIGH);
	
	// Perform ellipse fitting 3D
	FCC1.setZero(); FCC2.setZero(); FCC3.setZero(); FCC4.setZero();
	FA1.setIdentity(); FA2.setIdentity(); FA3.setIdentity(); FA4.setIdentity();
	ConicFit::fitEllipse(PP1, FCC1, FA1);
	ConicFit::fitEllipse(PP2, FCC2, FA2);
	ConicFit::fitEllipse(PP3, FCC3, FA3);
	ConicFit::fitEllipse(PP4, FCC4, FA4);
	
	// Check ellipse fitting 3D
	EXPECT_NEAR(0.0, (FCC1 - CC1).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FCC2 - CC2).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FCC3 - CC3).norm(), TOL_MED);
	EXPECT_NEAR(0.0, (FCC4 - CC4).norm(), TOL_MED);
	for(size_t i = 0; i < PP1.size(); i++)
	{
		Eigen::Vector2d v = (PP1[i] - FCC1).head<2>();
		double err = v.transpose() * FA1 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_HIGH);
	}
	for(size_t i = 0; i < PP2.size(); i++)
	{
		Eigen::Vector2d v = (PP2[i] - FCC2).head<2>();
		double err = v.transpose() * FA2 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < PP3.size(); i++)
	{
		Eigen::Vector2d v = (PP3[i] - FCC3).head<2>();
		double err = v.transpose() * FA3 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < PP4.size(); i++)
	{
		Eigen::Vector2d v = (PP4[i] - FCC4).head<2>();
		double err = v.transpose() * FA4 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_MED);
	}
	
	// Check fitting errors 3D
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP1, CC1, FA1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP2, CC2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP3, CC3, FA3), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP4, CC4, FA4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP1, FCC1, FA1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP2, FCC2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP3, FCC3, FA3), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP4, FCC4, FA4), TOL_HIGH);
	
	// Perform ellipse fitting 3D with a known centre
	FA1.setIdentity(); FA2.setIdentity(); FA3.setIdentity(); FA4.setIdentity();
	ConicFit::fitEllipseCentred(PP1, CC1, FA1);
	ConicFit::fitEllipseCentred(PP2, CC2, FA2);
	ConicFit::fitEllipseCentred(PP3, CC3, FA3);
	ConicFit::fitEllipseCentred(PP4, CC4, FA4);
	
	// Check ellipse fitting 3D with a known centre
	for(size_t i = 0; i < PP1.size(); i++)
	{
		Eigen::Vector2d v = (PP1[i] - CC1).head<2>();
		double err = v.transpose() * FA1 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_HIGH);
	}
	for(size_t i = 0; i < PP2.size(); i++)
	{
		Eigen::Vector2d v = (PP2[i] - CC2).head<2>();
		double err = v.transpose() * FA2 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < PP3.size(); i++)
	{
		Eigen::Vector2d v = (PP3[i] - CC3).head<2>();
		double err = v.transpose() * FA3 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < PP4.size(); i++)
	{
		Eigen::Vector2d v = (PP4[i] - CC4).head<2>();
		double err = v.transpose() * FA4 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_MED);
	}
	
	// Check fitting errors 3D with a known centre
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP1, CC1, FA1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP2, CC2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP3, CC3, FA3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseErrorCoeff(PP4, CC4, FA4), TOL_HIGH);
	
	// Convert the fitted coefficient matrices to rotation matrices and radii
	Eigen::Matrix2d FQ1, FQ2, FQ3, FQ4;
	Eigen::Vector2d FR1, FR2, FR3, FR4;
	double FAng1, FAng2, FAng3, FAng4;
	ASSERT_TRUE(ConicFit::ellipseMatrixToAxes(FA1, FQ1, FR1, FAng1));
	ASSERT_TRUE(ConicFit::ellipseMatrixToAxes(FA2, FQ2, FR2, FAng2));
	ASSERT_TRUE(ConicFit::ellipseMatrixToAxes(FA3, FQ3, FR3, FAng3));
	ASSERT_TRUE(ConicFit::ellipseMatrixToAxes(FA4, FQ4, FR4, FAng4));
	
	// Check the rotation matrices and radii
	double cFAng1 = cos(FAng1), sFAng1 = sin(FAng1);
	double cFAng2 = cos(FAng2), sFAng2 = sin(FAng2);
	double cFAng3 = cos(FAng3), sFAng3 = sin(FAng3);
	double cFAng4 = cos(FAng4), sFAng4 = sin(FAng4);
	Eigen::Vector2d lambda1(1.0/(FR1.x()*FR1.x()), 1.0/(FR1.y()*FR1.y()));
	Eigen::Vector2d lambda2(1.0/(FR2.x()*FR2.x()), 1.0/(FR2.y()*FR2.y()));
	Eigen::Vector2d lambda3(1.0/(FR3.x()*FR3.x()), 1.0/(FR3.y()*FR3.y()));
	Eigen::Vector2d lambda4(1.0/(FR4.x()*FR4.x()), 1.0/(FR4.y()*FR4.y()));
	EXPECT_NEAR(0.0, (FQ1 * lambda1.asDiagonal() * FQ1.transpose() - FA1).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ2 * lambda2.asDiagonal() * FQ2.transpose() - FA2).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ3 * lambda3.asDiagonal() * FQ3.transpose() - FA3).norm(), TOL_LOW);
	EXPECT_NEAR(0.0, (FQ4 * lambda4.asDiagonal() * FQ4.transpose() - FA4).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ1 - (Eigen::Matrix2d() << cFAng1, -sFAng1, sFAng1, cFAng1).finished()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ2 - (Eigen::Matrix2d() << cFAng2, -sFAng2, sFAng2, cFAng2).finished()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ3 - (Eigen::Matrix2d() << cFAng3, -sFAng3, sFAng3, cFAng3).finished()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ4 - (Eigen::Matrix2d() << cFAng4, -sFAng4, sFAng4, cFAng4).finished()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ1.inverse() - FQ1.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ2.inverse() - FQ2.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ3.inverse() - FQ3.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ4.inverse() - FQ4.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(1.0, FQ1.determinant(), TOL_HIGH);
	EXPECT_NEAR(1.0, FQ2.determinant(), TOL_HIGH);
	EXPECT_NEAR(1.0, FQ3.determinant(), TOL_HIGH);
	EXPECT_NEAR(1.0, FQ4.determinant(), TOL_HIGH);
	EXPECT_GT(FR1.x(), 0.0);
	EXPECT_GT(FR1.y(), 0.0);
	EXPECT_GT(FR2.x(), 0.0);
	EXPECT_GT(FR2.y(), 0.0);
	EXPECT_GT(FR3.x(), 0.0);
	EXPECT_GT(FR3.y(), 0.0);
	EXPECT_GT(FR4.x(), 0.0);
	EXPECT_GT(FR4.y(), 0.0);
	
	// Convert the rotation matrix and radii to a normalisation matrix
	Eigen::Matrix2d FW1, FW2, FW3, FW4;
	ASSERT_TRUE(ConicFit::ellipseAxesToTransform(FQ1, FR1, FW1));
	ASSERT_TRUE(ConicFit::ellipseAxesToTransform(FQ2, FR2, FW2));
	ASSERT_TRUE(ConicFit::ellipseAxesToTransform(FQ3, FR3, FW3));
	ASSERT_TRUE(ConicFit::ellipseAxesToTransform(FQ4, FR4, FW4));
	
	// Check the normalisation matrices
	EXPECT_NEAR(0.0, (FW1*FW1 - FA1).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW2*FW2 - FA2).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW3*FW3 - FA3).norm(), TOL_LOW);
	EXPECT_NEAR(0.0, (FW4*FW4 - FA4).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW1 - FW1.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW2 - FW2.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW3 - FW3.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW4 - FW4.transpose()).norm(), TOL_HIGH);
	for(size_t i = 1; i <= 2; i++)
	{
		EXPECT_GT(FW1.topLeftCorner(i,i).determinant(), 0.0); // A matrix is positive definite iff all leading principal minors are positive
		EXPECT_GT(FW2.topLeftCorner(i,i).determinant(), 0.0);
		EXPECT_GT(FW3.topLeftCorner(i,i).determinant(), 0.0);
		EXPECT_GT(FW4.topLeftCorner(i,i).determinant(), 0.0);
	}
	
	// Check fitting errors
	EXPECT_NEAR(0.0, ConicFit::fitEllipseError(P1, FC1, FW1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseError(P2, FC2, FW2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseError(P3, FC3, FW3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseError(P4, FC4, FW4), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseError(PP1, FCC1, FW1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseError(PP2, FCC2, FW2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseError(PP3, FCC3, FW3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipseError(PP4, FCC4, FW4), TOL_HIGH);
}

// Test sphere fitting
TEST(ConicFitTest, testFitSphere)
{
	// Declare variables
	Eigen::Vector3d FC1, FC2, FC3, FC4;
	double FR1, FR2, FR3, FR4;
	
	// Generate spherical test data
	Eigen::Vector3d C1(0.6, -0.8, 0.4), C2(21312.0, -934511.0, -13123.0), C3(0.0, 0.0, 0.0), C4(-5.6, 6123.0, 9.3);
	double R1 = 1.0, R2 = 4.2, R3 = 0.0, R4 = 17.0;
	ConicFit::Points3D PP1, PP2, PP3, PP4;
	ConicFit::genSphereData(PP1, 13, C1, R1);
	ConicFit::genSphereData(PP2, 15, C2, R2);
	ConicFit::genSphereData(PP3, 10, C3, R3);
	ConicFit::genSphereData(PP4, 45, C4, R4);
	
	// Sanity check the fitting errors of the generated data
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP1, C1, R1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP2, C2, R2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP3, C3, R3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP4, C4, R4), TOL_HIGH);
	
	// Perform sphere fitting
	FC1.setZero(); FC2.setZero(); FC3.setZero(); FC4.setZero();
	FR1 = FR2 = FR3 = FR4 = 0.0;
	ConicFit::fitSphere(PP1, FC1, FR1);
	ConicFit::fitSphere(PP2, FC2, FR2);
	ConicFit::fitSphere(PP3, FC3, FR3);
	ConicFit::fitSphere(PP4, FC4, FR4);
	
	// Check sphere fitting
	EXPECT_NEAR(C1.x(), FC1.x(), TOL_HIGH);
	EXPECT_NEAR(C1.y(), FC1.y(), TOL_HIGH);
	EXPECT_NEAR(C1.z(), FC1.z(), TOL_HIGH);
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(C2.x(), FC2.x(), TOL_HIGH);
	EXPECT_NEAR(C2.y(), FC2.y(), TOL_HIGH);
	EXPECT_NEAR(C2.z(), FC2.z(), TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW); // ULP size of C2
	EXPECT_NEAR(C3.x(), FC3.x(), TOL_HIGH);
	EXPECT_NEAR(C3.y(), FC3.y(), TOL_HIGH);
	EXPECT_NEAR(C3.z(), FC3.z(), TOL_HIGH);
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(C4.x(), FC4.x(), TOL_MED);
	EXPECT_NEAR(C4.y(), FC4.y(), TOL_MED);
	EXPECT_NEAR(C4.z(), FC4.z(), TOL_MED);
	EXPECT_NEAR(R4, FR4, TOL_MED); // ULP size of C4
	
	// Check fitting errors
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP1, FC1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP2, FC2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP3, FC3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP4, FC4, FR4), TOL_HIGH);
	
	// Perform sphere fitting with a known centre
	FR1 = FR2 = FR3 = FR4 = 0.0;
	ConicFit::fitSphereCentred(PP1, C1, FR1);
	ConicFit::fitSphereCentred(PP2, C2, FR2);
	ConicFit::fitSphereCentred(PP3, C3, FR3);
	ConicFit::fitSphereCentred(PP4, C4, FR4);
	
	// Check sphere fitting with a known centre
	EXPECT_NEAR(R1, FR1, TOL_HIGH);
	EXPECT_NEAR(R2, FR2, TOL_LOW); // ULP size of C2
	EXPECT_NEAR(R3, FR3, TOL_HIGH);
	EXPECT_NEAR(R4, FR4, TOL_MED); // ULP size of C4
	
	// Check fitting errors with a known centre
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP1, C1, FR1), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP2, C2, FR2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP3, C3, FR3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitSphereError(PP4, C4, FR4), TOL_MED);
}

// Test ellipsoid fitting
TEST(ConicFitTest, testFitEllipsoid)
{
	// Declare variables
	Eigen::Vector3d FC1, FC2, FC3, FC4;
	Eigen::Matrix3d FA1, FA2, FA3, FA4;
	
	// Generate ellipsoidal test data
	Eigen::Vector3d C1(0.6, -0.8, 0.4), C2(21312.0, -934511.0, -13123.0), C3(0.0, 0.0, 0.0), C4(-5.6, 6123.0, 9.3);
	Eigen::Vector3d R1(0.7, 0.5, 1.3), R2(6.0, 3.0, 4.5), R3(16.0, 3.0, 1.0), R4(3.0, 17.0, 8.0);
	Eigen::Matrix3d Q1(Eigen::AngleAxisd(0.83, Eigen::Vector3d(1.0, 1.0, 1.0).normalized()));
	Eigen::Matrix3d Q2(Eigen::AngleAxisd(2.81, Eigen::Vector3d(-0.4, 0.7, 1.0).normalized()));
	Eigen::Matrix3d Q3(Eigen::AngleAxisd(1.27, Eigen::Vector3d(0.0, 1.0, 2.0).normalized()));
	Eigen::Matrix3d Q4(Eigen::AngleAxisd(4.93, Eigen::Vector3d(2.0, -1.0, 0.4).normalized()));
	ConicFit::Points3D PP1, PP2, PP3, PP4;
	ConicFit::genEllipsoidData(PP1, 13, C1, R1, Q1);
	ConicFit::genEllipsoidData(PP2, 15, C2, R2, Q2);
	ConicFit::genEllipsoidData(PP3, 10, C3, R3, Q3);
	ConicFit::genEllipsoidData(PP4, 45, C4, R4, Q4);
	
	// Perform ellipsoid fitting
	FC1.setZero(); FC2.setZero(); FC3.setZero(); FC4.setZero();
	FA1.setIdentity(); FA2.setIdentity(); FA3.setIdentity(); FA4.setIdentity();
	ConicFit::fitEllipsoid(PP1, FC1, FA1);
	ConicFit::fitEllipsoid(PP2, FC2, FA2);
	ConicFit::fitEllipsoid(PP3, FC3, FA3);
	ConicFit::fitEllipsoid(PP4, FC4, FA4);
	
	// Check ellipsoid fitting
	EXPECT_NEAR(0.0, (FC1 - C1).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FC2 - C2).norm(), TOL_LOW);
	EXPECT_NEAR(0.0, (FC3 - C3).norm(), TOL_MED);
	EXPECT_NEAR(0.0, (FC4 - C4).norm(), TOL_HIGH);
	for(size_t i = 0; i < PP1.size(); i++)
	{
		Eigen::Vector3d v = PP1[i] - FC1;
		double err = v.transpose() * FA1 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_HIGH);
	}
	for(size_t i = 0; i < PP2.size(); i++)
	{
		Eigen::Vector3d v = PP2[i] - FC2;
		double err = v.transpose() * FA2 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < PP3.size(); i++)
	{
		Eigen::Vector3d v = PP3[i] - FC3;
		double err = v.transpose() * FA3 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_MED);
	}
	for(size_t i = 0; i < PP4.size(); i++)
	{
		Eigen::Vector3d v = PP4[i] - FC4;
		double err = v.transpose() * FA4 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_MED);
	}
	
	// Check fitting errors
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP1, C1, FA1), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP2, C2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP3, C3, FA3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP4, C4, FA4), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP1, FC1, FA1), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP2, FC2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP3, FC3, FA3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP4, FC4, FA4), TOL_MED);
	
	// Perform ellipsoid fitting with a known centre
	FA1.setIdentity(); FA2.setIdentity(); FA3.setIdentity(); FA4.setIdentity();
	ConicFit::fitEllipsoidCentred(PP1, C1, FA1);
	ConicFit::fitEllipsoidCentred(PP2, C2, FA2);
	ConicFit::fitEllipsoidCentred(PP3, C3, FA3);
	ConicFit::fitEllipsoidCentred(PP4, C4, FA4);
	
	// Check ellipsoid fitting with a known centre
	for(size_t i = 0; i < PP1.size(); i++)
	{
		Eigen::Vector3d v = PP1[i] - C1;
		double err = v.transpose() * FA1 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_HIGH);
	}
	for(size_t i = 0; i < PP2.size(); i++)
	{
		Eigen::Vector3d v = PP2[i] - C2;
		double err = v.transpose() * FA2 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_LOW);
	}
	for(size_t i = 0; i < PP3.size(); i++)
	{
		Eigen::Vector3d v = PP3[i] - C3;
		double err = v.transpose() * FA3 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_MED);
	}
	for(size_t i = 0; i < PP4.size(); i++)
	{
		Eigen::Vector3d v = PP4[i] - C4;
		double err = v.transpose() * FA4 * v - 1.0;
		EXPECT_NEAR(0.0, err, TOL_MED);
	}
	
	// Check fitting errors with a known centre
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP1, C1, FA1), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP2, C2, FA2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP3, C3, FA3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidErrorCoeff(PP4, C4, FA4), TOL_MED);
	
	// Convert the fitted coefficient matrices to rotation matrices and radii
	Eigen::Matrix3d FQ1, FQ2, FQ3, FQ4;
	Eigen::Vector3d FR1, FR2, FR3, FR4;
	ASSERT_TRUE(ConicFit::ellipsoidMatrixToAxes(FA1, FQ1, FR1));
	ASSERT_TRUE(ConicFit::ellipsoidMatrixToAxes(FA2, FQ2, FR2));
	ASSERT_TRUE(ConicFit::ellipsoidMatrixToAxes(FA3, FQ3, FR3));
	ASSERT_TRUE(ConicFit::ellipsoidMatrixToAxes(FA4, FQ4, FR4));
	
	// Check the rotation matrices and radii
	Eigen::Vector3d lambda1(1.0/(FR1.x()*FR1.x()), 1.0/(FR1.y()*FR1.y()), 1.0/(FR1.z()*FR1.z()));
	Eigen::Vector3d lambda2(1.0/(FR2.x()*FR2.x()), 1.0/(FR2.y()*FR2.y()), 1.0/(FR2.z()*FR2.z()));
	Eigen::Vector3d lambda3(1.0/(FR3.x()*FR3.x()), 1.0/(FR3.y()*FR3.y()), 1.0/(FR3.z()*FR3.z()));
	Eigen::Vector3d lambda4(1.0/(FR4.x()*FR4.x()), 1.0/(FR4.y()*FR4.y()), 1.0/(FR4.z()*FR4.z()));
	EXPECT_NEAR(0.0, (FQ1 * lambda1.asDiagonal() * FQ1.transpose() - FA1).norm(), TOL_MED);
	EXPECT_NEAR(0.0, (FQ2 * lambda2.asDiagonal() * FQ2.transpose() - FA2).norm(), TOL_MED);
	EXPECT_NEAR(0.0, (FQ3 * lambda3.asDiagonal() * FQ3.transpose() - FA3).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ4 * lambda4.asDiagonal() * FQ4.transpose() - FA4).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ1.inverse() - FQ1.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ2.inverse() - FQ2.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ3.inverse() - FQ3.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FQ4.inverse() - FQ4.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(1.0, FQ1.determinant(), TOL_HIGH);
	EXPECT_NEAR(1.0, FQ2.determinant(), TOL_HIGH);
	EXPECT_NEAR(1.0, FQ3.determinant(), TOL_HIGH);
	EXPECT_NEAR(1.0, FQ4.determinant(), TOL_HIGH);
	EXPECT_GT(FR1.x(), 0.0);
	EXPECT_GT(FR1.y(), 0.0);
	EXPECT_GT(FR1.z(), 0.0);
	EXPECT_GT(FR2.x(), 0.0);
	EXPECT_GT(FR2.y(), 0.0);
	EXPECT_GT(FR2.z(), 0.0);
	EXPECT_GT(FR3.x(), 0.0);
	EXPECT_GT(FR3.y(), 0.0);
	EXPECT_GT(FR3.z(), 0.0);
	EXPECT_GT(FR4.x(), 0.0);
	EXPECT_GT(FR4.y(), 0.0);
	EXPECT_GT(FR4.z(), 0.0);
	
	// Convert the rotation matrix and radii to a normalisation matrix
	Eigen::Matrix3d FW1, FW2, FW3, FW4;
	ASSERT_TRUE(ConicFit::ellipsoidAxesToTransform(FQ1, FR1, FW1));
	ASSERT_TRUE(ConicFit::ellipsoidAxesToTransform(FQ2, FR2, FW2));
	ASSERT_TRUE(ConicFit::ellipsoidAxesToTransform(FQ3, FR3, FW3));
	ASSERT_TRUE(ConicFit::ellipsoidAxesToTransform(FQ4, FR4, FW4));
	
	// Check the normalisation matrices
	EXPECT_NEAR(0.0, (FW1*FW1 - FA1).norm(), TOL_MED);
	EXPECT_NEAR(0.0, (FW2*FW2 - FA2).norm(), TOL_MED);
	EXPECT_NEAR(0.0, (FW3*FW3 - FA3).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW4*FW4 - FA4).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW1 - FW1.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW2 - FW2.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW3 - FW3.transpose()).norm(), TOL_HIGH);
	EXPECT_NEAR(0.0, (FW4 - FW4.transpose()).norm(), TOL_HIGH);
	for(size_t i = 1; i <= 3; i++)
	{
		EXPECT_GT(FW1.topLeftCorner(i,i).determinant(), 0.0); // A matrix is positive definite iff all leading principal minors are positive
		EXPECT_GT(FW2.topLeftCorner(i,i).determinant(), 0.0);
		EXPECT_GT(FW3.topLeftCorner(i,i).determinant(), 0.0);
		EXPECT_GT(FW4.topLeftCorner(i,i).determinant(), 0.0);
	}
	
	// Check fitting errors
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidError(PP1, C1, FW1), TOL_MED);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidError(PP2, C2, FW2), TOL_LOW);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidError(PP3, C3, FW3), TOL_HIGH);
	EXPECT_NEAR(0.0, ConicFit::fitEllipsoidError(PP4, C4, FW4), TOL_MED);
}

// Test fitting errors
TEST(ConicFitTest, testFitError)
{
	// Sanity check fitting error for circle
	ConicFit::Points2D P1;
	ConicFit::Weights W1;
	ConicFit::WeightedPoints2D WP1;
	Eigen::Vector2d C1(0.6, -0.8);
	double R1 = 0.5, S1 = 1.3;
	ConicFit::genCircleData(P1, 60, C1, R1*S1);
	genWeights(W1, P1.size(), 0.0, 1.0);
	mergeWeights(WP1, P1, W1);
	EXPECT_NEAR(fabs(S1 - 1.0), ConicFit::fitCircleError(P1, C1, R1), TOL_HIGH);
	EXPECT_NEAR(fabs(S1 - 1.0), ConicFit::fitCircleErrorWeighted(WP1, C1, R1), TOL_HIGH);
	EXPECT_NEAR(fabs(S1 - 1.0), ConicFit::fitCircleErrorWeighted(P1, W1, C1, R1), TOL_HIGH);
	
	// Sanity check fitting error for ellipse
	Eigen::Vector2d C2(-0.6, 0.8);
	Eigen::Vector2d R2(3.0, 1.2);
	Eigen::Matrix2d W2;
	double Ang2 = 2.19, S2 = 1.7;
	ConicFit::Points2D P2;
	ConicFit::genEllipseData(P2, 60, C2, R2*S2, Ang2);
	ConicFit::ellipseAxesToTransform((Eigen::Matrix2d() << cos(Ang2), -sin(Ang2), sin(Ang2), cos(Ang2)).finished(), R2, W2);
	EXPECT_NEAR(fabs(S2 - 1.0), ConicFit::fitEllipseError(P2, C2, W2), TOL_HIGH);
	
	// Sanity check fitting error for sphere
	ConicFit::Points3D PP1;
	Eigen::Vector3d CC1(0.6, -0.8, 0.4);
	double RR1 = 7.5, SS1 = 0.6;
	ConicFit::genSphereData(PP1, 20, CC1, RR1*SS1);
	EXPECT_NEAR(fabs(SS1 - 1.0), ConicFit::fitSphereError(PP1, CC1, RR1), TOL_HIGH);
	
	// Sanity check fitting error for ellipsoid
	Eigen::Vector3d CC2(1.4, -4.3, -0.5);
	Eigen::Vector3d RR2(3.3, 1.8, 0.7);
	Eigen::Matrix3d QQ2(Eigen::AngleAxisd(2.39, Eigen::Vector3d(-0.2, 0.8, 0.4).normalized()));
	Eigen::Matrix3d WW2;
	double SS2 = 0.84;
	ConicFit::Points3D PP2;
	ConicFit::genEllipsoidData(PP2, 15, CC2, RR2*SS2, QQ2);
	ConicFit::ellipsoidAxesToTransform(QQ2, RR2, WW2);
	EXPECT_NEAR(fabs(SS2 - 1.0), ConicFit::fitEllipsoidError(PP2, CC2, WW2), TOL_HIGH);
}

// Main function
int main(int argc, char **argv)
{
	// Ensure repeatability of random number generation
	srand48(0xF9EB283A);
	
	// Run all the tests
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF