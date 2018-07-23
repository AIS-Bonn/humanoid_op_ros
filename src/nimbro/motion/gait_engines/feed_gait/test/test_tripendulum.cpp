// Unit test for tripendulum model
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/model/tilt_phase/tripendulum_model.h>
#include <gtest/gtest.h>

// Namespaces
using namespace tripendulum;

// Test: Tripendulum model parameters
TEST(TripendulumTest, testTriPendModelParam)
{
	// Test the default model parameters
	TriPendModel TPM1;
	EXPECT_EQ(0.0, TPM1.thb);
	EXPECT_EQ(0.0, TPM1.thm);
	EXPECT_EQ(0.0, TPM1.thf);
	EXPECT_EQ(9.81, TPM1.Cbsq);
	EXPECT_EQ(9.81, TPM1.Cmsq);
	EXPECT_EQ(9.81, TPM1.Cfsq);
	EXPECT_DOUBLE_EQ(0.0, TPM1.tht);
	EXPECT_DOUBLE_EQ(0.0, TPM1.ths);

	// Test custom model parameters
	TriPendModel TPM2(-0.42, 0.03, 0.37, 21.0, -17.0, 34.0);
	EXPECT_EQ(-0.42, TPM2.thb);
	EXPECT_EQ( 0.03, TPM2.thm);
	EXPECT_EQ( 0.37, TPM2.thf);
	EXPECT_EQ(21.0, TPM2.Cbsq);
	EXPECT_EQ(17.0, TPM2.Cmsq);
	EXPECT_EQ(34.0, TPM2.Cfsq);
	EXPECT_DOUBLE_EQ(-0.2190874845759425, TPM2.tht);
	EXPECT_DOUBLE_EQ( 0.2571566180304092, TPM2.ths);

	// Test changing model parameters
	TPM2.setParam(-0.33, -0.02, 0.38, 6.0, 19.0, 12.0);
	EXPECT_EQ(-0.33, TPM2.thb);
	EXPECT_EQ(-0.02, TPM2.thm);
	EXPECT_EQ( 0.38, TPM2.thf);
	EXPECT_EQ(6.0, TPM2.Cbsq);
	EXPECT_EQ(19.0, TPM2.Cmsq);
	EXPECT_EQ(12.0, TPM2.Cfsq);
	EXPECT_DOUBLE_EQ(-0.0939263632746346, TPM2.tht);
	EXPECT_DOUBLE_EQ( 0.1342586939059573, TPM2.ths);

	// Test resetting parameters
	TPM2.resetParam();
	EXPECT_EQ(0.0, TPM2.thb);
	EXPECT_EQ(0.0, TPM2.thm);
	EXPECT_EQ(0.0, TPM2.thf);
	EXPECT_EQ(9.81, TPM2.Cbsq);
	EXPECT_EQ(9.81, TPM2.Cmsq);
	EXPECT_EQ(9.81, TPM2.Cfsq);
	EXPECT_DOUBLE_EQ(0.0, TPM2.tht);
	EXPECT_DOUBLE_EQ(0.0, TPM2.ths);

	// Test manually changing parameters and recalculating
	TPM2.thb = -0.34;
	TPM2.Cfsq = 14.0;
	EXPECT_EQ(-0.34, TPM2.thb);
	EXPECT_EQ( 0.0, TPM2.thm);
	EXPECT_EQ( 0.0, TPM2.thf);
	EXPECT_EQ(9.81, TPM2.Cbsq);
	EXPECT_EQ(9.81, TPM2.Cmsq);
	EXPECT_EQ(14.0, TPM2.Cfsq);
	EXPECT_DOUBLE_EQ(0.0, TPM2.tht);
	EXPECT_DOUBLE_EQ(0.0, TPM2.ths);
	TPM2.recalc();
	EXPECT_EQ(-0.34, TPM2.thb);
	EXPECT_EQ( 0.0, TPM2.thm);
	EXPECT_EQ( 0.0, TPM2.thf);
	EXPECT_EQ(9.81, TPM2.Cbsq);
	EXPECT_EQ(9.81, TPM2.Cmsq);
	EXPECT_EQ(14.0, TPM2.Cfsq);
	EXPECT_DOUBLE_EQ(-0.17, TPM2.tht);
	EXPECT_DOUBLE_EQ(0.0, TPM2.ths);
}

// Test: Tripendulum model parameters
TEST(TripendulumTest, testTriPendModelProp)
{
	// Declare variables
	CrossingEnergy CE;

	// Define the model parameters
	TriPendModel TPM(-0.32, 0.04, 0.31, 17.0, 32.0, 21.0);

	// Test the model parameters
	EXPECT_EQ(-0.32, TPM.thb);
	EXPECT_EQ( 0.04, TPM.thm);
	EXPECT_EQ( 0.31, TPM.thf);
	EXPECT_EQ(17.0, TPM.Cbsq);
	EXPECT_EQ(32.0, TPM.Cmsq);
	EXPECT_EQ(21.0, TPM.Cfsq);
	EXPECT_DOUBLE_EQ(-0.0843525533733478, TPM.tht);
	EXPECT_DOUBLE_EQ( 0.1468171313786484, TPM.ths);

	// Test the acceleration
	EXPECT_DOUBLE_EQ(-1.6971680829960782, TPM.accel(-0.42));
	EXPECT_EQ(0.0, TPM.accel(-0.32));
	EXPECT_DOUBLE_EQ(2.5572702070135263, TPM.accel(-0.04));
	EXPECT_EQ(0.0, TPM.accel(0.04));
	EXPECT_DOUBLE_EQ(-3.3456823388991657, TPM.accel(0.15));
	EXPECT_EQ(0.0, TPM.accel(0.31));
	EXPECT_DOUBLE_EQ(0.2099965000175001, TPM.accel(0.32));

	// Test the local energy
	EXPECT_DOUBLE_EQ(-0.0798583805471221, TPM.localEnergy(-0.42, 0.3));
	EXPECT_DOUBLE_EQ(-0.0798583805471221, TPM.localEnergyB(-0.42, 0.3));
	EXPECT_EQ(0.0, TPM.localEnergy(-0.32, 0.0));
	EXPECT_EQ(0.0, TPM.localEnergyB(-0.32, 0.0));
	EXPECT_DOUBLE_EQ(0.2446907966323559, TPM.localEnergy(-0.04, -0.2));
	EXPECT_DOUBLE_EQ(0.2446907966323559, TPM.localEnergyM(-0.04, -0.2));
	EXPECT_EQ(0.0, TPM.localEnergy(0.04, 0.0));
	EXPECT_EQ(0.0, TPM.localEnergyM(0.04, 0.0));
	EXPECT_DOUBLE_EQ(-0.3764540982236689, TPM.localEnergy(0.15, 0.4));
	EXPECT_DOUBLE_EQ(-0.3764540982236689, TPM.localEnergyF(0.15, 0.4));
	EXPECT_EQ(0.0, TPM.localEnergy(0.31, 0.0));
	EXPECT_EQ(0.0, TPM.localEnergyF(0.31, 0.0));
	EXPECT_DOUBLE_EQ(0.3579000174999410, TPM.localEnergy(0.32, -0.6));
	EXPECT_DOUBLE_EQ(0.3579000174999410, TPM.localEnergyF(0.32, -0.6));

	// Test the potential crossing energy
	CE = TPM.crossingEnergyPE(-0.42);
	EXPECT_DOUBLE_EQ(0.1698583805471188, CE.B);
	EXPECT_DOUBLE_EQ(0.3412514930310735, CE.F);
	CE = TPM.crossingEnergyPE(-0.32);
	EXPECT_EQ(0.0, CE.B);
	EXPECT_DOUBLE_EQ(0.5111098735781923, CE.F);
	CE = TPM.crossingEnergyPE(-0.04);
	EXPECT_DOUBLE_EQ(-1.2291506557357863, CE.B);
	EXPECT_DOUBLE_EQ(-0.7180407821575940, CE.F);
	CE = TPM.crossingEnergyPE(0.04);
	EXPECT_DOUBLE_EQ(-1.4338414523681422, CE.B);
	EXPECT_DOUBLE_EQ(-0.9227315787899499, CE.F);
	CE = TPM.crossingEnergyPE(0.15);
	EXPECT_DOUBLE_EQ(-1.0475639718018641, CE.B);
	EXPECT_DOUBLE_EQ(-0.5364540982236718, CE.F);
	CE = TPM.crossingEnergyPE(0.31);
	EXPECT_DOUBLE_EQ(-0.5111098735781923, CE.B);
	EXPECT_EQ(0.0, CE.F);
	CE = TPM.crossingEnergyPE(0.32);
	EXPECT_DOUBLE_EQ(-0.5132098560782481, CE.B);
	EXPECT_DOUBLE_EQ(0.0020999825000558303, CE.F);

	// Test the kinetic crossing energy
	CE = TPM.crossingEnergyKE(-0.7);
	EXPECT_DOUBLE_EQ(0.49, CE.B);
	EXPECT_DOUBLE_EQ(-0.49, CE.F);
	CE = TPM.crossingEnergyKE(0.0);
	EXPECT_DOUBLE_EQ(0.0, CE.B);
	EXPECT_DOUBLE_EQ(0.0, CE.F);
	CE = TPM.crossingEnergyKE(0.4);
	EXPECT_DOUBLE_EQ(-0.16, CE.B);
	EXPECT_DOUBLE_EQ(0.16, CE.F);

	// Test the crossing energy
	CE = TPM.crossingEnergy(-0.42, 0.3);
	EXPECT_DOUBLE_EQ(0.0798583805471188, CE.B);
	EXPECT_DOUBLE_EQ(0.4312514930310735, CE.F);
	CE = TPM.crossingEnergy(-0.32, 0.0);
	EXPECT_EQ(0.0, CE.B);
	EXPECT_DOUBLE_EQ(0.5111098735781923, CE.F);
	CE = TPM.crossingEnergy(-0.04, -0.2);
	EXPECT_DOUBLE_EQ(-1.1891506557357863, CE.B);
	EXPECT_DOUBLE_EQ(-0.7580407821575941, CE.F);
	CE = TPM.crossingEnergy(0.04, 0.0);
	EXPECT_DOUBLE_EQ(-1.4338414523681422, CE.B);
	EXPECT_DOUBLE_EQ(-0.9227315787899499, CE.F);
	CE = TPM.crossingEnergy(0.15, 0.4);
	EXPECT_DOUBLE_EQ(-1.2075639718018643, CE.B);
	EXPECT_DOUBLE_EQ(-0.3764540982236718, CE.F);
	CE = TPM.crossingEnergy(0.31, 0.0);
	EXPECT_DOUBLE_EQ(-0.5111098735781923, CE.B);
	EXPECT_EQ(0.0, CE.F);
	CE = TPM.crossingEnergy(0.32, -0.6);
	EXPECT_DOUBLE_EQ(-0.1532098560782481, CE.B);
	EXPECT_DOUBLE_EQ(-0.3579000174999442, CE.F);
}

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF