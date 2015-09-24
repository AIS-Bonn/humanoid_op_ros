// Unit test for the rotations conversion package
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rot_conv/rot_conv.h>
#include <gtest/gtest.h>

// Namespaces
using namespace rot_conv;

// Test the Fused struct
TEST(RotConvTest, testFusedStruct)
{
	// Test the constructors
	Fused fa; // This should compile and leave the members uninitialized
	Fused fb(1.0, 2.0);
	EXPECT_EQ(0.0, fb.fYaw);
	EXPECT_EQ(1.0, fb.fPitch);
	EXPECT_EQ(2.0, fb.fRoll);
	EXPECT_TRUE(true == fb.hemi);
	Fused fc(4.0, 3.0, 2.0);
	EXPECT_EQ(4.0, fc.fYaw);
	EXPECT_EQ(3.0, fc.fPitch);
	EXPECT_EQ(2.0, fc.fRoll);
	EXPECT_TRUE(true == fc.hemi);
	Fused fd(1.0, 2.0, 3.0, false);
	EXPECT_EQ(1.0, fd.fYaw);
	EXPECT_EQ(2.0, fd.fPitch);
	EXPECT_EQ(3.0, fd.fRoll);
	EXPECT_TRUE(false == fd.hemi);
}

// Worker function: testFusedFromQuat
void checkFusedFromQuat(const Quat& q, const Fused& ftrue)
{
	// Declare variables
	double fYaw = -100.0, fPitch = -100.0, fRoll = -100.0;
	bool hemi = false;

	// Test the various function overloads
	FusedFromQuat(q, fPitch, fRoll);
	EXPECT_DOUBLE_EQ(ftrue.fPitch, fPitch);
	EXPECT_DOUBLE_EQ(ftrue.fRoll, fRoll);
	FusedFromQuat(q, fYaw, fPitch, fRoll);
	EXPECT_DOUBLE_EQ(ftrue.fYaw, fYaw);
	EXPECT_DOUBLE_EQ(ftrue.fPitch, fPitch);
	EXPECT_DOUBLE_EQ(ftrue.fRoll, fRoll);
	FusedFromQuat(q, fYaw, fPitch, fRoll, hemi);
	EXPECT_DOUBLE_EQ(ftrue.fYaw, fYaw);
	EXPECT_DOUBLE_EQ(ftrue.fPitch, fPitch);
	EXPECT_DOUBLE_EQ(ftrue.fRoll, fRoll);
	EXPECT_TRUE(ftrue.hemi == hemi);
	Fused f = FusedFromQuat(q);
	EXPECT_DOUBLE_EQ(ftrue.fYaw, f.fYaw);
	EXPECT_DOUBLE_EQ(ftrue.fPitch, f.fPitch);
	EXPECT_DOUBLE_EQ(ftrue.fRoll, f.fRoll);
	EXPECT_TRUE(ftrue.hemi == f.hemi);
}

// Test the conversion from quaternion to fused angles
TEST(RotConvTest, testFusedFromQuat)
{
	// Test the identity case
	Quat  qa(1.0, 0.0, 0.0, 0.0);
	Fused fa(0.0, 0.0, 0.0, true);
	checkFusedFromQuat(qa, fa);

	// Test a random quaternion in the positive hemisphere
	Quat  qb(0.7222856332669739, 0.5029423807110762, 0.3807601824433657, 0.2835032788637483);
	Fused fb(0.7480631709823684, 0.2680624583862969, 1.2298230154711272, true);
	checkFusedFromQuat(qb, fb);

	// Test a random quaternion in the negative hemisphere
	Quat  qc(-0.1177398416985938, 0.6616531189965961, -0.3860555283890073, -0.6319126591540564);
	Fused fc( 2.7731713536579932, 1.1866593328830430,  0.3385300011717289, false);
	checkFusedFromQuat(qc, fc);
}

// Worker function: testQuatFromFused
void checkQuatFromFused(const Fused& f, const Quat& q1, const Quat& q2, const Quat& q3)
{
	// Declare variables
	Quat qout;

	// Test the various function overloads
	qout = QuatFromFused(f.fPitch, f.fRoll);
	EXPECT_DOUBLE_EQ(q1.w(), qout.w());
	EXPECT_DOUBLE_EQ(q1.x(), qout.x());
	EXPECT_DOUBLE_EQ(q1.y(), qout.y());
	EXPECT_DOUBLE_EQ(q1.z(), qout.z());
	qout = QuatFromFused(f.fYaw, f.fPitch, f.fRoll);
	EXPECT_DOUBLE_EQ(q2.w(), qout.w());
	EXPECT_DOUBLE_EQ(q2.x(), qout.x());
	EXPECT_DOUBLE_EQ(q2.y(), qout.y());
	EXPECT_DOUBLE_EQ(q2.z(), qout.z());
	qout = QuatFromFused(f.fYaw, f.fPitch, f.fRoll, f.hemi);
	EXPECT_DOUBLE_EQ(q3.w(), qout.w());
	EXPECT_DOUBLE_EQ(q3.x(), qout.x());
	EXPECT_DOUBLE_EQ(q3.y(), qout.y());
	EXPECT_DOUBLE_EQ(q3.z(), qout.z());
	qout = QuatFromFused(f);
	EXPECT_DOUBLE_EQ(q3.w(), qout.w());
	EXPECT_DOUBLE_EQ(q3.x(), qout.x());
	EXPECT_DOUBLE_EQ(q3.y(), qout.y());
	EXPECT_DOUBLE_EQ(q3.z(), qout.z());
}

// Test the conversion from fused angles to quaternion
TEST(RotConvTest, testQuatFromFused)
{
	// Test the identity case
	Fused fa(0.0, 0.0, 0.0, true);
	Quat qa1(1.0, 0.0, 0.0, 0.0);
	checkQuatFromFused(fa, qa1, qa1, qa1);
	
	// Test a random fused angles rotation in the positive hemisphere
	Fused fb(2.4534701287862291, -0.2650491423322411, 0.2746203053006582, true);
	Quat qb1(0.9813756008800009, 0.1381639656628278, -0.1334640348173924, 0.0000000000000000);
	Quat qb2(0.3310308515488257, 0.1722465598310650,  0.0850473551839241, 0.9238596459016067);
	checkQuatFromFused(fb, qb1, qb2, qb2);

	// Test a random fused angles rotation in the negative hemisphere
	Fused fc(2.3862836331701032, -0.3599342692652525, -0.7344627162396046, false);
	Quat qc1(0.9092014129497605, -0.3685588669926899, -0.1936934491696190, 0.0000000000000000);
	Quat qc2(0.3352600907094059,  0.0441414053240089, -0.4140100566727584, 0.8451318718918143);
	Quat qc3(0.1535278524138601,  0.0963919661500578, -0.9040773186910327, 0.3870167815784569);
	checkQuatFromFused(fc, qc1, qc2, qc3);

	// Test a fused angles rotation in violation of the sine sum criterion
	Fused fd(0.4, 1.2, -1.4, true);
	Quat qd1(0.7071067811865476, -0.5137279825527999, 0.4858843071578151, 0.0000000000000000);
	Quat qd2(0.6930117232058354, -0.6000179359486728, 0.3741369756384608, 0.1404804310189812);
	checkQuatFromFused(fd, qd1, qd2, qd2);
}

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF