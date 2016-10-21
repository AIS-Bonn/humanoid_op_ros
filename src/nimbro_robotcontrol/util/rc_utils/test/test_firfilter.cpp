// Unit test for the FIRFilter class
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/firfilter.h>
#include <gtest/gtest.h>

// Namespaces
using namespace rc_utils;

// Test construction of a FIRFilter object
TEST(FIRFilterTest, testConstruction)
{
	// Declare variables
	double val[9] = {0};
	int i, tmp;

	// Test default construction
	FIRFilter<> fira;
	tmp = fira.size; ASSERT_EQ(5, tmp);
	fira.getCoeff(val, 9);
	EXPECT_EQ(1.0, val[0]);
	for(i = 1; i < 9; i++)
		EXPECT_EQ(0.0, val[i]);
	fira.getBuf(val);
	for(i = 0; i < 5; i++)
		EXPECT_EQ(0.0, val[i]);
	EXPECT_EQ(0.0, fira.get());
	EXPECT_EQ(0.0, fira.value());

	// Test alternate constructor
	FIRFilter<5> firb(1.0, 2.0, 1.5, 0.5);
	tmp = firb.size; ASSERT_EQ(5, tmp);
	firb.getCoeff(val, 9);
	EXPECT_EQ(0.2, val[0]);
	EXPECT_EQ(0.4, val[1]);
	EXPECT_EQ(0.3, val[2]);
	EXPECT_EQ(0.1, val[3]);
	for(i = 4; i < 9; i++)
		EXPECT_EQ(0.0, val[i]);
	firb.getBuf(val);
	for(i = 0; i < 5; i++)
		EXPECT_EQ(0.0, val[i]);
	EXPECT_EQ(0.0, firb.get());
	EXPECT_EQ(0.0, firb.value());

	// Test different window size
	FIRFilter<9> firc;
	tmp = firc.size; ASSERT_EQ(9, tmp);
	firc.getCoeff(val, 9);
	EXPECT_EQ(1.0, val[0]);
	for(i = 1; i < 9; i++)
		EXPECT_EQ(0.0, val[i]);
	firc.getBuf(val);
	for(i = 0; i < 9; i++)
		EXPECT_EQ(0.0, val[i]);
	EXPECT_EQ(0.0, firc.get());
	EXPECT_EQ(0.0, firc.value());

	// Test reset
	firb.resetAll();
	tmp = firb.size; ASSERT_EQ(5, tmp);
	firb.getCoeff(val, 9);
	EXPECT_EQ(1.0, val[0]);
	for(i = 1; i < 9; i++)
		EXPECT_EQ(0.0, val[i]);
	firb.getBuf(val);
	for(i = 0; i < 5; i++)
		EXPECT_EQ(0.0, val[i]);
	EXPECT_EQ(0.0, firb.get());
	EXPECT_EQ(0.0, firb.value());
}

// Test setting the coefficients
TEST(FIRFilterTest, testSetCoeff)
{
	// Declare variables
	double val[9] = {0};
	int i;

	// FIR filter object
	FIRFilter<5> fir;

	// Test default coefficients
	fir.getCoeff(val, 9);
	EXPECT_EQ(1.0, val[0]);
	for(i = 1; i < 9; i++)
		EXPECT_EQ(0.0, val[i]);

	// Test first setCoeff overload
	fir.setCoeff(1.4, 1.0, -0.4);
	fir.getCoeff(val, 9);
	EXPECT_EQ(0.7, val[0]);
	EXPECT_EQ(0.5, val[1]);
	EXPECT_EQ(-0.2, val[2]);
	for(i = 3; i < 9; i++)
		EXPECT_EQ(0.0, val[i]);

	// Test second setCoeff overload
	double newval[9] = {2.0, 1.0, 1.5, 4.5, 1.0, 2.0, 3.0, 1.5, 0.5};
	fir.setCoeff(newval, 9);
	fir.getCoeff(val, 9);
	EXPECT_EQ(0.20, val[0]);
	EXPECT_EQ(0.10, val[1]);
	EXPECT_EQ(0.15, val[2]);
	EXPECT_EQ(0.45, val[3]);
	EXPECT_EQ(0.10, val[4]);
	for(i = 5; i < 9; i++)
		EXPECT_EQ(0.0, val[i]);
}

// Test setting the buffer contents
TEST(FIRFilterTest, testSetBuf)
{
	// Declare variables
	double buf[5] = {-1, -1, -1, -1, -1};
	int i;

	// FIR filter object
	FIRFilter<5> fir;

	// Test default buffer values
	fir.getBuf(buf);
	for(i = 0; i < 5; i++)
		EXPECT_EQ(0.0, buf[i]);

	// Test first setBuf overload
	fir.setBuf();
	fir.getBuf(buf);
	for(i = 0; i < 5; i++)
		EXPECT_EQ(0.0, buf[i]);
	fir.setBuf(0.3);
	fir.getBuf(buf);
	for(i = 0; i < 5; i++)
		EXPECT_EQ(0.3, buf[i]);
	fir.setBuf(1.0, -0.2);
	fir.getBuf(buf);
	EXPECT_EQ(1.8, buf[0]);
	EXPECT_EQ(1.6, buf[1]);
	EXPECT_EQ(1.4, buf[2]);
	EXPECT_EQ(1.2, buf[3]);
	EXPECT_EQ(1.0, buf[4]);

	// Test second setBuf overload
	double newbuf[5] = {1.7, 0.4, 1.3, 1.0, 0.8};
	fir.setBuf(newbuf);
	fir.getBuf(buf);
	EXPECT_EQ(1.7, buf[0]);
	EXPECT_EQ(0.4, buf[1]);
	EXPECT_EQ(1.3, buf[2]);
	EXPECT_EQ(1.0, buf[3]);
	EXPECT_EQ(0.8, buf[4]);

	// Test clearing the buffer
	fir.reset();
	fir.getBuf(buf);
	for(i = 0; i < 5; i++)
		EXPECT_EQ(0.0, buf[i]);
}

// Test putting values into the buffer
TEST(FIRFilterTest, testPutIntoBuffer)
{
	// Declare variables
	double buf[5] = {-1, -1, -1, -1, -1};
	int i;

	// FIR filter object
	FIRFilter<5> fir;

	// Test putting three values into the buffer
	fir.put(0.7);
	fir.put(1.1);
	fir.put(0.9);
	fir.getBuf(buf);
	EXPECT_EQ(0.0, buf[0]);
	EXPECT_EQ(0.0, buf[1]);
	EXPECT_EQ(0.7, buf[2]);
	EXPECT_EQ(1.1, buf[3]);
	EXPECT_EQ(0.9, buf[4]);

	// Test putting a further four values into the buffer (overflow)
	fir.put(2.1);
	fir.put(1.9);
	fir.put(2.3);
	fir.put(2.0);
	fir.getBuf(buf);
	EXPECT_EQ(0.9, buf[0]);
	EXPECT_EQ(2.1, buf[1]);
	EXPECT_EQ(1.9, buf[2]);
	EXPECT_EQ(2.3, buf[3]);
	EXPECT_EQ(2.0, buf[4]);

	// Test filling the buffer values
	fir.setBuf(1.5);
	fir.getBuf(buf);
	for(i = 0; i < 5; i++)
		EXPECT_EQ(1.5, buf[i]);

	// Test putting three values into the buffer
	fir.put(0.7);
	fir.put(1.1);
	fir.put(0.9);
	fir.getBuf(buf);
	EXPECT_EQ(1.5, buf[0]);
	EXPECT_EQ(1.5, buf[1]);
	EXPECT_EQ(0.7, buf[2]);
	EXPECT_EQ(1.1, buf[3]);
	EXPECT_EQ(0.9, buf[4]);
}

// Test the filter output
TEST(FIRFilterTest, testFilterOuput)
{
	// FIR filter object
	FIRFilter<5> fir(1, 2, 4, -1, 2);

	// Test the initial output
	EXPECT_EQ(0.0, fir.get());
	EXPECT_EQ(0.0, fir.value());
	EXPECT_EQ(0.0, fir.value());

	// Test the output after putting values into the buffer
	fir.put(1.0);
	fir.put(2.0);
	fir.put(-1.0);
	EXPECT_EQ(0.875, fir.get());
	EXPECT_EQ(0.875, fir.value());
	EXPECT_EQ(0.875, fir.value());

	// Test the output after putting more values into the buffer (overflow)
	fir.put(1.5);
	fir.put(0.5);
	fir.put(2.0);
	fir.put(1.0);
	EXPECT_EQ(0.4375, fir.get());
	EXPECT_EQ(0.4375, fir.value());
	EXPECT_EQ(0.4375, fir.value());
}

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF