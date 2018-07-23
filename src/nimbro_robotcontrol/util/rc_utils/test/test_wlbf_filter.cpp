// Unit test of WLBF filter
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/wlbf_filter.h>
#include <rc_utils/wlbf_filter_nd.h>
#include <gtest/gtest.h>

// Defines
#define PRINT_TESTING 0

// Namespaces
using namespace rc_utils;

// Override default print behaviour of Eigen vectors
template<class Scalar, int N> std::ostream& operator<<(std::ostream& os, const Eigen::Matrix<Scalar, N, 1>& v)
{
	// Print the vector
	os << "(" << v[0];
	for(int i = 1; i < N; i++)
		os << ", " << v[i];
	return os << ")";
}

// Custom testing macros
#define EXPECT_EQ3(e,a) do { EXPECT_EQ(e[0], a[0]); EXPECT_EQ(e[1], a[1]); EXPECT_EQ(e[2], a[2]); } while(0)
#define EXPECT_DOUBLE_EQ3(e,a) do { EXPECT_DOUBLE_EQ(e[0], a[0]); EXPECT_DOUBLE_EQ(e[1], a[1]); EXPECT_DOUBLE_EQ(e[2], a[2]); } while(0)

// Print state functions
void printState(const WLBFFilter& WF)
{
#if PRINT_TESTING
	// Print the state of the filter
	std::cout << std::endl << "PRINTING" << std::endl;
	for(std::size_t i = 0; i < WF.len(); i++)
		std::cout << "[" << i << "] = (" << WF.XBuf()[i] << ", " << WF.YBuf()[i] << ", " << WF.WBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", X = " << WF.getX() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
}
template<int N> void printState(const WLBFFilterND<N>& WF)
{
#if PRINT_TESTING
	// Print the state of the filter
	std::cout << std::endl << "PRINTING" << std::endl;
	for(std::size_t i = 0; i < WF.len(); i++)
		std::cout << "[" << i << "] = (" << WF.TBuf()[i] << ", " << WF.PBuf()[i] << ", " << WF.WBuf()[i] << ")" << std::endl;
	std::cout << "A = " << WF.getA() << ", B = " << WF.getB() << ", X = " << WF.getT() << ", Value = " << WF.value() << ", Deriv = " << WF.deriv() << std::endl;
#endif
}

// Test the Boost circular buffer class for an important property
TEST(WLBFTest, testBoostCircularBuffer)
{
	// Declare variables
	boost::circular_buffer<int> buf;

	// Test push_back/rset_capacity behaviour
	buf.clear();
	buf.rset_capacity(5);
	EXPECT_EQ(0, buf.size());
	EXPECT_EQ(5, buf.capacity());
	buf.push_back(1);
	EXPECT_EQ(1, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(1, buf[0]);
	buf.push_back(2);
	buf.push_back(3);
	buf.push_back(4);
	buf.push_back(5);
	EXPECT_EQ(5, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(1, buf[0]);
	EXPECT_EQ(2, buf[1]);
	EXPECT_EQ(3, buf[2]);
	EXPECT_EQ(4, buf[3]);
	EXPECT_EQ(5, buf[4]);
	buf.push_back(6);
	buf.push_back(7);
	EXPECT_EQ(5, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(3, buf[0]);
	EXPECT_EQ(4, buf[1]);
	EXPECT_EQ(5, buf[2]);
	EXPECT_EQ(6, buf[3]);
	EXPECT_EQ(7, buf[4]);
	buf.rset_capacity(3); // Note: This should retain the three most recent values pushed back
	EXPECT_EQ(3, buf.size());
	EXPECT_EQ(3, buf.capacity());
	EXPECT_EQ(5, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(7, buf[2]);
	buf.rset_capacity(5);
	EXPECT_EQ(3, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(5, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(7, buf[2]);

	// Test push_front/set_capacity behaviour
	buf.clear();
	buf.set_capacity(5);
	EXPECT_EQ(0, buf.size());
	EXPECT_EQ(5, buf.capacity());
	buf.push_front(1);
	EXPECT_EQ(1, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(1, buf[0]);
	buf.push_front(2);
	buf.push_front(3);
	buf.push_front(4);
	buf.push_front(5);
	EXPECT_EQ(5, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(5, buf[0]);
	EXPECT_EQ(4, buf[1]);
	EXPECT_EQ(3, buf[2]);
	EXPECT_EQ(2, buf[3]);
	EXPECT_EQ(1, buf[4]);
	buf.push_front(6);
	buf.push_front(7);
	EXPECT_EQ(5, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(7, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(5, buf[2]);
	EXPECT_EQ(4, buf[3]);
	EXPECT_EQ(3, buf[4]);
	buf.set_capacity(3); // Note: This should retain the three most recent values pushed front
	EXPECT_EQ(3, buf.size());
	EXPECT_EQ(3, buf.capacity());
	EXPECT_EQ(7, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(5, buf[2]);
	buf.set_capacity(5);
	EXPECT_EQ(3, buf.size());
	EXPECT_EQ(5, buf.capacity());
	EXPECT_EQ(7, buf[0]);
	EXPECT_EQ(6, buf[1]);
	EXPECT_EQ(5, buf[2]);
}

// Test the WLBFFilter class
TEST(WLBFTest, testWLBFFilter)
{
	// Create a default WLBF filter
	WLBFFilter WFD;

	// Print the filter state
	printState(WFD);

	// Testing
	EXPECT_EQ(0, WFD.len());
	EXPECT_TRUE(WFD.lenZero());
	EXPECT_EQ(WFD.XBuf().capacity(), WFD.XBuf().size());
	EXPECT_EQ(WFD.YBuf().capacity(), WFD.YBuf().size());
	EXPECT_EQ(WFD.WBuf().capacity(), WFD.WBuf().size());
	EXPECT_DOUBLE_EQ(0.0, WFD.getA());
	EXPECT_DOUBLE_EQ(0.0, WFD.getB());
	EXPECT_DOUBLE_EQ(0.0, WFD.getX());
	EXPECT_DOUBLE_EQ(0.0, WFD.value());
	EXPECT_DOUBLE_EQ(0.0, WFD.deriv());

	// Update the filter
	WFD.update();

	// Testing
	EXPECT_EQ(0, WFD.len());
	EXPECT_TRUE(WFD.lenZero());
	EXPECT_EQ(WFD.XBuf().capacity(), WFD.XBuf().size());
	EXPECT_EQ(WFD.YBuf().capacity(), WFD.YBuf().size());
	EXPECT_EQ(WFD.WBuf().capacity(), WFD.WBuf().size());
	EXPECT_DOUBLE_EQ(0.0, WFD.getA());
	EXPECT_DOUBLE_EQ(0.0, WFD.getB());
	EXPECT_DOUBLE_EQ(0.0, WFD.getX());
	EXPECT_DOUBLE_EQ(0.0, WFD.value());
	EXPECT_DOUBLE_EQ(0.0, WFD.deriv());

	// Create WLBF filter
	WLBFFilter WF(5);

	// Add some data points
	WF.addYW(1.0, 2.0, 0.0);
	WF.addYW(-1.0, 2.0, 0.5);
	WF.addYW(-3.0, 2.0, 1.0);
	WF.addYW(1.0, 4.0, 1.5);
	WF.addYW(2.0, 2.0, 2.0);
	WF.addYW(1.0, 7.0, 2.5);
	WF.addYW(-4.0, 2.0, 3.0);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.XBuf()[0]);
	EXPECT_EQ( 2.0, WF.XBuf()[2]);
	EXPECT_EQ(-3.0, WF.XBuf()[4]);
	EXPECT_EQ( 7.0, WF.YBuf()[1]);
	EXPECT_EQ( 4.0, WF.YBuf()[3]);
	EXPECT_EQ( 9.0, WF.WBuf()[0]);
	EXPECT_EQ( 6.25, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[4]);
	EXPECT_DOUBLE_EQ(3.5888888888888890, WF.getA());
	EXPECT_DOUBLE_EQ(0.4612903225806452, WF.getB());
	EXPECT_DOUBLE_EQ(-1.0, WF.getX());
	EXPECT_DOUBLE_EQ(2.2050179211469536, WF.value());
	EXPECT_DOUBLE_EQ(0.4612903225806452, WF.deriv());

	// Truncate some elements
	WF.resize(3);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(3, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.XBuf()[0]);
	EXPECT_EQ( 2.0, WF.XBuf()[2]);
	EXPECT_EQ( 7.0, WF.YBuf()[1]);
	EXPECT_EQ( 9.0, WF.WBuf()[0]);
	EXPECT_EQ( 6.25, WF.WBuf()[1]);
	EXPECT_DOUBLE_EQ(3.6233766233766231, WF.getA());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.getB());
	EXPECT_DOUBLE_EQ(-1.1298701298701299, WF.getX());
	EXPECT_DOUBLE_EQ(2.2750022916857642, WF.value());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.deriv());

	// Pad out some elements
	WF.resize(6);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(6, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.XBuf()[0]);
	EXPECT_EQ( 2.0, WF.XBuf()[2]);
	EXPECT_EQ( 0.0, WF.XBuf()[4]);
	EXPECT_EQ( 7.0, WF.YBuf()[1]);
	EXPECT_EQ( 0.0, WF.YBuf()[3]);
	EXPECT_EQ( 9.0, WF.WBuf()[0]);
	EXPECT_EQ( 6.25, WF.WBuf()[1]);
	EXPECT_EQ( 0.0, WF.WBuf()[5]);
	EXPECT_DOUBLE_EQ(3.6233766233766231, WF.getA());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.getB());
	EXPECT_DOUBLE_EQ(-1.1298701298701299, WF.getX());
	EXPECT_DOUBLE_EQ(2.2750022916857642, WF.value());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.deriv());

	// Add some data back in again
	WF.addYW(1.0, 2.0, 0.0);
	WF.addYW(-1.0, 2.0, 0.5);
	WF.addYW(-3.0, 2.0, 1.0);
	WF.zeroYBuf();
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(6, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-3.0, WF.XBuf()[0]);
	EXPECT_EQ( 1.0, WF.XBuf()[2]);
	EXPECT_EQ( 1.0, WF.XBuf()[4]);
	EXPECT_EQ( 0.0, WF.YBuf()[1]);
	EXPECT_EQ( 0.0, WF.YBuf()[2]);
	EXPECT_EQ( 0.0, WF.YBuf()[3]);
	EXPECT_EQ( 1.0, WF.WBuf()[0]);
	EXPECT_EQ( 0.25, WF.WBuf()[1]);
	EXPECT_EQ( 4.0, WF.WBuf()[5]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(-1.2195121951219512, WF.getX());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Set the buffers manually
	double tmpx[3] = {0.2, 0.7, 0.3};
	double tmpy[3] = {100, -42, 17};
	double tmpw[3] = {0.666, 0.777, 0.333};
	WF.setXBuf(tmpx, tmpx + 3);
	WF.setYBuf(tmpy, tmpy + 2);
	WF.setWBuf(tmpw, tmpw + 3);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(6, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.2, WF.XBuf()[0]);
	EXPECT_EQ( 0.3, WF.XBuf()[2]);
	EXPECT_EQ( 0.0, WF.XBuf()[4]);
	EXPECT_EQ( -42, WF.YBuf()[1]);
	EXPECT_EQ( 0.0, WF.YBuf()[2]);
	EXPECT_EQ( 0.0, WF.YBuf()[3]);
	EXPECT_EQ( 0.666*0.666, WF.WBuf()[0]);
	EXPECT_EQ( 0.777*0.777, WF.WBuf()[1]);
	EXPECT_EQ( 0.0, WF.WBuf()[5]);
	EXPECT_NEAR(  16.4042553191489127, WF.getA(), 1e-12);
	EXPECT_NEAR(-263.9720279720280018, WF.getB(), 1e-12);
	EXPECT_NEAR(   0.4702127659574469, WF.getX(), 1e-12);
	EXPECT_NEAR(  87.7328671328672, WF.value(), 1e-12);
	EXPECT_NEAR(-263.9720279720282, WF.deriv(), 1e-12);

	// Set the x and w buffers
	WF.zeroXBuf();
	WF.setUnitW();
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(6, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.0, WF.XBuf()[0]);
	EXPECT_EQ( 0.0, WF.XBuf()[2]);
	EXPECT_EQ( 0.0, WF.XBuf()[4]);
	EXPECT_EQ( -42, WF.YBuf()[1]);
	EXPECT_EQ( 0.0, WF.YBuf()[2]);
	EXPECT_EQ( 0.0, WF.YBuf()[3]);
	EXPECT_EQ( 1.0, WF.WBuf()[0]);
	EXPECT_EQ( 1.0, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[5]);
	EXPECT_DOUBLE_EQ(9.6666666666666667, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.getX());
	EXPECT_DOUBLE_EQ(9.6666666666666667, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Reduce the size and push back elements
	WF.resize(4);
	WF.addYW(1.0, 2.0, 0.0);
	WF.addYW(-1.0, 7.0, 0.5);
	WF.addYW(-3.0, 1.0, 1.0);
	WF.addYW(2.0, 3.0, 0.0);
	WF.addYW(-2.0, 1.0, 0.5);
	WF.addYW(-4.0, 4.0, 1.0);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(4, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.XBuf()[0]);
	EXPECT_EQ( 2.0, WF.XBuf()[2]);
	EXPECT_EQ(-3.0, WF.XBuf()[3]);
	EXPECT_EQ( 1.0, WF.YBuf()[1]);
	EXPECT_EQ( 3.0, WF.YBuf()[2]);
	EXPECT_EQ( 1.0, WF.YBuf()[3]);
	EXPECT_EQ( 1.0, WF.WBuf()[0]);
	EXPECT_EQ(0.25, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[3]);

	// Set the init XY
	WF.resize(5);
	WF.initConstY(0.3, 0.1, 0.7);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.3, WF.XBuf()[0]);
	EXPECT_DOUBLE_EQ( 0.1, WF.XBuf()[2]);
	EXPECT_NEAR( 0.0, WF.XBuf()[3], 1e-15);
	EXPECT_EQ( 0.7, WF.YBuf()[1]);
	EXPECT_EQ( 0.7, WF.YBuf()[2]);
	EXPECT_EQ( 0.7, WF.YBuf()[4]);
	EXPECT_EQ( 1.0, WF.WBuf()[0]);
	EXPECT_EQ(0.25, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[3]);
	EXPECT_DOUBLE_EQ(0.7, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.1555555555555556, WF.getX());
	EXPECT_DOUBLE_EQ(0.7, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Reset the filter
	WF.reset();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.0, WF.XBuf()[0]);
	EXPECT_EQ( 0.0, WF.XBuf()[2]);
	EXPECT_EQ( 0.0, WF.XBuf()[3]);
	EXPECT_EQ( 0.0, WF.YBuf()[1]);
	EXPECT_EQ( 0.0, WF.YBuf()[2]);
	EXPECT_EQ( 0.0, WF.YBuf()[4]);
	EXPECT_EQ( 0.0, WF.WBuf()[0]);
	EXPECT_EQ( 0.0, WF.WBuf()[1]);
	EXPECT_EQ( 0.0, WF.WBuf()[4]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.getX());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Set the init XYW
	WF.resize(5);
	WF.initConstYW(0.5, 0.2, 0.1, 2.0);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.5, WF.XBuf()[0]);
	EXPECT_DOUBLE_EQ( 0.1, WF.XBuf()[2]);
	EXPECT_NEAR(-0.1, WF.XBuf()[3], 1e-15);
	EXPECT_EQ( 0.1, WF.YBuf()[1]);
	EXPECT_EQ( 0.1, WF.YBuf()[2]);
	EXPECT_EQ( 0.1, WF.YBuf()[4]);
	EXPECT_EQ( 4.0, WF.WBuf()[0]);
	EXPECT_EQ( 4.0, WF.WBuf()[1]);
	EXPECT_EQ( 4.0, WF.WBuf()[4]);
	EXPECT_DOUBLE_EQ(0.1, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.1, WF.getX());
	EXPECT_DOUBLE_EQ(0.1, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Reset the filter
	WF.resetAll(1);
	printState(WF);

	// Testing
	EXPECT_EQ(1, WF.len());
	EXPECT_FALSE(WF.lenZero());
	EXPECT_EQ(WF.XBuf().capacity(), WF.XBuf().size());
	EXPECT_EQ(WF.YBuf().capacity(), WF.YBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.0, WF.XBuf()[0]);
	EXPECT_EQ( 0.0, WF.YBuf()[0]);
	EXPECT_EQ( 0.0, WF.WBuf()[0]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.getX());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());
}

// Test the 1D WLBFFilterND class
TEST(WLBFTest, testWLBFFilterND1D)
{
	// Create a default WLBF filter
	WLBFFilterND<1> WFD;

	// Print the filter state
	printState(WFD);

	// Testing
	EXPECT_EQ(0, WFD.len());
	EXPECT_TRUE(WFD.lenZero());
	EXPECT_EQ(WFD.TBuf().capacity(), WFD.TBuf().size());
	EXPECT_EQ(WFD.PBuf().capacity(), WFD.PBuf().size());
	EXPECT_EQ(WFD.WBuf().capacity(), WFD.WBuf().size());
	EXPECT_DOUBLE_EQ(0.0, WFD.getA());
	EXPECT_DOUBLE_EQ(0.0, WFD.getB());
	EXPECT_DOUBLE_EQ(0.0, WFD.getT());
	EXPECT_DOUBLE_EQ(0.0, WFD.value());
	EXPECT_DOUBLE_EQ(0.0, WFD.deriv());

	// Update the filter
	WFD.update();

	// Testing
	EXPECT_EQ(0, WFD.len());
	EXPECT_TRUE(WFD.lenZero());
	EXPECT_EQ(WFD.TBuf().capacity(), WFD.TBuf().size());
	EXPECT_EQ(WFD.PBuf().capacity(), WFD.PBuf().size());
	EXPECT_EQ(WFD.WBuf().capacity(), WFD.WBuf().size());
	EXPECT_DOUBLE_EQ(0.0, WFD.getA());
	EXPECT_DOUBLE_EQ(0.0, WFD.getB());
	EXPECT_DOUBLE_EQ(0.0, WFD.getT());
	EXPECT_DOUBLE_EQ(0.0, WFD.value());
	EXPECT_DOUBLE_EQ(0.0, WFD.deriv());

	// Create WLBF filter
	WLBFFilterND<1> WF(5);

	// Add some data points
	WF.addPW(1.0, 2.0, 0.0);
	WF.addPW(-1.0, 2.0, 0.5);
	WF.addPW(-3.0, 2.0, 1.0);
	WF.addPW(1.0, 4.0, 1.5);
	WF.addPW(2.0, 2.0, 2.0);
	WF.addPW(1.0, 7.0, 2.5);
	WF.addPW(-4.0, 2.0, 3.0);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.TBuf()[0]);
	EXPECT_EQ( 2.0, WF.TBuf()[2]);
	EXPECT_EQ(-3.0, WF.TBuf()[4]);
	EXPECT_EQ( 7.0, WF.PBuf()[1]);
	EXPECT_EQ( 4.0, WF.PBuf()[3]);
	EXPECT_EQ( 9.0, WF.WBuf()[0]);
	EXPECT_EQ( 6.25, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[4]);
	EXPECT_DOUBLE_EQ(3.5888888888888890, WF.getA());
	EXPECT_DOUBLE_EQ(0.4612903225806452, WF.getB());
	EXPECT_DOUBLE_EQ(-1.0, WF.getT());
	EXPECT_DOUBLE_EQ(2.2050179211469536, WF.value());
	EXPECT_DOUBLE_EQ(0.4612903225806452, WF.deriv());

	// Truncate some elements
	WF.resize(3);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(3, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.TBuf()[0]);
	EXPECT_EQ( 2.0, WF.TBuf()[2]);
	EXPECT_EQ( 7.0, WF.PBuf()[1]);
	EXPECT_EQ( 9.0, WF.WBuf()[0]);
	EXPECT_EQ( 6.25, WF.WBuf()[1]);
	EXPECT_DOUBLE_EQ(3.6233766233766231, WF.getA());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.getB());
	EXPECT_DOUBLE_EQ(-1.1298701298701299, WF.getT());
	EXPECT_DOUBLE_EQ(2.2750022916857642, WF.value());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.deriv());

	// Pad out some elements
	WF.resize(6);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(6, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.TBuf()[0]);
	EXPECT_EQ( 2.0, WF.TBuf()[2]);
	EXPECT_EQ( 0.0, WF.TBuf()[4]);
	EXPECT_EQ( 7.0, WF.PBuf()[1]);
	EXPECT_EQ( 0.0, WF.PBuf()[3]);
	EXPECT_EQ( 9.0, WF.WBuf()[0]);
	EXPECT_EQ( 6.25, WF.WBuf()[1]);
	EXPECT_EQ( 0.0, WF.WBuf()[5]);
	EXPECT_DOUBLE_EQ(3.6233766233766231, WF.getA());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.getB());
	EXPECT_DOUBLE_EQ(-1.1298701298701299, WF.getT());
	EXPECT_DOUBLE_EQ(2.2750022916857642, WF.value());
	EXPECT_DOUBLE_EQ(0.4697955816298469, WF.deriv());

	// Add some data back in again
	WF.addPW(1.0, 2.0, 0.0);
	WF.addPW(-1.0, 2.0, 0.5);
	WF.addPW(-3.0, 2.0, 1.0);
	WF.zeroPBuf();
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(6, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-3.0, WF.TBuf()[0]);
	EXPECT_EQ( 1.0, WF.TBuf()[2]);
	EXPECT_EQ( 1.0, WF.TBuf()[4]);
	EXPECT_EQ( 0.0, WF.PBuf()[1]);
	EXPECT_EQ( 0.0, WF.PBuf()[2]);
	EXPECT_EQ( 0.0, WF.PBuf()[3]);
	EXPECT_EQ( 1.0, WF.WBuf()[0]);
	EXPECT_EQ( 0.25, WF.WBuf()[1]);
	EXPECT_EQ( 4.0, WF.WBuf()[5]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(-1.2195121951219512, WF.getT());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Set the buffers manually
	double tmpt[3] = {0.2, 0.7, 0.3};
	double tmpp[3] = {100, -42, 17};
	double tmpw[3] = {0.666, 0.777, 0.333};
	WF.setTBuf(tmpt, tmpt + 3);
	WF.setPBuf(tmpp, tmpp + 2);
	WF.setWBuf(tmpw, tmpw + 3);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(6, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.2, WF.TBuf()[0]);
	EXPECT_EQ( 0.3, WF.TBuf()[2]);
	EXPECT_EQ( 0.0, WF.TBuf()[4]);
	EXPECT_EQ( -42, WF.PBuf()[1]);
	EXPECT_EQ( 0.0, WF.PBuf()[2]);
	EXPECT_EQ( 0.0, WF.PBuf()[3]);
	EXPECT_EQ( 0.666*0.666, WF.WBuf()[0]);
	EXPECT_EQ( 0.777*0.777, WF.WBuf()[1]);
	EXPECT_EQ( 0.0, WF.WBuf()[5]);
	EXPECT_NEAR(  16.4042553191489127, WF.getA(), 1e-12);
	EXPECT_NEAR(-263.9720279720280018, WF.getB(), 1e-12);
	EXPECT_NEAR(   0.4702127659574469, WF.getT(), 1e-12);
	EXPECT_NEAR(  87.7328671328672, WF.value(), 1e-12);
	EXPECT_NEAR(-263.9720279720282, WF.deriv(), 1e-12);

	// Set the t and w buffers
	WF.zeroTBuf();
	WF.setUnitW();
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(6, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.0, WF.TBuf()[0]);
	EXPECT_EQ( 0.0, WF.TBuf()[2]);
	EXPECT_EQ( 0.0, WF.TBuf()[4]);
	EXPECT_EQ( -42, WF.PBuf()[1]);
	EXPECT_EQ( 0.0, WF.PBuf()[2]);
	EXPECT_EQ( 0.0, WF.PBuf()[3]);
	EXPECT_EQ( 1.0, WF.WBuf()[0]);
	EXPECT_EQ( 1.0, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[5]);
	EXPECT_DOUBLE_EQ(9.6666666666666667, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.getT());
	EXPECT_DOUBLE_EQ(9.6666666666666667, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Reduce the size and push back elements
	WF.resize(4);
	WF.addPW(1.0, 2.0, 0.0);
	WF.addPW(-1.0, 7.0, 0.5);
	WF.addPW(-3.0, 1.0, 1.0);
	WF.addPW(2.0, 3.0, 0.0);
	WF.addPW(-2.0, 1.0, 0.5);
	WF.addPW(-4.0, 4.0, 1.0);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(4, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.TBuf()[0]);
	EXPECT_EQ( 2.0, WF.TBuf()[2]);
	EXPECT_EQ(-3.0, WF.TBuf()[3]);
	EXPECT_EQ( 1.0, WF.PBuf()[1]);
	EXPECT_EQ( 3.0, WF.PBuf()[2]);
	EXPECT_EQ( 1.0, WF.PBuf()[3]);
	EXPECT_EQ( 1.0, WF.WBuf()[0]);
	EXPECT_EQ(0.25, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[3]);

	// Set constant points
	WF.resize(5);
	WF.initConstP(0.3, 0.1, 0.7);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.3, WF.TBuf()[0]);
	EXPECT_DOUBLE_EQ( 0.1, WF.TBuf()[2]);
	EXPECT_NEAR( 0.0, WF.TBuf()[3], 1e-15);
	EXPECT_EQ( 0.7, WF.PBuf()[1]);
	EXPECT_EQ( 0.7, WF.PBuf()[2]);
	EXPECT_EQ( 0.7, WF.PBuf()[4]);
	EXPECT_EQ( 1.0, WF.WBuf()[0]);
	EXPECT_EQ(0.25, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[3]);
	EXPECT_DOUBLE_EQ(0.7, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.1555555555555556, WF.getT());
	EXPECT_DOUBLE_EQ(0.7, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Reset the filter
	WF.reset();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.0, WF.TBuf()[0]);
	EXPECT_EQ( 0.0, WF.TBuf()[2]);
	EXPECT_EQ( 0.0, WF.TBuf()[3]);
	EXPECT_EQ( 0.0, WF.PBuf()[1]);
	EXPECT_EQ( 0.0, WF.PBuf()[2]);
	EXPECT_EQ( 0.0, WF.PBuf()[4]);
	EXPECT_EQ( 0.0, WF.WBuf()[0]);
	EXPECT_EQ( 0.0, WF.WBuf()[1]);
	EXPECT_EQ( 0.0, WF.WBuf()[4]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.getT());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Set constant points with weights
	WF.resize(5);
	WF.initConstPW(0.5, 0.2, 0.1, 2.0);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.5, WF.TBuf()[0]);
	EXPECT_DOUBLE_EQ( 0.1, WF.TBuf()[2]);
	EXPECT_NEAR(-0.1, WF.TBuf()[3], 1e-15);
	EXPECT_EQ( 0.1, WF.PBuf()[1]);
	EXPECT_EQ( 0.1, WF.PBuf()[2]);
	EXPECT_EQ( 0.1, WF.PBuf()[4]);
	EXPECT_EQ( 4.0, WF.WBuf()[0]);
	EXPECT_EQ( 4.0, WF.WBuf()[1]);
	EXPECT_EQ( 4.0, WF.WBuf()[4]);
	EXPECT_DOUBLE_EQ(0.1, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.1, WF.getT());
	EXPECT_DOUBLE_EQ(0.1, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());

	// Reset the filter
	WF.resetAll(1);
	printState(WF);

	// Testing
	EXPECT_EQ(1, WF.len());
	EXPECT_FALSE(WF.lenZero());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ( 0.0, WF.TBuf()[0]);
	EXPECT_EQ( 0.0, WF.PBuf()[0]);
	EXPECT_EQ( 0.0, WF.WBuf()[0]);
	EXPECT_DOUBLE_EQ(0.0, WF.getA());
	EXPECT_DOUBLE_EQ(0.0, WF.getB());
	EXPECT_DOUBLE_EQ(0.0, WF.getT());
	EXPECT_DOUBLE_EQ(0.0, WF.value());
	EXPECT_DOUBLE_EQ(0.0, WF.deriv());
}

// Test the WLBFFilterND class
TEST(WLBFTest, testWLBFFilterND)
{
	// Typedefs
	typedef Eigen::Vector3d V;

	// Constants
	V Zero = V::Zero();

	// Create a default WLBF filter
	WLBFFilterND<3> WFD;

	// Print the filter state
	printState(WFD);

	// Testing
	EXPECT_EQ(3, WFD.ND);
	EXPECT_EQ(0, WFD.len());
	EXPECT_TRUE(WFD.lenZero());
	EXPECT_EQ(WFD.TBuf().capacity(), WFD.TBuf().size());
	EXPECT_EQ(WFD.PBuf().capacity(), WFD.PBuf().size());
	EXPECT_EQ(WFD.WBuf().capacity(), WFD.WBuf().size());
	EXPECT_DOUBLE_EQ3(Zero, WFD.getA());
	EXPECT_DOUBLE_EQ3(Zero, WFD.getB());
	EXPECT_DOUBLE_EQ(0.0, WFD.getT());
	EXPECT_DOUBLE_EQ3(Zero, WFD.value());
	EXPECT_DOUBLE_EQ3(Zero, WFD.deriv());

	// Update the filter
	WFD.update();

	// Testing
	EXPECT_EQ(3, WFD.ND);
	EXPECT_EQ(0, WFD.len());
	EXPECT_TRUE(WFD.lenZero());
	EXPECT_EQ(WFD.TBuf().capacity(), WFD.TBuf().size());
	EXPECT_EQ(WFD.PBuf().capacity(), WFD.PBuf().size());
	EXPECT_EQ(WFD.WBuf().capacity(), WFD.WBuf().size());
	EXPECT_DOUBLE_EQ3(Zero, WFD.getA());
	EXPECT_DOUBLE_EQ3(Zero, WFD.getB());
	EXPECT_DOUBLE_EQ(0.0, WFD.getT());
	EXPECT_DOUBLE_EQ3(Zero, WFD.value());
	EXPECT_DOUBLE_EQ3(Zero, WFD.deriv());

	// Create WLBF filter
	WLBFFilterND<3> WF(5);

	// Add some data points
	WF.addPW(1.0, V(2.0, 3.0, 0.4), 0.0);
	WF.addPW(-1.0, V(2.0, -1.0, 1.7), 0.5);
	WF.addPW(-3.0, V(2.0, 1.5, 0.6), 1.0);
	WF.addPW(1.0, V(4.0, 3.7, 2.2), 1.5);
	WF.addPW(2.0, V(2.0, 0.6, 1.9), 2.0);
	WF.addPW(1.0, V(7.0, 1.2, 3.8), 2.5);
	WF.addPW(-4.0, V(2.0, 4.2, 10.2), 3.0);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.TBuf()[0]);
	EXPECT_EQ( 2.0, WF.TBuf()[2]);
	EXPECT_EQ(-3.0, WF.TBuf()[4]);
	EXPECT_EQ3(V(7.0, 1.2, 3.8), WF.PBuf()[1]);
	EXPECT_EQ3(V(4.0, 3.7, 2.2), WF.PBuf()[3]);
	EXPECT_EQ( 9.0, WF.WBuf()[0]);
	EXPECT_EQ( 6.25, WF.WBuf()[1]);
	EXPECT_EQ( 1.0, WF.WBuf()[4]);
	EXPECT_DOUBLE_EQ3(V(3.5888888888888890, 2.5566666666666666, 5.7199999999999998), WF.getA());
	EXPECT_DOUBLE_EQ3(V(0.4612903225806452, -0.5003225806451613, -1.2670967741935482), WF.getB());
	EXPECT_DOUBLE_EQ(-1.0, WF.getT());
	EXPECT_DOUBLE_EQ3(V(2.2050179211469536, 4.0576344086021505, 9.5212903225806436), WF.value());
	EXPECT_DOUBLE_EQ3(V(0.4612903225806452, -0.5003225806451613, -1.2670967741935482), WF.deriv());

	// Modify some data points
	WF.setW(0, 0.5);
	WF.setP(1, 1.5, V(-5.0, 0.0, -1.0));
	WF.setPW(4, 0.8, V(3.0, 3.1, 3.3), 1.7);
	WF.update();
	printState(WF);

	// Testing
	EXPECT_EQ(5, WF.len());
	EXPECT_EQ(WF.TBuf().capacity(), WF.TBuf().size());
	EXPECT_EQ(WF.PBuf().capacity(), WF.PBuf().size());
	EXPECT_EQ(WF.WBuf().capacity(), WF.WBuf().size());
	EXPECT_EQ(-4.0, WF.TBuf()[0]);
	EXPECT_EQ( 1.5, WF.TBuf()[1]);
	EXPECT_EQ( 2.0, WF.TBuf()[2]);
	EXPECT_EQ( 0.8, WF.TBuf()[4]);
	EXPECT_EQ3(V(-5.0, 0.0, -1.0), WF.PBuf()[1]);
	EXPECT_EQ3(V(4.0, 3.7, 2.2), WF.PBuf()[3]);
	EXPECT_EQ3(V(3.0, 3.1, 3.3), WF.PBuf()[4]);
	EXPECT_EQ( 0.25, WF.WBuf()[0]);
	EXPECT_EQ( 6.25, WF.WBuf()[1]);
	EXPECT_EQ( 1.7*1.7, WF.WBuf()[4]);
	EXPECT_DOUBLE_EQ3(V(-0.3248081841432227, 1.3257033248081842, 1.1756393861892585), WF.getA());
	EXPECT_DOUBLE_EQ3(V(-1.0004336036692112, -1.1509708920863151, -1.6192623816403151), WF.getB());
	EXPECT_DOUBLE_EQ(1.3386828644501279, WF.getT());
	EXPECT_DOUBLE_EQ3(V(5.0161895527856863, 7.4703719038702712, 9.8203677161011118), WF.value());
	EXPECT_DOUBLE_EQ3(V(-1.0004336036692112, -1.1509708920863151, -1.6192623816403151), WF.deriv());
}

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF