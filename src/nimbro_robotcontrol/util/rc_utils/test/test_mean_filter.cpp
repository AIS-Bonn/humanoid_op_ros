// Unit test of mean filter
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/mean_filter.h>
#include <rc_utils/mean_filter_nd.h>
#include <test_utilities/test_eigen.h>
#include <gtest/gtest.h>

// Namespaces
using namespace rc_utils;
using namespace testutilities;

// MeanFilter tester class (scalar)
template<class Filter> class MeanFilterTester
{
public:
	// Test function
	static void test()
	{
		// Declare variables
		const typename Filter::Buffer* buf;
		
		// Test construction
		Filter MF1;
		buf = &MF1.buf();
		EXPECT_EQ(0, MF1.len());
		EXPECT_EQ(0, buf->capacity());
		EXPECT_EQ(0, buf->size());
		EXPECT_EQ(0.0, MF1.mean());
		MF1.put(5.0);
		MF1.update();
		EXPECT_EQ(0, MF1.len());
		EXPECT_EQ(0, buf->capacity());
		EXPECT_EQ(0, buf->size());
		EXPECT_EQ(0.0, MF1.mean());
		Filter MF2(5);
		buf = &MF2.buf();
		EXPECT_EQ(5, MF2.len());
		EXPECT_EQ(5, buf->capacity());
		EXPECT_EQ(5, buf->size());
		EXPECT_EQ(0.0, MF2.mean());
		for(std::size_t i = 0; i < buf->size(); i++)
			EXPECT_EQ(0.0, buf->at(i));
		
		// Test reset all
		MF2.resetAll(2);
		EXPECT_EQ(2, MF2.len());
		EXPECT_EQ(2, buf->capacity());
		EXPECT_EQ(2, buf->size());
		EXPECT_EQ(0.0, MF2.mean());
		for(std::size_t i = 0; i < buf->size(); i++)
			EXPECT_EQ(0.0, buf->at(i));
		MF2.resetAll();
		EXPECT_EQ(0, MF2.len());
		EXPECT_EQ(0, buf->capacity());
		EXPECT_EQ(0, buf->size());
		EXPECT_EQ(0.0, MF2.mean());
		
		// Test resizing and putting
		MF2.resize(5, 3.0);                 // 3 3 3 3 3 NOW
		EXPECT_EQ(5, MF2.len());
		EXPECT_EQ(5, buf->capacity());
		EXPECT_EQ(5, buf->size());
		EXPECT_EQ(3.0, MF2.mean());
		for(std::size_t i = 0; i < buf->size(); i++)
			EXPECT_EQ(3.0, buf->at(i));
		MF2.resize(8);                      // 0 0 0 3 3 3 3 3 NOW
		EXPECT_EQ(8, MF2.len());
		EXPECT_EQ(8, buf->capacity());
		EXPECT_EQ(8, buf->size());
		EXPECT_EQ(1.875, MF2.updatedMean());
		for(std::size_t i = 0; i < 5; i++)
			EXPECT_EQ(3.0, buf->at(i));
		for(std::size_t i = 5; i < buf->size(); i++)
			EXPECT_EQ(0.0, buf->at(i));
		MF2.update(5.0);
		EXPECT_EQ(2.5, MF2.meanC());        // 0 0 3 3 3 3 3 5 NOW
		MF2.put(5.0);
		EXPECT_EQ(3.125, MF2.mean());       // 0 3 3 3 3 3 5 5 NOW
		EXPECT_EQ(3.75, MF2.update(5.0));   // 3 3 3 3 3 5 5 5 NOW
		MF2.put(5.0);
		EXPECT_EQ(4.0, MF2.updatedMean());  // 3 3 3 3 5 5 5 5 NOW
		MF2.resize(2);                      // 5 5 NOW
		EXPECT_EQ(2, MF2.len());
		EXPECT_EQ(2, buf->capacity());
		EXPECT_EQ(2, buf->size());
		EXPECT_EQ(5.0, MF2.mean());
		EXPECT_EQ(5.0, buf->at(0));
		EXPECT_EQ(5.0, buf->at(1));
		
		// Test reset
		MF2.reset();
		EXPECT_EQ(2, MF2.len());
		EXPECT_EQ(2, buf->capacity());
		EXPECT_EQ(2, buf->size());
		EXPECT_EQ(0.0, MF2.mean());
		
		// Test setting the buffer
		std::vector<double> data;
		data.push_back(1.0);
		data.push_back(3.0);
		data.push_back(2.0);
		data.push_back(5.0);
		data.push_back(4.0);
		MF2.setBuf(data.begin(), data.end()); // 3 1 NOW
		EXPECT_EQ(2, MF2.len());
		EXPECT_EQ(2, buf->capacity());
		EXPECT_EQ(2, buf->size());
		EXPECT_EQ(2.0, MF2.update().mean());
		MF2.resize(8);
		MF2.setBuf(data.begin(), data.end()); // 0 0 0 4 5 2 3 1 NOW
		EXPECT_EQ(8, MF2.len());
		EXPECT_EQ(8, buf->capacity());
		EXPECT_EQ(8, buf->size());
		EXPECT_EQ(1.875, MF2.mean());
		MF2.put(9.0);                         // 0 0 4 5 2 3 1 9 NOW
		MF2.update();
		EXPECT_EQ(3.0, MF2.mean());
		MF2.resize(5);
		MF2.setBuf(data.begin(), data.end()); // 4 5 2 3 1 NOW
		MF2.update();
		EXPECT_EQ(5, MF2.len());
		EXPECT_EQ(5, buf->capacity());
		EXPECT_EQ(5, buf->size());
		EXPECT_EQ(3.0, MF2.meanC());
		MF2.put(9.0);                         // 5 2 3 1 9 NOW
		MF2.update();
		EXPECT_EQ(4.0, MF2.meanC());
		MF2.zeroBuf();
		EXPECT_EQ(5, MF2.len());
		EXPECT_EQ(5, buf->capacity());
		EXPECT_EQ(5, buf->size());
		EXPECT_EQ(0.0, MF2.mean());
		for(std::size_t i = 0; i < buf->size(); i++)
			EXPECT_EQ(0.0, buf->at(i));
		MF2.fillBuf(-1.0);
		EXPECT_EQ(5, MF2.len());
		EXPECT_EQ(5, buf->capacity());
		EXPECT_EQ(5, buf->size());
		EXPECT_EQ(-1.0, MF2.mean());
		for(std::size_t i = 0; i < buf->size(); i++)
			EXPECT_EQ(-1.0, buf->at(i));
	}
};

// Test the MeanFilter class
TEST(RCUtilsMiscTest, testMeanFilter)
{
	// Common testing of MeanFilter class
	MeanFilterTester<MeanFilter>::test();
}

// Test the MeanFilter1D class
TEST(RCUtilsMiscTest, testMeanFilter1D)
{
	// Common testing of MeanFilter class
	MeanFilterTester<MeanFilter1D>::test();
}

// Test the MeanFilterND class
TEST(RCUtilsMiscTest, testMeanFilterND)
{
	// Typedefs
	typedef Eigen::Vector3d V;

	// Constants
	V Zero = V::Zero();
	Eigen::Vector2d Zero2d = Eigen::Vector2d::Zero();

	// Declare variables
	const MeanFilter2D::Buffer* buf2;
	const MeanFilter3D::Buffer* buf3;

	// Test construction
	MeanFilter2D MF2(4);
	buf2 = &MF2.buf();
	EXPECT_EQ(4, buf2->capacity());
	EXPECT_EQ(4, buf2->size());
	EXPECT_EQ(4, MF2.len());
	EXPECT_FALSE(MF2.lenZero());
	EXPECT_EIGEQ(2, Zero2d, MF2.meanC());
	EXPECT_TRUE(MF2.changed());
	EXPECT_EIGEQ(2, Zero2d, MF2.mean());
	EXPECT_FALSE(MF2.changed());
	for(std::size_t i = 0; i < buf2->size(); i++)
		EXPECT_EIGEQ(2, Zero2d, buf2->at(i));
	MeanFilter3D MF3;
	buf3 = &MF3.buf();
	EXPECT_EQ(0, buf3->capacity());
	EXPECT_EQ(0, buf3->size());
	EXPECT_EQ(0, MF3.len());
	EXPECT_TRUE(MF3.lenZero());
	EXPECT_EIGEQ(3, Zero, MF3.meanC());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, Zero, MF3.mean());
	EXPECT_FALSE(MF3.changed());

	// Test reset all
	MF3.put(V(1.0, 2.0, 3.0));
	MF3.put(V(2.0, 3.0, 4.0));
	MF3.resetAll(2);
	EXPECT_EQ(2, buf3->capacity());
	EXPECT_EQ(2, buf3->size());
	EXPECT_EQ(2, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_EIGEQ(3, Zero, MF3.meanC());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, Zero, MF3.mean());
	EXPECT_FALSE(MF3.changed());
	for(std::size_t i = 0; i < buf3->size(); i++)
		EXPECT_EIGEQ(3, Zero, buf3->at(i));
	MF3.resetAll();
	EXPECT_EQ(0, buf3->capacity());
	EXPECT_EQ(0, buf3->size());
	EXPECT_EQ(0, MF3.len());
	EXPECT_TRUE(MF3.lenZero());
	EXPECT_EIGEQ(3, Zero, MF3.meanC());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, Zero, MF3.mean());
	EXPECT_FALSE(MF3.changed());

	// Test put
	MF3.resetAll(3);
	MF3.put(V(0.0, 3.0, 1.0));
	MF3.put(V(4.0, 5.0, 6.0));
	MF3.put(1.0, 2.0, 3.0);
	MF3.put(V(2.0, 3.0, 4.0));
	MF3.put(0.0, 7.0, 2.0);
	EXPECT_EQ(3, buf3->capacity());
	EXPECT_EQ(3, buf3->size());
	EXPECT_EQ(3, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_EIGEQ(3, Zero, MF3.meanC());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, V(1.0, 4.0, 3.0), MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, V(1.0, 4.0, 3.0), MF3.meanC());
	EXPECT_EIGEQ(3, V(0.0, 7.0, 2.0), buf3->at(0));
	EXPECT_EIGEQ(3, V(2.0, 3.0, 4.0), buf3->at(1));
	EXPECT_EIGEQ(3, V(1.0, 2.0, 3.0), buf3->at(2));

	// Test resize
	MF3.resize(1);
	EXPECT_EQ(1, buf3->capacity());
	EXPECT_EQ(1, buf3->size());
	EXPECT_EQ(1, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_EIGEQ(3, V(1.0, 4.0, 3.0), MF3.meanC());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, V(0.0, 7.0, 2.0), MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, V(0.0, 7.0, 2.0), MF3.meanC());
	EXPECT_EIGEQ(3, V(0.0, 7.0, 2.0), buf3->at(0));
	MF3.resize(2);
	EXPECT_EQ(2, buf3->capacity());
	EXPECT_EQ(2, buf3->size());
	EXPECT_EQ(2, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, V(0.0, 3.5, 1.0), MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, MF3.mean(), MF3.meanC());
	EXPECT_EIGEQ(3, V(0.0, 7.0, 2.0), buf3->at(0));
	EXPECT_EIGEQ(3, Zero, buf3->at(1));
	MF3.resize(3, V(-3.0, -1.0, 1.0));
	EXPECT_EQ(3, buf3->capacity());
	EXPECT_EQ(3, buf3->size());
	EXPECT_EQ(3, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, V(-1.0, 2.0, 1.0), MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, MF3.mean(), MF3.meanC());
	EXPECT_EIGEQ(3, V(0.0, 7.0, 2.0), buf3->at(0));
	EXPECT_EIGEQ(3, Zero, buf3->at(1));
	EXPECT_EIGEQ(3, V(-3.0, -1.0, 1.0), buf3->at(2));

	// Test reset
	MF3.reset();
	EXPECT_EQ(3, buf3->capacity());
	EXPECT_EQ(3, buf3->size());
	EXPECT_EQ(3, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, Zero, MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, MF3.mean(), MF3.meanC());
	for(std::size_t i = 0; i < buf3->size(); i++)
		EXPECT_EIGEQ(3, Zero, buf3->at(i));

	// Test filling buffer
	MF3.fillBuf(V(0.1, 0.2, 0.3));
	EXPECT_EQ(3, buf3->capacity());
	EXPECT_EQ(3, buf3->size());
	EXPECT_EQ(3, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ_UT(3, V(0.1, 0.2, 0.3), MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, MF3.mean(), MF3.meanC());
	for(std::size_t i = 0; i < buf3->size(); i++)
		EXPECT_EIGEQ(3, V(0.1, 0.2, 0.3), buf3->at(i));
	MF3.zeroBuf();
	EXPECT_EQ(3, buf3->capacity());
	EXPECT_EQ(3, buf3->size());
	EXPECT_EQ(3, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_EIGEQ_UT(3, V(0.1, 0.2, 0.3), MF3.meanC());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, Zero, MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, MF3.mean(), MF3.meanC());
	for(std::size_t i = 0; i < buf3->size(); i++)
		EXPECT_EIGEQ(3, Zero, buf3->at(i));

	// Test setting buffer
	V tmp[2] = {V(2.0, 5.0, 3.0), V(1.0, -2.0, 0.0)};
	MF3.setBuf(tmp, tmp + 2);
	EXPECT_EQ(3, buf3->capacity());
	EXPECT_EQ(3, buf3->size());
	EXPECT_EQ(3, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, V(1.0, 1.0, 1.0), MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, MF3.mean(), MF3.meanC());
	EXPECT_EIGEQ(3, V(2.0, 5.0, 3.0), buf3->at(0));
	EXPECT_EIGEQ(3, V(1.0, -2.0, 0.0), buf3->at(1));
	EXPECT_EIGEQ(3, Zero, buf3->at(2));
	MF3.setBuf(tmp, tmp + 2, V(-3.0, 0.0, 3.0));
	EXPECT_EQ(3, buf3->capacity());
	EXPECT_EQ(3, buf3->size());
	EXPECT_EQ(3, MF3.len());
	EXPECT_FALSE(MF3.lenZero());
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, V(0.0, 1.0, 2.0), MF3.mean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, MF3.mean(), MF3.meanC());
	EXPECT_EIGEQ(3, V(2.0, 5.0, 3.0), buf3->at(0));
	EXPECT_EIGEQ(3, V(1.0, -2.0, 0.0), buf3->at(1));
	EXPECT_EIGEQ(3, V(-3.0, 0.0, 3.0), buf3->at(2));

	// Test update
	EXPECT_FALSE(MF3.changed());
	MF3.put(0.0, 0.0, 6.0);
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, V(0.0, 1.0, 2.0), MF3.meanC());
	EXPECT_TRUE(MF3.changed());
	MF3.update();
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, V(1.0, 1.0, 3.0), MF3.meanC());
	EXPECT_FALSE(MF3.changed());
	MF3.put(1.0, 1.0, -3.0);
	EXPECT_TRUE(MF3.changed());
	EXPECT_EIGEQ(3, V(1.0, 2.0, 2.0), MF3.updatedMean());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, V(1.0, 2.0, 2.0), MF3.meanC());
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, V(3.0, -1.0, 2.0), MF3.update(V(8.0, -4.0, 3.0)));
	EXPECT_FALSE(MF3.changed());
	EXPECT_EIGEQ(3, V(5.0, 2.0, 1.0), MF3.update(6.0, 9.0, 3.0));
	EXPECT_FALSE(MF3.changed());
}

// Main function
int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF