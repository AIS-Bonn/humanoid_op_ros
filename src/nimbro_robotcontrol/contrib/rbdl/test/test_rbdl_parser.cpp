// Unit tests for rbdl_parser module
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <rbdl/rbdl_parser.h>
#include <rbdl/treestream.h>
#include <urdf/model.h>

#include <rbdl/Kinematics.h>
#include <rbdl/Dynamics.h>

#include <gtest/gtest.h>

#include <stdio.h>

#include "test_config.h"


#define DUMP_TREES 0

namespace Math = RigidBodyDynamics::Math;

::testing::AssertionResult AssertNormDiff(
	const char* a_expr,
	const char* b_expr,
	const char* norm_expr,
	const Eigen::VectorXd& a,
	const Eigen::VectorXd& b,
	double max_norm)
{
	Eigen::VectorXd diff = a - b;
	double norm = diff.norm();

	if(norm < max_norm)
		return ::testing::AssertionSuccess();

	return ::testing::AssertionFailure()
		<< "Vectors a and b are not equal. a (expected) = " << a.transpose()
		<< ", b (actual) = " << b.transpose() << ", diff norm: " << norm;
}

#define ASSERT_VECTOR_EQ_N(a, b, max_norm) \
	ASSERT_PRED_FORMAT3(AssertNormDiff, a, b, max_norm)

#define ASSERT_VECTOR_EQ(a, b) \
	ASSERT_PRED_FORMAT3(AssertNormDiff, a, b, 1e-8)

TEST(RBDLParserTest, test_names)
{
	urdf::Model model;

	ASSERT_TRUE(model.initFile(SOURCE_DIRECTORY "/test/tree.xml"));

	rbdl_parser::URDF_RBDL_Model rbdl;
	ASSERT_TRUE(rbdl.initFrom(model));

	int id1 = rbdl.jointIndex("arm_joint");
	int id2 = rbdl.jointIndex("arm2_joint");

	ASSERT_NE(id1, id2);

	ASSERT_EQ("arm_joint", rbdl.jointName(id1));
	ASSERT_EQ("arm2_joint", rbdl.jointName(id2));
}

TEST(RBDLParserTest, test_simple)
{
	urdf::Model model;

	ASSERT_TRUE(model.initFile(SOURCE_DIRECTORY "/test/simple.xml"));

	rbdl_parser::URDF_RBDL_Model rbdl;
	ASSERT_TRUE(rbdl.initFrom(model));

	Math::VectorNd q(1);
	q(0) = 0.0;
	Math::Vector3d bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d::Zero());
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, 0.0, 1.0), bodypos);

	bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d(0.0, 0.0, 1.0));
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, 0.0, 2.0), bodypos);

	q(0) = M_PI / 2.0;
	bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d(0.0, 0.0, 1.0));
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, -1.0, 1.0), bodypos);

	Math::VectorNd zero(1);
	Math::VectorNd tau(1);
	zero(0) = 0.0;

	q(0) = 0.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_DOUBLE_EQ(0, tau(0));

	q(0) = M_PI / 2.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_DOUBLE_EQ(-9.81, tau(0));
}

TEST(RBDLParserTest, test_rotated)
{
	urdf::Model model;

	ASSERT_TRUE(model.initFile(SOURCE_DIRECTORY "/test/rotated.xml"));

	rbdl_parser::URDF_RBDL_Model rbdl;
	ASSERT_TRUE(rbdl.initFrom(model));

#if DUMP_TREES
	TreeStream stream(&std::cerr);
	stream << rbdl;
#endif

	Math::VectorNd q(1);
	q(0) = 0.0;
	Math::Vector3d bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d::Zero());
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, 0.0, 1.0), bodypos);

	bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d(0.0, 0.0, 1.0));
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, -1.0, 1.0), bodypos);

	q(0) = M_PI / 2.0;
	bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d(0.0, 0.0, 1.0));
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, 0.0, 0.0), bodypos);

	Math::VectorNd zero(1);
	Math::VectorNd tau(1);
	zero(0) = 0.0;

	q(0) = 0.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_NEAR(-9.81, tau(0), 1e-8);

	q(0) = M_PI / 2.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_NEAR(0.0, tau(0), 1e-8);
}

TEST(RBDLParserTest, test_rotated_inverse)
{
	urdf::Model model;

	ASSERT_TRUE(model.initFile(SOURCE_DIRECTORY "/test/rotated.xml"));

	rbdl_parser::URDF_RBDL_Model rbdl;
	ASSERT_TRUE(rbdl.initFrom(model, "arm"));

#if DUMP_TREES
	TreeStream stream(&std::cerr);
	stream << rbdl;
#endif

	Math::VectorNd q(1);
	q(0) = 0.0;
	Math::Vector3d bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d::Zero());
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, 0.0, 0.0), bodypos);

	bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d(0.0, 0.0, 1.0));
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, 1.0, 0.0), bodypos);

	q(0) = M_PI / 2.0;
	bodypos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, 1, Math::Vector3d(0.0, 0.0, 1.0));
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, 0.0, -1.0), bodypos);

	Math::VectorNd zero(1);
	Math::VectorNd tau(1);
	zero(0) = 0.0;

	rbdl.gravity << 0, -9.81, 0;

	q(0) = 0.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_NEAR(0.0, tau(0), 1e-8);

	q(0) = M_PI / 2.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_NEAR(9.81, tau(0), 1e-8);
}

TEST(RBDLParserTest, test_tree_inverse)
{
	urdf::Model model;

	ASSERT_TRUE(model.initFile(SOURCE_DIRECTORY "/test/tree.xml"));

	rbdl_parser::URDF_RBDL_Model rbdl;
	ASSERT_TRUE(rbdl.initFrom(model, "arm"));

#if DUMP_TREES
	TreeStream stream(&std::cerr);
	stream << rbdl;
#endif

	Math::VectorNd q(2);
	q << 0.0, 0.0;

	Math::Vector3d pos;

	int trunk = rbdl.GetBodyId("trunk_link");
	//int arm = rbdl.GetBodyId("arm");
	int arm2 = rbdl.GetBodyId("arm2");

	pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, arm2, Math::Vector3d::Zero());
	ASSERT_VECTOR_EQ(Math::Vector3d(0.0, 0.0, 1.0), pos);

	pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, arm2, Math::Vector3d(-1.0, 1.0, 1.0));
	ASSERT_VECTOR_EQ(Math::Vector3d(-1.0, 1.0, 0.0), pos);

	pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl, q, trunk, Math::Vector3d(3.0, 1.0, -2.0));
	ASSERT_VECTOR_EQ(Math::Vector3d(3.0, -2.0, -1.0), pos);



	Math::VectorNd zero(2);
	Math::VectorNd tau(2);
	zero << 0, 0;

	rbdl.gravity << 0, -9.81, 0;

	q << 0.0, 0.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_NEAR(0.0, tau(0), 1e-8);
	ASSERT_NEAR(0.0, tau(1), 1e-8);

	q << M_PI/2.0, 0.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_NEAR(-9.81, tau(0), 1e-8);
	ASSERT_NEAR(0.0, tau(1), 1e-8);

	q << 0.0, M_PI/2.0;
	RigidBodyDynamics::InverseDynamics(rbdl, q, zero, zero, tau);
	ASSERT_NEAR(0.0, tau(0), 1e-8);
	ASSERT_NEAR(9.81, tau(1), 1e-8);
}


int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
