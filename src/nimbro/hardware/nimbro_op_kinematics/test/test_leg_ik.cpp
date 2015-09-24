// Unit tests for LegIK class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_op_kinematics/leg_ik.h>
#include <rbdl/rbdl_parser.h>

#include <rbdl/Kinematics.h>
#include <rbdl/Dynamics.h>

#include <robotcontrol/model/robotmodel.h>

#include <gtest/gtest.h>

#include <ros/package.h>

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

void setupModel(robotcontrol::RobotModel* model)
{
	boost::shared_ptr<urdf::Model> urdf = boost::make_shared<urdf::Model>();

	ASSERT_TRUE(urdf->initParam("/robot_description"));
// 	ASSERT_TRUE(urdf->initFile(ros::package::getPath("nimbro_op_model") + "/robots/nimbro_op.xml"));

	model->setModel(urdf);

	std::vector<boost::shared_ptr<urdf::Link> > links;
	urdf->getLinks(links);

	for(size_t i = 0; i < links.size(); ++i)
	{
		boost::shared_ptr<urdf::Joint> urdfJoint = links[i]->parent_joint;

		if(!urdfJoint)
			continue;

		robotcontrol::Joint::Ptr joint = boost::make_shared<robotcontrol::Joint>();
		joint->modelJoint = urdfJoint;
		joint->name = urdfJoint->name;

		model->addJoint(joint);
	}

	model->initTrees();
}

TEST(LegIKTest, test_z_motions)
{
	robotcontrol::RobotModel model;
	setupModel(&model);

	boost::shared_ptr<robotcontrol::SingleSupportModel> sup = model.supportModel("trunk_link");

	unsigned int body_id = sup->GetBodyId("left_foot_plane_link");
	Math::SpatialTransform fX;

	if(body_id >= sup->fixed_body_discriminator)
	{
		unsigned int fbody_id = body_id - sup->fixed_body_discriminator;
		fX = sup->mFixedBodies[fbody_id].mParentTransform;
		body_id = sup->mFixedBodies[fbody_id].mMovableParent;
	}

	nimbro_op_kinematics::LegIK ik(sup, "left_foot_plane_link");

	for(int iz = -100; iz < 100; ++iz)
	{
		Eigen::Vector3d target(
			0.0041,
			0.0562,
			0.001 * iz - 0.3
		);

		ASSERT_TRUE(ik.sendTargetsFor(target));

		sup->updateRBDLJointPos(robotcontrol::SingleSupportModel::CommandData);

		Math::SpatialTransform X = sup->X_base[body_id];

		ASSERT_VECTOR_EQ_N(target, X.r + X.E.transpose() * fX.r, 0.00005);
	}
}

TEST(LegIKTest, test_planar_motions)
{
	robotcontrol::RobotModel model;
	setupModel(&model);

	boost::shared_ptr<robotcontrol::SingleSupportModel> sup = model.supportModel("trunk_link");

	unsigned int body_id = sup->GetBodyId("left_foot_plane_link");
	Math::SpatialTransform fX;

	if(body_id >= sup->fixed_body_discriminator)
	{
		unsigned int fbody_id = body_id - sup->fixed_body_discriminator;
		fX = sup->mFixedBodies[fbody_id].mParentTransform;
		body_id = sup->mFixedBodies[fbody_id].mMovableParent;
	}

	nimbro_op_kinematics::LegIK ik(sup, "left_foot_plane_link");

	for(int ix = -10; ix < 10; ++ix)
	{
		for(int iy = -10; iy < 10; ++iy)
		{
			Eigen::Vector3d target(
				0.01 * ix,
				0.01 * iy,
				-0.3
			);

			ASSERT_TRUE(ik.sendTargetsFor(target));

			sup->updateRBDLJointPos(robotcontrol::SingleSupportModel::CommandData);

			Math::SpatialTransform& X = sup->X_base[body_id];

			ASSERT_VECTOR_EQ_N(target, X.r + X.E.transpose() * fX.r, 0.00005);
		}
	}
}

TEST(LegIKTest, test_rotations)
{
	robotcontrol::RobotModel model;
	setupModel(&model);

	boost::shared_ptr<robotcontrol::SingleSupportModel> sup = model.supportModel("trunk_link");

	unsigned int body_id = sup->GetBodyId("left_foot_plane_link");
	Math::SpatialTransform fX;

	if(body_id >= sup->fixed_body_discriminator)
	{
		unsigned int fbody_id = body_id - sup->fixed_body_discriminator;
		fX = sup->mFixedBodies[fbody_id].mParentTransform;
		body_id = sup->mFixedBodies[fbody_id].mMovableParent;
	}

	nimbro_op_kinematics::LegIK ik(sup, "left_foot_plane_link");

	double max_diff = 0;
	double max_rot = 0;

	for(int ir = -10; ir < 10; ++ir)
	{
		for(int ip = -10; ip < 10; ++ip)
		{
			for(int iy = -10; iy < 10; ++iy)
			{
				Eigen::Vector3d target(0.0, 0.0, -0.3);
				Eigen::Matrix3d rot;

				double yaw = M_PI * iy / 10;
				double pitch = M_PI/4.0 * ip / 10;
				double roll = M_PI/4.0 * ir / 10;

				rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
				    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
			        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

				ASSERT_TRUE(ik.sendTargetsFor(target, rot));

				sup->updateRBDLJointPos(robotcontrol::SingleSupportModel::CommandData);

				Math::SpatialTransform X = sup->X_base[body_id];

				ASSERT_VECTOR_EQ_N(target, X.r + X.E.transpose() * fX.r, 0.00005);
				double norm = (target - (X.r + X.E * fX.r)).norm();

				if(norm > max_diff)
					max_diff = norm;

				Eigen::Vector3d x = rot * Eigen::Vector3d::UnitX();
				Eigen::Vector3d y = X.E.transpose() * Eigen::Vector3d::UnitX();

				double dot = x.dot(y);
				if(dot > 1.0)
					dot = 1.0;

				double angle = fabs(acos(dot));
				ASSERT_GT(0.1 * M_PI / 180.0, angle);

				if(angle > max_rot)
					max_rot = angle;
			}
		}
	}

	fprintf(stderr, "Max norm diff: %3.8lf m, %3.8lf°\n", max_diff, max_rot * 180.0 / M_PI);
}

TEST(LegIKTest, test_rotations_right)
{
	robotcontrol::RobotModel model;
	setupModel(&model);

	boost::shared_ptr<robotcontrol::SingleSupportModel> sup = model.supportModel("trunk_link");

	unsigned int body_id = sup->GetBodyId("right_foot_plane_link");
	Math::SpatialTransform fX;

	if(body_id >= sup->fixed_body_discriminator)
	{
		unsigned int fbody_id = body_id - sup->fixed_body_discriminator;
		fX = sup->mFixedBodies[fbody_id].mParentTransform;
		body_id = sup->mFixedBodies[fbody_id].mMovableParent;
	}

	nimbro_op_kinematics::LegIK ik(sup, "right_foot_plane_link");

	for(int ir = -10; ir < 10; ++ir)
	{
		for(int ip = -10; ip < 10; ++ip)
		{
			for(int iy = -10; iy < 10; ++iy)
			{
				Eigen::Vector3d target(0.0, 0.0, -0.3);
				Eigen::Matrix3d rot;

				rot = Eigen::AngleAxisd(M_PI     * iy / 10, Eigen::Vector3d::UnitZ())
				* Eigen::AngleAxisd(M_PI/4.0 * ip / 10, Eigen::Vector3d::UnitY())
				* Eigen::AngleAxisd(M_PI/4.0 * ir / 10, Eigen::Vector3d::UnitX());

				ASSERT_TRUE(ik.sendTargetsFor(target, rot));

				sup->updateRBDLJointPos(robotcontrol::SingleSupportModel::CommandData);

				Math::SpatialTransform X = sup->X_base[body_id];

				ASSERT_VECTOR_EQ_N(target, X.r + X.E.transpose() * fX.r, 0.0005);

				Eigen::Vector3d x = rot * Eigen::Vector3d::UnitX();
				Eigen::Vector3d y = X.E.transpose() * Eigen::Vector3d::UnitX();

				double dot = x.dot(y);
				if(dot > 1.0)
					dot = 1.0;

				ASSERT_GT(0.2 * M_PI / 180.0, fabs(acos(dot)));
			}
		}
	}
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_leg_ik");

	return RUN_ALL_TESTS();
}
