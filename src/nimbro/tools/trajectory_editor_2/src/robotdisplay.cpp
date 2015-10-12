// Displays the robot in a pose defined by joint angles
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <trajectory_editor_2/robotdisplay.h>

#include <rviz/robot/robot.h>
#include <rviz/robot/link_updater.h>

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include <OGRE/OgreMatrix3.h>

class RBDLLinkUpdater : public rviz::LinkUpdater
{
public:
	RBDLLinkUpdater(RigidBodyDynamics::Model* model)
	 : m_model(model)
	{}

	virtual bool getLinkTransforms(const std::string& link_name,
		Ogre::Vector3& visual_position,
		Ogre::Quaternion& visual_orientation,
		Ogre::Vector3& collision_position,
		Ogre::Quaternion& collision_orientation) const
	{
		unsigned int id = m_model->GetBodyId(link_name.c_str());

		if(id >= m_model->X_base.size())
		{
			visual_position = Ogre::Vector3::ZERO;
			visual_orientation = Ogre::Quaternion::IDENTITY;
			return true;
		}

		RigidBodyDynamics::Math::SpatialTransform trans = m_model->X_base[id];

		visual_position = Ogre::Vector3(
			trans.r.x(), trans.r.y(), trans.r.z()
		);

		Ogre::Matrix3 mat(
			trans.E(0, 0), trans.E(1, 0), trans.E(2, 0),
			trans.E(0, 1), trans.E(1, 1), trans.E(2, 1),
			trans.E(0, 2), trans.E(1, 2), trans.E(2, 2)
		);
		visual_orientation.FromRotationMatrix(mat);

		return true;
	}
private:
	RigidBodyDynamics::Model* m_model;
};

RobotDisplay::RobotDisplay(const boost::shared_ptr< urdf::Model >& model)
 : m_model(model)
{
	m_rbdl = boost::make_shared<rbdl_parser::URDF_RBDL_Model>();
	m_rbdl->initFrom(*model);
}

RobotDisplay::~RobotDisplay()
{
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 6)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#endif


	// rviz::Robot class has a non-virtual destructor, but is polymorphic.
	// This is bad design, but not an issue here, because it is assured
	// to be exactly rviz::Robot (see below).
	delete m_robot;

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 6)
#pragma GCC diagnostic pop
#endif
}

void RobotDisplay::onInitialize()
{
	rviz::Display::onInitialize();

	m_robot = new rviz::Robot(scene_node_, context_, "robot", this);

	m_robot->load(*m_model, true, false);

	RigidBodyDynamics::Math::VectorNd Q = RigidBodyDynamics::Math::VectorNd::Constant(m_rbdl->dof_count, 0);
	RigidBodyDynamics::UpdateKinematics(*m_rbdl, Q, Q, Q);

	m_robot->update(RBDLLinkUpdater(m_rbdl.get()));
	
	
	
	// Set up publisher
	/*marker_pub = n.advertise<visualization_msgs::Marker>("/foo/bar", 100);
	
	// Set up marker
	uint32_t shape = visualization_msgs::Marker::CUBE;
	
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/left_foot_plane_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();*/
}

void RobotDisplay::update(std::vector<double> positions)
{
	double* p = &positions[0];
	RigidBodyDynamics::Math::VectorNd Q = Eigen::Map<Eigen::VectorXd>(p, m_rbdl->dof_count, 1);

	RigidBodyDynamics::Math::VectorNd QZero = RigidBodyDynamics::Math::VectorNd::Constant(m_rbdl->dof_count, 0);
	RigidBodyDynamics::UpdateKinematics(*m_rbdl, Q, QZero, QZero);

	m_robot->update(RBDLLinkUpdater(m_rbdl.get()));
	
 	/*marker.header.stamp = ros::Time::now();
 	marker_pub.publish(marker);*/
}
