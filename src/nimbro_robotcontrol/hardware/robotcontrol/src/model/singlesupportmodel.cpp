// RBDL model for tip link to body chain
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <robotcontrol/model/singlesupportmodel.h>
#include <robotcontrol/model/robotmodel.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Dynamics.h>
#include <ros/console.h>
#include <tf/tf.h>

// Namespaces
using namespace robotcontrol;

//
// SingleSupportModel class
//

// Constructor
SingleSupportModel::SingleSupportModel(RobotModel* model, const boost::shared_ptr<const urdf::Link>& link)
 : m_model(model)
 , m_link(link)
 , m_coeff(0.0)
 , m_normedCoeff(0.0)
 , m_com(Math::Vector3d::Zero())
 , m_zmp(Math::Vector3d::Zero())
 , m_zmpTorque(Math::Vector3d::Zero())
 , m_footForce(Math::Vector3d::Zero())
{
}

// Initialise the RBDL model from the given URDF model using the given link as root
bool SingleSupportModel::initFrom(const urdf::Model& model, const std::string& root)
{
	// Issue a warning if there is a root link mismatch
	if(root != m_link->name)
		ROS_WARN("Provided root link name does not match the name of the URDF link that was provided in the constructor of this single support model!");
	
	// Initialise the RBDL model as required
	if(!rbdl_parser::URDF_RBDL_Model::initFrom(model, root)) // This function sets up m_joints using multiple calls to setupJoint()
		return false;
	
	// Initialise the gravity vector as pointing down in the chosen root link coordinate frame
	gravity << 0, 0, -9.81;

	// Initialise the required generalised vectors
	int nj = dof_count;
	m_q.resize(nj);
	m_q.setZero();
	m_qdot.resize(nj);
	m_qdot.setZero();
	m_qdotdot.resize(nj);
	m_qdotdot.setZero();
	m_tau.resize(nj);
	m_tau.setZero();
	m_measuredTorque.resize(nj);
	m_measuredTorque.setZero();
	
	// Return that initialisation was successful
	return true;
}

// Virtual override for setting up a joint (called by the base initFrom() implementation)
void SingleSupportModel::setupJoint(unsigned int index, const urdf::Joint& urdf, bool reverse)
{
	// Call the base implementation
	URDF_RBDL_Model::setupJoint(index, urdf, reverse); // This enables use of jointIndex() and jointName()
	
	// Protect against bad index
	if(index <= 0)
	{
		ROS_WARN("setupJoint() for single support model %s joint %s was called with a bad index (%u)", m_link->name.c_str(), urdf.name.c_str(), index);
		return;
	}
	
	// Increase the joints array size if required
	if(index - 1 >= m_joints.size())
		m_joints.resize(index);

	// Retrieve the joint struct pointer from the robot model and save it in our joint array (Note: The resulting joint array is in tree hierarchy order from the chosen root link onwards)
	Joint::Ptr joint = m_model->getJoint(urdf.name);
	ROS_ASSERT(joint);
	m_joints[index - 1] = joint;
}

// Set the commanded position of a joint (see Joint::setFrom*() for alternatives)
void SingleSupportModel::setJointCmd(int idx, double pos)
{
	// Set the required commanded position
	m_joints[idx]->cmd.setFromPos(m_model->timerDuration(), pos);
}

// Update the joint positions stored inside the RBDL model
void SingleSupportModel::updateRBDLJointPos(DataSource source)
{
	// Update the m_q vector
	for(size_t i = 0; i < m_joints.size(); i++)
	{
		// Retrieve a reference to the joint
		if(!m_joints[i]) continue;
		const Joint& joint = *m_joints[i];
		
		// Decide between measurement and commanded data
		double pos = 0.0;
		if(source == MeasurementData)
			pos = joint.feedback.pos;
		else
			pos = joint.cmd.pos;
		if(isnan(pos))
		{
			ROS_ERROR("Joint position used for RBDL computation is NaN for joint '%s'!", joint.name.c_str());
			pos = 0.0;
		}
		m_q[i] = pos;
	}
	
	// Update the joint positions in the RBDL model
	RigidBodyDynamics::UpdateKinematicsCustom(*this, &m_q, NULL, NULL); // This updates the positions and leaves the velocities/accelerations untouched (after this only functions that use kinematic position should be used as the velocities/accelerations haven't been updated)
}

// Update the measured torques from the joint structs
void SingleSupportModel::updateMeasuredTorques()
{
	// Update the m_measuredTorque vector
	for(size_t i = 0; i < m_joints.size(); i++)
	{
		if(!m_joints[i]) continue;
		m_measuredTorque[i] = m_joints[i]->feedback.torque;
	}
}

// Perform an inverse dynamics calculation on the single support model
double SingleSupportModel::doInverseDynamics(DataSource source, bool useCoeff, bool applyVelAcc, bool applyGrav)
{
	// Retrieve the coefficient with which this support model contributes to the model torques
	double coeff = (useCoeff ? m_normedCoeff : 1.0);
	if(coeff <= 0.0) return 0.0; // If the coefficient is zero then this function will have no effect

	// Update the kinematic positions of the RBDL model (updates m_q in the process)
	updateRBDLJointPos(source);

	// Apply gravity to the model if required
	if(applyGrav)
	{
		// Calculate the required gravity vector (always assumed to point in the negative z direction in the URDF root link frame)
		Math::Vector3d gravInURDFRoot(0.0, 0.0, -9.81);
		if(URDFRootIndex() != (unsigned int) -1)
		{
			const Math::SpatialTransform& X = X_base[URDFRootIndex()]; // This is the transform from the RBDL root link frame to the URDF root link frame
			Math::Vector3d gravInRBDLRoot = X.E.transpose() * gravInURDFRoot; // Transform gravity from URDF root link coordinates to RBDL root link coordinates
			setRBDLGravity(gravInRBDLRoot);
		}
		else
		{
			ROS_WARN_THROTTLE(1.0, "I don't know which index my URDF root link '%s' is at, so I don't know how to apply gravity for the inverse dynamics", URDFRootName().c_str());
			resetRBDLGravity();
		}
	}
	else
	{
		// Reset the gravity to zero
		resetRBDLGravity();
	}

	// Apply velocities and accelerations to the model if required
	if(applyVelAcc)
	{
		// Transcribe the commanded joint velocities and accelerations
		for(size_t i = 0; i < m_joints.size(); i++)
		{
			if(!m_joints[i]) continue;
			const Joint& joint = *m_joints[i];
			m_qdot[i] = (isnan(joint.cmd.vel) ? 0.0 : joint.cmd.vel);
			m_qdotdot[i] = (isnan(joint.cmd.acc) ? 0.0 : joint.cmd.acc);
		}
	}
	else
	{
		// Zero the commanded joint velocities and accelerations used for the inverse dynamics
		m_qdot.setZero();
		m_qdotdot.setZero();
	}

	// Calculate the inverse dynamics (updates the kinematics with m_q, m_qdot and m_qdotdot, uses the gravity set above, and returns the calculated torques in m_tau)
	RigidBodyDynamics::InverseDynamics(*this, m_q, m_qdot, m_qdotdot, m_tau, NULL);

	// Add the calculated joint torque, weighted by the required coefficient,
	for(size_t i = 0; i < m_joints.size(); i++)
	{
		if(!m_joints[i]) continue;
		m_joints[i]->feedback.modelTorque += coeff * m_tau[i];
	}
	
	// Return the coefficient that was used
	return coeff;
}

// Compute the center of mass
void SingleSupportModel::computeCoM()
{
	Math::Vector3d com = Math::Vector3d::Zero();
	double mass = 0.0;

	updateRBDLJointPos(MeasurementData);

	for(size_t i = 0; i < mBodies.size(); ++i)
	{
		const Math::SpatialTransform& X = X_base[i];
		const RigidBodyDynamics::Body& body = mBodies[i];

		Math::Vector3d bodyCom = X.E.transpose() * body.mCenterOfMass + X.r;

		com += body.mMass * bodyCom;
		mass += body.mMass;
	}

	m_com = com / mass;
}

// Compute the zero moment point
void SingleSupportModel::computeZMP()
{
	// TODO: This should be moved into a subclass, and RobotModel should be subclassed to create instances of this subclass.
	double ANKLE_Z_HEIGHT = 0.039;

	if(m_coeff == 0)
	{
		ROS_ERROR("SingleSupportModel: computeZMP() was called with zero support coefficient");
		return;
	}

	Math::SpatialVector spatialFootForce = -X_base[1].applyTranspose(f[1]);

	double mass = link()->inertial->mass;
	Eigen::Vector3d centerOfMass(
		link()->inertial->origin.position.x,
		link()->inertial->origin.position.y,
		link()->inertial->origin.position.z
	);

	double F = -mass * 9.81;

	double total_Fz = spatialFootForce[5] + F;
	double total_Mx = spatialFootForce[0] + F * centerOfMass.y() - spatialFootForce[4] * ANKLE_Z_HEIGHT;
	double total_My = spatialFootForce[1] - F * centerOfMass.x() + spatialFootForce[3] * ANKLE_Z_HEIGHT;

	m_footForce = Eigen::Vector3d(0.0, 0.0, total_Fz);

	m_zmp = Eigen::Vector3d(-total_My / total_Fz, total_Mx / total_Fz, 0);

	// ZMP estimation based on the measured ankle roll torque
	// TODO: Do this in a more general way?
	Joint::Ptr ankle_roll = m_joints[0];
	double total_Mx_torque = ankle_roll->feedback.torque + F * centerOfMass.y() - spatialFootForce[4] * ANKLE_Z_HEIGHT;
	m_zmpTorque = Eigen::Vector3d(-total_My / total_Fz, total_Mx_torque / total_Fz, 0);
}

// Calculate the jacobian matrix (3xNDof) for some point on a body (it is assumed m_q already contains up to date measurement/command data as desired, call updateRBDLJointPos() yourself before as appropriate)
Math::MatrixNd SingleSupportModel::pointJacobian(unsigned int body, const Math::Vector3d& pos)
{
	// Calculate the required jacobian
	Math::MatrixNd ret(3, dof_count);
	RigidBodyDynamics::CalcPointJacobian(*this, m_q, body, pos, ret, false);
	return ret;
}

// Estimate the force generated by joint torques (it is assumed m_q already contains up to date measurement/command data as desired, call updateRBDLJointPos() yourself before as appropriate)
Math::Vector3d SingleSupportModel::estimateContactForce(unsigned int body, const Math::Vector3d& pos)
{
	// Retrieve the latest feedback torques
	updateMeasuredTorques();
	
	// Return the required estimation
	return pointJacobian(body, pos) * m_measuredTorque;
}
// EOF