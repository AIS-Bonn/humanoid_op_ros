// Estimates outside torques working on our joints
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <servomodel/torqueestimator.h>

#include <kdl_parser/kdl_parser.hpp>
#include <ros/console.h>

#include <map>

TorqueEstimator::TorqueEstimator()
{
}

TorqueEstimator::~TorqueEstimator()
{
}

struct JointSpec
{
	JointSpec()
	{}

	JointSpec(int _id, int _offset, bool _invert, bool _active)
	 : id(_id), offset(_offset), invert(_invert), active(_active)
	{}

	int id;
	int offset;
	bool invert;
	bool active;
};

bool TorqueEstimator::init(const boost::shared_ptr<KDL::Chain>& chain)
{
// 	if(!kdl_parser::treeFromUrdfModel(*m_model, m_tree))
// 	{
// 		ROS_ERROR("Could not get KDL tree from URDF model");
// 		return false;
// 	}
//
// 	KDL::Chain chain;
// 	if(!m_tree.getChain(base, tip, chain))
// 	{
// 		ROS_ERROR("Could not get chain from '%s' to '%s'",
// 			base.c_str(), tip.c_str()
// 		);
// 		return false;
// 	}
	m_chain = chain;

	std::map<std::string, JointSpec> specs;
	specs["left_hip_yaw"] = JointSpec(8, 2048, false, false);
	specs["left_hip_roll"] = JointSpec(10, 1792, false, false);
	specs["left_hip_pitch"] = JointSpec(12, 2040, false, true);
	specs["left_knee_pitch"] = JointSpec(14, 1205, false, true);
	specs["left_ankle_pitch"] = JointSpec(16, 2046, true, false);
	specs["left_ankle_roll"] = JointSpec(18, 2304, true, false);

	int njoints = m_chain->getNrOfJoints();
	int nsegm = m_chain->getNrOfSegments();
	ROS_INFO("TorqueEstimator: Got %d segments with %d joints", nsegm, njoints);

	m_positions.resize(njoints);
	m_velocities.resize(njoints);
	m_accelerations.resize(njoints);
	m_torques.resize(njoints);
	m_forces.resize(nsegm);

	for(int i = 0; i < nsegm; ++i)
		m_forces[i] = KDL::Wrench(KDL::Vector::Zero(), KDL::Vector::Zero());

	m_chain->getSegment(0);

	for(size_t i = 0; i < m_chain->getNrOfSegments(); ++i)
	{
		const KDL::Segment& segment = m_chain->getSegment(i);
		const KDL::Joint& joint = segment.getJoint();

		if(joint.getType() == KDL::Joint::None)
			continue;

		const std::string& name = joint.getName();
		int id = specs[name].id;

		if(id >= (int)m_idMap.size())
			m_idMap.resize(id+1, -1);

		m_idMap[id] = i;
	}

	m_solver.reset(
		new KDL::ChainIdSolver_RNE(
			*chain,
			KDL::Vector(0, 0, -9.81)
		)
	);

	return true;
}

void TorqueEstimator::setJointState(int id, double pos, double vel, double acc)
{
	int idx = m_idMap[id];
	m_positions(idx) = pos;
	m_velocities(idx) = vel;
	m_accelerations(idx) = acc;
}

bool TorqueEstimator::estimate()
{
	if(m_solver->CartToJnt(
		m_positions, m_velocities, m_accelerations, m_forces, // in
		m_torques // out
	) != 0)
	{
		ROS_ERROR("Could not solve dynamic IK");
		return false;
	}

	return true;
}

double TorqueEstimator::torque(int id) const
{
	assert(id >= 0 && ((size_t) id) < m_idMap.size());
	return m_torques(m_idMap[id]);
}
