// Estimates outside torques working on our joints
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TORQUEESTIMATOR_H
#define TORQUEESTIMATOR_H

#include <boost/shared_ptr.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

class TorqueEstimator
{
public:
	TorqueEstimator();
	virtual ~TorqueEstimator();

	bool init(const boost::shared_ptr<KDL::Chain>& chain);

	void setJointState(int id, double pos, double vel, double acc);
	bool estimate();
	double torque(int id) const;
private:
	boost::shared_ptr<KDL::Chain> m_chain;
	boost::shared_ptr<KDL::ChainIdSolver_RNE> m_solver;

	std::vector<int> m_idMap;
	KDL::JntArray m_positions;
	KDL::JntArray m_velocities;
	KDL::JntArray m_accelerations;
	KDL::JntArray m_torques;
	KDL::Wrenches m_forces;
};

#endif
