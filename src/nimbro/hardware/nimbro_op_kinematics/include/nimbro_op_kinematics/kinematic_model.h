// Kinematic model of NimbRo-OP
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef KINEMATIC_MODEL_H
#define KINEMATIC_MODEL_H

#include <nimbro_op_kinematics/leg.h>
#include <nimbro_op_kinematics/types.h>

#include <robotcontrol/model/robotmodel.h>

namespace nimbro_op_kinematics
{

class KinematicModel
{
public:
	KinematicModel(robotcontrol::RobotModel* robotModel);
	virtual ~KinematicModel();

	//! @name Leg accessor methods
	//@{
	inline Leg* leftLeg()               { return &m_leftLeg; }
	inline const Leg* leftLeg() const   { return &m_leftLeg; }

	inline Leg* rightLeg()              { return &m_rightLeg; }
	inline const Leg* rightLeg() const  { return &m_rightLeg; }
	//@}

	//! Compute forward kinematics (both from measurement and from command data)
	void updateKinematics();

	const boost::shared_ptr<robotcontrol::SingleSupportModel>& trunkModel()
	{ return m_trunkModel; }
private:
	robotcontrol::RobotModel* m_robotModel;
	boost::shared_ptr<robotcontrol::SingleSupportModel> m_trunkModel;

	Leg m_leftLeg;
	Leg m_rightLeg;
};

};

#endif
