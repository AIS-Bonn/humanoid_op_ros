// Trunk orientation stabilization
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/motion_utils/trunk_stabilizer.h>
#include <model/robotmodel.h>

namespace robotcontrol
{

TrunkStabilizer::TrunkStabilizer(RobotModel* model)
 : CONFIG_PARAM_PATH("trunkStabilizer/")
 , m_model(model)
 , m_pitch_P(CONFIG_PARAM_PATH + "pitch/P", -2.0, 0.01, 2.0, 0.0)
 , m_pitch_D(CONFIG_PARAM_PATH + "pitch/D", -2.0, 0.01, 2.0, 0.0)
 , m_pitch_hip(CONFIG_PARAM_PATH + "pitch/hip", 0.0, 0.01, 2.0, 0.0)
 , m_pitch_knee(CONFIG_PARAM_PATH + "pitch/knee", 0.0, 0.01, 2.0, 0.0)
 , m_pitch_ankle(CONFIG_PARAM_PATH + "pitch/ankle", 0.0, 0.01, 2.0, 0.0)
 , m_roll_P(CONFIG_PARAM_PATH + "roll/P", -2.0, 0.01, 2.0, 0.0)
 , m_roll_D(CONFIG_PARAM_PATH + "roll/D", -2.0, 0.01, 2.0, 0.0)
 , m_roll_hip(CONFIG_PARAM_PATH + "roll/hip", 0.0, 0.01, 2.0, 0.0)
 , m_roll_ankle(CONFIG_PARAM_PATH + "roll/ankle", 0.0, 0.01, 2.0, 0.0)
{
	m_legs[LeftFoot].hip_pitch = model->getJoint("left_hip_pitch");
	m_legs[LeftFoot].knee_pitch = model->getJoint("left_knee_pitch");
	m_legs[LeftFoot].ankle_pitch = model->getJoint("left_ankle_pitch");

	m_legs[LeftFoot].hip_roll = model->getJoint("left_hip_roll");
	m_legs[LeftFoot].ankle_roll = model->getJoint("left_ankle_roll");

	m_legs[LeftFoot].supportModel = model->supportModel("left_foot_link");

	m_legs[RightFoot].hip_pitch = model->getJoint("right_hip_pitch");
	m_legs[RightFoot].knee_pitch = model->getJoint("right_knee_pitch");
	m_legs[RightFoot].ankle_pitch = model->getJoint("right_ankle_pitch");

	m_legs[RightFoot].hip_roll = model->getJoint("right_hip_roll");
	m_legs[RightFoot].ankle_roll = model->getJoint("right_ankle_roll");

	m_legs[RightFoot].supportModel = model->supportModel("right_foot_link");
}

TrunkStabilizer::~TrunkStabilizer()
{
}

/**
 * @param supportFoot Foot on the ground
 * @param goal Goal orientation (projected roll, projected pitch)
 **/
void TrunkStabilizer::update(const Eigen::Vector2d& goal)
{
	// TODO: Remove smoothing if unneeded
	Eigen::Vector3d gyro = m_model->robotAngularVelocity();
	m_gyro_golay.put(gyro.head<2>());
	Eigen::Vector2d gyro_smooth = m_gyro_golay.value();
	
	double fusedPitch = m_model->robotFPitchPR();
	double fusedRoll  = m_model->robotFRollPR();

	double pitch_cmd =
		  m_pitch_P() * (fusedPitch - goal.y())
		+ m_pitch_D() * gyro.y()
	;

	double roll_cmd =
		  m_roll_P() * (fusedRoll - goal.x())
		+ m_roll_D() * gyro_smooth.x()
	;

	for(int i = 0; i < 2; ++i)
	{
		Leg* leg = &m_legs[i];
		double coeff = leg->supportModel->coeff();

		// Move both legs at the hip
		leg->hip_pitch->cmd.pos += m_pitch_hip() * pitch_cmd;
		leg->hip_roll->cmd.pos += m_roll_hip() * roll_cmd;

		// Move just the support leg (and fade during double support)
		leg->knee_pitch->cmd.pos += coeff * m_pitch_knee() * pitch_cmd;

		leg->ankle_pitch->cmd.pos += coeff * m_pitch_ankle() * pitch_cmd;
		leg->ankle_roll->cmd.pos += coeff * m_roll_ankle() * roll_cmd;
	}
}

void TrunkStabilizer::plot(plot_msgs::Plot* plot)
{
	Eigen::Vector3d gyro = m_model->robotAngularVelocity();

	plot_msgs::PlotPoint p;
	p.name = "/trunkStabilizer/pitch/gyro";
	p.value = gyro.y();
	plot->points.push_back(p);

	p.name = "/trunkStabilizer/pitch/gyro_smooth";
	p.value = m_gyro_golay.value().y();
	plot->points.push_back(p);

	p.name = "/trunkStabilizer/pitch/fusedPitch";
	p.value = m_model->robotFPitchPR();
	plot->points.push_back(p);
}

}
