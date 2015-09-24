// Trunk orientation stabilization
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROBOTCONTROL_TRUNK_STABILIZER_H
#define ROBOTCONTROL_TRUNK_STABILIZER_H

#include <robotcontrol/model/singlesupportmodel.h>
#include <robotcontrol/model/joint.h>
#include <config_server/parameter.h>

#include <plot_msgs/Plot.h>

namespace robotcontrol
{
class RobotModel;

class TrunkStabilizer
{
public:
	TrunkStabilizer(RobotModel* model);
	virtual ~TrunkStabilizer();

	enum Foot
	{
		LeftFoot,
		RightFoot
	};

	void update(const Eigen::Vector2d& goal = Eigen::Vector2d::Zero());
	void plot(plot_msgs::Plot* plot);

private:
	// Constants
	const std::string CONFIG_PARAM_PATH;

	RobotModel* m_model;

	struct Leg
	{
		Joint::Ptr hip_pitch;
		Joint::Ptr knee_pitch;
		Joint::Ptr ankle_pitch;

		Joint::Ptr hip_roll;
		Joint::Ptr ankle_roll;

		boost::shared_ptr<SingleSupportModel> supportModel;
	};

	Leg m_legs[2];

	GolayDerivative<Eigen::Vector2d, 0, 5, Eigen::aligned_allocator<Eigen::Vector2d> > m_gyro_golay;

	config_server::Parameter<float> m_pitch_P;
	config_server::Parameter<float> m_pitch_D;

	config_server::Parameter<float> m_pitch_hip;
	config_server::Parameter<float> m_pitch_knee;
	config_server::Parameter<float> m_pitch_ankle;

	config_server::Parameter<float> m_roll_P;
	config_server::Parameter<float> m_roll_D;

	config_server::Parameter<float> m_roll_hip;
	config_server::Parameter<float> m_roll_ankle;
};

}

#endif
