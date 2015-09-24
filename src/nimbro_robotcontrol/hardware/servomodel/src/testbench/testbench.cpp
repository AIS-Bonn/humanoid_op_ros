// Testbench main window
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "testbench.h"
#include "singleservotestbench.h"
#include "ijointinterface.h"
#include "dxljointinterface.h"
#include "tunablecommandgenerator.h"
#include "trajectory.h"
#include "compositetrajectory.h"
#include "trajectorypart.h"

#include "ui_testbench.h"

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <QtGui/QVBoxLayout>

Testbench::Testbench()
 : QWidget()
{
	m_ui = new Ui::Testbench;
	m_ui->setupUi(this);
	connect(m_ui->startButton, SIGNAL(clicked()), SLOT(start()));
	connect(m_ui->stopButton, SIGNAL(clicked()), SLOT(stop()));

	m_timer = new QTimer(this);
	m_timer->setInterval(20);
	connect(m_timer, SIGNAL(timeout()), SLOT(doStep()));

	m_repTimer = new QTimer(this);
	m_repTimer->setInterval(4000);
	m_repTimer->setSingleShot(true);
	connect(m_repTimer, SIGNAL(timeout()), SLOT(start()));

	m_traj = new CompositeTrajectory;
}

Testbench::~Testbench()
{
	delete m_ui;
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

bool Testbench::init()
{
	ROS_ERROR("Testbench::init");
	boost::shared_ptr<urdf::Model> model = boost::make_shared<urdf::Model>();
	if(!model->initParam("robot_description"))
	{
		ROS_ERROR("Could not get robot description");
		return false;
	}

	if(!kdl_parser::treeFromUrdfModel(*model, m_tree))
	{
		ROS_ERROR("Could not get KDL tree from URDF model");
		return false;
	}

	boost::shared_ptr<KDL::Chain> chain = boost::make_shared<KDL::Chain>();
	// FIXME
	if(!m_tree.getChain("trunk_link", "left_foot_link", *chain))
	{
		ROS_ERROR("Could not get chain");
		return false;
	}

	ROS_ERROR("Initializing torque estimator...");
	fflush(stderr);
	m_estimator.reset(new TorqueEstimator);
	if(!m_estimator->init(chain))
	{
		ROS_ERROR("Could not initialize torque estimator");
		return false;
	}

	QMap<QString, JointSpec> specs;
	specs["left_hip_yaw"] = JointSpec(8, 2048, false, false);
	specs["left_hip_roll"] = JointSpec(10, 1792, false, false);
	specs["left_hip_pitch"] = JointSpec(12, 2040, false, true);
	specs["left_knee_pitch"] = JointSpec(14, 1205, false, true);
	specs["left_ankle_pitch"] = JointSpec(16, 2046, true, false);
	specs["left_ankle_roll"] = JointSpec(18, 2304, true, false);

	std::vector<boost::shared_ptr<urdf::Link> > links;
	model->getLinks(links);

	for(int i = 0; i < links.size(); ++i)
	{
		const urdf::Link& link = *links[i];
		if(!link.parent_joint)
			continue;

		const urdf::Joint& joint = *link.parent_joint;
		QString joint_name = joint.name.c_str();

		const KDL::Joint* kdlJoint = 0;
		for(int i = 0; i < chain->getNrOfSegments(); ++i)
		{
			const KDL::Segment& segment = chain->getSegment(i);
			if(segment.getJoint().getName() == joint.name)
			{
				kdlJoint = &segment.getJoint();
				break;
			}
		}

		if(!kdlJoint)
			continue;

		if(!specs.contains(joint_name))
		{
			ROS_ERROR("Unknown joint '%s'", joint.name.c_str());
			return false;
		}

		const JointSpec& spec = specs[joint_name];

		DXLJointInterface* iface = new DXLJointInterface(spec.id);
		iface->setOffsetTicks(spec.offset);
		if(!iface->init())
		{
			ROS_ERROR("Could not initialize joint interface for joint '%s'\n", joint.name.c_str());
			return false;
		}

		if(spec.active)
		{
			SingleServoTestbench* tb = new SingleServoTestbench(iface, m_estimator, this);
			m_ui->tabWidget->addTab(tb, joint_name);

			tb->takeTrajectory(new TrajectoryPart(m_traj, spec.id));
			tb->setObjectName(QString("Testbench_%1").arg(joint_name));
			tb->loadSettings();

			m_benches << tb;
			m_jointNames << joint_name;

			connect(tb, SIGNAL(trajectoryChanged()), SLOT(generateCommands()));
		}
		else
		{
			iface->setGoalPosition(0);
			iface->setPValue(5);
			delete iface;
		}
	}

	generateCommands();

	return true;
}

int Testbench::trajectoryTicks()
{
	double duration = 0;
	Q_FOREACH(SingleServoTestbench* tb, m_benches)
	{
		if(tb->trajectory().endTime() >= duration)
			duration = tb->trajectory().endTime();
	}

	return duration * PROCESS_FREQ;
}

void Testbench::generateCommands()
{
	int ticks = trajectoryTicks();

	QVector<QVector<double> > discreteTrajectories(m_benches.size());
	QVector<Keyframe> motionStates(m_benches.count());

	Q_FOREACH(SingleServoTestbench* tb, m_benches)
		tb->resetOutsideTorques();

	for(int t = 0; t < ticks; ++t)
	{
		double time = PROCESS_PERIOD * t;

		for(int i = 0; i < m_benches.count(); ++i)
		{
			const Trajectory& traj = m_benches[i]->trajectory();
			int id = m_benches[i]->jointInterface()->id();
			Keyframe state;
			double stime = time + m_benches[i]->generator()->latency();
			state.x = traj.position(stime);
			state.v = traj.velocity(stime);
			state.a = traj.acceleration(stime);

			m_estimator->setJointState(id, state.x, state.v, state.a);

			motionStates[i] = state;
		}

		if(!m_estimator->estimate())
		{
			ROS_ERROR("Could not estimate servo torques for time %lf", time);
			return;
		}

		for(int i = 0; i < m_benches.count(); ++i)
		{
			const Keyframe& state = motionStates[i];

			double outsideTorque = m_estimator->torque(m_benches[i]->jointInterface()->id());

			discreteTrajectories[i]
				<< m_benches[i]->generator()->servoCommandFor(state.x, state.v, state.a, outsideTorque);

			m_benches[i]->logOutsideTorque(time + m_benches[i]->generator()->latency());
		}
	}

	for(int i = 0; i < m_benches.count(); ++i)
	{
		m_benches[i]->setModelTrajectory(discreteTrajectories[i]);
	}
}

void Testbench::start()
{
	Q_FOREACH(SingleServoTestbench* tb, m_benches)
		tb->reset();

	m_stopWatch.restart();
	m_timeStamp = 0;
	m_timer->start();
}

void Testbench::stop()
{
	Q_FOREACH(SingleServoTestbench* tb, m_benches)
		tb->jointInterface()->relax();

	m_timer->stop();
	m_repTimer->stop();
}

void Testbench::doStep()
{
	double currentTime = m_stopWatch.elapsedTime();
	double timePassed = currentTime - m_timeStamp;
	m_timeStamp = currentTime;

	if(currentTime / PROCESS_PERIOD >= trajectoryTicks())
	{
		stop();

		Q_FOREACH(SingleServoTestbench* tb, m_benches)
			tb->updateStatistics();

		if(m_ui->learnCheckBox->isChecked())
		{
			Q_FOREACH(SingleServoTestbench* tb, m_benches)
				tb->updateTrajectory();

			m_repTimer->start();
		}

		return;
	}

	Q_FOREACH(SingleServoTestbench* tb, m_benches)
	{
		tb->doStep(currentTime);
	}
}

void Testbench::closeEvent(QCloseEvent* )
{
	Q_FOREACH(SingleServoTestbench* tb, m_benches)
		tb->saveSettings();
}





