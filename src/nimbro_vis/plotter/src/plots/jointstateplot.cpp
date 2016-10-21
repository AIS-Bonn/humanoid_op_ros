// Provides plots of the joint angles and efforts
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include "plotter/plots/jointstateplot.h"
#include <ros/node_handle.h>
#include <ros/console.h>
#include <qmetatype.h>

// Plotter namespace
namespace plotter
{

// Class constructor
JointStatePlot::JointStatePlot(ros::NodeHandle& nh, Plot* parent) : Plot(JS_PLOT_TOPIC, parent)
{
	// Create plot groups to hold all the joint-specific plots
	m_group.positionPlot        = new Plot("Positions", this);
	m_group.velocityPlot        = new Plot("Velocities", this);
	m_group.torquePlot          = new Plot("Torques", this);
	m_group.cmdPositionPlot     = new Plot("Cmd Positions", this);
	m_group.cmdVelocityPlot     = new Plot("Cmd Velocities", this);
	m_group.cmdAccelerationPlot = new Plot("Cmd Accelerations", this);
	m_group.cmdRawPositionPlot  = new Plot("Cmd Raw Positions", this);
	m_group.cmdTorquePlot       = new Plot("Cmd Torques", this);
	m_group.cmdEffortPlot       = new Plot("Cmd Efforts", this);

	// Subscribe to the required joint data topics
	boost::function<void(const sensor_msgs::JointStateConstPtr&)> gotDataCb = boost::bind(&JointStatePlot::gotData, this, _1, false);
	boost::function<void(const plot_msgs::JointCommandConstPtr&)> gotCmdDataCb = boost::bind(&JointStatePlot::gotCmdData, this, _1, false);
	m_sub_js = nh.subscribe<sensor_msgs::JointStateConstPtr>("/joint_states", 50, gotDataCb);
	m_sub_cmd_js = nh.subscribe<plot_msgs::JointCommandConstPtr>("/joint_commands", 50, gotCmdDataCb);

	// Register the plot message types with Qt
	qRegisterMetaType<sensor_msgs::JointStateConstPtr>("sensor_msgs::JointStateConstPtr");
	qRegisterMetaType<plot_msgs::JointCommandConstPtr>("plot_msgs::JointCommandConstPtr");

	// Connect the required Qt signals and slots
	connect(this, SIGNAL(gotData(sensor_msgs::JointStateConstPtr, bool)),    SLOT(handleData(sensor_msgs::JointStateConstPtr, bool)),    Qt::QueuedConnection);
	connect(this, SIGNAL(gotCmdData(plot_msgs::JointCommandConstPtr, bool)), SLOT(handleCmdData(plot_msgs::JointCommandConstPtr, bool)), Qt::QueuedConnection);
}

// Class destructor
JointStatePlot::~JointStatePlot()
{
	// Shut down the required subscribers
	shutdownNode();
}

void JointStatePlot::shutdownNode()
{
	// Shut down the subscribers that we own
	m_sub_js.shutdown();
	m_sub_cmd_js.shutdown();
}

void JointStatePlot::shutdown()
{
	// Shut down this node
	shutdownNode();

	// Shut down whatever the base implementation needs to
	Plot::shutdown();
}

// Find or create a plot by path
Plot* JointStatePlot::findOrCreatePlotByPath(const QString& path)
{
	// Get the local path component
	QString local = path.section('/', 0, 0, QString::SectionSkipEmpty);
	QString subPath = path.section('/', 1, -1, QString::SectionSkipEmpty);

	// See whether the local path component is special
	bool isSpecialLocal = (local == m_group.positionPlot->name()       || local == m_group.velocityPlot->name()    || local == m_group.torquePlot->name()          ||
	                       local == m_group.cmdPositionPlot->name()    || local == m_group.cmdVelocityPlot->name() || local == m_group.cmdAccelerationPlot->name() ||
	                       local == m_group.cmdRawPositionPlot->name() || local == m_group.cmdTorquePlot->name()   || local == m_group.cmdEffortPlot->name());

	// If not special then just proceed as normal
	if(!isSpecialLocal)
		return plotter::Plot::findOrCreatePlotByPath(path);

	// See whether the subpath is special
	QString subLocal = path.section('/', 1, 1, QString::SectionSkipEmpty);
	bool isSpecialSubPath = (subPath == subLocal);

	// If not special then just proceed as normal
	if(!isSpecialSubPath)
		return plotter::Plot::findOrCreatePlotByPath(path);

	// Create plotters for the entry if we don't already have one
	if(m_plotters.find(subLocal) == m_plotters.end())
		createPlottersFor(subLocal);

	// Now that we have created the required plotter we proceed as normal, and this should just retrieve the already created plot
	return plotter::Plot::findOrCreatePlotByPath(path);
}

// Handle received joint state data
void JointStatePlot::handleData(const sensor_msgs::JointStateConstPtr& data, bool overridePause)
{
	// Don't accept any data if paused
	if(paused() && !overridePause) return;

	// Handle the received data
	for(std::size_t i = 0; i < data->position.size(); i++)
	{
		// Retrieve the i-th joint name and check whether we already have plots for it
		QString name = QString::fromStdString(data->name[i]);
		PlotHash::iterator it = m_plotters.find(name);

		// If we do not have plots for a joint of this name then create some
		if(it == m_plotters.end())
			it = createPlottersFor(name);

		// Update the joint state plots for the joint
		JointPlotters* plotters = &(*it);
		plotters->positionPlot->put(data->header.stamp, data->position[i], NULL, true, overridePause);
		if(i < data->velocity.size()) plotters->velocityPlot->put(data->header.stamp, data->velocity[i], NULL, true, overridePause);
		if(i < data->effort.size()) plotters->torquePlot->put(data->header.stamp, data->effort[i], NULL, true, overridePause);
	}
}

// Handle received joint command data
void JointStatePlot::handleCmdData(const plot_msgs::JointCommandConstPtr& data, bool overridePause)
{
	// Don't accept any data if paused
	if(paused() && !overridePause) return;

	// Handle the received data
	for(std::size_t i = 0; i < data->position.size(); i++)
	{
		// Retrieve the i-th joint name and check whether we already have plots for it
		QString name = QString::fromStdString(data->name[i]);
		PlotHash::iterator it = m_plotters.find(name);

		// If we do not have plots for a joint of this name then create some
		if(it == m_plotters.end())
			it = createPlottersFor(name);

		// Update the joint state plots for the joint
		JointPlotters* plotters = &(*it);
		plotters->cmdPositionPlot->put(data->header.stamp, data->position[i], NULL, true, overridePause);
		if(i < data->velocity.size()) plotters->cmdVelocityPlot->put(data->header.stamp, data->velocity[i], NULL, true, overridePause);
		if(i < data->acceleration.size()) plotters->cmdAccelerationPlot->put(data->header.stamp, data->acceleration[i], NULL, true, overridePause);
		if(i < data->rawPosition.size()) plotters->cmdRawPositionPlot->put(data->header.stamp, data->rawPosition[i], NULL, true, overridePause);
		if(i < data->torque.size()) plotters->cmdTorquePlot->put(data->header.stamp, data->torque[i], NULL, true, overridePause);
		if(i < data->effort.size()) plotters->cmdEffortPlot->put(data->header.stamp, data->effort[i], NULL, true, overridePause);
	}
}

// Create an entry in the plot hash for a joint of the given name
JointStatePlot::PlotHash::iterator JointStatePlot::createPlottersFor(const QString& name)
{
	// Create a JointPlotters struct instance
	JointPlotters newPlotters;

	// Create the new plots
	newPlotters.positionPlot        = new Plot(name, m_group.positionPlot);
	newPlotters.velocityPlot        = new Plot(name, m_group.velocityPlot);
	newPlotters.torquePlot          = new Plot(name, m_group.torquePlot);
	newPlotters.cmdPositionPlot     = new Plot(name, m_group.cmdPositionPlot);
	newPlotters.cmdVelocityPlot     = new Plot(name, m_group.cmdVelocityPlot);
	newPlotters.cmdAccelerationPlot = new Plot(name, m_group.cmdAccelerationPlot);
	newPlotters.cmdRawPositionPlot  = new Plot(name, m_group.cmdRawPositionPlot);
	newPlotters.cmdTorquePlot       = new Plot(name, m_group.cmdTorquePlot);
	newPlotters.cmdEffortPlot       = new Plot(name, m_group.cmdEffortPlot);

	// Insert the new joint plotters into the plot hash and return an iterator to it
	return m_plotters.insert(name, newPlotters);
}

}
// EOF