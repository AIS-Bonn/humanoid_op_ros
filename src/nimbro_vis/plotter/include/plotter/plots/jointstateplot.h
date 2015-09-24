// Provides plots of the joint angles & efforts
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotter/plot.h"

#include <QtCore/QHash>
#include <sensor_msgs/JointState.h>
#include <plot_msgs/JointCommand.h>
#include <ros/subscriber.h>

namespace plotter
{

class JointStatePlot : public Plot
{
Q_OBJECT
public:
	JointStatePlot(ros::NodeHandle& nh, Plot* parent = 0);
	virtual ~JointStatePlot();

	//! Shut the ROS subscribers in this plot down
	void shutdown();
Q_SIGNALS:
	void gotData(const sensor_msgs::JointStateConstPtr& data);
	void gotCmdData(const plot_msgs::JointCommandConstPtr& data);
private Q_SLOTS:
	void handleData(const sensor_msgs::JointStateConstPtr& data);
	void handleCmdData(const plot_msgs::JointCommandConstPtr& data);
private:
	struct JointPlotters
	{
		Plot* positionPlot;
		Plot* velocityPlot;
		Plot* torquePlot;
		Plot* cmdPositionPlot;
		Plot* cmdVelocityPlot;
		Plot* cmdAccelerationPlot;
		Plot* cmdRawPositionPlot;
		Plot* cmdTorquePlot;
		Plot* cmdEffortPlot;
	};
	
	JointPlotters m_group;

	typedef QHash<QString, JointPlotters> PlotHash;
	PlotHash m_plotters; // Map from joint name (string) to the struct of plots corresponding to that joint

	PlotHash::iterator createPlottersFor(const QString& name);

	ros::Subscriber m_sub_js;
	ros::Subscriber m_sub_cmd_js;
};

}