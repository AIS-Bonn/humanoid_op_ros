// Provides file serialization for plots
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTTER_PLOTIO_H
#define PLOTTER_PLOTIO_H

#include <QtCore/QObject>

#include <ros/time.h>
#include <ros/node_handle.h>

namespace plotter
{

class Plot;

class PlotIO : public QObject
{
Q_OBJECT
public:
	PlotIO(ros::NodeHandle& nh, Plot* rootPlot, QObject* parent);
	virtual ~PlotIO();

	bool read(const QString& path);
	bool write(const QString& path, const ros::Time& start, const ros::Time& end);
signals:
	void progress(double done);
private:
	ros::NodeHandle m_nh;
	Plot* m_rootPlot;
};

}

#endif
