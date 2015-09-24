// Provides file serialization for plots
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTTER_PLOTIO_H
#define PLOTTER_PLOTIO_H

#include <QtCore/QObject>

#include <ros/time.h>

namespace plotter
{

class Plot;

class PlotIO : public QObject
{
Q_OBJECT
public:
	PlotIO(Plot* rootPlot, QObject* parent);
	virtual ~PlotIO();

	bool read(const QString& path);
	bool write(const QString& path, const ros::Time& start, const ros::Time& end);
signals:
	void progress(double done);
private:
	Plot* m_rootPlot;
};

}

#endif
