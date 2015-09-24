// Legend widget for plot
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTLEGENDWIDGET_H
#define PLOTLEGENDWIDGET_H

#include <QtGui/QWidget>

class Plot;
class QGridLayout;

class PlotLegendWidget : public QWidget
{
Q_OBJECT
public:
	explicit PlotLegendWidget(QWidget* parent = 0);
	virtual ~PlotLegendWidget();

	void setPlot(Plot* plot);
Q_SIGNALS:
	void plotSettingsChanged();
private Q_SLOTS:
	void plotChanged();
	void loggerChanged();
private:
	Plot* m_plot;
	QGridLayout* m_layout;
};

#endif
