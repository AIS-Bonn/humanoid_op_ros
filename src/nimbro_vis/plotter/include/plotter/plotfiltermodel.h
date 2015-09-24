// Filter the available plots
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTFILTERMODEL_H
#define PLOTFILTERMODEL_H

#include <QtGui/QSortFilterProxyModel>

namespace plotter
{

class PlotFilterModel : public QSortFilterProxyModel
{
Q_OBJECT
public:
	PlotFilterModel(QObject* parent = 0);
	virtual ~PlotFilterModel();
public slots:
	void setHideDisabledPlots(bool on);
protected:
	virtual bool filterAcceptsRow(int source_row, const QModelIndex& source_parent) const;
private:
	bool m_hideDisabledPlots;
};

}

#endif


