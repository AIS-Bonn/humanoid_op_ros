// Filter the available plots
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <plotter/plotfiltermodel.h>
#include <plotter/plotmodel.h>

namespace plotter
{

PlotFilterModel::PlotFilterModel(QObject* parent)
 : QSortFilterProxyModel(parent)
 , m_hideDisabledPlots(false)
{
	setDynamicSortFilter(true);
	setFilterCaseSensitivity(Qt::CaseInsensitive);
}

PlotFilterModel::~PlotFilterModel()
{
}

void PlotFilterModel::setHideDisabledPlots(bool on)
{
	m_hideDisabledPlots = on;
	invalidateFilter();
}

bool PlotFilterModel::filterAcceptsRow(int source_row, const QModelIndex& source_parent) const
{
	if(!source_parent.isValid())
		return true;

	if(m_hideDisabledPlots && !source_parent.child(source_row, PlotModel::COL_ENABLED).data().toBool())
		return false;

	QModelIndex child = source_parent.child(source_row, 0);
	if(!child.model()->hasChildren(child))
		return QSortFilterProxyModel::filterAcceptsRow(source_row, source_parent);

	return true;
}

}
