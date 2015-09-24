// Plot tree model
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotter/plotmodel.h"

#include <QtCore/QStringList>
#include <QtCore/QMimeData>

#include <ros/console.h>

namespace plotter
{

PlotModel::PlotModel(QObject* parent)
 : QAbstractItemModel(parent)
 , m_topLevelPlot("topLevel")
{
	connect(&m_topLevelPlot, SIGNAL(hierarchyChanged()), SLOT(doReset()));
	connect(&m_topLevelPlot, SIGNAL(changed(Plot*)), SLOT(doChange(Plot*)));
	connect(&m_topLevelPlot, SIGNAL(beginAddChild(Plot*,int)), SLOT(doBeginAddChild(Plot*,int)));
	connect(&m_topLevelPlot, SIGNAL(childAdded(Plot*)), SLOT(doEndAddChild(Plot*)));

	m_topLevelPlot.setEnabled(true);

	m_valueFont = QFont("monospace");
}

PlotModel::~PlotModel()
{
}

QVariant PlotModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(role != Qt::DisplayRole || orientation != Qt::Horizontal)
		return QVariant();

	switch(section)
	{
		case COL_NAME:    return "Name";
		case COL_VALUE:   return "Current value";
		case COL_ENABLED: return "X";
		case COL_COLOR:   return "Color";
	}

	return QVariant();
}

void PlotModel::doReset()
{
	reset();
}

void PlotModel::addPlot(Plot* plot)
{
	plot->setParent(&m_topLevelPlot);
}

Plot* PlotModel::plotByPath(const QString& path)
{
	return m_topLevelPlot.findPlotByPath(path);
}

int PlotModel::columnCount(const QModelIndex& parent) const
{
	return COL_COUNT;
}

int PlotModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid() && parent.column() != 0)
		return 0;

	const Plot* plot = plotFromIndex(parent);

	return plot->childCount();
}

QModelIndex PlotModel::index(int row, int column, const QModelIndex& parent) const
{
	const Plot* parentPlot = plotFromIndex(parent);

	if(row < 0 || row >= parentPlot->childCount())
		return QModelIndex();

	if(column < 0 || column >= COL_COUNT)
		return QModelIndex();

	const Plot* plot = parentPlot->child(row);

	return createIndex(row, column, (void*)plot);
}

Plot* PlotModel::plotFromIndex(const QModelIndex& index)
{
	if(!index.isValid())
		return &m_topLevelPlot;
	else
		return (Plot*)index.internalPointer();
}

const Plot* PlotModel::plotFromIndex(const QModelIndex& index) const
{
	if(!index.isValid())
		return &m_topLevelPlot;
	else
		return (Plot*)index.internalPointer();
}

QModelIndex PlotModel::parent(const QModelIndex& child) const
{
	if(!child.isValid())
		return QModelIndex();

	const Plot* childPlot = plotFromIndex(child);

	if(childPlot == &m_topLevelPlot)
		return QModelIndex();

	const Plot* parentPlot = (const Plot*)childPlot->parent();

	if(parentPlot == &m_topLevelPlot)
		return QModelIndex();

	const Plot* grandParent = qobject_cast<Plot*>(parentPlot->parent());

	return createIndex(grandParent->indexOfChild(parentPlot), 0, (void*)parentPlot);
}

QVariant PlotModel::data(const QModelIndex& index, int role) const
{
	const Plot* plot = plotFromIndex(index);

	if(role == Qt::TextAlignmentRole)
	{
		if(index.column() == COL_VALUE)
			return QVariant(Qt::AlignVCenter | Qt::AlignRight);
		else
			return QVariant(Qt::AlignVCenter | Qt::AlignLeft);
	}

	if(role == Qt::FontRole)
	{
		if(index.column() == COL_VALUE)
			return m_valueFont;
		else
			return QVariant();
	}

	if(role != Qt::DisplayRole)
		return QVariant();

	switch(index.column())
	{
		case COL_NAME:
			return plot->name();
		case COL_VALUE:
		{
			if(!plot->hasData())
				return QVariant();

			double val;
			if(m_currentTime.isValid())
			{
				val = plot->value(m_currentTime);

				if(isnan(val))
					return "---";
			}
			else
				val = plot->lastValue();

			QByteArray buf(100, Qt::Uninitialized);
			snprintf(buf.data(), buf.size(), "%.4lf", val);
			return QString(buf);
		}
		case COL_ENABLED:
			return plot->isEnabled();
		case COL_COLOR:
			if(!plot->isEnabled() || !plot->hasData())
				return QColor(Qt::transparent);

			return plot->color();
	}

	return QVariant();
}

bool PlotModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if(role != Qt::EditRole)
		return false;

	Plot* plot = plotFromIndex(index);

	switch(index.column())
	{
		case COL_ENABLED:
			plot->setEnabled(value.toBool());
			dataChanged(index, index);
			return true;
		case COL_COLOR:
			plot->setColor(value.value<QColor>());
			return true;
	}

	return false;
}

Qt::ItemFlags PlotModel::flags(const QModelIndex& index) const
{
	if(!index.isValid())
		return 0;

	Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;

	switch(index.column())
	{
		case COL_COLOR:
			flags |= Qt::ItemIsEditable;
	}
	return flags;
}

QStringList PlotModel::mimeTypes() const
{
	QStringList types;
	types << "application/x-nimbro-plot";
	return types;
}

QMimeData* PlotModel::mimeData(const QModelIndexList& indexes) const
{
	if(indexes.count() == 0)
		return 0;

	QModelIndex index = indexes[0];

	QMimeData* mimeData = new QMimeData;
	mimeData->setData("application/x-nimbro-plot", plotFromIndex(index)->path().toLatin1());

	return mimeData;
}

void PlotModel::doChange(Plot* source)
{
	if(source == &m_topLevelPlot)
		return;

	Plot* parent = (Plot*)source->parent();

	int row = parent->indexOfChild(source);
	QModelIndex left = createIndex(row, 0, (void*)source);
	QModelIndex right = createIndex(row, COL_COUNT-1, parent->indexOfChild(source));
	dataChanged(left, right);
}

void PlotModel::doBeginAddChild(Plot* parent, int idx)
{
	int row = 0;
	QModelIndex index;

	if(parent != &m_topLevelPlot)
	{
		Plot* grandParent = qobject_cast<Plot*>(parent->parent());
		if(grandParent)
			row = grandParent->indexOfChild(parent);

		index = createIndex(row, 0, (void*)parent);
	}

	beginInsertRows(index, idx, idx);
}

void PlotModel::doEndAddChild(Plot* parent)
{
	endInsertRows();
}

Plot* PlotModel::rootPlot()
{
	return &m_topLevelPlot;
}

void PlotModel::setCurrentTime(const ros::Time& time)
{
	m_currentTime = time;
	columnChanged(COL_VALUE);
}

void PlotModel::columnChanged(PlotModel::Columns column, const QModelIndex& parent)
{
	int rows = rowCount(parent);
	dataChanged(index(0, column), index(rows, column));

	for(int i = 0; i < rows; ++i)
	{
		QModelIndex idx = index(i, 0, parent);
		if(!rowCount(idx))
			continue;

		columnChanged(column, idx);
	}
}


}
