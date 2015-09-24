// Plot tree model
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTMODEL_H
#define PLOTMODEL_H

#include <QtCore/QAbstractItemModel>
#include <QtGui/QFont>

#include "plotter/plot.h"

class QSettings;

namespace plotter
{

/**
 * Contains a tree of all available Plot objects.
 **/
class PlotModel : public QAbstractItemModel
{
Q_OBJECT
public:
	enum Columns
	{
		COL_NAME,
		COL_VALUE,
		COL_ENABLED,
		COL_COLOR,
		COL_COUNT
	};

	explicit PlotModel(QObject* parent = 0);
	virtual ~PlotModel();

	//! @name Qt model methods
	//@{
	virtual int columnCount(const QModelIndex& parent = QModelIndex()) const;
	virtual int rowCount(const QModelIndex& parent = QModelIndex()) const;

	virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

	virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const;
	virtual QModelIndex parent(const QModelIndex& child) const;

	virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
	virtual bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole);

	virtual Qt::ItemFlags flags(const QModelIndex& index) const;
	virtual QStringList mimeTypes() const;
	virtual QMimeData* mimeData(const QModelIndexList& indexes) const;
	//@}

	void addPlot(Plot* plot);
	Plot* plotByPath(const QString& path);

	Plot* rootPlot();

public slots:
	void setCurrentTime(const ros::Time& time);
private slots:
	void doReset();
	void doChange(Plot* source);
	void doBeginAddChild(Plot* parent, int idx);
	void doEndAddChild(Plot* parent);
private:
	Plot m_topLevelPlot;
	ros::Time m_currentTime;
	QFont m_valueFont;

	Plot* plotFromIndex(const QModelIndex& index);
	const Plot* plotFromIndex(const QModelIndex& index) const;

	void columnChanged(Columns column, const QModelIndex& parent = QModelIndex());
};

}

#endif
