// Tree model for parameters
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PARAMETER_MODEL_H
#define PARAMETER_MODEL_H

#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QPointer>

#include <config_server/ParameterValueList.h>

#include "parameter_item.h"

namespace remote_tuner
{

class ParameterModel : public QAbstractItemModel
{
Q_OBJECT
public:
	enum Columns
	{
		COL_NAME,
		COL_VALUE,

		COL_COUNT
	};

	explicit ParameterModel(QObject* parent = 0);
	virtual ~ParameterModel();

	virtual int rowCount(const QModelIndex& index) const override;
	virtual int columnCount(const QModelIndex& index) const override;

	virtual QModelIndex parent(const QModelIndex& index) const override;

	virtual QModelIndex index(int row, int col, const QModelIndex& parent = QModelIndex()) const override;

	virtual QVariant data(const QModelIndex& index, int role) const override;
	virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
	virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
	virtual bool setData(const QModelIndex& index, const QVariant& data, int role) override;

	config_server::ParameterValue value(const QModelIndex& index) const;

	// HACK
	void initializeView(QAbstractItemView* view);

public Q_SLOTS:
	void update(const config_server::ParameterValueList& list);
Q_SIGNALS:
	void setRequested(const QString& name, const QString& value);
private:
	struct ParameterNode
	{
		ParameterNode(ParameterNode* parent = nullptr);
		~ParameterNode();

		int row() const;

		ParameterNode* parent;

		std::string path;
		std::string name;
		std::vector<ParameterNode*> children;

		std::string value;
		config_server::ParameterValue pValue;
		bool pending;

		QPointer<ParameterWidgetBase> widget;
	};

	const ParameterNode* nodeFromIndex(const QModelIndex& index) const;
	ParameterNode* nodeFromIndex(const QModelIndex& index);

	QModelIndex indexForNode(const ParameterNode* node, int col = 0);

	void insertNode(const std::string& name, const config_server::ParameterValue& value);

	ParameterWidgetBase* widgetForNode(const ParameterNode* node) const;

	ParameterNode m_rootNode;

	std::vector<ParameterNode*> m_linearizedNodeList;

	// HACK
	QAbstractItemView* m_view;
};

}

#endif
