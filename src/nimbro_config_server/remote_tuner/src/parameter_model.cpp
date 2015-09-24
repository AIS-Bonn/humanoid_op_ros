// Tree model for parameters
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "parameter_model.h"
#include "parameter_item.h"

#include <stack>

#include <boost/tokenizer.hpp>

#include <config_server/SetParameter.h>

#include <ros/service.h>

#include <QMessageBox>

namespace remote_tuner
{

ParameterModel::ParameterNode::ParameterNode(ParameterNode* parent)
 : parent(parent)
 , pending(false)
{
}

ParameterModel::ParameterNode::~ParameterNode()
{
	qDeleteAll(children);
}

int ParameterModel::ParameterNode::row() const
{
	if(parent)
		return std::find(parent->children.begin(), parent->children.end(), this) - parent->children.begin();

	return 0;
}

ParameterModel::ParameterModel(QObject* parent)
 : QAbstractItemModel(parent)
 , m_view(nullptr)
{
}

ParameterModel::~ParameterModel()
{
}

const ParameterModel::ParameterNode* ParameterModel::nodeFromIndex(const QModelIndex& index) const
{
	if(!index.isValid())
		return &m_rootNode;

	return reinterpret_cast<const ParameterNode*>(index.internalPointer());
}
ParameterModel::ParameterNode* ParameterModel::nodeFromIndex(const QModelIndex& index)
{
	if(!index.isValid())
		return nullptr;

	if(index.internalPointer())
		return reinterpret_cast<ParameterNode*>(index.internalPointer());
	else
		return &m_rootNode;
}

QModelIndex ParameterModel::indexForNode(const ParameterNode* node, int col)
{
	if(node == &m_rootNode)
		return QModelIndex();
	else
		return createIndex(node->row(), col, const_cast<ParameterNode*>(node));
}

int ParameterModel::rowCount(const QModelIndex& index) const
{
	auto node = nodeFromIndex(index);

	if(!node)
		return 0;

	return node->children.size();
}

int ParameterModel::columnCount(const QModelIndex& index) const
{
	auto node = nodeFromIndex(index);

	if(!node)
		return 0;

	return COL_COUNT;
}

QModelIndex ParameterModel::parent(const QModelIndex& index) const
{
	if(!index.isValid())
		return QModelIndex();

	auto node = nodeFromIndex(index);
	auto parent = node->parent;

	if(parent == &m_rootNode)
		return QModelIndex();

	return createIndex(parent->row(), 0, parent);
}

QModelIndex ParameterModel::index(int row, int col, const QModelIndex& parent) const
{
	if(!hasIndex(row, col, parent))
		return QModelIndex();

	auto node = nodeFromIndex(parent);

	if(row < 0 || row >= (int)node->children.size())
		return QModelIndex();

	return createIndex(row, col, node->children[row]);
}

QVariant ParameterModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(orientation != Qt::Horizontal || role != Qt::DisplayRole)
		return QVariant();

	switch(section)
	{
		case COL_NAME: return "Name";
		case COL_VALUE: return "Value";
	}

	return QVariant();
}

QVariant ParameterModel::data(const QModelIndex& index, int role) const
{
	auto node = nodeFromIndex(index);

	if(!node)
		return QVariant();

	switch(role)
	{
		case Qt::DisplayRole:
		case Qt::EditRole:
			switch(index.column())
			{
				case COL_NAME:
					return QString::fromStdString(node->name);
				case COL_VALUE:
					if(node->widget)
						return QVariant();
					else
						return QString::fromStdString(node->value);
			}
			break;
		case Qt::BackgroundColorRole:
			switch(index.column())
			{
				case COL_VALUE:
					if(node->pending)
						return Qt::yellow;
					break;
			}
			break;
	}

	return QVariant();
}

Qt::ItemFlags ParameterModel::flags(const QModelIndex& index) const
{
	Qt::ItemFlags flags = QAbstractItemModel::flags(index);

	if(!index.isValid())
		return flags;

	auto node = nodeFromIndex(index);

	if(index.column() == COL_VALUE && node->children.size() == 0)
		flags |= Qt::ItemIsEditable;

	return flags;
}

bool ParameterModel::setData(const QModelIndex& index, const QVariant& data, int role)
{
	if(role != Qt::EditRole)
		return false;

	if(!index.isValid() || index.column() != COL_VALUE)
		return false;

	auto node = nodeFromIndex(index);

	setRequested(QString::fromStdString(node->path), data.toString());

	node->pending = true;
	node->value = data.toString().toStdString();
	dataChanged(index, index);

	return true;
}

void ParameterModel::update(const config_server::ParameterValueList& list)
{
	// Sort the update list (should be no-op)
	std::vector<config_server::ParameterValue> sorted = list.parameters;
	std::sort(sorted.begin(), sorted.end(), [](const config_server::ParameterValue& a, const config_server::ParameterValue& b) {
		return a.name < b.name;
	});

	// left: current in model
	// right: msg

	std::size_t leftIdx = 0;
	std::size_t rightIdx = 0;

	while(leftIdx < m_linearizedNodeList.size() && rightIdx < sorted.size())
	{
		auto& left = m_linearizedNodeList[leftIdx];
		const auto& right = sorted[rightIdx];

// 		printf("left: %30s, right: %30s\n", left->path.c_str(), right.name.c_str());

		if(left->path == right.name)
		{
			// Value update
			left->value = right.value;
			left->pValue = right;
			left->pending = false;
			if(left->widget)
				left->widget->setValue(right);

			QModelIndex idx = indexForNode(left, COL_VALUE);
			dataChanged(idx, idx);

			leftIdx++;
			rightIdx++;
		}
		else if(right.name > left->path)
		{
			// left has to catch up -> we need to remove this index
			printf("Deleting entry '%s'\n", left->path.c_str());

			auto parent = left->parent;
			QModelIndex idx = indexForNode(parent);
			beginRemoveRows(idx, left->row(), left->row());

			auto it = std::find(parent->children.begin(), parent->children.end(), left);

			if(it == parent->children.end())
				throw std::runtime_error("Invalid children list");

			parent->children.erase(it);

			m_linearizedNodeList.erase(m_linearizedNodeList.begin() + leftIdx);

			endRemoveRows();

			// TODO: We actually need to remove in-between nodes as well
		}
		else if(right.name < left->path)
		{
			// left is ahead -> we need to add this index

			insertNode(right.name, right);

			leftIdx++;
			rightIdx++;
		}
	}

	// Handle leftovers in both lists
	while(leftIdx < m_linearizedNodeList.size())
	{
		// TODO: It might be more efficient to handle this list from the end.
		auto left = m_linearizedNodeList[leftIdx];

		auto parent = left->parent;
		QModelIndex idx = indexForNode(parent);
		beginRemoveRows(idx, left->row(), left->row());

		auto it = std::find(parent->children.begin(), parent->children.end(), left);
		parent->children.erase(it);

		m_linearizedNodeList.erase(m_linearizedNodeList.begin() + leftIdx);

		endRemoveRows();
	}

	for(; rightIdx < sorted.size(); ++rightIdx)
	{
		const auto& right = sorted[rightIdx];
		insertNode(right.name, right);
	}

// 	for(ParameterNode* n : m_linearizedNodeList)
// 	{
// 		printf("node: %s\n", n->path.c_str());
// 	}
}

void ParameterModel::insertNode(const std::string& name, const config_server::ParameterValue& value)
{
	// We look for the name mentioned in the right index.
	boost::char_separator<char> pathSep("/");
	boost::tokenizer<boost::char_separator<char>> tokens(name, pathSep);
	std::vector<std::string> path;
	std::copy(tokens.begin(), tokens.end(), std::back_inserter(path));

	std::size_t knownPath;
	auto node = &m_rootNode;

	for(knownPath = 0; knownPath < path.size(); ++knownPath)
	{
		auto& segment = path[knownPath];
		auto it = std::find_if(node->children.begin(), node->children.end(),
			[&](const ParameterNode* n) { return n->name == segment; }
		);

		if(it == node->children.end())
			break;
		else
			node = *it;
	}

	auto listComp = [](const ParameterNode* a, const ParameterNode* b) {
		return a->path < b->path;
	};

	for(std::size_t i = knownPath; i < path.size(); ++i)
	{
		auto& segment = path[i];

		ParameterNode* child = new ParameterNode(node);
		child->name = segment;
		child->path = node->path + "/" + segment;

		auto it = std::lower_bound(node->children.begin(), node->children.end(), child, listComp);

		QModelIndex idx = indexForNode(node);
		beginInsertRows(idx, it - node->children.begin(), it - node->children.begin());

		node->children.insert(it, child);

		endInsertRows();

		node = child;
	}

	node->value = value.value;
	node->pValue = value;

	node->widget = widgetForNode(node);
	if(m_view && node->widget)
		m_view->setIndexWidget(indexForNode(node, COL_VALUE), node->widget);

	auto linIt = std::lower_bound(m_linearizedNodeList.begin(), m_linearizedNodeList.end(), node, listComp);
	if(linIt == m_linearizedNodeList.end() || (*linIt)->path != node->path)
		m_linearizedNodeList.insert(linIt, node);
}

config_server::ParameterValue ParameterModel::value(const QModelIndex& index) const
{
	const ParameterNode* node = nodeFromIndex(index);
	if(!node)
		return config_server::ParameterValue();

	return node->pValue;
}

ParameterWidgetBase* ParameterModel::widgetForNode(const ParameterNode* node) const
{
	ParameterWidgetBase* w = 0;

	if(node->pValue.type == "float")
		w = new FloatParameterWidget;
	else if(node->pValue.type == "int")
		w = new IntParameterWidget;
	else if(node->pValue.type == "string")
		w = new StringParameterWidget;
	else if(node->pValue.type == "bool")
		w = new BoolParameterWidget;

	if(w)
	{
		w->setValue(node->pValue);
		connect(w, SIGNAL(setRequested(QString,QString)),
			SIGNAL(setRequested(QString,QString))
		);
	}

	return w;
}

void ParameterModel::initializeView(QAbstractItemView* view)
{
	m_view = view;

	for(auto& node : m_linearizedNodeList)
	{
		node->widget = widgetForNode(node);
		if(node->widget)
			view->setIndexWidget(indexForNode(node, COL_VALUE), node->widget);
	}
}


}
