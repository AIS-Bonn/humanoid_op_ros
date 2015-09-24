// Delegate for rendering & editing parameters
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "parameter_delegate.h"
#include "parameter_model.h"
#include "parameter_item.h"

namespace remote_tuner
{

ParameterDelegate::ParameterDelegate(QObject* parent)
 : QStyledItemDelegate(parent)
{
}

QWidget* ParameterDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	printf("Create editor\n");
	const ParameterModel* model = qobject_cast<const ParameterModel*>(index.model());
	if(!model)
	{
		printf("no model\n");
		return nullptr;
	}

	config_server::ParameterValue value = model->value(index);

	ParameterWidgetBase* w = nullptr;

	if(value.type == "float")
		w = new FloatParameterWidget;

	if(!w)
	{
		printf("Unknown value type '%s'\n", value.type.c_str());
		return nullptr;
	}

	w->setParent(parent);
	w->update(value);

	return w;
}

QSize ParameterDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	return QSize(0, 20);
}

}
