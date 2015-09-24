// Delegate for rendering & editing parameters
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PARAMETER_DELEGATE

#include <QStyledItemDelegate>

namespace remote_tuner
{

class ParameterDelegate : public QStyledItemDelegate
{
public:
	explicit ParameterDelegate(QObject* parent = nullptr);

	virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	virtual QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const override;
};

}

#endif
