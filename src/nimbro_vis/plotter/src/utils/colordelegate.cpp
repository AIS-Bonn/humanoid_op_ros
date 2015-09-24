// Delegate for color selection
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotter/utils/colordelegate.h"

#include <QtGui/QPainter>
#include <QtGui/QMouseEvent>
#include <QtGui/QColorDialog>

ColorDelegate::ColorDelegate(QObject* parent)
 : QStyledItemDelegate(parent)
{
}

ColorDelegate::~ColorDelegate()
{
}

void ColorDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	QStyledItemDelegate::paint(painter, option, QModelIndex());

	painter->fillRect(option.rect, index.data(Qt::DisplayRole).value<QColor>());
}

bool ColorDelegate::editorEvent(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index)
{
	if(event->type() == QEvent::MouseButtonRelease)
	{
		QMouseEvent* mouse = (QMouseEvent*)event;
		if(mouse->button() == Qt::LeftButton)
		{
			QColor color = QColorDialog::getColor(index.data(Qt::DisplayRole).value<QColor>(), 0);

			return model->setData(index, color);
		}
	}

	return QStyledItemDelegate::editorEvent(event, model, option, index);
}

QSize ColorDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	return QSize(60, 10);
}
