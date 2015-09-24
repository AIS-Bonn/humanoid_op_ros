// Delegate for color selection
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COLORDELEGATE_H
#define COLORDELEGATE_H

#include <QtGui/QStyledItemDelegate>

class ColorDelegate : public QStyledItemDelegate
{
Q_OBJECT
public:
	explicit ColorDelegate(QObject* parent = 0);
	virtual ~ColorDelegate();

	virtual void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const;
	virtual bool editorEvent(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index);

	virtual QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const;
};

#endif
