// Displays a combo box
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COMBOBOX_DELEGATE_H
#define COMBOBOX_DELEGATE_H

#include <QtGui/QStyledItemDelegate>

class ComboBoxDelegate : public QStyledItemDelegate
{
Q_OBJECT
public:
	explicit ComboBoxDelegate(QObject* parent = 0);
	virtual ~ComboBoxDelegate();

	virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const;
	virtual void setEditorData(QWidget* editor, const QModelIndex& index) const;
	virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const;
private Q_SLOTS:
	void updateData();
};

#endif
