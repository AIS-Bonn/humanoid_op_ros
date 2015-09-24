// Displays a combo box
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "combobox_delegate.h"

#include <QtGui/QComboBox>

Q_DECLARE_METATYPE(QModelIndex)

ComboBoxDelegate::ComboBoxDelegate(QObject* parent)
 : QStyledItemDelegate(parent)
{
}

ComboBoxDelegate::~ComboBoxDelegate()
{
}

QWidget* ComboBoxDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	// Create the combobox and populate it
	QComboBox *cb = new QComboBox(parent);
	cb->setProperty("combobox-delegate-index", QVariant::fromValue(index));

	connect(cb, SIGNAL(currentIndexChanged(int)), SLOT(updateData()));

	cb->insertItems(0, index.data(Qt::UserRole).toStringList());
	return cb;
}

void ComboBoxDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
	QComboBox* cb = qobject_cast<QComboBox*>(editor);

	cb->setCurrentIndex(index.data(Qt::EditRole).toInt());
}

void ComboBoxDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
	QComboBox* cb = qobject_cast<QComboBox*>(editor);

	model->setData(index, cb->currentIndex());
}

void ComboBoxDelegate::updateData()
{
	QComboBox* comboBox = qobject_cast<QComboBox*>(sender());

	QModelIndex index = comboBox->property("combobox-delegate-index").value<QModelIndex>();

	const_cast<QAbstractItemModel*>(index.model())->setData(index, comboBox->currentIndex());
}
