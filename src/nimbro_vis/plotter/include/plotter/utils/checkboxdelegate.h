// QCheckBox delegate taken from
// http://stackoverflow.com/questions/3363190/qt-qtableview-how-to-have-a-checkbox-only-column

#include <QStyledItemDelegate>

class CheckBoxDelegate : public QStyledItemDelegate
{
Q_OBJECT
public:
	explicit CheckBoxDelegate(QObject* parent = 0);
	virtual ~CheckBoxDelegate();

	virtual QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const;
	virtual void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const;
	virtual bool editorEvent(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index);
};
