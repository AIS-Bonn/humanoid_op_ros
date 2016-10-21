// A class that maintains an undo/redo history for a particular variable
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//         Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef HISTORYKEEPER_H
#define HISTORYKEEPER_H

#include <QObject>
#include <QVariant>
#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QKeyEvent>
#include <QTreeWidgetItem>

namespace parametertuner
{
	class HistoryKeeper : public QObject
	{
	Q_OBJECT
	public:
		explicit HistoryKeeper(int historySize, double waitTime = 1.0, double emphasisTime = 3.0);
		virtual ~HistoryKeeper();

		void reset();
		void clearHistory();

		void addNewValue(QVariant value);

		bool undo(); // Emits valueChanged if an undo operation was performed
		bool redo(); // Emits valueChanged if a redo operation was performed

		void clearFilterWidgets();
		void addFilterWidget(QWidget* widget);

		void clearTreeItem() { if(m_treeItem) m_treeItem->treeWidget()->removeEventFilter(this); m_treeItem = NULL; }
		void setTreeItem(QTreeWidgetItem* treeItem) { m_treeItem = treeItem; if(m_treeItem) m_treeItem->treeWidget()->installEventFilter(this); }
		QTreeWidgetItem* getTreeItem() const { return m_treeItem; }

		virtual bool eventFilter(QObject *object, QEvent *event);

	Q_SIGNALS:
		void valueChanged(QVariant);

	private Q_SLOTS:
		void handlePendingTimerTimeout();
		void handleWidgetTimerTimeout();

	private:
		void performAdd(QVariant value);

		bool canUndo() const;
		bool canRedo() const;
		QVariant performUndo();
		QVariant performRedo();

		void setBackground(const QColor& color);
		void clearBackground();

		int m_historySize;

		QList<QVariant> m_history;
		QList<QVariant>::iterator m_current;

		QVariant m_pending;
		QTimer m_pendingTimer;

		QTimer m_widgetTimer;
		QVector<QWidget*> m_filterWidgets;
		QTreeWidgetItem* m_treeItem;
	};
}

#endif // HISTORYKEEPER_H
// EOF