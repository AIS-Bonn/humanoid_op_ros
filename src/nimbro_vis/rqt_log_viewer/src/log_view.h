// Customized Qt view for log entries
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LOG_VIEW_H
#define LOG_VIEW_H

#include <QtGui/QAbstractItemView>
#include <QtCore/QTimer>

namespace rqt_log_viewer
{

class LogView : public QAbstractItemView
{
Q_OBJECT
public:
	explicit LogView(QWidget* parent = 0);
	virtual ~LogView();

	virtual int horizontalOffset() const;
	virtual int verticalOffset() const;

	virtual QModelIndex indexAt(const QPoint& point) const;
	virtual bool isIndexHidden(const QModelIndex& index) const;

	virtual QModelIndex moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers);
	virtual void scrollTo(const QModelIndex& index, ScrollHint hint = EnsureVisible);

	virtual QRect visualRect(const QModelIndex& index) const;

	virtual void setSelection(const QRect& rect, QItemSelectionModel::SelectionFlags command);
	virtual QRegion visualRegionForSelection(const QItemSelection& selection) const;

	virtual void rowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
	virtual void rowsInserted(const QModelIndex& parent, int start, int end);
	virtual void reset();
	virtual void dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight);
public Q_SLOTS:
	void deselect();
	void setDisplayAbsoluteTimeStamps(bool on);
protected:
	virtual void paintEvent(QPaintEvent*);
	virtual bool viewportEvent(QEvent* event);

	virtual void enterEvent(QEvent*);
	virtual void mouseMoveEvent(QMouseEvent* event);
	virtual void mousePressEvent(QMouseEvent* event);
	virtual void mouseDoubleClickEvent(QMouseEvent* event);
	virtual void mouseReleaseEvent(QMouseEvent* event);
	virtual void leaveEvent(QEvent*);

	virtual void keyPressEvent(QKeyEvent* event);
private:
	void calculateMessageHeights();
	int findHeightIndex(int* ypos);
	void selectMessage(int idx);
	int numLinesForText(const QString& text);

	bool m_lastItemVisible;

	bool m_sticky;
	int m_lastStickyScrollValue;

	int m_textWidth;
	int m_lines;
	QList<int> m_messageLines;

	QList<int> m_shownMessages;
	QList<int> m_shownMessageHeights;

	std::string m_highlightedNode;

	bool m_displayAbsoluteStamps;

	int m_lineSpacing;
};

}

#endif
