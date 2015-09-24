// Y histogram display widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef HISTWIDGET_H
#define HISTWIDGET_H

#include <QtGui/QWidget>

class CalibScene;
class CalibEllipse;
class Histogram;

class HistWidget : public QWidget
{
Q_OBJECT
public:
	explicit HistWidget(CalibScene* scene, Histogram* hist, QWidget* parent = 0);
	virtual ~HistWidget();

	virtual void paintEvent(QPaintEvent*);
	virtual void mouseMoveEvent(QMouseEvent*);
	virtual void mousePressEvent(QMouseEvent*);
	virtual void mouseReleaseEvent(QMouseEvent*);
signals:
	void selectionChanged();
private:
	CalibScene* m_scene;
	Histogram* m_histogram;
	QPixmap m_y_ruler;

	enum {
		GR_None,
		GR_Min,
		GR_Max
	} m_grabState;

	int m_colorID;

	int m_x_minY;
	int m_x_maxY;
};

#endif
