// YUV color classification calibration widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CALIBWIDGET_H
#define CALIBWIDGET_H

#include <QtGui/QWidget>

class QToolBar;
class CalibScene;
class CalibView;
class QGraphicsView;

class CalibWidget : public QWidget
{
	Q_OBJECT
public:
	CalibWidget(CalibScene* scene, QWidget* parent = 0);
	virtual ~CalibWidget();

	virtual void resizeEvent(QResizeEvent*);
public slots:
	virtual void fitInView();
	virtual void disableFitInView();
private:
	CalibScene* m_scene;
	CalibView* m_view;
	QToolBar* m_toolBar;

	QAction* m_act_fitInView;
};

#endif
