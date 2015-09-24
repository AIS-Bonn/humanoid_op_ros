// YUV color classification calibration widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "calibwidget.h"

#include "calibscene.h"
#include <QtGui/QGraphicsView>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QAction>

#include <QtCore/QTimer>

#include "calibview.h"

CalibWidget::CalibWidget(CalibScene* scene, QWidget* parent)
 : QWidget(parent)
 , m_scene(scene)
{
	m_view = new CalibView(this);
	m_toolBar = new QToolBar(this);

	m_view->setScene(m_scene);
	m_view->setRenderHint(QPainter::Antialiasing, 0);

	QLayout* layout = new QVBoxLayout(this);
	layout->addWidget(m_toolBar);
	layout->addWidget(m_view);

	m_act_fitInView = new QAction("Fit in view", this);
	m_act_fitInView->setCheckable(true);
	m_act_fitInView->setChecked(true);
	m_toolBar->addAction(m_act_fitInView);
	connect(m_act_fitInView, SIGNAL(changed()), SLOT(fitInView()));

	connect(m_view, SIGNAL(zoomed()), SLOT(disableFitInView()));

	setMinimumWidth(400);

	QTimer::singleShot(300, this, SLOT(fitInView()));
}

CalibWidget::~CalibWidget()
{
}

void CalibWidget::resizeEvent(QResizeEvent*)
{
	fitInView();
}

void CalibWidget::fitInView()
{
	if(m_act_fitInView->isChecked())
		m_view->fitInView(m_scene->sceneRect(), Qt::KeepAspectRatio);
}

void CalibWidget::disableFitInView()
{
	m_act_fitInView->setChecked(false);
}




