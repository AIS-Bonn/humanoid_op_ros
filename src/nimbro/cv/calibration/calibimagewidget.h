// Calibration Image widget (handles rect selection)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CALIBIMAGEWIDGET_H
#define CALIBIMAGEWIDGET_H

#include <imagewidget.h>

class CalibImageWidget : public ImageWidget
{
Q_OBJECT
public:
	CalibImageWidget(QWidget* parent = 0);
	virtual ~CalibImageWidget();

	virtual void paintEvent(QPaintEvent*);
	virtual void mousePressEvent(QMouseEvent*);
	virtual void mouseMoveEvent(QMouseEvent*);
	virtual void mouseReleaseEvent(QMouseEvent*);

	inline const QRect& selection() const
	{ return m_selection; }
signals:
	void selectionChanged();
private:
	QRect m_selection;
};

#endif
