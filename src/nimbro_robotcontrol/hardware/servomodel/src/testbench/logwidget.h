// Log widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LOGWIDGET_H
#define LOGWIDGET_H

#include <QtGui/QTextEdit>

class LogWidget : public QTextEdit
{
Q_OBJECT
public:
	explicit LogWidget(QWidget* parent = 0);
	virtual ~LogWidget();

public Q_SLOTS:
	void append(const QString& text);
};

#endif
