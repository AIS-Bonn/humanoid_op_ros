// Log widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "logwidget.h"

LogWidget::LogWidget(QWidget* parent) : QTextEdit(parent)
{
}

LogWidget::~LogWidget()
{
}

void LogWidget::append(const QString& new_text)
{
	insertPlainText(new_text + "\n");
}
