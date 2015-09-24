/****************************************************************************
**
** Copyright (c) 2007 Trolltech ASA <info@trolltech.com>
**
** Use, modification and distribution is allowed without limitation,
** warranty, liability or support of any kind.
**
****************************************************************************/

#ifndef LINEEDIT_H
#define LINEEDIT_H

#include <QLineEdit>

class QToolButton;

namespace rqt_log_viewer
{

class FilterLineEdit : public QLineEdit
{
Q_OBJECT
public:
	FilterLineEdit(QWidget *parent = 0);

protected:
	void resizeEvent(QResizeEvent *);

private Q_SLOTS:
	void updateCloseButton(const QString &text);

private:
	QToolButton *clearButton;
	QPalette m_activePalette;
	QPalette m_palette;
};

}

#endif // LIENEDIT_H

