// Displays details about a single message
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MESSAGE_DISPLAY_H
#define MESSAGE_DISPLAY_H

#include <QtWebKit/QWebView>

#include <rosgraph_msgs/Log.h>

class QToolButton;

namespace rqt_log_viewer
{

class MessageDisplay : public QWebView
{
Q_OBJECT
public:
	explicit MessageDisplay(QWidget* parent = 0);
	virtual ~MessageDisplay();
Q_SIGNALS:
	void closeRequested();
public Q_SLOTS:
	void displayMessage(const rosgraph_msgs::Log& msg);
protected:
	virtual void resizeEvent(QResizeEvent* event);
private:
	QToolButton* m_closeButton;
};

}

#endif
