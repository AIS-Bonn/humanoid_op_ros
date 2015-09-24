// Displays details about a single message
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "message_display.h"

#include <QtGui/QTextDocument>
#include <QtGui/QApplication>
#include <QtGui/QToolButton>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>

namespace rqt_log_viewer
{

MessageDisplay::MessageDisplay(QWidget* parent)
 : QWebView(parent)
{
	m_closeButton = new QToolButton(this);
	m_closeButton->setIcon(QIcon::fromTheme("dialog-close"));

	connect(m_closeButton, SIGNAL(clicked(bool)), SIGNAL(closeRequested()));
}

MessageDisplay::~MessageDisplay()
{
}

void MessageDisplay::displayMessage(const rosgraph_msgs::Log& msg)
{
	QString html;

	html = "<html><head><style>body { font-family: \"sans-serif\"; font-size: 9pt; }</style>";

	html += "<body>";

	html += "<div style=\"font-size: 11pt\"><b>Message details</b></div>";

	html += "<table style=\"font-size: 9pt\">";

	typedef boost::date_time::c_local_adjustor<boost::posix_time::ptime> local_adj;

	html += "<tr><td><b>Time: </b></td><td>"
		+ QString::number(msg.header.stamp.toSec(), 'f', 3)
		+ " (" + QString::fromStdString(boost::posix_time::to_simple_string(local_adj::utc_to_local(msg.header.stamp.toBoost()))) + ")</td></tr>";

	html += "<tr><td><b>Text: </b></td><td><pre>" + Qt::escape(QString::fromStdString(msg.msg)) + "</pre></td></tr>";

	html += "<tr><td><b>Node: </b></td><td>" + Qt::escape(QString::fromStdString(msg.name)) + "</td></tr>";

	html += "<tr><td><b>Src: </b></td><td>" + Qt::escape(QString::fromStdString(msg.file) + ":" + QString::number(msg.line)) + "</td></tr>";

	html += "</body></html>";

	setHtml(html);

	connect(page(), SIGNAL(scrollRequested(int,int,QRect)), SLOT(update()));
}

void MessageDisplay::resizeEvent(QResizeEvent* event)
{
	QWebView::resizeEvent(event);

	m_closeButton->move(width() - m_closeButton->sizeHint().width() - 20, 3);

	// TODO: Repaint?
}


}
