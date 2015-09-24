// Support for KDevelop sessions
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "kdevelop.h"

#include <QtCore/QDir>
#include <QtCore/QSettings>
#include <QtGui/QAction>
#include <QtGui/QMenu>
#include <QMessageBox>

#include <QtDBus/QDBusInterface>
#include <QtDBus/QDBusReply>

namespace rqt_log_viewer
{
namespace editors
{

KDevelop::KDevelop()
{
}

KDevelop::~KDevelop()
{
}

void KDevelop::createAction(QMenu* menu, QAction* action, const QDir& sessionsDir, const QString& file, int line)
{
	Q_FOREACH(const QString& entry, sessionsDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot))
	{
		QDir sessionDir(sessionsDir.absoluteFilePath(entry));

		if(!sessionDir.exists())
			continue;

		QFile lock(sessionDir.absoluteFilePath("lock"));
		if(!lock.exists())
			continue;

		if(!lock.open(QIODevice::ReadOnly))
			continue;

		QString pidStr = lock.readLine();
		pidStr = pidStr.trimmed();
		bool ok;
		int pid = pidStr.toInt(&ok);

		if(!ok)
			continue;

		fprintf(stderr, "Found running KDevelop instance at PID %d\n", pid);
		QSettings sessionrc(sessionDir.absoluteFilePath("sessionrc"), QSettings::IniFormat);

		QString sessionName = QString::number(pid);
		if(sessionrc.contains("SessionName"))
		{
			fprintf(stderr, "has sessionname\n");
			sessionName = sessionrc.value("SessionName").toString();
		}
		else if(sessionrc.contains("SessionPrettyContents"))
		{
			QVariant value = sessionrc.value("SessionPrettyContents");

			if(value.type() == QVariant::StringList)
				sessionName = value.toStringList().join(", ");
			else
				sessionName = value.toString();
			fprintf(stderr, "has pretty contents of type %d\n", sessionrc.value("SessionPrettyContents").type());
		}

		QAction* sessionAction = new QAction(
			QIcon::fromTheme("kdevelop"),
			QString("Open with KDevelop: %1").arg(sessionName),
			action
		);
		sessionAction->setProperty("kdevelop-pid", pid);
		sessionAction->setProperty("kdevelop-file", file);
		sessionAction->setProperty("kdevelop-line", line);
		connect(sessionAction, SIGNAL(triggered(bool)), SLOT(openFileInKDevelop()));
		menu->addAction(sessionAction);
	}
}

QAction* KDevelop::createAction(const QString& file, int line)
{
	// Try to find running KDevelop sessions
	QAction* action = new QAction(QIcon::fromTheme("kdevelop"), "KDevelop", 0);
	QMenu* menu = new QMenu;

	createAction(menu, action, QDir(QDir::homePath() + + "/.kde/share/apps/kdevelop/sessions"), file, line);
	createAction(menu, action, QDir(QDir::homePath() + + "/.local5/share/kdevelop/sessions"), file, line);

	if(menu->actions().count() == 0)
	{
		delete menu;
		delete action;
		return 0;
	}

	if(menu->actions().count() == 1)
	{
		QAction* sessionAction = menu->actions()[0];
		sessionAction->setParent(0);
		delete menu;
		delete action;
		return sessionAction;
	}

	action->setMenu(menu);
	return action;
}

void KDevelop::openFileInKDevelop()
{
	QAction* action = qobject_cast<QAction*>(sender());

	int pid = action->property("kdevelop-pid").toInt();
	QString file = action->property("kdevelop-file").toString();
	int line = action->property("kdevelop-line").toInt();

	QString service = QString("org.kdevelop.kdevelop-%1").arg(pid);

	// Try the new dbus call first
	QDBusInterface iface(
		service,
		"/org/kdevelop/DocumentController",
		"org.kdevelop.DocumentController"
	);

	QDBusReply<bool> ret = iface.call("openDocumentSimple", file, line-1);
	if(!ret.isValid())
	{
		ret = iface.call("openDocumentSimple", file);
		if(!ret.isValid())
		{
			QMessageBox::critical(0, "Error", "Could not call KDevelop via D-Bus.");
			return;
		}
	}

	QDBusInterface mwindowIface(
		service,
		"/kdevelop/MainWindow",
		"org.kdevelop.MainWindow"
	);

	mwindowIface.call("ensureVisible");
}

}
}
