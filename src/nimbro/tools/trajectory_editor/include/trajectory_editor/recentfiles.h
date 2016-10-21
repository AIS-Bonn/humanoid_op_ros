// Implementation of a "recent files" feature
// Stores "recent files" in the file on disk
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef RECENTFILES_H
#define RECENTFILES_H

#include <QMenu>
#include <QAction>

#include <vector>

class RecentFiles : public QObject
{
Q_OBJECT
public:
	RecentFiles(QMenu *menu);
	~RecentFiles();
	
	void addRecentFile(QString path);
	
Q_SIGNALS:
	void requestOpen(QString path);
	
private Q_SLOTS:
	void onMenuTriggered(QAction *action);
	
private:
	void load();
	void save();
	
private:
	int m_max_files;
	QMenu *m_menu;
};

#endif // RECENTFILES_H