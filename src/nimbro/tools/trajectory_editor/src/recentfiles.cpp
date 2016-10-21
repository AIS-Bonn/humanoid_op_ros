// Implementation of a "recent files" feature
// Stores "recent files" in the file on disk
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor/recentfiles.h>

#include <QFile>
#include <QTextStream>

#include <ros/package.h>
#include <ros/console.h>

RecentFiles::RecentFiles(QMenu *menu) : QObject()
{
	m_menu = menu;
	m_max_files = 5;
	
	load();
	
	connect(m_menu, SIGNAL(triggered(QAction*)), this, SLOT(onMenuTriggered(QAction*)));
	
	ROS_INFO("initialized successfully");
}

void RecentFiles::addRecentFile(QString path)
{
	// Check if file is alredy in the list
	int id = -1;
	
	for(int i = 0; i < m_menu->actions().size(); i++)
	{
		if(m_menu->actions().at(i)->text() == path)
		{
			id = i;
			break;
		}
	}
	
	if(id >= 0) // Remove "old" record
		m_menu->removeAction(m_menu->actions().at(id));
	
	// Add action at the first position
	QAction *action = new QAction(path, m_menu);
	
	if(m_menu->actions().size() == 0)
		m_menu->insertAction(0, action);
	else
		m_menu->insertAction(m_menu->actions().at(0), action);
	
	// Check if size exceeded maximum. If yes - remove the oldest
	if(m_menu->actions().size() > m_max_files)
		m_menu->removeAction(m_menu->actions().last());
}

void RecentFiles::onMenuTriggered(QAction* action)
{
	requestOpen(action->text());
}

// Load recent files from file on disk
void RecentFiles::load()
{
	QString path = QString::fromStdString(ros::package::getPath("launch") + "/motions/perspectives/recent_files");
	QFile file(path);
	
	if(!file.open(QIODevice::ReadOnly)) 
		return;

	QTextStream in(&file);

	while(!in.atEnd()) 
	{
		QString filePath = in.readLine();    
		
		if(filePath.isEmpty() == false)
			m_menu->addAction(filePath);
	}
	
	file.close();
}

// Saves (probably)updated list of recent files into file on disk
void RecentFiles::save()
{
	QString path = QString::fromStdString(ros::package::getPath("launch") + "/motions/perspectives/recent_files");
	QFile file(path);
	
	if(!file.open(QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text)) 
		return;

	QTextStream out(&file);
	
	for(int i = 0; i < m_menu->actions().size(); i++)
		out << m_menu->actions().at(i)->text() << endl;
	
	file.close();
}

RecentFiles::~RecentFiles()
{
	save();
}