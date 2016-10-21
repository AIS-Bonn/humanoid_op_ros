//rqt plugin for tuning parameters
//Author: Sebastian Schüller

#ifndef PARAMETERTUNER_H
#define PARAMETERTUNER_H

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <sensor_msgs/Joy.h>
#include <ui_parametertuner.h>

#include <config_server/ParameterList.h>
#include <QMutex>

#endif

namespace parametertuner
{
	
struct SavedTreeNode
{
	QString name;
	bool selected;
};


class Parametertuner : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	Parametertuner();
	virtual ~Parametertuner();

	void handleList(const config_server::ParameterListConstPtr& list);
	void handleJoystickInput(const sensor_msgs::JoyConstPtr& joy);

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();

Q_SIGNALS:
	void updateRequested();
	void moveSelectionRequested(int);
	void valueChangeRequested(int);
	void expansionRequested(int);

public slots:
	void save();
	void reset();
	void update();
	
	void handleJoystickButton();
	void handleSearchClicked();
	void handleEnterPressed();
	
	QTreeWidgetItem* itemGetParent(const QTreeWidgetItem* item);
	bool itemHasParent(const QTreeWidgetItem* item);
	void moveSelection(int dir);
	void changeValue(int dir);
	void handleExpansion(int dir);
	
private slots:
	void customButtonClicked();

private:
	void load(std::string path);
	void handleResetRightClick();
	
	void expandCollapseParents(QTreeWidgetItem *item, bool expand); // Recursively expands/collapses parents of 'item'
	
	void saveExpanded_and_Selected(QTreeWidgetItem *item); // Recursively saves all expanded children
	void restoreExpanded_and_Selected(); // Expands all previously saved (as expanded) items
	
	// Finds all items with 'text', expands the tree such that all items are visible and selected
	void search(QString text); 
	void unsearch(); // Removes all consequences of previous search (collapses and deselects all found items)
	
	QList<QVector<SavedTreeNode> > m_branches;
	void saveBranches();
	void restoreBranches();
	
	void getExpandedItems(QTreeWidgetItem* item, QList<QTreeWidgetItem*> &list);
	void saveBranch(QTreeWidgetItem* item, QVector<SavedTreeNode> &branch);
	bool checkBranch(QTreeWidgetItem* item, const QVector<SavedTreeNode> &branch, int index);
	
	void restoreSelection(QTreeWidgetItem* item, const QVector<SavedTreeNode> &branch, int index);
	
	bool eventFilter(QObject *obj, QEvent *event);
	
	void parseCustomCommand();
	
	QList<QTreeWidgetItem*> m_items_selected;
	QList<QTreeWidgetItem*> m_items_expanded;
	
	QList<QTreeWidgetItem*> m_items_found;
	bool isSearchTurn; // If true, search() is called, otherwise - unsearch()
	
	ros::NodeHandle m_nh;
	QMutex m_mutex;
	ros::Subscriber m_sub_paramList;
	config_server::ParameterListConstPtr m_list;
	
	std::string m_custom_command;

	Ui::parametertuner m_ui;

	ros::Subscriber m_sub_joystick;

	bool m_useJoystick;
	bool m_buttonUp;
	bool m_buttonDown;
	bool m_buttonInc;
	bool m_buttonDec;
	bool m_buttonUpLeft;

	ros::WallTimer m_updateTimer;
	int m_updateCounter;
	void handleUpdateTimer();

	void insertParameter(QString param, QTreeWidgetItem* ancestor, const config_server::ParameterDescription& description);
	QTreeWidgetItem* insertParameterNode(const config_server::ParameterDescription& description, QTreeWidgetItem* ancestor);

	void updateSelection(QTreeWidgetItem* old, QTreeWidgetItem* next);
	QTreeWidgetItem* getSelectedItem();

	void deleteWidget(QTreeWidgetItem* item);

	enum direction
	{
		UP = -1,
		DOWN = 1,
		LEFT = -1,
		RIGHT = 1,
		COLLAPSE = 0
	};
};

}
#endif
