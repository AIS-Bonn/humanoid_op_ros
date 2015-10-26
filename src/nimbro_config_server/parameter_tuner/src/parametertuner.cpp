//rqt plugin for tuning parameters
//Author: Sebastian Schüller

#include <pluginlib/class_list_macros.h>
#include <boost/iterator/iterator_concepts.hpp>

#include <config_server/Save.h>
#include <config_server/Load.h>
#include <config_server/parameterclient.h>

#include <ros/this_node.h>

#include "parametertuner.h"
#include "parameteritem.h"


namespace parametertuner
{

Parametertuner::Parametertuner()
 : m_useJoystick(false)
 , m_updateCounter(0)
{}

Parametertuner::~Parametertuner()
{
}


void Parametertuner::initPlugin(qt_gui_cpp::PluginContext& context)
{
	// Initialize config_server library with the correct node handle.
	// We cannot use getPrivateNodeHandle() here, as it returns something like
	// /parametertuner/Parametertuner_1 as namespace, which is not unique -.-
	// Instead, use the global node handle with the ROS node name, which is
	// unique up to the rqt process (which is what we want).
	m_nh = ros::NodeHandle(getNodeHandle(), ros::this_node::getName());
	config_server::ParameterClient::initialize(m_nh);

	QWidget* w = new QWidget();

	m_ui.setupUi(w);
	
	qRegisterMetaType<std::string>("std::string");

	connect(m_ui.joystick_button, SIGNAL(toggled(bool)), this, SLOT(handleJoystickButton()));
	connect(m_ui.reset_button, SIGNAL(clicked(bool)), this, SLOT(reset()));
	connect(m_ui.save_button, SIGNAL(clicked(bool)), this, SLOT(save()));

	connect(this, SIGNAL(updateRequested()), this, SLOT(update()), Qt::QueuedConnection);
	m_sub_paramList = getNodeHandle().subscribe("/config_server/parameter_list", 1, &Parametertuner::handleList, this);
	m_sub_joystick = getNodeHandle().subscribe("/joy", 1, &Parametertuner::handleJoystickInput, this);

	connect(this, SIGNAL(moveSelectionRequested(int)), this, SLOT(moveSelection(int)), Qt::QueuedConnection);
	connect(this, SIGNAL(valueChangeRequested(int)), this, SLOT(changeValue(int)), Qt::QueuedConnection);
	connect(this, SIGNAL(expansionRequested(int)), this, SLOT(handleExpansion(int)), Qt::QueuedConnection);

	m_ui.parameter_root_widget->setColumnCount(2);
	m_ui.parameter_root_widget->setColumnWidth(0, 260);

	context.addWidget(w);
	
	m_updateCounter = 0;
	m_updateTimer = m_nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&Parametertuner::handleUpdateTimer, this), true, false);
}

void Parametertuner::shutdownPlugin()
{
	m_sub_joystick.shutdown();
	m_sub_paramList.shutdown();
}

void Parametertuner::handleList(const config_server::ParameterListConstPtr& list)
{
	QMutexLocker locker(&m_mutex);
	m_list = list;
	m_updateCounter++;
	if(m_updateCounter >= 20)
	{
		emit updateRequested();
		m_updateCounter = 0;
	}
	else
	{
		m_updateTimer.stop();
		m_updateTimer.start();
	}
}

void Parametertuner::handleUpdateTimer()
{
	m_updateTimer.stop(); // Required!
	emit updateRequested();
}

void Parametertuner::deleteWidget(QTreeWidgetItem* item)
{
	QWidget* w = m_ui.parameter_root_widget->itemWidget(item, 1);
	m_ui.parameter_root_widget->removeItemWidget(item, 1);

	if(w)
		delete w;

	for(int i = 0; i < item->childCount(); ++i)
		deleteWidget(item->child(i));
}

void Parametertuner::update()
{
	deleteWidget(m_ui.parameter_root_widget->invisibleRootItem());
	m_ui.parameter_root_widget->clear();

	config_server::ParameterClient* client = config_server::ParameterClient::instance();

	client->cork();

	QMutexLocker locker (&m_mutex);
	for(size_t i = 0; i < m_list->parameters.size(); i++)
	{
		QString param = QString::fromStdString(m_list->parameters[i].name);
		insertParameter(param, 0, m_list->parameters[i]);
	}

	client->uncork();
}

void Parametertuner::insertParameter(QString param, QTreeWidgetItem* ancestor, const config_server::ParameterDescription& description)
{
	if (param.isEmpty())
		return;

	QString levelParam, strippedParam;
	bool isLeaf;
	levelParam = param.section("/", 1, 1);
	strippedParam = param.section("/", 2);
	isLeaf = strippedParam.isEmpty();
	if (!isLeaf)
		strippedParam = "/" + strippedParam;


	QTreeWidgetItem* nextItem = 0;
	if (!ancestor)
	{
		QList<QTreeWidgetItem *> topLevelItems;
		topLevelItems = m_ui.parameter_root_widget->findItems(levelParam, 0);
		if (topLevelItems.isEmpty())
		{
			QTreeWidgetItem* newItem = 0;
			newItem = new QTreeWidgetItem();
			newItem->setText(0, levelParam);
			m_ui.parameter_root_widget->addTopLevelItem(newItem);
			nextItem  = newItem;
		}
		else
		{
			nextItem = topLevelItems.at(0);
		}
		insertParameter(strippedParam, nextItem, description);
		return;
	}

	int childCount = ancestor->childCount();
	int i;
	for (i = 0; i < childCount; i++)
	{
		if (levelParam == ancestor->child(i)->text(0))
		{
			insertParameter(strippedParam, ancestor->child(i), description);
			return;
		}
	}

	if (isLeaf)
		nextItem = insertParameterNode(description, ancestor);
	else
		nextItem = new QTreeWidgetItem(ancestor);

	nextItem->setText(0, levelParam);
	insertParameter(strippedParam, nextItem, description);
	return;

}

QTreeWidgetItem* Parametertuner::insertParameterNode(const config_server::ParameterDescription& description, QTreeWidgetItem* ancestor)
{
	QTreeWidgetItem* newItem = new QTreeWidgetItem(ancestor);

	QWidget* newWidget = 0;
	QString type = QString::fromStdString(description.type);
	if (type == "int")
		newWidget = new IntParameterWidget(getNodeHandle(), description);
	else if (type == "string")
		newWidget = new StringParameterWidget(getNodeHandle(), description);
	else if (type == "float")
		newWidget = new FloatParameterWidget(getNodeHandle(), description);
	else if (type == "bool")
		newWidget = new BoolParameterWidget(getNodeHandle(), description);

	if (newWidget != 0)
		m_ui.parameter_root_widget->setItemWidget(newItem, 1, newWidget);

	return newItem;
}

void Parametertuner::save()
{
	config_server::Save srv;

	if (!ros::service::call("/config_server/save",srv))
	{
		m_ui.save_button->setText("Saving failed");
		ROS_WARN("Could not call config_server service 'save'!");
		return;
	}
	m_ui.save_button->setText("Save Config");
}

void Parametertuner::reset()
{
	config_server::Load srv;
	srv.request.filename = "";

	if(!ros::service::call("/config_server/load", srv))
	{
		m_ui.reset_button->setText("Reset Failed");
		ROS_WARN("Could not call config_server service 'load'!");
	}
	m_ui.reset_button->setText("Reset Config");
}

void Parametertuner::handleJoystickButton()
{
	m_useJoystick = m_ui.joystick_button->isChecked();
}

void Parametertuner::handleJoystickInput(const sensor_msgs::JoyConstPtr& joy)
{
	if(!m_useJoystick)
		return;

	if(joy->buttons.size() < 8)
		return;

	int buttonUp = 4;
	int buttonDown = 5;
	int buttonLeft = 6;
	int buttonRight = 7;

	bool collapse = false;
	bool pressedUpLeft = joy->buttons[buttonUp] && joy->buttons[buttonLeft];
	if(m_buttonUpLeft && !pressedUpLeft)
	{
		expansionRequested(COLLAPSE);
		collapse = true;
	}
	m_buttonUpLeft = pressedUpLeft;

	if(m_buttonUp && !joy->buttons[buttonUp] && !collapse)
		moveSelectionRequested(UP);
	m_buttonUp = joy->buttons[buttonUp];

	if(m_buttonDown && !joy->buttons[buttonDown])
		moveSelectionRequested(DOWN);
	m_buttonDown = joy->buttons[buttonDown];

	if(m_buttonDec && !joy->buttons[buttonLeft] && !collapse)
		valueChangeRequested(LEFT);
	m_buttonDec = joy->buttons[buttonLeft];

	if(m_buttonInc && !joy->buttons[buttonRight])
		valueChangeRequested(RIGHT);
	m_buttonInc = joy->buttons[buttonRight];

}

// Get the parent of a tree item, including if it is a top level or root item already
QTreeWidgetItem* Parametertuner::itemGetParent(const QTreeWidgetItem* item)
{
	QTreeWidgetItem* parentItem = item->parent();
	if(!parentItem)
		parentItem = m_ui.parameter_root_widget->invisibleRootItem();
	return parentItem;
}

// Get whether an item has a parent tree item (i.e. is not top level or root)
bool Parametertuner::itemHasParent(const QTreeWidgetItem* item)
{
	// If an item has no parent then NULL is returned, evaluating to a return value of false...
	return item->parent();
}

// Move the selection within the parameter tuner tree widget
void Parametertuner::moveSelection(int dir)
{
	// Error checking
	if(dir != UP && dir != DOWN) return;

	// Declare variables
	QTreeWidgetItem* item;
	QTreeWidgetItem* tmpItem;
	QTreeWidgetItem* thisItem;
	QTreeWidgetItem* parentItem;
	int itemIndex;

	// Get the currently selected item
	thisItem = getSelectedItem();

	// If no item is selected then select the first top level item
	if(!thisItem)
	{
		if(dir == DOWN)
			updateSelection(NULL, m_ui.parameter_root_widget->topLevelItem(0));
		else
			updateSelection(NULL, m_ui.parameter_root_widget->topLevelItem(m_ui.parameter_root_widget->topLevelItemCount() - 1));
		return;
	}

	// Handle UP case
	if(dir == UP)
	{
		// Go to the previous item in the tree
		parentItem = itemGetParent(thisItem);
		itemIndex = parentItem->indexOfChild(thisItem);
		tmpItem = parentItem->child(itemIndex + UP);
		if(tmpItem)
		{
			item = tmpItem;
			while(item->childCount() > 0 && item->isExpanded())
				item = item->child(item->childCount() - 1);
			updateSelection(thisItem, item);
			return;
		}
		else if(itemHasParent(thisItem))
		{
			updateSelection(thisItem, parentItem);
			return;
		}
		else return; // <-- This is the case that we are at the very top-most element
	}

	// Handle DOWN case
	if(dir == DOWN)
	{
		// If the item has children and is expanded then go to the first child
		if(thisItem->childCount() > 0 && thisItem->isExpanded())
		{
			updateSelection(thisItem, thisItem->child(0));
			return;
		}

		// Go to the next item in the tree
		item = thisItem;
		while(true)
		{
			parentItem = itemGetParent(item);
			itemIndex = parentItem->indexOfChild(item);
			tmpItem = parentItem->child(itemIndex + DOWN);
			if(tmpItem)
			{
				updateSelection(thisItem, tmpItem);
				return;
			}
			else
			{
				if(itemHasParent(item))
					item = parentItem;
				else return; // <-- This is the case that we are at the very bottom-most element
			}
		}
	}
}

QTreeWidgetItem* Parametertuner::getSelectedItem()
{
	QList<QTreeWidgetItem *> selectList = m_ui.parameter_root_widget->selectedItems();
	if (selectList.size() == 0)
	{
		return 0;
	}
	else
		return selectList.at(0);
}

void Parametertuner::updateSelection(QTreeWidgetItem* old, QTreeWidgetItem* next)
{
	if(old) old->setSelected(false);
	if(next)
	{
		next->setSelected(true);
		m_ui.parameter_root_widget->scrollToItem(next);
	}
}

void Parametertuner::changeValue(int dir)
{
	QTreeWidgetItem* thisItem;
	if (!(thisItem = getSelectedItem()))
		return;

	if (thisItem->childCount() != 0)
	{
		if (dir == RIGHT && (!thisItem->isExpanded()))
			thisItem->setExpanded(true);
		else if (dir == LEFT && thisItem->isExpanded())
			thisItem->setExpanded(false);
		return;
	}
	ParameterWidgetBase* param;
	param = (ParameterWidgetBase *)m_ui.parameter_root_widget->itemWidget(thisItem, 1);
	if (!param)
		return;

	if (dir == LEFT)
		param->DecValue();
	else
		param->IncValue();
}

void Parametertuner::handleExpansion(int dir)
{
	// Error checking
	if(dir != COLLAPSE) return;

	// Declare variables
	QTreeWidgetItem* thisItem;
	QTreeWidgetItem* parentItem;
	
	// Get the currently selected item
	thisItem = getSelectedItem(); // Careful: This may still be null!
	
	// Handle case where collapse has been requested
	if(dir == COLLAPSE)
	{
		// Can't collapse the whole root
		if(!thisItem) return;
		if(!itemHasParent(thisItem)) return;

		// Collapse the parent item of the currently selected item
		parentItem = itemGetParent(thisItem);
		updateSelection(thisItem, parentItem);
		parentItem->setExpanded(false);

		// Done
		return;
	}
}

}

PLUGINLIB_EXPORT_CLASS(parametertuner::Parametertuner, rqt_gui_cpp::Plugin)
// EOF