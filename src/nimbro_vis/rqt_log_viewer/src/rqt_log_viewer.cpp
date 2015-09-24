// RQT plugin
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "rqt_log_viewer.h"

#include <QtGui/QMenu>

#include <ros/node_handle.h>

#include <pluginlib/class_list_macros.h>

#include "ui_rqt_log_viewer.h"

#include "log_view.h"
#include "checkboxdelegate.h"
#include "combobox_delegate.h"

#include "editors/kdevelop.h"

Q_DECLARE_METATYPE(rosgraph_msgs::Log);

namespace rqt_log_viewer
{

RQTLogViewer::RQTLogViewer()
{
	m_ui = new Ui_RQTLogViewer;
}

RQTLogViewer::~RQTLogViewer()
{
	if(m_ui)
		delete m_ui;
}

void RQTLogViewer::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	rqt_gui_cpp::Plugin::initPlugin(ctx);

	m_nodeModel = new NodeModel(this);

	m_model = new LogModel(this);

	qRegisterMetaType<rosgraph_msgs::Log>();
	connect(this, SIGNAL(messageReceived(rosgraph_msgs::Log)),
		m_model, SLOT(addMessage(rosgraph_msgs::Log)),
		Qt::QueuedConnection
	);

	m_filter = new LogFilter(m_nodeModel, this);
	m_filter->setSourceModel(m_model);

	m_sub_log = getPrivateNodeHandle().subscribe("/rosout_agg", 1000,
		&RQTLogViewer::messageReceived, this
	);

	QWidget* w = new QWidget;
	m_ui->setupUi(w);

	m_ui->logView->setModel(m_filter);
	m_ui->logView->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(m_ui->logView, SIGNAL(customContextMenuRequested(QPoint)), SLOT(displayContextMenu(QPoint)));

	m_ui->nodeView->setModel(m_nodeModel);
	m_ui->nodeView->setItemDelegateForColumn(NodeModel::COL_SHOW, new CheckBoxDelegate(m_ui->nodeView));
	m_ui->nodeView->setItemDelegateForColumn(NodeModel::COL_DEBUG, new CheckBoxDelegate(m_ui->nodeView));
	m_ui->nodeView->setItemDelegateForColumn(NodeModel::COL_LEVEL, new ComboBoxDelegate(m_ui->nodeView));
	m_ui->nodeView->setColumnWidth(NodeModel::COL_SHOW, 45);
	m_ui->nodeView->setColumnWidth(NodeModel::COL_NAME, 110);
	m_ui->nodeView->setColumnWidth(NodeModel::COL_DEBUG, 30);
	m_ui->nodeView->setColumnWidth(NodeModel::COL_LEVEL, 40);

	connect(m_ui->configButton, SIGNAL(clicked(bool)), SLOT(toggleConfig()));
	m_ui->nodeView->hide();

	connect(m_ui->filterEdit, SIGNAL(textChanged(QString)), m_filter, SLOT(setFilterRegExp(QString)));

	connect(m_ui->logView->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)),
		SLOT(handleMessageSelection(QModelIndex))
	);
	connect(m_ui->detailView, SIGNAL(closeRequested()), SLOT(handleMessageClosing()));
	m_ui->detailView->hide();

	QMenu* menu = new QMenu(w);
	menu->addAction(QIcon::fromTheme("edit-clear"), "Clear messages", m_model, SLOT(clear()));

	QAction* act = menu->addAction(QIcon::fromTheme("clock"), "Display absolute timestamps");
	act->setCheckable(true);
	connect(act, SIGNAL(triggered(bool)), m_ui->logView, SLOT(setDisplayAbsoluteTimeStamps(bool)));

	m_ui->toolsButton->setMenu(menu);

	ctx.addWidget(w);

	m_editors << new editors::KDevelop;
}

void RQTLogViewer::shutdownPlugin()
{
	rqt_gui_cpp::Plugin::shutdownPlugin();

	m_sub_log.shutdown();

	// NodeModel has a ROS background thread. This needs to be stopped here
	m_nodeModel->stopBackend();

	qDeleteAll(m_editors);
}

void RQTLogViewer::toggleConfig()
{
	if(m_ui->nodeView->isVisible())
	{
		m_ui->configButton->setText(">> Tree");
		m_ui->nodeView->hide();
	}
	else
	{
		m_ui->configButton->setText("<< Tree");
		m_ui->nodeView->show();
	}
}

void RQTLogViewer::handleMessageSelection(const QModelIndex& index)
{
	if(!index.isValid())
	{
		m_ui->detailView->hide();
		return;
	}

	m_ui->detailView->displayMessage(
		*((rosgraph_msgs::Log*)index.data(LogModel::ROLE_PTR).value<void*>())
	);
	m_ui->detailView->show();
}

void RQTLogViewer::handleMessageClosing()
{
	m_ui->logView->deselect();
}

void RQTLogViewer::saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const
{
	qt_gui_cpp::Plugin::saveSettings(pluginSettings, instanceSettings);

	instanceSettings.setValue("nodeViewSplitter", m_ui->nodeViewSplitter->saveState());

	instanceSettings.setValue("nodeView", m_ui->nodeView->header()->saveState());
}

void RQTLogViewer::restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings)
{
	qt_gui_cpp::Plugin::restoreSettings(pluginSettings, instanceSettings);

	if(instanceSettings.contains("nodeViewSplitter"))
		m_ui->nodeViewSplitter->restoreState(instanceSettings.value("nodeViewSplitter").toByteArray());

	if(instanceSettings.contains("nodeView"))
		m_ui->nodeView->header()->restoreState(instanceSettings.value("nodeView").toByteArray());
}

void RQTLogViewer::displayContextMenu(const QPoint& point)
{
	QModelIndex index = m_ui->logView->indexAt(point);
	if(!index.isValid())
		return;

	void *ptr = index.data(LogModel::ROLE_PTR).value<void*>();
	ROS_ASSERT(ptr);

	const rosgraph_msgs::Log& msg = *(reinterpret_cast<rosgraph_msgs::Log*>(ptr));

	QMenu menu;

	Q_FOREACH(editors::Editor* editor, m_editors)
	{
		QAction* editorAction = editor->createAction(QString::fromStdString(msg.file), msg.line);
		if(editorAction)
			menu.addAction(editorAction);
	}

	menu.exec(m_ui->logView->mapToGlobal(point));
}

}

PLUGINLIB_EXPORT_CLASS(rqt_log_viewer::RQTLogViewer, rqt_gui_cpp::Plugin)
