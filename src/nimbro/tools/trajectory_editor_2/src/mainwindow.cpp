// Trajectory editor main window
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <trajectory_editor_2/mainwindow.h>
#include <trajectory_editor_2/rvizwidget.h>

#include "ui_mainwindow_2.h"
#include "ui_controls.h"
#include "ui_about.h"

#include <qt4/QtCore/QObject>
#include <qt4/QtGui/QAction>
#include <qt4/QtGui/QStatusBar>
#include <QKeyEvent>

#include <ros/node_handle.h>

MainWindow::MainWindow()
 : QMainWindow()
{
	// GUI setup
    m_ui = new Ui::MainWindow;
    m_ui->setupUi(this);
	
	this->setMinimumSize(1200, 650);
	m_ui->jointsTabWidget->setMinimumSize(850, 200);
	
	QFont font = m_ui->headerLabel->font();
    font.setPointSize(14);
    m_ui->headerLabel->setFont(font);
	
	m_ui->trajectoryView->setAcceptDrops(true);
	m_ui->trajectoryView->setDragEnabled(true);
	m_ui->trajectoryView->setDropIndicatorShown(true);
	m_ui->trajectoryView->setDragDropMode(QAbstractItemView::DragDrop);
	m_ui->trajectoryView->setSelectionMode(QAbstractItemView::ExtendedSelection);
	
	this->updateStatusBar(QString("Nothing is opened"));
	
	// About/Controls views setup
	controlsView = new QWidget();
	aboutView    = new QWidget();
	
	Ui::Controls controlsUI;
    controlsUI.setupUi(controlsView);
	
	Ui::About aboutUI;
    aboutUI.setupUi(aboutView);
	
	// Core setup
    ros::NodeHandle nh("~");
    m_ui->rviz->initialize(&nh);

    m_kModel = new KeyframeModel(this);
    m_ui->trajectoryView->setModel(m_kModel);
    
    m_kModel->initRobot(m_ui->rviz->getRobot());
    m_kModel->initJointDisplay(m_ui->jointsTabWidget, m_ui->headerTabWidget);

	// Connections setup
    connect(m_ui->addFrameButton, SIGNAL(clicked(bool)), m_kModel, SLOT(addFrame()));
    connect(m_ui->remFrameButton, SIGNAL(clicked(bool)), this, SLOT(handleRemoveButton()));
    connect(m_ui->playFrameButton, SIGNAL(clicked(bool)), m_kModel, SLOT(playFrame()));
	connect(m_ui->playFrameSlowButton, SIGNAL(clicked(bool)), m_kModel, SLOT(playFrameSlow()));
	
    connect(m_ui->moveUpButton, SIGNAL(clicked(bool)), this, SLOT(handleMoveUp()));
    connect(m_ui->moveDownButton, SIGNAL(clicked(bool)), this, SLOT(handleMoveDown()));
	
	connect(m_ui->playMotionButton, SIGNAL(clicked(bool)), m_kModel, SLOT(playMotion()));
	connect(m_ui->playMotionSlowButton, SIGNAL(clicked(bool)), m_kModel, SLOT(playMotionSLow()));
	
	connect(m_ui->openButton, SIGNAL(clicked(bool)), m_kModel, SLOT(load()));
	connect(m_ui->saveButton, SIGNAL(clicked(bool)), m_kModel, SLOT(save()));
	
    connect(m_ui->actionNew_motion, SIGNAL(triggered()), m_kModel, SLOT(createMotion()));
    connect(m_ui->actionSave, SIGNAL(triggered()), m_kModel, SLOT(save()));
	connect(m_ui->actionSave_as, SIGNAL(triggered()), m_kModel, SLOT(saveAs()));
	connect(m_ui->actionSave_mirrored_as, SIGNAL(triggered()), m_kModel, SLOT(saveMirroredAs()));
    connect(m_ui->actionOpen, SIGNAL(triggered()), m_kModel, SLOT(load()));
	
    connect(m_ui->actionPlay, SIGNAL(triggered()), m_kModel, SLOT(playMotion()));
	connect(m_ui->actionPlayMotionSlow, SIGNAL(triggered()), m_kModel, SLOT(playMotionSLow()));
	
	connect(m_ui->actionControls, SIGNAL(triggered()), this, SLOT(showControls()));
	connect(m_ui->actionAbout, SIGNAL(triggered()), this, SLOT(showAbout()));

	connect(m_ui->trajectoryView->selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)), 
      this, SLOT(handleSelectionChanged(QItemSelection)));
	
	connect(m_kModel, SIGNAL(setViewSelection(int)), this, SLOT(setViewSelection(int)));
	connect(m_kModel, SIGNAL(setViewSelection(QItemSelection)), this, SLOT(setViewSelection(QItemSelection)));
}

void MainWindow::handleSelectionChanged(QItemSelection selection)
{
	QModelIndexList selectedIndexes = selection.indexes();
	
	if(selectedIndexes.size() > 0)
		m_kModel->handleIndexChange(selectedIndexes.first());
}

void MainWindow::setViewSelection(int row)
{
	QModelIndex index = m_kModel->index(row, 0, QModelIndex());
	
	m_ui->trajectoryView->setCurrentIndex(index);
	m_ui->trajectoryView->update();
	m_ui->trajectoryView->setFocus();
}

void MainWindow::setViewSelection(QItemSelection selection)
{
	QItemSelectionModel *selectionModel = m_ui->trajectoryView->selectionModel();
	
	m_ui->trajectoryView->clearSelection();
    selectionModel->select(selection, QItemSelectionModel::Select);
	
	m_ui->trajectoryView->update();
	m_ui->trajectoryView->setFocus();
}

void MainWindow::updateStatusBar(QString status)
{
    m_ui->statusbar->showMessage(status);
}

void MainWindow::updateHeaderLabel(QString text)
{
	m_ui->headerLabel->setText(text);
}

void MainWindow::handleMoveUp()
{
	QModelIndexList selectedIndexes = m_ui->trajectoryView->selectionModel()->selectedIndexes();
	qSort(selectedIndexes.begin(), selectedIndexes.end(), qLess<QModelIndex>());
	
	if(selectedIndexes.isEmpty())
		return;
	
	if(selectedIndexes.at(0).row() == 0)
		return;
	
	QItemSelection selection;

	for(int i = 0; i < selectedIndexes.size(); i++)
	{
		m_kModel->moveFrame(selectedIndexes.at(i).row(), -1); // move
		
		QModelIndex index = m_kModel->index(selectedIndexes.at(i).row()-1, 0, QModelIndex());
		selection.select(index, index); // create new selection
	}
	
	this->setViewSelection(selection);
}

void MainWindow::handleMoveDown()
{
	QModelIndexList selectedIndexes = m_ui->trajectoryView->selectionModel()->selectedIndexes();
	qSort(selectedIndexes.begin(), selectedIndexes.end(), qGreater<QModelIndex>());
	
	if(selectedIndexes.isEmpty())
		return;
	
	if(selectedIndexes.at(0).row() == m_kModel->rowCount()-1)
		return;
	
	QItemSelection selection;
	
	for(int i = 0; i < selectedIndexes.size(); i++)
	{
		m_kModel->moveFrame(selectedIndexes.at(i).row(), 1); // move
		
		QModelIndex index = m_kModel->index(selectedIndexes.at(i).row()+1, 0, QModelIndex());
		selection.select(index, index); // create new selection
	}
	
	this->setViewSelection(selection);
}

void MainWindow::handleRemoveButton()
{
	QModelIndexList list = m_ui->trajectoryView->selectionModel()->selectedIndexes();
	qSort(list.begin(), list.end(), qGreater<QModelIndex>());

	for(int i = 0; i < list.size(); i++)
		m_kModel->removeFrame(list.at(i).row());
}

void MainWindow::showAbout()
{
	aboutView->show();
}

void MainWindow::showControls()
{
	controlsView->show();
}

MainWindow::~MainWindow()
{
	delete m_kModel;
    delete m_ui;
	
	delete controlsView;
	delete aboutView;
}

