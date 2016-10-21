// Trajectory editor main window
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>, Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor/mainwindow.h>
#include <trajectory_editor/rvizwidget.h>
#include <trajectory_editor/headerview.h>

#include "ui_mainwindow_2.h"
#include "ui_controls.h"
#include "ui_about.h"

#include <QAction>
#include <QStatusBar>
#include <QKeyEvent>
#include <QShortcut>
#include <QMessageBox>

#include <ros/node_handle.h>

MainWindow::MainWindow()
 : QMainWindow()
{
	// GUI setup
    m_ui = new Ui::MainWindow;
    m_ui->setupUi(this);
	
	setMinimumSize(1300, 650);
	m_ui->jointsTabWidget->setMinimumSize(850, 200);
	m_ui->trajectoryView->setMinimumHeight(350);
	
	QFont font = m_ui->headerLabel->font();
    font.setPointSize(14);
    m_ui->headerLabel->setFont(font);
	
	m_ui->trajectoryView->setAcceptDrops(true);
	m_ui->trajectoryView->setDragEnabled(true);
	m_ui->trajectoryView->setDropIndicatorShown(true);
	m_ui->trajectoryView->setDragDropMode(QAbstractItemView::DragDrop);
	m_ui->trajectoryView->setSelectionMode(QAbstractItemView::ExtendedSelection);
	
	setControlButtonsEnabled(false);
	setPlayButtonsEnabled(false);
	updateCurrentFileLabel(QString("Nothing is opened"));
	
	// About/Controls views setup
	m_controls_view = new QWidget();
	m_about_view    = new QWidget();
	
	Ui::Controls controlsUI;
    controlsUI.setupUi(m_controls_view);
	
	Ui::About aboutUI;
    aboutUI.setupUi(m_about_view);
	
	// Progress bar setup
	m_progress_bar = new QProgressBar(this);
	m_progress_bar->hide();
	m_ui->statusbar->addPermanentWidget(m_progress_bar);
	
	// Core setup
    ros::NodeHandle nh("~");
    m_ui->rviz->initialize(&nh);

	initHeaderWidgets();
    m_kModel = new KeyframeModel(this);
    m_ui->trajectoryView->setModel(m_kModel);
    
	m_motion_player_client = new MotionPlayerClient();
	m_save_controller = new SaveController(m_header_view, m_kModel->getPerspectiveManager());
	m_recent_files = new RecentFiles(m_ui->menuMotion->addMenu("Recent"));
	
	initSpaces();
	
	// Subscribe to MotionPlayerState
	m_state_subscriber = nh.subscribe("/motion_player/state", 1, &MainWindow::motionPlayerStateReceived, this);

	// Connections setup TODO: refactor
	
	connect(m_kModel, SIGNAL(modelChanged(std::string)), m_ui->rviz, SLOT(setModel(std::string)));
	
	connect(m_kModel, SIGNAL(updateRobot(std::vector<std::string>&,std::vector<double>&))
			, m_ui->rviz, SLOT(updateRobotDisplay(std::vector<std::string>&,std::vector<double>&)));
	
	// Header and Frame views
	connect(m_header_view, SIGNAL(dataChanged(HeaderData)), m_kModel, SLOT(updateHeaderData(HeaderData)));
	connect(m_kModel, SIGNAL(headerDataChanged(HeaderData)), m_header_view, SLOT(setData(HeaderData)));
	connect(m_header_view, SIGNAL(pidEnabledChanged(bool)), m_pid_space, SLOT(setEnabled(bool)));
	
	connect(m_frame_view, SIGNAL(updateFrame()), m_kModel, SLOT(handleUpdateFrame()));
	connect(m_frame_view, SIGNAL(frameLoaded(KeyframePtr)), m_kModel, SLOT(handeFrameLoaded(KeyframePtr)));
	connect(m_frame_view, SIGNAL(changeEffortForAllFrames(std::vector<int>, double))
						, m_kModel, SLOT(handleChangeEffortForAllFrames(std::vector<int>, double)));
	
	connect(m_save_controller, SIGNAL(newPath(QString)), m_frame_view, SLOT(setPathAndName(QString)));
	connect(m_save_controller, SIGNAL(modelChanged(std::string, std::vector<std::string> &))
											, m_ui->rviz, SLOT(setModel(std::string, std::vector<std::string> &)));
	
	connect(m_kModel, SIGNAL(jointListChanged(std::vector<std::string>))
						, m_frame_view, SLOT(updateJointList(std::vector<std::string>)));
	
	connect(m_kModel, SIGNAL(currentFrameChanged(KeyframePtr)), m_frame_view, SLOT(setFrame(KeyframePtr)));
	
	// Rule view
	connect(m_kModel, SIGNAL(gotRules(std::vector<motionfile::Rule>)), m_rule_view, SLOT(handleRulesLoaded(std::vector<motionfile::Rule>)));
	connect(m_rule_view, SIGNAL(applyRule(double, int)), m_kModel, SLOT(applyRule(double, int)));
	
	// Play frame/motion
	connect(m_ui->playFrameButton, SIGNAL(clicked(bool)), this, SLOT(playFrameClicked()));
	connect(m_ui->playFrameSlowButton, SIGNAL(clicked(bool)), this, SLOT(playFrameSlowClicked()));
	
	connect(m_ui->playMotionButton, SIGNAL(clicked(bool)), this, SLOT(playMotionClicked()));
	connect(m_ui->playMotionSlowButton, SIGNAL(clicked(bool)), this, SLOT(playSlowMotionClicked()));
	
	connect(m_ui->actionPlay, SIGNAL(triggered()), this, SLOT(playMotionClicked()));
	connect(m_ui->actionPlayMotionSlow, SIGNAL(triggered()), this, SLOT(playSlowMotionClicked()));
	
	connect(m_ui->playSequenceButton, SIGNAL(clicked(bool)), this, SLOT(playSequenceClicked()));
	
	connect(m_ui->updateMotionButton, SIGNAL(clicked(bool)), this, SLOT(updateMotionClicked()));
	
	// Add/remove frame
	connect(m_ui->addFrameButton, SIGNAL(clicked(bool)), this, SLOT(handleAddFrameButton()));
    connect(m_ui->remFrameButton, SIGNAL(clicked(bool)), this, SLOT(handleRemoveButton()));
	
	// Move selection up/down
    connect(m_ui->moveUpButton, SIGNAL(clicked(bool)), this, SLOT(handleMoveUp()));
    connect(m_ui->moveDownButton, SIGNAL(clicked(bool)), this, SLOT(handleMoveDown()));
	
	// Open/New/Save
	connect(m_ui->openButton, SIGNAL(clicked(bool)), this, SLOT(handleLoad()));
	connect(m_ui->actionOpen, SIGNAL(triggered()), this, SLOT(handleLoad()));
	
	connect(m_ui->saveButton, SIGNAL(clicked(bool)), this, SLOT(handleSave()));
	connect(m_ui->actionSave, SIGNAL(triggered()), this, SLOT(handleSave()));
	
	connect(m_ui->actionSave_as, SIGNAL(triggered()), this, SLOT(handleSaveAs()));
	connect(m_ui->actionSave_mirrored_as, SIGNAL(triggered()), this, SLOT(handleSaveMirroredAs()));
	connect(m_ui->actionNew_motion, SIGNAL(triggered()), this, SLOT(handleNewMotion()));
	
	connect(m_recent_files, SIGNAL(requestOpen(QString)), this, SLOT(handleLoad(QString)));

	connect(m_ui->actionShow_in_folder, SIGNAL(triggered()), m_kModel, SLOT(showMotionInFolder()));
	
    // Show widgets with help info
	connect(m_ui->actionControls, SIGNAL(triggered()), this, SLOT(showControls()));
	connect(m_ui->actionAbout, SIGNAL(triggered()), this, SLOT(showAbout()));

	connect(m_ui->trajectoryView->selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)), 
      this, SLOT(handleSelectionChanged(QItemSelection)));
	
	// Model view
	connect(m_kModel, SIGNAL(setViewSelection(int)), this, SLOT(setViewSelection(int)));
	connect(m_kModel, SIGNAL(setViewSelection(QItemSelection)), this, SLOT(setViewSelection(QItemSelection)));
	connect(m_kModel, SIGNAL(updateCurrentFileLabel(QString)), this, SLOT(updateCurrentFileLabel(QString)));
	connect(m_kModel, SIGNAL(gotMotion(bool)), this, SLOT(setControlButtonsEnabled(bool)));
	
	// Set up shortcuts
	QShortcut *shortcutCopy = new QShortcut(QKeySequence(tr("Ctrl+C", "copy")), this);
	QShortcut *shortcutPaste = new QShortcut(QKeySequence(tr("Ctrl+V", "paste")), this);
	QShortcut *shortcutDisable = new QShortcut(QKeySequence(tr("Ctrl+D", "disable")), this);
	
	connect(shortcutCopy, SIGNAL(activated()), this, SLOT(ctrlC()));
	connect(shortcutPaste, SIGNAL(activated()), this, SLOT(ctrlV()));
	connect(shortcutDisable, SIGNAL(activated()), this, SLOT(handleDisableSelected()));
}

void MainWindow::handleNewMotion()
{
	motionfile::Motion newMotion;
	QString path = m_save_controller->newMotion(newMotion);
	
	if(path != "")
	{
		if(m_kModel->hasMotion() && m_kModel->hasUnsavedChanges())
			proposeSave();
		
		m_kModel->setMotion(newMotion);
		m_recent_files->addRecentFile(path);
	}
}

void MainWindow::handleLoad()
{
	motionfile::Motion loadedMotion;
	QString path = m_save_controller->open(loadedMotion);
	
	handleLoad(path);
}

void MainWindow::handleLoad(QString path)
{
	motionfile::Motion loadedMotion;
	QString ckeckPath = m_save_controller->open(loadedMotion, path);
	
	if(ckeckPath != "")
	{
		if(m_kModel->hasMotion() && m_kModel->hasUnsavedChanges())
			proposeSave();
		
		m_kModel->setMotion(loadedMotion);
		m_recent_files->addRecentFile(ckeckPath);
	}
	else
		QMessageBox::critical(0, "Error", "File you tried to open is not a valid motion!");
}

void MainWindow::handleSave()
{
	m_save_controller->save(m_kModel->getMotion());
	m_kModel->setSaved();
}

void MainWindow::handleSaveAs()
{
	m_save_controller->saveAs(m_kModel->getMotion());
}

void MainWindow::handleSaveMirroredAs()
{
	motionfile::Motion motion = m_kModel->getMotionCopy();
	m_save_controller->saveMirroredAs(m_kModel->getMotion(), motion);
}

void MainWindow::initHeaderWidgets()
{
	// Create Header
	m_header_view = new HeaderView(m_ui->headerTabWidget);
	m_header_view->enableEdit(false);
    m_ui->headerTabWidget->addTab(createAreaWithWidget(m_header_view), "Header");
	
	m_header_view->resize(800, 250);
	m_ui->headerTabWidget->updateGeometry();
	
	// Create Frame view
	m_frame_view = new FrameView(m_ui->headerTabWidget);
    m_ui->headerTabWidget->addTab(createAreaWithWidget(m_frame_view), "Frame");
	
	// Create Rule view
	m_rule_view = new RuleView(m_ui->headerTabWidget);
	m_ui->headerTabWidget->addTab(createAreaWithWidget(m_rule_view), "Rule");
}

void MainWindow::initSpaces()
{
	// Create Joint tabs
	m_joint_space = new JointManager(m_kModel->getJointList(), m_ui->jointsTabWidget);
    m_ui->jointsTabWidget->addTab(createAreaWithWidget(m_joint_space), "Joint Space");
	
	m_abstract_space = new AbstractSpace(m_kModel->getJointList(), m_ui->jointsTabWidget);
	m_ui->jointsTabWidget->addTab(createAreaWithWidget(m_abstract_space), "Abstract Space");
	
	m_inverse_space = new InverseSpace(m_kModel->getJointList(), m_ui->jointsTabWidget);
	m_ui->jointsTabWidget->addTab(createAreaWithWidget(m_inverse_space), "Inverse Space");
	
	m_pid_space = new PIDSpace(m_kModel->getJointList(), m_ui->jointsTabWidget);
	m_ui->jointsTabWidget->addTab(createAreaWithWidget(m_pid_space), "PID Space");
	
	m_joint_space->resize(800, 150);
	m_ui->jointsTabWidget->updateGeometry();
	
	m_spaces.push_back(m_joint_space);
	m_spaces.push_back(m_abstract_space);
	m_spaces.push_back(m_inverse_space);
	m_spaces.push_back(m_pid_space);
	
	// Set up connections
	for(unsigned i = 0; i < m_spaces.size(); i++)
	{
		m_spaces.at(i)->setEnabled(false);
		
		connect(m_spaces.at(i), SIGNAL(updateRobot()), m_kModel, SLOT(updateRobotDisplay()));
		
		connect(m_kModel, SIGNAL(currentFrameChanged(KeyframePtr)), m_spaces.at(i), SLOT(setFrame(KeyframePtr)));
		connect(m_kModel, SIGNAL(jointListChanged(std::vector<std::string>)), m_spaces.at(i), SLOT(updateJointList(std::vector<std::string>)));
		
		connect(m_kModel, SIGNAL(perspectiveChanged(joint_perspective::JointPerspective)),
			m_spaces.at(i), SLOT(handlePerspectiveUpdate(joint_perspective::JointPerspective)));
	}
	
	connect(m_kModel, SIGNAL(getInverseLimits(bool&,double&)), m_inverse_space, SLOT(getInverseLimits(bool&,double&)));
	connect(m_ui->jointsTabWidget, SIGNAL(currentChanged(int)), m_kModel, SLOT(handleTabChange(int)));
}

QScrollArea* MainWindow::createAreaWithWidget(QWidget* widget)
{
	QScrollArea *area = new QScrollArea;
    area->setWidget(widget);
    area->setWidgetResizable(true);
	return area;
}

void MainWindow::motionPlayerStateReceived(const motion_player::MotionPlayerState& stateMsg)
{
	if(m_kModel->hasMotion() == false)
		return;
	
	QString state;
	
	if(stateMsg.isPlaying)
	{
		setPlayButtonsEnabled(false);
		state = QString("Motion %1 is being played").arg(QString::fromStdString(stateMsg.motionName));
		
		// Visualize duration via progress bar
		m_progress_bar->setMinimum(0);
		m_progress_bar->setMaximum((int)(stateMsg.motionDuration*100));
		
		m_progress_bar->setValue((int)(stateMsg.currentTime*100));
		
		QString progressBarText = QString("%1 / %2 seconds (%p%)" ).arg(stateMsg.currentTime)
																	.arg(stateMsg.motionDuration);
		
		m_progress_bar->setFormat(progressBarText);
		
		if(m_progress_bar->isHidden())
			m_progress_bar->show();
	}
	else
	{
		setPlayButtonsEnabled(true);
		state = QString("Nothing is being played");
		
		m_motion_player_client->tryPlayDelayedMotion();
		
		if(m_progress_bar->isHidden() == false)
			m_progress_bar->hide();
	}
	
	m_ui->statusbar->showMessage(state);
}

void MainWindow::playFrameClicked()
{
	m_motion_player_client->playFrame(m_kModel->getCurrentFrameCopy(), m_kModel->getJointList(), 1);
}

void MainWindow::playFrameSlowClicked()
{
	m_motion_player_client->playFrame(m_kModel->getCurrentFrameCopy()
										, m_kModel->getJointList(), m_header_view->getFactor());
}

void MainWindow::playMotionClicked()
{
	m_save_controller->saveBackup(m_kModel->getMotion());
	m_motion_player_client->playMotion(m_kModel->getMotionCopy(), 1);
}

void MainWindow::playSlowMotionClicked()
{
	m_save_controller->saveBackup(m_kModel->getMotion());
	m_motion_player_client->playMotion(m_kModel->getMotionCopy(), m_header_view->getFactor());
}

// If there are at least 2 frames selected - play motion constructed from these frames
void MainWindow::playSequenceClicked()
{
	std::vector<int> indices = getSelectedIndices(true);
	
	if(indices.size() >= 2)
		m_motion_player_client->playSequence(m_kModel->getMotionCopy(), indices);
}

void MainWindow::updateMotionClicked()
{
	m_motion_player_client->updateMotion(m_kModel->getMotionCopy());
}

void MainWindow::setPlayButtonsEnabled(bool flag)
{
	m_ui->playFrameButton->setEnabled(flag);
	m_ui->playFrameSlowButton->setEnabled(flag);
	
	m_ui->playMotionButton->setEnabled(flag);
	m_ui->playMotionSlowButton->setEnabled(flag);
	
	m_ui->playSequenceButton->setEnabled(flag);
	
	m_ui->updateMotionButton->setEnabled(flag);
	
	m_ui->actionPlay->setEnabled(flag);
	m_ui->actionPlayMotionSlow->setEnabled(flag);
}

void MainWindow::setControlButtonsEnabled(bool flag)
{
	m_ui->saveButton->setEnabled(flag);
	m_ui->actionSave->setEnabled(flag);
	m_ui->actionSave_as->setEnabled(flag);
	m_ui->actionSave_mirrored_as->setEnabled(flag);
	
	m_ui->moveUpButton->setEnabled(flag);
	m_ui->moveDownButton->setEnabled(flag);
	m_ui->addFrameButton->setEnabled(flag);
	m_ui->remFrameButton->setEnabled(flag);
}

void MainWindow::ctrlC()
{
	QModelIndexList selectedIndices = m_ui->trajectoryView->selectionModel()->selectedIndexes();
	m_kModel->copySelectedFrames(selectedIndices);
}

void MainWindow::ctrlV()
{
	m_kModel->insertCopiedFrames();
}

void MainWindow::handleAddFrameButton()
{
	// Get current selection
	QModelIndexList selectedIndices = m_ui->trajectoryView->selectionModel()->selectedIndexes();
	
	if(selectedIndices.size() > 0) 
		m_kModel->addFrame(selectedIndices.last().row() + 1); // If something is selected - add frame after it
	else
		m_kModel->addFrame(m_kModel->rowCount()); // Otherwise - add frame to the end
}

// Rturn vector of currently selected indices. Sorted if sort == true
std::vector<int> MainWindow::getSelectedIndices(bool sort)
{
	// Get current selection
	QModelIndexList selectedIndices = m_ui->trajectoryView->selectionModel()->selectedIndexes();
	std::vector<int> indices;
	
	for(int i = 0; i < selectedIndices.size(); i++)
		indices.push_back(selectedIndices.at(i).row());
	
	// Sort indices from lesser to bigger if requested
	if(sort)
		std::sort(indices.begin(), indices.end());  
	
	return indices;
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

void MainWindow::updateCurrentFileLabel(QString newTitle)
{
	m_ui->headerLabel->setText(newTitle);
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

void MainWindow::handleDisableSelected()
{
	QModelIndexList list = m_ui->trajectoryView->selectionModel()->selectedIndexes();
	qSort(list.begin(), list.end(), qGreater<QModelIndex>());

	for(int i = 0; i < list.size(); i++)
		m_kModel->disableFrame(list.at(i).row());
}

void MainWindow::showAbout()
{
	m_about_view->show();
}

void MainWindow::showControls()
{
	m_controls_view->show();
}

void MainWindow::proposeSave()
{
	QMessageBox proposeSaveBox;
	proposeSaveBox.setWindowTitle("Trajectory Editor warning");
	proposeSaveBox.setText("You have unsaved changes! Do you want to save them?");;
	proposeSaveBox.setStandardButtons(QMessageBox::Save | QMessageBox::Close);
	proposeSaveBox.setDefaultButton(QMessageBox::Save);
		
	int answer = proposeSaveBox.exec();
		
	if(answer == QMessageBox::Save)
		handleSave();
}

void MainWindow::onQuit()
{
	if(m_kModel->hasMotion() && m_kModel->hasUnsavedChanges())
		proposeSave();
}

MainWindow::~MainWindow()
{
	onQuit();
	
	delete m_kModel;
	delete m_motion_player_client;
	delete m_save_controller;
	delete m_recent_files;
    delete m_ui;
	
	delete m_controls_view;
	delete m_about_view;
}

