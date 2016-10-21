// Trajectory editor main window
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>, Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QItemSelection>
#include <QProgressBar>
#include <QScrollArea>
#include <QString>

#include <motion_player/MotionPlayerState.h>

#include <trajectory_editor/keyframemodel.h>
#include <trajectory_editor/motionplayerclient.h>
#include <trajectory_editor/savecontroller.h>
#include <trajectory_editor/recentfiles.h>

#include <trajectory_editor/headerview.h>
#include <trajectory_editor/frameview.h>
#include <trajectory_editor/ruleview.h>

#include <trajectory_editor/spaces/jointdisplay.h>
#include <trajectory_editor/spaces/abstractspace.h>
#include <trajectory_editor/spaces/inversespace.h>
#include <trajectory_editor/spaces/pidspace.h>

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
Q_OBJECT
public:
    MainWindow();
    virtual ~MainWindow();

private Q_SLOTS:
	void playFrameClicked();
	void playFrameSlowClicked();
	
	void playMotionClicked();
	void playSlowMotionClicked();
	
	void playSequenceClicked();
	void updateMotionClicked();
	
	void handleNewMotion();
	
	void handleLoad();
	void handleLoad(QString path);
	void handleSave();
	void handleSaveAs();
	void handleSaveMirroredAs();
	
	void handleAddFrameButton();
    void handleRemoveButton();
	void handleSelectionChanged(QItemSelection);
	void handleDisableSelected();
	void setViewSelection(int row);
	void setViewSelection(QItemSelection selection);
	
	void handleMoveUp();
	void handleMoveDown();
	
	void showAbout();
	void showControls();
	
	void ctrlC();
	void ctrlV();
	
	void updateCurrentFileLabel(QString newTitle);
	
	void setPlayButtonsEnabled(bool flag);
	void setControlButtonsEnabled(bool flag);
	
private:
	void motionPlayerStateReceived(const motion_player::MotionPlayerState& stateMsg);
	std::vector<int> getSelectedIndices(bool sort);
	
	void proposeSave(); // Proposes to save the motion. If user clicks yes - saves it
	void initSpaces();
	void initHeaderWidgets();
	void onQuit();
	
	QScrollArea* createAreaWithWidget(QWidget *widget);

private:
    Ui::MainWindow* m_ui;
    KeyframeModel* m_kModel;
	MotionPlayerClient *m_motion_player_client;
	SaveController *m_save_controller;
	RecentFiles *m_recent_files;
	
	HeaderView *m_header_view;
	FrameView  *m_frame_view;
	RuleView   *m_rule_view;
	
	std::vector<BasicSpace*> m_spaces;
	JointManager  *m_joint_space;
	AbstractSpace *m_abstract_space;
	InverseSpace  *m_inverse_space;
	PIDSpace      *m_pid_space;
	
	QWidget *m_controls_view;
	QWidget *m_about_view;
	
	QProgressBar *m_progress_bar;
	ros::Subscriber m_state_subscriber;
};

#endif
