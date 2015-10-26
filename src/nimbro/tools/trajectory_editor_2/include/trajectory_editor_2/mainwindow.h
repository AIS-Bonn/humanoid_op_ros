// Trajectory editor main window
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <qt4/QtGui/QMainWindow>
#include <qt4/QtCore/QString>
#include <QItemSelection>

#include <trajectory_editor_2/keyframemodel.h>

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
Q_OBJECT
public:
    MainWindow();
    virtual ~MainWindow();

    void updateStatusBar(QString status);
	void updateHeaderLabel(QString text);

private Q_SLOTS:
    void handleRemoveButton();
	void handleSelectionChanged(QItemSelection);
	void setViewSelection(int row);
	void setViewSelection(QItemSelection selection);
	
	void handleMoveUp();
	void handleMoveDown();
	
	void showAbout();
	void showControls();

private:
    Ui::MainWindow* m_ui;
    KeyframeModel* m_kModel;
	
	QWidget *controlsView;
	QWidget *aboutView;
};

#endif
