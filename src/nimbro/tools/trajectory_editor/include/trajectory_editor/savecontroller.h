// Performs saving/loading motions
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef SAVE_CONTROLLER_H
#define SAVE_CONTROLLER_H

#include <trajectory_editor/headerview.h>
#include <trajectory_editor/frameview.h>
#include <trajectory_editor/jointperspective.h>

#include <QObject>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QTime>

#include <ros/package.h>
#include <ros/console.h>

#include <motion_file/motionfile.h>

class SaveController : public QObject
{
    Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	SaveController(HeaderView *view, joint_perspective::PerspectiveManager *manager);
	~SaveController();
	
	// Return path to motion. Empty string if operation failed
	QString newMotion(motionfile::Motion &motion);
	QString open(motionfile::Motion &motion);
	QString open(motionfile::Motion &motion, QString path);
	
	void save(motionfile::Motion &motion);
	void saveAs(motionfile::Motion &motion);
	void saveMirroredAs(motionfile::Motion &original_motion, motionfile::Motion &mirrored);
	void saveBackup(motionfile::Motion &motion); // Save backup in /tmp/trajectory_editor
	
Q_SIGNALS:
	void newPath(QString path);
	void modelChanged(std::string model_path, std::vector<std::string> &new_joint_list);
	
private:
	KeyframePtr firstFrame(unsigned joints_amount);
	std::string getFileNameFromPath(QString path);
	void showMessageBox(QString title, QString text);
	
	bool isMotionValid(motionfile::Motion &motion);
	
	// Mirror values of specified joint to create a mirrored motion
	void mirrorValues(KeyframePtr frame, KeyframePtr mirrored, std::string 
		 left, std::string right, bool swap, bool invert, std::vector<std::string> jointList);
	
private:
	HeaderView *m_header_view;
	joint_perspective::PerspectiveManager *m_perspective_manager;
};

#endif // SAVE_CONTROLLER_H
