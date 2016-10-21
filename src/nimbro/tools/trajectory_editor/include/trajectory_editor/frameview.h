#ifndef FRAMEVIEW_H
#define FRAMEVIEW_H

// Widget to edit propertirs of KeyFrame:
// Duration
// Efforts

// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QWidget>
#include <QObject>

#include <boost/shared_ptr.hpp>
#include <motion_file/motionfile.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <robotcontrol/RobotState.h>

namespace Ui
{
	class FrameView; 
}

// TODO refactor
class FrameView : public QWidget
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	FrameView(QWidget *parent = 0);
	~FrameView();
	
	virtual bool eventFilter(QObject *object, QEvent *event);
	
	QString getPathToDir(); // Return path to dirrectory where current motion file is located
	
	double getEffort();
	int nameToIndex(std::string name);
	
	static bool parseSupport(QString support, double &left, double &right);
	
Q_SIGNALS:
	void updateFrame();
	void frameLoaded(KeyframePtr frame);
	void changeEffortForAllFrames(std::vector<int> indexesToApply, double newEffort);
	
public Q_SLOTS:
	void saveFrame(); // Saves current frame near opened motion with ".frame" extension
	void loadFrame();
	
	void setFrame(KeyframePtr frame);
	void updateJointList(const std::vector<std::string> &jointList);
	
	void setPathAndName(QString path); // Extracts path to file and file name for further saving
	
private Q_SLOTS:
	void nameChanged();
	void updateRollPitchYaw();
	
	void durationSpinChanged();
	void durationSliderChanged();
	
	void enableSupport(bool enable);
	
	void supportLeftSpinChanged();
	void supportRightSpinChanged();
	void supportSliderChanged();
	
	void handleRollSpinChanged();
	void handlePitchSpinChanged();
	void handleYawSpinChanged();
	
	void changeEffort();
	void setAllChecked(bool flag);
	
private:
	void findIndices(std::vector<int> &indices);
	void setCurrentSupport(); // Set support from spins
	void setSupportFromFrame(); // Parse support from frame, set sliders/spins
	
	void robotStateReceived(const robotcontrol::RobotStateConstPtr& state);
	
private:
	Ui::FrameView *ui;
	
	KeyframePtr frame;
	std::vector<std::string> joints;
	
	QString path;
	QString motionName;
	
	ros::NodeHandle nh;
	ros::Subscriber subscriber;
	bool updateRPY; // If true, values of Roll,Pitch,Yaw from /robotmodel/robot_state are set to current frame
};

#endif // FRAMEVIEW_H