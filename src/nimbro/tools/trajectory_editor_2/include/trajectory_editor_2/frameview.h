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

namespace Ui
{
	class FrameView; 
}

class FrameView : public QWidget
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	FrameView(const std::vector<std::string> &jointList, QWidget *parent = 0);
	~FrameView();
	
	virtual bool eventFilter(QObject *object, QEvent *event);
	
	void setFrame(KeyframePtr frame);
	void setPathAndName(QString path); // Extracts path to file and file name for further saving
	void updateJointList(const std::vector<std::string> &jointList);
	
	double getEffort();
	int nameToIndex(std::string name);
	
Q_SIGNALS:
	void updateFrame();
	void frameLoaded(KeyframePtr frame);
	void changeEffortForAllFrames(std::vector<int> indexesToApply);
	
private Q_SLOTS:
	void nameChanged();
	
	void durationSpinChanged();
	void durationSliderChanged();
	
	void enableSupport(bool enable);
	
	void supportLeftSpinChanged();
	void supportRightSpinChanged();
	void supportSliderChanged();
	
	void changeEffort();
	void setAllChecked(bool flag);
	
	void saveFrame(); // Saves current frame near opened motion with ".frame" extension
	void loadFrame();
	
private:
	void findIndices(std::vector<int> &indices);
	void setCurrentSupport(); // Set support from spins
	void setSupportFromFrame(); // Parse support from frame, set sliders/spins
	
private:
	Ui::FrameView *ui;
	
	KeyframePtr frame;
	std::vector<std::string> joints;
	
	QString path;
	QString motionName;
};

#endif // FRAMEVIEW_H