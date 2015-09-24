#ifndef ABSTRACTSPACE_H
#define ABSTRACTSPACE_H

// Widget to edit joints in Abstract Space
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <trajectory_editor_2/poseconverter.h>
#include <trajectory_editor_2/rateangleview.h>

#include <QWidget>
#include <QObject>
#include <QFrame>
#include <QLabel>
#include <QGridLayout>

#include <gait/util/gait_abstract_pose.h>
#include <boost/shared_ptr.hpp>
#include <motion_file/motionfile.h>

class AbstractSpace : public QWidget
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	AbstractSpace(const std::vector<std::string> &jointList, QWidget *parent = 0);
	~AbstractSpace();
	
	void setFrame(KeyframePtr frame);
	void updateJointList(const std::vector<std::string>& jointList);

Q_SIGNALS:
	void frameDataChanged();
	
private Q_SLOTS:
	void handleRateChanged();
	
private:
	void initViews();
	void updateFrame();
	
	void updateRate(double rate, std::string jointName);
	void updateRateFromUI(string name, double rate);
	
	int  indexToName(std::string name);
	
	// Find joint jointName in jointList and create view for it. Put it on layout
	// If jointName was not found, put warning on layout
	void findAndPutView(std::string jointName, std::string label, int row
	, RateAngleView::Alignment alignment, RateAngleView::Type type);
	
	void createHeaderLabels(RateAngleView::Alignment alignment, int row);
	QFrame* createLine();
	
private:
	KeyframePtr frame;
	gait::AbstractPose *abstractPose;
	
	std::vector<std::string> joints;
	std::vector<RateAngleView*> jointViews;
	
	QGridLayout *jointsLayout;
};

#endif // ABSTRACTSPACE_H