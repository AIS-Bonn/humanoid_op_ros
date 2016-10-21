// Widget to edit joints in Abstract Space
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef ABSTRACTSPACE_H
#define ABSTRACTSPACE_H

#include <QLabel>
#include <QGridLayout>

#include <gait/util/gait_abstract_pose.h>
#include <boost/shared_ptr.hpp>

#include <motion_file/poseconverter.h>
#include <trajectory_editor/spaces/rateangleview.h>
#include <trajectory_editor/spaces/basicspace.h>

class AbstractSpace : public BasicSpace
{
Q_OBJECT
public:
	AbstractSpace(const std::vector<std::string> &jointList, QWidget *parent = 0);
	~AbstractSpace();
	
public Q_SLOTS:
	void handlePerspectiveUpdate(const joint_perspective::JointPerspective &perspective);
	
private Q_SLOTS:
	void handleRateChanged(std::string jointName);
	void handleChangeForInverse(const std::string joint_name, const RateAngleView::Field field, const double value);
	
private:
	void initViews();
	void updateFrame();
	
	void updateRate(double rate, std::string jointName);
	void updateRateFromUI(std::string name, double rate);
	
	// Find joint jointName in jointList and create view for it. Put it on layout
	// If jointName was not found, put warning on layout
	void findAndPutView(std::string jointName, std::string label, int row
	, RateAngleView::Alignment alignment, RateAngleView::Type type, bool shiftMirrored);
	
	void createHeaderLabels(RateAngleView::Alignment alignment, int row);
	
private:
	gait::AbstractPose *abstractPose;
	
	std::vector<RateAngleView*> jointViews;
	QGridLayout *jointsLayout;
	
	motionfile::PoseConverter p;
};

#endif // ABSTRACTSPACE_H