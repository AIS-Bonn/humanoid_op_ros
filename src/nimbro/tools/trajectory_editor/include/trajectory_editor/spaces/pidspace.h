// Displays PID of loaded frame
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef PIDSPACE_H
#define PIDSPACE_H

#include <QGridLayout>
#include <QLabel>

#include <trajectory_editor/spaces/pidview.h>
#include <trajectory_editor/spaces/basicspace.h>

class PIDSpace : public BasicSpace
{
Q_OBJECT
public:
	PIDSpace(const std::vector<std::string> &modelJointList, QWidget *parent = 0);
	~PIDSpace(){};

public Q_SLOTS:
	void handleFieldChanged(PIDView::Field field);
	void handlePerspectiveUpdate(const joint_perspective::JointPerspective &perspective);
	void handleApplyRulePart(KeyframePtr frame, const motionfile::RulePart &part, double delta){};

private:
	void updateFrame();
	void initGUI(const joint_perspective::JointPerspective &perspective);
	
	// Find joint jointName in jointList and create view for it. Put it on layout
	// If jointName was not found, put warning on layout
	void findAndPutView(std::string jointName, std::string label, int row, BasicSmallView::Alignment alignment, BasicSmallView::Type type, bool shiftMirrored);
	void createHeaderLabels(BasicSmallView::Alignment alignment, int row);
	
private:
	std::vector<PIDView*> jointViews;
	QGridLayout *jointsLayout;
};


#endif