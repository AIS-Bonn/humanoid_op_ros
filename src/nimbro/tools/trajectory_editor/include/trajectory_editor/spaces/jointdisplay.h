// Displays joints of loaded frame
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef JOINTDISPLAY_H
#define JOINTDISPLAY_H

#include <QGridLayout>
#include <QLabel>

#include <trajectory_editor/spaces/posveleffview.h>
#include <trajectory_editor/spaces/basicspace.h>

// TODO: rename to JointSpace
class JointManager : public BasicSpace
{
Q_OBJECT
public:
	JointManager(const std::vector<std::string> &modelJointList, QWidget *parent = 0);
	~JointManager(){};
	
	void unsetFrame();

public Q_SLOTS:
	void handleFieldChanged(PosVelEffView::Field, std::string jointName);
	void handleChangeForInverse(std::string jointName, PosVelEffView::Field field, double value);
	
	void handlePerspectiveUpdate(const joint_perspective::JointPerspective &perspective);

private:
	void updateFrame();
	void initGUI(const joint_perspective::JointPerspective &perspective);
	
	// Find joint jointName in jointList and create view for it. Put it on layout
	// If jointName was not found, put warning on layout
	void findAndPutView(std::string jointName, std::string label, int row, BasicSmallView::Alignment alignment, BasicSmallView::Type type, bool shiftMirrored);
	PosVelEffView* findView(const std::string &jointName);
	void dynapedTransform(bool flag);
	void adultTransform();
	void setMotionFromView(PosVelEffView* view);
	void createHeaderLabels(BasicSmallView::Alignment alignment, int row);
	
private:
	std::vector<PosVelEffView*> jointViews;
	QGridLayout *jointsLayout;
	std::string m_perspective_name;
};


#endif