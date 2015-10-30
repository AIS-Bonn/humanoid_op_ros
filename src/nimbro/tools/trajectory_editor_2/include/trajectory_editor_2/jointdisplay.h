//Displays joints of loaded frame
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef JOINTDISPLAY_H
#define JOINTDISPLAY_H

#include <QAbstractItemModel>
#include <qitemdelegate.h>
#include <qstyleditemdelegate.h>
#include <QLabel>
#include <QSpinBox>
#include <QWidget>
#include <QFrame>
#include <QGridLayout>
#include <QVBoxLayout>

#include <boost/shared_ptr.hpp>
#include <boost/concept_check.hpp>

#include <gait/util/gait_joint_pose.h>

#include <motion_file/motionfile.h>
#include <trajectory_editor_2/posveleffview.h>

class JointManager : public QWidget
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;

	JointManager(const std::vector<std::string> &modelJointList, QWidget *parent = 0);
	~JointManager(){};

	void setFrame(KeyframePtr frame);
	void unsetFrame();
	void updateJointList(const std::vector<std::string>& jointList);
	
	int  indexToName(std::string name);

public Q_SLOTS:
	void updateFrame();
	
	void handlePositionChanged();
	void handleVelocityChanged();
	void handleEffortChanged();
	
	void handleChangeForID(int id, PosVelEffView::Field field, float value);

Q_SIGNALS:
	void frameDataChanged();

private:
	void initFrames(std::vector<std::string> jointList);
	
	// Find joint jointName in jointList and create view for it. Put it on layout
	// If jointName was not found, put warning on layout
	void findAndPutView(std::vector<std::string>  &jointList, std::string jointName, std::string label
						, int row, PosVelEffView::Alignment alignment, int id, bool shiftMirrored);
	void createHeaderLabels(PosVelEffView::Alignment alignment, int row);
	QFrame* createLine();
	
private:
	KeyframePtr m_currentFrame;
	std::vector<std::string> m_jointList;
	std::vector<PosVelEffView*> jointViews;
	
	QGridLayout *jointsLayout;
};


#endif