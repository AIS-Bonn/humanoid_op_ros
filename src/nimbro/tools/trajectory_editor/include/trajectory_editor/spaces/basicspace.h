// Base class for space view
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef BASICSPACE_H
#define BASICSPACE_H

#include <QWidget>
#include <QFrame>
#include <QLayoutItem>
#include <QLayout>

#include <boost/shared_ptr.hpp>
#include <motion_file/motionfile.h>

#include <trajectory_editor/jointperspective.h>

class BasicSpace : public QWidget
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	BasicSpace(const std::vector<std::string> &joint_list, QWidget *parent = 0);
	~BasicSpace(){};

public Q_SLOTS:
	void setFrame(KeyframePtr frame);
	void updateJointList(const std::vector<std::string>& jointList);
	
	virtual void handlePerspectiveUpdate(const joint_perspective::JointPerspective &perspective){};
	virtual void handleApplyRulePart(KeyframePtr frame, const motionfile::RulePart &part, double delta){};

Q_SIGNALS:
	void updateRobot(); // Emitted when robot view has to be updated
	
protected:
	virtual void updateFrame();
	QFrame* createLine();
	void clearlayout(QLayout *layout);
	
protected:
	KeyframePtr m_current_frame;
	std::vector<std::string> m_joint_list;
};


#endif