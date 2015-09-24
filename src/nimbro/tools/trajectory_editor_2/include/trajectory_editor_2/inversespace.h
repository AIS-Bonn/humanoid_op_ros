#ifndef INVERSESPACE_H
#define INVERSESPACE_H

// Widget to edit joints in Inverse Space
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QWidget>
#include <QObject>
#include <QFrame>
#include <QLabel>
#include <QGridLayout>

#include <gait/util/gait_inverse_pose.h>
#include <boost/shared_ptr.hpp>
#include <motion_file/motionfile.h>
#include <Eigen/Geometry>

#include <trajectory_editor_2/positionview.h>

class InverseSpace : public QWidget
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;
	
	struct Angles
	{
		double yaw;
		double roll;
		double pitch;
	};
	
	InverseSpace(const std::vector<std::string> &jointList, QWidget *parent = 0);
	~InverseSpace();
	
	void setFrame(KeyframePtr frame);
	void updateJointList(const std::vector<std::string>& jointList);

Q_SIGNALS:
	void frameDataChanged();
	
private Q_SLOTS:
	void handlePositionChanged();
	
private:
	void test();
	void initViews();
	void updateFrame();
	
	void updatePosition(std::string jointName, double pos);
	void updatePositionFromUI();
	void updateRotationFromUI();

	InverseSpace::Angles anglesFromQuaternion(Eigen::Quaterniond q);
	Eigen::Quaterniond quaternionFromAngles(InverseSpace::Angles angles);
	
	double getPos(std::string name);
	int  indexToName(std::string name);
	
	// Find joint jointName in jointList and create view for it. Put it on layout
	// If jointName was not found, put warning on layout
	void findAndPutView(std::string jointName, std::string label, int row
	, PositionView::Alignment alignment, PositionView::Type type);
	
	void createHeaderLabels();
	QFrame* createLine();
	
private:
	KeyframePtr frame;
	gait::InversePose *inversePose;
	
	std::vector<std::string> joints;
	std::vector<PositionView*> views;
	
	QGridLayout *layout;
};

#endif // INVERSESPACE_H