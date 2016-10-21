// Widget to edit joints in Inverse Space
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#ifndef INVERSESPACE_H
#define INVERSESPACE_H

#include <QLabel>
#include <QLayout>
#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QCheckBox>

#include <gait/util/gait_inverse_pose.h>
#include <Eigen/Geometry>

#include <trajectory_editor/spaces/positionview.h>
#include <trajectory_editor/spaces/basicspace.h>
#include <motion_file/poseconverter.h>

class InverseSpace : public BasicSpace
{
Q_OBJECT
public:
	InverseSpace(const std::vector<std::string> &jointList, QWidget *parent = 0);
	~InverseSpace();
	
public Q_SLOTS:
	void handlePerspectiveUpdate(const joint_perspective::JointPerspective &perspective);
	void getInverseLimits(bool &limit, double &epsilon); // Returns values from GUI
	
private Q_SLOTS:
	void handlePositionChanged(const std::string joint_name, const double prev, const double current);
	
private:
	void initViews();
	void updateFrame();
	
	void updatePosition(std::string jointName, double pos);
	void updatePositionFromUI(gait::InversePose &inverse_pose);
	void updateRotationFromUI(gait::InversePose &inverse_pose);
	
	double getPos(std::string name);
	
	// Find joint jointName in jointList and create view for it. Put it on layout
	// If jointName was not found, put warning on layout
	void findAndPutView(std::string jointName, std::string label, int row
	, PositionView::Alignment alignment, PositionView::Type type);
	
	void createHeaderLabels();
	
private:
	std::vector<PositionView*> m_views;
	QVBoxLayout *m_main_layout;
	QHBoxLayout *m_epsilon_layout;
	QGridLayout *m_joint_layout;
	
	QCheckBox      *m_limit_inverse; // If true, the changes in inverse space are only applied, if only one inverse dimension changes
	QDoubleSpinBox *m_epsilon_spin;
	
	gait::InversePose *m_inverse_pose;
	gait::InversePose *m_temp_inverse_pose;
	
	motionfile::PoseConverter p;
};

#endif // INVERSESPACE_H