#include <trajectory_editor/spaces/inversespace.h>

#include <ros/package.h>
#include <ros/console.h>

#include <QSpacerItem>

#include <math.h>

using namespace gait;
using namespace std;

InverseSpace::InverseSpace(const std::vector<std::string> &jointList, QWidget *parent) 
: BasicSpace(jointList, parent)
{
	m_inverse_pose = new InversePose();
	m_inverse_pose->setLinkLengths(0.2, 0.2);
	
	m_temp_inverse_pose = new InversePose();
	m_temp_inverse_pose->setLinkLengths(0.2, 0.2);
	
	m_limit_inverse = new QCheckBox();
	m_limit_inverse->setText("Change 1 DOF only:");
	m_limit_inverse->setChecked(false);
	
	QLabel *epsilon_label = new QLabel("  Epsilon:");
	
	m_epsilon_spin = new QDoubleSpinBox();
	m_epsilon_spin->setMinimum(0);
	m_epsilon_spin->setMaximum(1);
	m_epsilon_spin->setSingleStep(0.005);
	m_epsilon_spin->setDecimals(3);
	m_epsilon_spin->setToolTip("Epsilon");
	m_epsilon_spin->installEventFilter(this);
	m_epsilon_spin->setValue(0.005);
	
	QSpacerItem *spacer = new QSpacerItem(0,0, QSizePolicy::Expanding, QSizePolicy::Expanding);
	
	// Set up layouts
	m_main_layout    = new QVBoxLayout();
	m_epsilon_layout = new QHBoxLayout();
	m_joint_layout   = new QGridLayout();
	
	m_epsilon_layout->addWidget(m_limit_inverse);
	m_epsilon_layout->addWidget(epsilon_label);
	m_epsilon_layout->addWidget(m_epsilon_spin);
	m_epsilon_layout->addItem(spacer);
	
	m_main_layout->addLayout(m_epsilon_layout);
	m_main_layout->addLayout(m_joint_layout);
	
	initViews();
	
	setLayout(m_main_layout);
	
	ROS_INFO("initialized successfully");
}

InverseSpace::~InverseSpace()
{
	delete m_inverse_pose;
	delete m_temp_inverse_pose;
}

void InverseSpace::handlePerspectiveUpdate(const joint_perspective::JointPerspective& perspective)
{
	setEnabled(perspective.m_inverse_space_allowed);
	m_inverse_pose->setLinkLengths(perspective.m_link_length, perspective.m_link_length);
	m_temp_inverse_pose->setLinkLengths(perspective.m_link_length, perspective.m_link_length);
}

void InverseSpace::getInverseLimits(bool& limit, double& epsilon)
{
	limit = m_limit_inverse->isChecked();
	epsilon = m_epsilon_spin->value();
}

void InverseSpace::updateFrame()
{
	if(!m_current_frame || isEnabled() == false)
		return;
	
	JointPose jointPose = p.getJointPose(m_current_frame, m_joint_list);
	
	m_inverse_pose->leftLeg.setFromJointPose(jointPose.leftLeg);
	m_inverse_pose->rightLeg.setFromJointPose(jointPose.rightLeg);
	
	// Update left leg
	// Position
	updatePosition("left_leg_x", m_inverse_pose->leftLeg.footPos.x());
	updatePosition("left_leg_y", m_inverse_pose->leftLeg.footPos.y());
	updatePosition("left_leg_z", m_inverse_pose->leftLeg.footPos.z());
	
	// Rotation
	motionfile::PoseConverter::Angles left = motionfile::PoseConverter::anglesFromQuaternion(m_inverse_pose->leftLeg.footRot);
	
	updatePosition("left_foot_yaw", left.yaw);
	updatePosition("left_foot_pitch", left.pitch);
	updatePosition("left_foot_roll", left.roll);

	// Update right leg
	// Position
	updatePosition("right_leg_x", m_inverse_pose->rightLeg.footPos.x());
	updatePosition("right_leg_y", m_inverse_pose->rightLeg.footPos.y());
	updatePosition("right_leg_z", m_inverse_pose->rightLeg.footPos.z());
	
	// Rotation
	motionfile::PoseConverter::Angles right = motionfile::PoseConverter::anglesFromQuaternion(m_inverse_pose->rightLeg.footRot);
	
	updatePosition("right_foot_yaw", right.yaw);
	updatePosition("right_foot_pitch", right.pitch);
	updatePosition("right_foot_roll", right.roll);
	
	updatePositionFromUI(*m_inverse_pose);
	updateRotationFromUI(*m_inverse_pose);
}

void InverseSpace::updatePosition(std::string jointName, double pos)
{
	for(unsigned i = 0; i < m_views.size(); i++)
	{
		PositionView *view = m_views.at(i);
		
		if(view->jointName == jointName) // Update position
		{
			view->setPosition(pos);
			break;
		}
	}
}

void InverseSpace::updatePositionFromUI(gait::InversePose &inverse_pose)
{
	// Left leg
	inverse_pose.leftLeg.footPos.x() = getPos("left_leg_x");
	inverse_pose.leftLeg.footPos.y() = getPos("left_leg_y");
	inverse_pose.leftLeg.footPos.z() = getPos("left_leg_z");

	// Right leg
	inverse_pose.rightLeg.footPos.x() = getPos("right_leg_x");
	inverse_pose.rightLeg.footPos.y() = getPos("right_leg_y");
	inverse_pose.rightLeg.footPos.z() = getPos("right_leg_z");
}

void InverseSpace::updateRotationFromUI(gait::InversePose &inverse_pose)
{
	motionfile::PoseConverter::Angles angles;
	
	// Update Left foot
	angles.yaw = getPos("left_foot_yaw");
	angles.pitch = getPos("left_foot_pitch");
	angles.roll = getPos("left_foot_roll");
		
	inverse_pose.leftLeg.footRot = motionfile::PoseConverter::quaternionFromAngles(angles);
	
	// Update Right foot
	angles.yaw = getPos("right_foot_yaw");
	angles.pitch = getPos("right_foot_pitch");
	angles.roll = getPos("right_foot_roll");
	
	inverse_pose.rightLeg.footRot = motionfile::PoseConverter::quaternionFromAngles(angles);
}

double InverseSpace::getPos(std::string name)
{
	for(unsigned i = 0; i < m_views.size(); i++)
	{
		PositionView *view = m_views.at(i);
		
		if(view->jointName == name)
			return view->getPosition();
	}
	
	return 0;
}

void InverseSpace::handlePositionChanged(const std::string joint_name, const double prev, const double current)
{
	if(!m_current_frame)
		return;
	
	if(m_limit_inverse->isChecked()) // Chenge 1 DOF only
	{
		// Some preliminary conversions
		JointPose joint_pose = p.getJointPose(m_current_frame, m_joint_list);
	
		m_temp_inverse_pose->leftLeg.setFromJointPose(joint_pose.leftLeg);
		m_temp_inverse_pose->rightLeg.setFromJointPose(joint_pose.rightLeg);
		
		updatePositionFromUI(*m_temp_inverse_pose);
		updateRotationFromUI(*m_temp_inverse_pose);
		
		joint_pose.leftLeg.setFromInversePose(m_temp_inverse_pose->leftLeg);
		joint_pose.rightLeg.setFromInversePose(m_temp_inverse_pose->rightLeg);
		
		m_temp_inverse_pose->leftLeg.setFromJointPose(joint_pose.leftLeg);
		m_temp_inverse_pose->rightLeg.setFromJointPose(joint_pose.rightLeg);
		
		// Check if this change affects joints other than 'joint_name'
		if(!motionfile::PoseConverter::equalExcept(*m_inverse_pose, *m_temp_inverse_pose, joint_name, m_epsilon_spin->value()))
		{
			updatePosition(joint_name, prev);
			return;
		}
	}
	
	updatePositionFromUI(*m_inverse_pose);
	updateRotationFromUI(*m_inverse_pose);
		
	p.updateFrame(*m_inverse_pose, m_current_frame, m_joint_list);
	
	updateRobot();
	setFrame(m_current_frame);
}

void InverseSpace::initViews()
{
	createHeaderLabels();
	
	// Init joints
	int row = 1;
	
	findAndPutView("left_leg_x", "Leg X", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_leg_x", "Leg X", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	findAndPutView("left_leg_y", "Leg Y", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_leg_y", "Leg Y", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	findAndPutView("left_leg_z", "Leg Z", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_leg_z", "Leg Z", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	for(int i = 0; i < 3; i++) // gap between blocks
		m_joint_layout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView("left_foot_yaw", "Foot Yaw", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_foot_yaw", "Foot Yaw", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	findAndPutView("left_foot_pitch", "Foot Pitch", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_foot_pitch", "Foot Pitch", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	findAndPutView("left_foot_roll", "Foot Roll", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_foot_roll", "Foot Roll", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	// Add spacers
	row++;
	for(int i = 0; i < 3; i++) 
		m_joint_layout->addItem(new QSpacerItem(0,0, QSizePolicy::Minimum, QSizePolicy::Expanding), row, i); 
	
	m_joint_layout->setMargin(0);
	m_joint_layout->setSpacing(0);
	m_joint_layout->setContentsMargins(0,0,0,0);
}

void InverseSpace::findAndPutView(std::string jointName, std::string label, int row
			, PositionView::Alignment alignment, PositionView::Type type)
{

	PositionView *view = new PositionView(alignment, type, jointName, this);
	connect(view, SIGNAL(positionChanged(std::string,double,double)), this, SLOT(handlePositionChanged(std::string,double,double)));
			
	if(alignment == PositionView::LEFT)
		m_joint_layout->addWidget(view, row, 0);
	else
		m_joint_layout->addWidget(view, row, 2);
			
	QLabel *nameLabel = new QLabel();
	nameLabel->setAlignment(Qt::AlignCenter);
	nameLabel->setText(QString::fromStdString(label));

	m_joint_layout->addWidget(nameLabel, row, 1);	
	m_views.push_back(view);
	return;
}

void InverseSpace::createHeaderLabels()
{
	QLabel *jointsLabel = new QLabel("Joint");
	QLabel *positionLabel = new QLabel("Value");
	QLabel *positionLabel2 = new QLabel("Value");
	
	QFont font = jointsLabel->font();
	font.setBold(true);
	
	positionLabel->setFont(font);
	positionLabel2->setFont(font);
	jointsLabel->setFont(font);
	
	jointsLabel->setAlignment(Qt::AlignCenter);
	positionLabel->setAlignment(Qt::AlignCenter);
	positionLabel2->setAlignment(Qt::AlignCenter);
	
	m_joint_layout->addWidget(positionLabel, 0, 0);
	m_joint_layout->addWidget(jointsLabel, 0, 1);
	m_joint_layout->addWidget(positionLabel2, 0, 2);
}