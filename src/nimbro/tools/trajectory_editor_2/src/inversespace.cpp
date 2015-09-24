#include <trajectory_editor_2/inversespace.h>
#include <trajectory_editor_2/poseconverter.h>

#include <ros/package.h>
#include <ros/console.h>

#include <math.h>

InverseSpace::InverseSpace(const std::vector<std::string> &jointList, QWidget *parent) : QWidget(parent)
{
	layout = new QGridLayout();
	setLayout(layout);
	
	inversePose = new InversePose();
	inversePose->setLinkLengths(1, 1);
	
	//test();
	
	this->joints = jointList;
	initViews();
}

void InverseSpace::test()
{
	JointPose jointPose;
	
	double value = 0;
	
	// Set left arm
	jointPose.leftArm.shoulderPitch = value;
	jointPose.leftArm.shoulderRoll =  value;
	jointPose.leftArm.elbowPitch =    value;
	
	// Set right arm
	jointPose.rightArm.shoulderPitch = value;
	jointPose.rightArm.shoulderRoll =  value;
	jointPose.rightArm.elbowPitch =    value;
	
	// Set left leg
	jointPose.leftLeg.anklePitch = value;
	jointPose.leftLeg.ankleRoll =  value;
	jointPose.leftLeg.hipPitch =   value;
	jointPose.leftLeg.hipRoll =    value;
	jointPose.leftLeg.hipYaw =     value;
	jointPose.leftLeg.kneePitch =  value;
	
	// Set right leg
	jointPose.rightLeg.anklePitch = value;
	jointPose.rightLeg.ankleRoll =  value;
	jointPose.rightLeg.hipPitch =   value;
	jointPose.rightLeg.hipRoll =    value;
	jointPose.rightLeg.hipYaw =     value;
	jointPose.rightLeg.kneePitch =  value;
	
	ROS_INFO("_");
	ROS_INFO("JOINT:");
	ROS_INFO("_");
	ROS_INFO("Left Ankle Pitch: %f", jointPose.leftLeg.anklePitch);
	ROS_INFO("Left Ankle Roll: %f", jointPose.leftLeg.ankleRoll);
	ROS_INFO("Left Hip Pitch: %f", jointPose.leftLeg.hipPitch);
	ROS_INFO("Left Hip Roll: %f", jointPose.leftLeg.hipRoll);
	ROS_INFO("Left Hip Yaw: %f", jointPose.leftLeg.hipYaw);
	ROS_INFO("Left Knee Pitch: %f", jointPose.leftLeg.kneePitch);
	ROS_INFO("_");
	
	InversePose inv;
	inv.setFromJointPose(jointPose);
	
	InverseSpace::Angles angles = anglesFromQuaternion(inv.leftLeg.footRot);
	
	ROS_INFO("_");
	ROS_INFO("INVERSE FORM JOINT:");
	ROS_INFO("_");
	ROS_INFO("Left Leg X: %f", inv.leftLeg.footPos.x());
	ROS_INFO("Left Leg Y: %f", inv.leftLeg.footPos.y());
	ROS_INFO("Left Leg Z: %f", inv.leftLeg.footPos.z());
	ROS_INFO("_");
	ROS_INFO("Yaw: %f", angles.yaw);
	ROS_INFO("Pitch: %f", angles.pitch);
	ROS_INFO("Roll: %f", angles.roll);
	ROS_INFO("_");
	
	JointPose p2;
	p2.setFromInversePose(inv);
	
	ROS_INFO("_");
	ROS_INFO("JOINT FORM INVERSE:");
	ROS_INFO("_");
	ROS_INFO("Left Ankle Pitch: %f", p2.leftLeg.anklePitch);
	ROS_INFO("Left Ankle Roll: %f", p2.leftLeg.ankleRoll);
	ROS_INFO("Left Hip Pitch: %f", p2.leftLeg.hipPitch);
	ROS_INFO("Left Hip Roll: %f", p2.leftLeg.hipRoll);
	ROS_INFO("Left Hip Yaw: %f", p2.leftLeg.hipYaw);
	ROS_INFO("Left Knee Pitch: %f", p2.leftLeg.kneePitch);
	ROS_INFO("_");
}

InverseSpace::~InverseSpace()
{
	delete inversePose;
}

void InverseSpace::setFrame(KeyframePtr frame)
{
	this->frame = frame;
	updateFrame();
}

void InverseSpace::updateJointList(const std::vector<std::string>& jointList)
{
	this->joints = jointList;
}

void InverseSpace::updateFrame()
{
	if(!frame)
		return;
	
	JointPose jointPose = PoseConverter::getJointPose(frame, joints);
	
	inversePose->leftLeg.setFromJointPose(jointPose.leftLeg);
	inversePose->rightLeg.setFromJointPose(jointPose.rightLeg);
	
	// Update left leg
	// Position
	updatePosition("left_leg_x", inversePose->leftLeg.footPos.x());
	updatePosition("left_leg_y", inversePose->leftLeg.footPos.y());
	updatePosition("left_leg_z", inversePose->leftLeg.footPos.z());
	
	// Rotation
	InverseSpace::Angles left = anglesFromQuaternion(inversePose->leftLeg.footRot);
	
	updatePosition("left_foot_yaw", left.yaw);
	updatePosition("left_foot_pitch", left.pitch);
	updatePosition("left_foot_roll", left.roll);

	// Update right leg
	// Position
	updatePosition("right_leg_x", inversePose->rightLeg.footPos.x());
	updatePosition("right_leg_y", inversePose->rightLeg.footPos.y());
	updatePosition("right_leg_z", inversePose->rightLeg.footPos.z());
	
	// Rotation
	InverseSpace::Angles right = anglesFromQuaternion(inversePose->rightLeg.footRot);
	
	updatePosition("right_foot_yaw", right.yaw);
	updatePosition("right_foot_pitch", right.pitch);
	updatePosition("right_foot_roll", right.roll);
}

InverseSpace::Angles InverseSpace::anglesFromQuaternion(Eigen::Quaterniond q)
{
	// Calculate pitch
	// wxyz
	double stheta = 2.0*(q.w()*q.y() - q.z()*q.x());
	stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
	double pitch = asin(stheta);

	// Calculate yaw and roll
	double ysq  = q.y()*q.y();
	double yaw  = atan2(q.w()*q.z()+q.x()*q.y(), 0.5-(ysq+q.z()*q.z()));
	double roll = atan2(q.w()*q.x()+q.y()*q.z(), 0.5-(ysq+q.x()*q.x()));
	
	InverseSpace::Angles angles;
	angles.pitch = pitch;
	angles.yaw   = yaw;
	angles.roll  = roll;
	
	return angles;
}

void InverseSpace::updatePosition(std::string jointName, double pos)
{
	for(unsigned i = 0; i < views.size(); i++)
	{
		PositionView *view = views.at(i);
		
		if(view->jointName == jointName) // Update position
		{
			view->setPosition(pos);
			break;
		}
	}
}

void InverseSpace::updatePositionFromUI()
{
	// Left leg
	inversePose->leftLeg.footPos.x() = getPos("left_leg_x");
	inversePose->leftLeg.footPos.y() = getPos("left_leg_y");
	inversePose->leftLeg.footPos.z() = getPos("left_leg_z");

	// Right leg
	inversePose->rightLeg.footPos.x() = getPos("right_leg_x");
	inversePose->rightLeg.footPos.y() = getPos("right_leg_y");
	inversePose->rightLeg.footPos.z() = getPos("right_leg_z");
}

void InverseSpace::updateRotationFromUI()
{
	InverseSpace::Angles angles;
	
	// Update Left foot
	angles.yaw = getPos("left_foot_yaw");
	angles.pitch = getPos("left_foot_pitch");
	angles.roll = getPos("left_foot_roll");
		
	inversePose->leftLeg.footRot = quaternionFromAngles(angles);
	
	// Update Right foot
	angles.yaw = getPos("right_foot_yaw");
	angles.pitch = getPos("right_foot_pitch");
	angles.roll = getPos("right_foot_roll");
	
	inversePose->rightLeg.footRot = quaternionFromAngles(angles);
}

Eigen::Quaterniond InverseSpace::quaternionFromAngles(InverseSpace::Angles angles)
{
	Eigen::Quaterniond q;
	double cpsi, spsi, cth, sth, cphi, sphi;

	// Halve the yaw, pitch and roll values (for calculation purposes only)
	angles.yaw   *= 0.5;
	angles.pitch *= 0.5;
	angles.roll  *= 0.5;

	// Precalculate the required sin and cos values
	cpsi = cos(angles.yaw);
	spsi = sin(angles.yaw);
	cth  = cos(angles.pitch);
	sth  = sin(angles.pitch);
	cphi = cos(angles.roll);
	sphi = sin(angles.roll);

	// Calculate the required quaternion components
	q.w() = cpsi*cth*cphi + spsi*sth*sphi;
	q.x() = cpsi*cth*sphi - spsi*sth*cphi;
	q.y() = cpsi*sth*cphi + spsi*cth*sphi;
	q.z() = spsi*cth*cphi - cpsi*sth*sphi;
	
	return q;
}

double InverseSpace::getPos(std::string name)
{
	for(unsigned i = 0; i < views.size(); i++)
	{
		PositionView *view = views.at(i);
		
		if(view->jointName == name)
			return view->getPosition();
	}
	
	return 0;
}
	
int InverseSpace::indexToName(std::string name)
{
	for (unsigned i = 0; i < joints.size(); i++)
	{
		if (joints[i] == name)
			return i;
	}
	return -1;
}

void InverseSpace::handlePositionChanged()
{
	if(!frame)
		return;
	
	updatePositionFromUI();
	updateRotationFromUI();
	
	PoseConverter::updateFrame(*inversePose, frame, joints);
	frameDataChanged();
	
	setFrame(frame);
}

void InverseSpace::findAndPutView(std::string jointName, std::string label, int row
			, PositionView::Alignment alignment, PositionView::Type type)
{

	PositionView *view = new PositionView(alignment, type, jointName, this);
	connect(view, SIGNAL(positionChanged()), this, SLOT(handlePositionChanged()));
			
	if(alignment == PositionView::LEFT)
		layout->addWidget(view, row, 0);
	else
		layout->addWidget(view, row, 2);
			
	QLabel *nameLabel = new QLabel();
	nameLabel->setAlignment(Qt::AlignCenter);
	nameLabel->setText(QString::fromStdString(label));

	layout->addWidget(nameLabel, row, 1);	
	views.push_back(view);
	return;
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
		layout->addWidget(createLine(), row, i); 
	row++;
	
	findAndPutView("left_foot_yaw", "Foot Yaw", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_foot_yaw", "Foot Yaw", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	findAndPutView("left_foot_pitch", "Foot Pitch", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_foot_pitch", "Foot Pitch", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	findAndPutView("left_foot_roll", "Foot Roll", row, PositionView::LEFT, PositionView::REGULAR);
	findAndPutView("right_foot_roll", "Foot Roll", row++, PositionView::RIGHT, PositionView::REGULAR);
	
	layout->setMargin(0);
	layout->setSpacing(0);
	layout->setContentsMargins(0,0,0,0);
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
	
	layout->addWidget(positionLabel, 0, 0);
	layout->addWidget(jointsLabel, 0, 1);
	layout->addWidget(positionLabel2, 0, 2);
}

QFrame* InverseSpace::createLine()
{
	QFrame *line = new QFrame();
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);
	line->setMidLineWidth(2);
	
	return line;
}