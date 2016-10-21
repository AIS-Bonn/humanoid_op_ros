#include <trajectory_editor/spaces/basicsmallview.h>

#include <QApplication>
#include <QGridLayout>
#include <QString>

#include <stdio.h>
#include <math.h>

BasicSmallView::BasicSmallView(Alignment alignment, Type type, std::string jointName, bool shiftMirrored, QWidget *parent) 
	: QWidget(parent)
{
	if(type == BasicSmallView::REGULAR)
	{
		min = -M_PI;
		max = M_PI;
	}
	else if(type == BasicSmallView::EXTENSION)
	{
		min = 0;
		max = 1;
	}
	else if(type == BasicSmallView::LEG)
	{
		min = -M_PI/2;
		max = M_PI/2;
	}
	else if(type == BasicSmallView::MINUS_ONE_PLUS_ONE)
	{
		min = -1;
		max = 1;
	}
	
	this->jointName = jointName;
	m_inverse_joint_name = determineInverseJoint(jointName);
	onShiftMirrored = shiftMirrored;
}

std::string BasicSmallView::determineInverseJoint(std::string joint)
{
	std::string inverse_joint = "";
	
	QString name = QString::fromStdString(joint);
	QStringList parts = name.split("_");
	
	if(parts.first() == "left") // Insert "right" instead of "left"
	{
		name.remove(0, 4);
		name.insert(0, QString("right"));
		
		return name.toStdString();
	}
	else if (parts.first() == "right") // Insert "left" instead of "right"
	{
		name.remove(0, 5);
		name.insert(0, QString("left"));
		
		return name.toStdString();
	}
	else
		return "";
}

void BasicSmallView::setUpLayout(std::vector<QWidget*> widgets, Alignment alignment)
{
	QGridLayout *layout = new QGridLayout(this);
	
	if(alignment == LEFT || alignment == NO_PAIR)
	{
		for(unsigned i = 0; i < widgets.size(); i++)
			layout->addWidget(widgets.at(i), 1, i);
	}
	else if(alignment == RIGHT)
	{
		for(unsigned i = 0; i < widgets.size(); i++)
			layout->addWidget(widgets.at(i), 1, widgets.size()-1-i);
	}

	layout->setContentsMargins(5,0,5,0);
	setLayout(layout);
	
	this->widgets = widgets;
	for(unsigned i = 0; i < widgets.size(); i++)
		widgets.at(i)->installEventFilter(this);
}

void BasicSmallView::setEnabled(bool enabled)
{
	for(unsigned i = 0; i < widgets.size(); i++)
		widgets.at(i)->setEnabled(enabled);
}

std::string BasicSmallView::getJointName()
{
	return jointName;
}

bool BasicSmallView::isShiftPressed()
{
	Qt::KeyboardModifiers modifier = QApplication::keyboardModifiers();
	return modifier == Qt::ShiftModifier;
}

bool BasicSmallView::withinRange(double value)
{
	return (value >= min && value <= max);
}

bool BasicSmallView::eventFilter(QObject *object, QEvent *event)
{
    if(event->type() == QEvent::Wheel) // Ignore wheel scrolling
	{
		for(unsigned i = 0; i < widgets.size(); i++)
		{
			if(object == widgets.at(i))
				return true;
		}
    }
    
    return false;
}

BasicSmallView::~BasicSmallView()
{
	
}