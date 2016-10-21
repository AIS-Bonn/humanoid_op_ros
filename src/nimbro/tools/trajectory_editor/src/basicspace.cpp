#include <trajectory_editor/spaces/basicspace.h>

BasicSpace::BasicSpace(const std::vector< std::string >& joint_list, QWidget *parent)
	: QWidget(parent)
	, m_current_frame()
{
	m_joint_list = joint_list;
}

void BasicSpace::setFrame(BasicSpace::KeyframePtr frame)
{
	m_current_frame = frame;
	updateFrame();
}

void BasicSpace::updateJointList(const std::vector< std::string >& jointList)
{
	m_joint_list = jointList;
}

QFrame* BasicSpace::createLine()
{
	QFrame *line = new QFrame();
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);
	line->setMidLineWidth(2);
	
	return line;
}

void BasicSpace::clearlayout(QLayout* layout)
{
	if(layout != NULL)
	{
		QLayoutItem* item;
		while ( (item = layout->takeAt(0)) != NULL)
		{
			delete item->widget();
			delete item;
		}
	}
}

void BasicSpace::updateFrame()
{

}

