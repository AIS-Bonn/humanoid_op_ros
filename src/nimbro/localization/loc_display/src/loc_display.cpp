// Localization display for RViz
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "loc_display.h"

#include <pluginlib/class_list_macros.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/display_context.h>
#include <ros/package.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>

#include <field_model/field_model.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMaterialManager.h>

#include <boost/foreach.hpp>

namespace loc_display
{

LocDisplay::LocDisplay()
{
	ROS_INFO("Adding ogre path: %s", (ros::package::getPath("loc_display") + "/ogre/scripts").c_str());
	Ogre::ResourceGroupManager::getSingleton().addResourceLocation(ros::package::getPath("loc_display") + "/ogre/scripts", "FileSystem", "loc_display");
	Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("loc_display");

	m_frame_property = new rviz::TfFrameProperty(
		"Reference Frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
		"The TF frame the field will use for its origin.",
		this, 0, true
	);
}

LocDisplay::~LocDisplay()
{
}

void LocDisplay::onInitialize()
{
	rviz::Display::onInitialize();

	m_frame_property->setFrameManager(context_->getFrameManager());

	field_model::FieldModel* field = field_model::FieldModel::getInstance();

	rviz::Shape* plane = new rviz::Shape(rviz::Shape::Cube, scene_manager_, scene_node_);

	const double MARGIN = 0.5;
	plane->setScale(Ogre::Vector3(field->length() + 2.0*MARGIN, field->width() + 2.0*MARGIN, 0.01));
	plane->setPosition(Ogre::Vector3(0.0, 0.0, -0.005));
	Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName("Soccer/Field", "loc_display");
	if(mat.isNull())
		ROS_ERROR("Could not find field material");

	plane->getEntity()->setMaterial(mat);


	const std::vector<field_model::WorldObject>& lines = field->objects(field_model::WorldObject::Type_FieldLine);

	for(size_t i = 0; i < lines.size(); ++i)
	{
		const field_model::WorldObject& obj = lines[i];

		Eigen::Vector2d mid = (obj.points()[0] + obj.points()[1]) / 2.0;
		Eigen::Vector2d dir = obj.points()[1] - obj.points()[0];

		Ogre::Quaternion rot;
		rot.FromAngleAxis(Ogre::Radian(atan2(dir.y(), dir.x())), Ogre::Vector3::UNIT_Z);

		rviz::Shape* line = new rviz::Shape(rviz::Shape::Cube, scene_manager_, scene_node_);
		line->setPosition(Ogre::Vector3(mid.x(), mid.y(), -0.005+0.001));
		line->setOrientation(rot);
		line->setScale(Ogre::Vector3(dir.norm() + 0.05, 0.05, 0.01));
		line->getEntity()->setMaterialName("Soccer/Field/Line", "loc_display");
	}

	const std::vector<field_model::WorldObject>& posts = field->objects(field_model::WorldObject::Type_GoalPost);
	Ogre::Quaternion post_rot;
	post_rot.FromAngleAxis(Ogre::Radian(M_PI/2.0), Ogre::Vector3::UNIT_X);
	for(size_t i = 0; i < posts.size(); ++i)
	{
		const field_model::WorldObject& obj = posts[i];

		const double HEIGHT = 1.0;

		rviz::Shape* post = new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, scene_node_);
		post->setPosition(Ogre::Vector3(obj.pose().x(), obj.pose().y(), HEIGHT/2.0));
		post->setScale(Ogre::Vector3(0.1, HEIGHT, 0.1));
		post->getEntity()->setMaterialName("Soccer/Field/GoalPost", "loc_display");
		post->setOrientation(post_rot);
	}

	// Center circle
	int CIRCLE_LINE_COUNT = 200;
	double r = field->centerCircleDiameter()/2.0;
	for(int i = 0; i < CIRCLE_LINE_COUNT; ++i)
	{
		rviz::Shape* line = new rviz::Shape(rviz::Shape::Cube, scene_manager_, scene_node_);

		double angle = i * 2.0 * M_PI / CIRCLE_LINE_COUNT;
		line->setPosition(Ogre::Vector3(cos(angle)*r, sin(angle)*r, -0.005+0.001));
		line->setScale(Ogre::Vector3(0.04, r * 2.0*M_PI / CIRCLE_LINE_COUNT, 0.01));

		Ogre::Quaternion rot;
		rot.FromAngleAxis(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z);
		line->setOrientation(rot);

		line->getEntity()->setMaterialName("Soccer/Field/Line", "loc_display");
	}

	addMarkers(field_model::WorldObject::Type_GoalPost);
	addMarkers(field_model::WorldObject::Type_LineXingL);
	addMarkers(field_model::WorldObject::Type_LineXingT);
	addMarkers(field_model::WorldObject::Type_LineXingX);
	addMarkers(field_model::WorldObject::Type_XMarker);

	// Coordinate system arrows
	const double ARROW_OFFSET = 0.1;
	const Ogre::Vector3 ARROW_ORIGIN(-field->length()/2.0 - MARGIN, -field->width()/2.0 - MARGIN - ARROW_OFFSET, 0.0);
	Ogre::Quaternion rot;

	rviz::Arrow* x_arrow = new rviz::Arrow(scene_manager_, scene_node_, field->length() + 2.0*MARGIN-0.3, 0.05, 0.3, 0.1);
	x_arrow->setPosition(ARROW_ORIGIN);
	rot.FromAngleAxis(Ogre::Radian(-M_PI/2.0), Ogre::Vector3::UNIT_Y);
	x_arrow->setOrientation(rot);
	x_arrow->setColor(1.0, 0.0, 0.0, 1.0);

// 	rviz::Arrow* y_arrow = new rviz::Arrow(scene_manager_, scene_node_, field->width());
// 	x_arrow->setPosition(ARROW_ORIGIN);
// 	rot.FromAngleAxis(Ogre::Radian(-M_PI/2.0), Ogre::Vector3::UNIT_Y);
// 	x_arrow->setOrientation(rot);

	// Table
	rviz::Shape* table = new rviz::Shape(rviz::Shape::Cube, scene_manager_, scene_node_);
	table->setScale(Ogre::Vector3(field->length() + 2.0 * MARGIN, 0.5, 0.6));
	table->setPosition(Ogre::Vector3(0.0, -field->width()/2.0 - MARGIN - 0.2 - 0.25, 0.3));
	table->setColor(0.2, 0.2, 0.2, 1.0);

	m_manual_object = scene_manager_->createManualObject();
	m_manual_object->setDynamic( true );
	scene_node_->attachObject( m_manual_object );

	scene_node_->setVisible(isEnabled());
}

void LocDisplay::addMarkers(field_model::WorldObject::Type type)
{
	const std::vector<field_model::WorldObject>& objs = field_model::FieldModel::getInstance()->objects(type);

	BOOST_FOREACH(const field_model::WorldObject& obj, objs)
	{
		rviz::Shape* cube = new rviz::Shape(rviz::Shape::Cube, scene_manager_, scene_node_);

		cube->setScale(Ogre::Vector3(0.05, 0.05, 0.05));
		cube->setPosition(Ogre::Vector3(obj.pose().x(), obj.pose().y(), 0.0));
		cube->getEntity()->setMaterialName("Soccer/Field/FloorMarker", "loc_display");
	}
}

void LocDisplay::update(float wall_dt, float ros_dt)
{
	QString qframe = m_frame_property->getFrame();
	std::string frame = qframe.toStdString();

	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if( context_->getFrameManager()->getTransform( frame, ros::Time(), position, orientation ))
	{
		scene_node_->setPosition( position + Ogre::Vector3(0, 0, -0.001) );
		scene_node_->setOrientation( orientation );
		setStatus( rviz::StatusProperty::Ok, "Transform", "Transform OK" );
	}
	else
	{
		std::string error;
		if( context_->getFrameManager()->transformHasProblems( frame, ros::Time(), error ))
		{
			setStatus( rviz::StatusProperty::Error, "Transform", QString::fromStdString( error ));
		}
		else
		{
			setStatus( rviz::StatusProperty::Error, "Transform",
					   "Could not transform from [" + qframe + "] to [" + fixed_frame_ + "]" );
		}
	}
}

}

PLUGINLIB_EXPORT_CLASS(loc_display::LocDisplay, rviz::Display)
