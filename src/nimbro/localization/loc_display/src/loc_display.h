// Localization display for RViz
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LOC_DISPLAY_H
#define LOC_DISPLAY_H

#include <rviz/display.h>
#include <rviz/properties/tf_frame_property.h>
#include <field_model/field_model.h>
#include <OGRE/OgreManualObject.h>

namespace loc_display
{

class LocDisplay : public rviz::Display
{
public:
	LocDisplay();
	virtual ~LocDisplay();

	virtual void onInitialize();

	virtual void update(float wall_dt, float ros_dt);
private:
	void addMarkers(field_model::WorldObject::Type type);

	rviz::TfFrameProperty* m_frame_property;

	Ogre::ManualObject* m_manual_object;
};

}

#endif
