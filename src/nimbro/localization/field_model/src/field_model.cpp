// Model for the soccer field
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <field_model/field_model.h>
#include <boost/concept_check.hpp>
#include <ros/node_handle.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <ros/package.h>

namespace field_model
{

FieldModel* FieldModel::m_instance = 0;

inline double picut(double a)
{
	while (a > M_PI)
		a -= 2.0 * M_PI;
	while (a < -M_PI)
		a += 2.0 * M_PI;
	return a;
}

WorldObject::WorldObject(WorldObject::Type type, const Eigen::Vector3d& pose) : m_type(type), m_pos(pose)
{
}

WorldObject::WorldObject(WorldObject::Type type, double x, double y, double t) : m_type(type)
{
	m_pos << x, y, t;
}

WorldObject::WorldObject(WorldObject::Type type, const PointsType& points) : m_type(type), m_pos(0.0, 0.0, 0.0), m_points(points)
{
}

void WorldObject::setPose(const Eigen::Vector3d& pose)
{
	m_pos = pose;
}

WorldObject WorldObject::mirrorX() const
{
	WorldObject ret;

	ret.m_type = m_type;
	ret.m_pos << -m_pos.x(), m_pos.y(), picut(M_PI - m_pos.z());

	for (size_t i = 0; i < m_points.size(); ++i)
		ret.m_points.push_back(Eigen::Vector2d(-m_points[i].x(), m_points[i].y()));

	return ret;
}

WorldObject WorldObject::mirrorY() const
{
	WorldObject ret;

	ret.m_type = m_type;
	ret.m_pos << m_pos.x(), -m_pos.y(), -m_pos.z();

	for (size_t i = 0; i < m_points.size(); ++i)
		ret.m_points.push_back(Eigen::Vector2d(m_points[i].x(), -m_points[i].y()));

	return ret;
}

const std::string FieldModel::FieldTypeName[FieldModel::NumFieldTypes] = {
	"unknown",
	"teensize",
	"kidsize",
	"bonn"
};

FieldModel::FieldModel()
 : m_attEstMagCalibX("/nimbro_op_interface/attEstMagCalib/x", -1.5, 0.01, 1.5, 1.0) // Note: This should be shadowing the corresponding parameter in nimbro_op_interface
 , m_attEstMagCalibY("/nimbro_op_interface/attEstMagCalib/y", -1.5, 0.01, 1.5, 0.0) // Note: This should be shadowing the corresponding parameter in nimbro_op_interface
{
	std::string fieldModelPath = ros::package::getPath("launch") + "/config/field_model.yaml";
	ROS_INFO("Field model: %s", fieldModelPath.c_str());

	try
	{
		YAML::Node fieldConfig = YAML::LoadFile(fieldModelPath);

		if (fieldConfig["field_type"])
		{
			std::string type = fieldConfig["field_type"].as<std::string>();
			int id = UnknownField;
			for(; id < NumFieldTypes; id++)
			{
				if(type == FieldTypeName[id])
					break;
			}
			if(id <= UnknownField || id >= NumFieldTypes)
			{
				id = TeenSizeField;
				ROS_WARN("Field type is unknown => Using %s by default...", FieldTypeName[id].c_str());
			}
			ROS_INFO("Loading field model for field type '%s'", FieldTypeName[id].c_str());
			m_type = (FieldType) id;
			m_length = fieldConfig[type]["length"].as<float>();
			m_width = fieldConfig[type]["width"].as<float>();
			m_boundary = fieldConfig[type]["boundary"].as<float>();
			m_centerCircleDiameter = fieldConfig[type]["centerCircleDiameter"].as<float>();
			m_goalWidth = fieldConfig[type]["goalWidth"].as<float>();
			m_goalAreaWidth = fieldConfig[type]["goalAreaWidth"].as<float>();
			m_goalAreaDepth = fieldConfig[type]["goalAreaDepth"].as<float>();
			m_penaltyMarkerDist = fieldConfig[type]["penaltyMarkerDist"].as<float>();
			m_ballDiameter = fieldConfig[type]["ballDiameter"].as<float>();
			m_borderTop = fieldConfig[type]["borderTop"].as<float>();
			m_borderBottom = fieldConfig[type]["borderBottom"].as<float>();
			m_borderLeft = fieldConfig[type]["borderLeft"].as<float>();
			m_borderRight = fieldConfig[type]["borderRight"].as<float>();
		}
		else
			throw std::runtime_error("No 'field_type' parameter is specified in the field model config file!");
	}
	catch(std::runtime_error& ex)
	{
		ROS_FATAL("Failed to load and parse field model config file: %s", ex.what());
		exit(1);
	}
	catch(...)
	{
		ROS_FATAL("Failed to load and parse field model config file: Unknown error");
		exit(1);
	}

	double hw = m_width / 2.0;
	double hl = m_length / 2.0;
	WorldObject::PointsType points;

	// Goals & posts
	addObject(WorldObject::Type_Goal, -hl, 0.0, 0.0, MirrorX);
	addObject(WorldObject::Type_GoalPost, -hl, m_goalWidth / 2.0, 0.0, MirrorAll);

	// Center circle
	addObject(WorldObject::Type_Circle, 0.0, 0.0, 0.0, NoMirror);

	// Penalty markers
	addObject(WorldObject::Type_XMarker, -hl + m_penaltyMarkerDist, 0.0, 0.0, MirrorX);

	// Center line
	points.clear();
	points.push_back(Eigen::Vector2d(0.0, hw));
	points.push_back(Eigen::Vector2d(0.0, -hw));
	addLine(points, NoMirror);

	// Side lines
	points.clear();
	points.push_back(Eigen::Vector2d(-hl, hw));
	points.push_back(Eigen::Vector2d(hl, hw));
	addLine(points, MirrorY);

	// Goal lines
	points.clear();
	points.push_back(Eigen::Vector2d(-hl, hw));
	points.push_back(Eigen::Vector2d(-hl, -hw));
	addLine(points, MirrorX);

	// Goal area lines (long line)
	points.clear();
	points.push_back(Eigen::Vector2d(-hl + m_goalAreaDepth, m_goalAreaWidth / 2.0));
	points.push_back(Eigen::Vector2d(-hl + m_goalAreaDepth, -m_goalAreaWidth / 2.0));
	addLine(points, MirrorX);

	// Goal area lines (small line)
	points.clear();
	points.push_back(Eigen::Vector2d(-hl, m_goalAreaWidth / 2.0));
	points.push_back(Eigen::Vector2d(-hl + m_goalAreaDepth, m_goalAreaWidth / 2.0));
	addLine(points, MirrorAll);

	// Corner L Xings (theta is the half angle between the two lines)
	addObject(WorldObject::Type_LineXingL, -hl, hw, -M_PI / 4.0, MirrorAll);

	// Goal area L Xings (theta is the half angle between the two lines)
	addObject(WorldObject::Type_LineXingL, -hl + m_goalAreaDepth, m_goalAreaWidth / 2.0, -3.0 * M_PI / 4.0, MirrorAll);

	// Goal area T Xings (theta points along the central line)
	addObject(WorldObject::Type_LineXingT, -hl, m_goalAreaWidth / 2.0, 0.0, MirrorAll);
	addObject(WorldObject::Type_LineXingT, 0.0, -hw, MirrorY);

	// Central X Xings
	addObject(WorldObject::Type_LineXingX, 0.0, m_centerCircleDiameter / 2.0, 0.0, MirrorY);

	// Magnetic orientation
	m_magneticHeading = addObject(WorldObject::Type_MagneticHeading, NAN, NAN, 0.0, NoMirror);
	boost::function<void()> updateCallback = boost::bind(&FieldModel::updateMagneticHeading, this);
	m_attEstMagCalibX.setCallback(boost::bind(updateCallback));
	m_attEstMagCalibY.setCallback(boost::bind(updateCallback));
	updateMagneticHeading(); // Initialises m_heading...
}

WorldObject* FieldModel::addObject(WorldObject::Type type, double x, double y, double t, int flags)
{
	std::vector<WorldObject>* list = &m_objects[type];

	WorldObject obj(type, x, y, t);
	list->push_back(obj);

	WorldObject* ret = &list->back();

	if (flags & MirrorX)
		list->push_back(obj.mirrorX());

	if (flags & MirrorY)
		list->push_back(obj.mirrorY());

	if ((flags & MirrorX) && (flags & MirrorY))
		list->push_back(obj.mirrorX().mirrorY());

	return ret;
}

void FieldModel::addLine(const WorldObject::PointsType& points, int flags)
{
	std::vector<WorldObject>* list = &m_objects[WorldObject::Type_FieldLine];

	WorldObject obj(WorldObject::Type_FieldLine, points);
	list->push_back(obj);

	if (flags & MirrorX)
		list->push_back(obj.mirrorX());

	if (flags & MirrorY)
		list->push_back(obj.mirrorY());

	if ((flags & MirrorX) && (flags & MirrorY))
		list->push_back(obj.mirrorX().mirrorY());
}

void FieldModel::updateMagneticHeading()
{
	m_heading = atan2(m_attEstMagCalibY(), m_attEstMagCalibX()); // This is the angle from the field's positive x-axis to the direction of the magnetic field, measured in a CCW direction
	m_magneticHeading->setPose(Eigen::Vector3d(NAN, NAN, m_heading));
}

double FieldModel::magneticHeading() const
{
	return m_heading;
}

FieldModel* FieldModel::getInstance()
{
	if (!m_instance)
		m_instance = new FieldModel;

	return m_instance;
}

}
