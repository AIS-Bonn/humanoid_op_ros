// Model for the soccer field
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef FIELD_MODEL_H
#define FIELD_MODEL_H

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <config_server/parameter.h>

namespace field_model
{

/**
 * Objects occuring in our soccer world.
 **/
class WorldObject
{
public:
	//! Object type
	enum Type
	{
		Type_Circle,      //!< Center circle
		Type_Goal,        //!< Goal (center position)
		Type_GoalPost,    //!< A single goal post
		Type_XMarker,     //!< One of the two penalty markers
		Type_FieldLine,   //!< Field line
		Type_LineXingT,   //!< T-crossing of two lines
		Type_LineXingX,   //!< X-crossing of two lines
		Type_LineXingL,   //!< L-crossing of two lines
		Type_MagneticHeading, //!< Magnetic heading towards X+
		NumTypes
	};

	typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > PointsType;

	WorldObject(Type type, const Eigen::Vector3d& pose);
	WorldObject(Type type, double x, double y, double t = 0.0);
	WorldObject(Type type, const PointsType& points);

	WorldObject mirrorX() const;
	WorldObject mirrorY() const;

	//! Object pose (x, y, theta)
	inline Eigen::Vector3d pose() const { return m_pos; }

	//! Points belonging to the object (e.g. line start and end)
	inline const PointsType& points() const { return m_points; }

	//! Object type
	inline Type type() const { return m_type; }

protected:
	void setPose(const Eigen::Vector3d& pose);

private:
	friend class FieldModel;

	WorldObject() {}

	Type m_type;
	Eigen::Vector3d m_pos;
	PointsType m_points;
};

/**
 * @brief Model of the soccer field
 *
 * All distances are given in meters.
 **/
class FieldModel
{
public:
	static FieldModel* getInstance();

	enum FieldType
	{
		UnknownField = 0,
		TeenSizeField,
		KidSizeField,
		BonnField,
		NumFieldTypes
	};
	static const std::string FieldTypeName[NumFieldTypes];
	static const std::string& fieldTypeName(FieldType type) { if(type >= UnknownField && type < NumFieldTypes) return FieldTypeName[type]; else return FieldTypeName[UnknownField]; }

	//! Field type
	inline FieldType type() const
	{ return m_type; }

	//! Field type name
	inline const std::string& typeName() const
	{ return fieldTypeName(m_type); }

	//! Field width (inside the lines)
	inline double width() const
	{ return m_width; }

	//! Field length (inside the lines)
	inline double length() const
	{ return m_length; }

	//! Field boundary (amount of green outside the field boundary)
	inline double boundary() const
	{ return m_boundary; }

	//! Goal width
	inline double goalWidth() const
	{ return m_goalWidth; }

	//! Width of the penalty area before each goal
	inline double goalAreaWidth() const
	{ return m_goalAreaWidth; }

	//! Depth of the penalty area before each goal
	inline double goalAreaDepth() const
	{ return m_goalAreaDepth; }

	inline double centerCircleDiameter() const
	{ return m_centerCircleDiameter; }

	//! Distance from the goal line to the penalty marker
	inline double penaltyMarkerDist() const
	{ return m_penaltyMarkerDist; }

	//! Diameter of the ball
	inline double ballDiameter() const
	{ return m_ballDiameter; }

	//! Border for the top(positive) part of the field
	inline double borderTop() const
	{ return m_borderTop; }

	//! Border for the bottom(negative) part of the field
	inline double borderBottom() const
	{ return m_borderBottom; }

	//! Border for the left(when looking to positive goal) part of the field
	inline double borderLeft() const
	{ return m_borderLeft; }

	//! Border for the right(when looking to positive goal) part of the field
	inline double borderRight() const
	{ return m_borderRight; }

	//! Objects for a specific WorldObject::Type
	inline const std::vector<WorldObject>& objects(WorldObject::Type type) const
	{ return m_objects[type]; }

	void setMagneticHeading(double heading);
	double magneticHeading() const;

private:
	FieldModel();

	enum MirrorFlag
	{
		NoMirror = 0,
		MirrorX = (1 << 0),
		MirrorY = (1 << 1),
		MirrorAll = MirrorX | MirrorY
	};
	WorldObject* addObject(WorldObject::Type type, double x, double y, double t, int flags = MirrorX | MirrorY);
	void addLine(const WorldObject::PointsType& points, int flags = MirrorX | MirrorY);

	static FieldModel* m_instance;

	FieldType m_type;
	double m_width;
	double m_length;
	double m_boundary;
	double m_goalWidth;
	double m_goalAreaWidth;
	double m_goalAreaDepth;
	double m_centerCircleDiameter;
	double m_penaltyMarkerDist;
	double m_ballDiameter;
	double m_borderTop;
	double m_borderBottom;
	double m_borderLeft;
	double m_borderRight;

	std::vector<WorldObject> m_objects[WorldObject::NumTypes];

	WorldObject* m_magneticHeading;
	void updateMagneticHeading();
	config_server::Parameter<float> m_attEstMagCalibX;
	config_server::Parameter<float> m_attEstMagCalibY;
	double m_heading;
};

}

#endif
