// Walk and kick: Behaviour utilities
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
 
 // Ensure header is only included once
#ifndef WAK_UTILS_H
#define WAK_UTILS_H

// Includes
#include <walk_and_kick/wak_common.h>
#include <config_server/parameter.h>
#include <field_model/field_model.h>
#include <rc_utils/math_spline.h>

// Walk and kick namespace
namespace walk_and_kick
{
	// Using declarations
	using field_model::FieldModel;

	/**
	* @class WAKUtils
	* 
	* @brief A collection of independent helper functions that should be available to all.
	**/
	class WAKUtils
	{
	public:
		// Tangent information struct
		struct TangentInfo
		{
			TangentInfo() : posAngle(0.0f), negAngle(0.0f), maxAdjust(0.0f), length(0.0f) {}
			float posAngle;
			float negAngle;
			float maxAdjust;
			float length;
		};

		// Calculate the tangents from a point to a circle of given centre and radius
		static void calcCircleTangent(TangentInfo& TI, const Vec2f& point, const Vec2f& centre, float radius);

		// Calculate a path from one position to another, subject to a halo (circle) of given centre and radius
		static float calculateHaloPath(Vec2f& fromNormal, const Vec2f& fromPos, const Vec2f& toPos, const Vec2f& centre, float radius, bool smoothFromNormal = true) { Vec2fArray path; return calculateHaloPath(fromNormal, path, fromPos, toPos, centre, radius, smoothFromNormal); }
		static float calculateHaloPath(Vec2f& fromNormal, Vec2fArray& path, const Vec2f& fromPos, const Vec2f& toPos, const Vec2f& centre, float radius, bool smoothFromNormal = true);

		// Calculate the wedge from a particular angle, to another particular angle, possibly forcing the sign of the output in the process
		static float calcWedge(float fromAngle, float toAngle, int forceSign = 0);

	private:
		// Constructor
		WAKUtils() {}
	};

	/**
	* @class Counter
	* 
	* @brief A simple counter class for counting events.
	* 
	* Be very careful about using this counter because it does not saturate!! So if expr is true for a while,
	* then the internal count winds up to a significantly high number, and it then takes an equally long period
	* of expr being false for reached(NUM) to eventually return false again. This is normally not the desired
	* behaviour in an application, so be careful to reset the counter at some point to alleviate this problem.
	**/
	class Counter
	{
	public:
		Counter() { reset(); }
		void reset() { m_count = 0; }
		int count() const { return m_count; }
		void add(bool expr = true) { if(expr) m_count++; else m_count = (m_count > 0 ? m_count - 1 : 0); }
		void increment() { m_count++; }
		bool reached(int limit) const { return (m_count >= limit); }
	private:
		int m_count;
	};

	/**
	* @class TheWorm
	* 
	* @brief A class that implements the worm, for making a live decision between two opposites.
	* 
	* To understand the name of the class, refer to the televised debates in Australian politics and [Worm (Marketing)](https://en.wikipedia.org/wiki/Worm_%28marketing%29).
	**/
	class TheWorm
	{
	public:
		explicit TheWorm(int limit = 100) { reset(limit); }
		void reset() { m_worm = 0; }
		void reset(bool decision) { if(decision) m_worm = m_limit; else m_worm = -m_limit; }
		void reset(int limit) { reset(); setLimit(limit); }
		void resetR(int range) { reset(); setRange(range); }
		void setLimit(int limit) { m_limit = (limit >= 1 ? limit : 1); } // The count from zero to a positive or negative decision
		void setRange(int range) { m_limit = (range >= 2 ? range >> 1 : 1); } // The count from a decision to the opposite decision
		void setWorm(int value) { m_worm = (value >= m_limit ? m_limit : (value <= -m_limit ? -m_limit : value)); }
		void vote(bool decision) { if(decision) m_worm = (m_worm < m_limit ? m_worm + 1 : m_limit); else m_worm = (m_worm > -m_limit ? m_worm - 1 : -m_limit); }
		void vote(bool decision, int numVotes) { if(numVotes <= 0) return; if(decision) m_worm = std::min(m_worm + numVotes, m_limit); else m_worm = std::max(m_worm - numVotes, -m_limit); }
		void neutralise() { if(m_worm > 0) m_worm = std::min(m_worm - 1, m_limit); else if(m_worm < 0) m_worm = std::max(m_worm + 1, -m_limit); }
		int  limit() const { return m_limit; }
		int  range() const { return m_limit << 1; }
		int  count() const { return m_worm; }
		bool decision() const { return (m_worm >= 0); } // Balanced votes is a decision of true!
		bool unanimousTrue() const { return (m_worm >= m_limit); }
		bool unanimousFalse() const { return (m_worm <= -m_limit); }
		bool unanimous() const { return (unanimousTrue() || unanimousFalse()); }
		void updateWormTime(const config_server::Parameter<float>* param) { setRange(TO_COUNT(param->get())); }
	private:
		int m_limit;
		int m_worm;
	};

	/**
	* @class LiveTrapVelSpline
	* 
	* @brief A wrapper class for the trapezoidal velocity spline class `rc_utils::TrapVelSpline` with a focus on smoothly supporting live updates of the spline.
	**/
	class LiveTrapVelSpline
	{
	public:
		LiveTrapVelSpline() { reset(); }
		void reset() { m_valid = false; m_x = m_v = m_t = 0.0; }
		void setState(double x, double v = 0.0) { m_x = x; m_v = v; m_t = 0.0; m_valid = false; }
		bool valid() const { return m_valid; }
		bool finished() const { return (!m_valid || m_t >= m_spline.T()); }
		double curX() const { return m_x; }
		double curV() const { return m_v; }
		void newTarget(double x, double v, double maxVel, double maxAcc, bool ctsVel = true)
		{
			m_spline.setParams(m_x, (ctsVel ? m_v : 0.0), x, v, maxVel, maxAcc);
			m_valid = true;
			m_t = 0.0;
		}
		double forward(double dT)
		{
			if(!m_valid) return m_x;
			m_t += dT;
			m_x = m_spline.x(m_t);
			m_v = m_spline.v(m_t);
			return m_x;
		}
	private:
		rc_utils::TrapVelSpline m_spline;
		bool m_valid; // Flag whether the spline is currently valid
		double m_x;   // Current position
		double m_v;   // Current velocity
		double m_t;   // Current time relative to the start of the spline (only if valid)
	};

	/**
	* @class TrapVelSpline2D
	* 
	* @brief A trapezoidal velocity spline that operates in 2D, based on the `rc_utils::TrapVelSpline` class.
	* 
	* Note that the spline velocity is not necessarily continuous if a new target is set before a previous one ends.
	* It is assumed that the total trajectory that is followed is piecewise linear in 2D with zero instantaneous
	* velocities at the cusps.
	**/
	class TrapVelSpline2D
	{
	public:
		TrapVelSpline2D() { reset(); }
		void reset() { m_valid = false; m_x = m_xi = m_xf = 0.0; m_y = m_yi = m_yf = 0.0; m_t = 0.0; }
		void setState(double x, double y) { m_valid = false; m_x = m_xi = m_xf = x; m_y = m_yi = m_yf = y; m_t = 0.0; }
		bool valid() const { return m_valid; }
		bool finished() const { return (!m_valid || m_t >= m_spline.T()); }
		double curX() const { return m_x; }
		double curY() const { return m_y; }
		double targetX() const { return m_xf; }
		double targetY() const { return m_yf; }
		void newTarget(double x, double y, double maxVel, double maxAcc)
		{
			m_xi = m_x;
			m_yi = m_y;
			m_xf = x;
			m_yf = y;
			double dx = m_xf - m_xi;
			double dy = m_yf - m_yi;
			double D = sqrt(dx*dx + dy*dy);
			m_spline.setParams(0.0, 0.0, D, 0.0, maxVel, maxAcc);
			m_valid = true;
			m_t = 0.0;
		}
		void forward(double dT)
		{
			if(!m_valid) return;
			m_t += dT;
			double D = m_spline.xf();
			if(D == 0.0)
			{
				m_x = m_xf;
				m_y = m_yf;
			}
			else
			{
				double d = m_spline.x(m_t);
				double r = d/D;
				m_x = m_xi + r*(m_xf - m_xi);
				m_y = m_yi + r*(m_yf - m_yi);
			}
		}
	private:
		rc_utils::TrapVelSpline m_spline;
		bool m_valid; // Flag whether the spline is currently valid
		double m_xi;  // Initial x position of the current spline (only if valid)
		double m_yi;  // Initial y position of the current spline (only if valid)
		double m_xf;  // Final x position of the current spline (only if valid)
		double m_yf;  // Final y position of the current spline (only if valid)
		double m_x;   // Current x position
		double m_y;   // Current y position
		double m_t;   // Current time relative to the start of the spline (only if valid)
	};

	/**
	* @class FieldDimensions
	* 
	* @brief A simple class for abstracting away the source of the field dimensions.
	**/
	class FieldDimensions
	{
	public:
		// Constructor
		FieldDimensions() : m_field(FieldModel::getInstance()) {}

		// Field parameters
		FieldModel::FieldType fieldType() const { return m_field->type(); } //!< @brief The type of the field that is being played on.
		const std::string& fieldTypeName() const { return m_field->typeName(); } //!< @brief A string representation of the type of the field that is being played on.
		float fieldLength() const { return m_field->length(); } //!< @brief The length of the field from one goal side to the other.
		float fieldWidth() const { return m_field->width(); } //!< @brief The width of the field from one sideline to the other.
		float circleDiameter() const { return m_field->centerCircleDiameter(); } //!< @brief The diameter of the centre circle.
		float boundary() const { return m_field->boundary(); } //!< @brief The width of the green boundary (on all four sides) outside the outer white lines of the field.
		float goalWidth() const { return m_field->goalWidth(); } //!< @brief The width of the goals from one goal post to the other.
		float goalAreaLength() const { return m_field->goalAreaDepth(); } //!< @brief The smaller dimension of the goal area box, from the goal line to the manual robot starting line.
		float goalAreaWidth() const { return m_field->goalAreaWidth(); } //!< @brief The larger dimension of the goal area box, from outside of the left goal post to outside of 
		float penaltyMarkDist() const { return m_field->penaltyMarkerDist(); } //!< @brief The distance from the goal line to the penalty marker in that half of the field.
		float ballDiameter() const { return m_field->ballDiameter(); } //!< @brief The diameter of the ball.
		float fieldLengthH() const { return 0.5*m_field->length(); } //!< @brief Half of the length of the field, i.e. the distance from the centre line to the goal lines.
		float fieldWidthH() const { return 0.5*m_field->width(); } //!< @brief Half of the width of the field, i.e. the distance from centre point to the side lines.
		float circleRadius() const { return 0.5*m_field->centerCircleDiameter(); } //!< @brief The radius of the centre circle.
		float goalWidthH() const { return 0.5*m_field->goalWidth(); } //!< @brief Half of the width of the goals, i.e. half of the distance separation between the two goal posts.
		float ballRadius() const { return 0.5*m_field->ballDiameter(); } //!< @brief The radius of the ball.

	private:
		// Field model
		const FieldModel* const m_field;
	};
}

#endif
// EOF