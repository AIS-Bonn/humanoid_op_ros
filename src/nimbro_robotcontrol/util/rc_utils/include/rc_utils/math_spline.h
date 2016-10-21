// Utilities for working with splines
// File: math_spline.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MATH_SPLINE_H
#define MATH_SPLINE_H

// Includes
#include <rc_utils/math_funcs.h>
#include <cmath>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class LinearSpline
	*
	* @brief Class that calculates a linear spline to fit between two boundary conditions.
	*
	* To use this class simply instantiate an instance of it, set up the boundary conditions
	* using either the overloaded constructor or the `setParams()` function, and then dereference
	* the class via the `()` operator to evaluate the spline at a particular time. A static method
	* for one-time calculations has also been implemented. Individual position and velocity
	* components can also be retrieved.
	*
	* Outside of the [0,dT] time range the linear spline simply takes on either the initial or
	* final position value, with zero velocity.
	*
	* @b Examples:
	* @code
	* // Sample time we wish to evaluate the spline at
	* myt = 0.25;
	*
	* // Params:           xi   xf   dT
	* LinearSpline spline(0.0, 1.0, 0.5);
	* myx = spline(myt);
	* myx = spline.x(myt);
	* myv = spline.v();
	*
	* // OR...
	*
	* LinearSpline spline;
	* spline.setParams(0.0, 1.0, 0.5);
	* myx = spline(myt); // Can also use x() and v() as before
	*
	* // OR...
	*
	* myx = LinearSpline::eval(0.0, 1.0, 0.5, myt);
	* @endcode
	*
	* Note that this class evaluates splines in terms of @c x vs. @c t, as opposed to
	* in the 2D plane between arbitary points.
	**/
	class LinearSpline
	{
	public:
		//! @brief Default constructor. If you use this you need to call `setParams()` manually.
		LinearSpline()
		 : m_xi(0.0)
		 , m_xf(0.0)
		 , m_tf(0.0)
		 , m_v(0.0)
		{}

		//! @brief Constructor overload: Sets the internal parameters as required, based on the provided boundary conditions. Refer to `setParams()` for more details.
		LinearSpline(double xi, double xf, double dT)
		{
			// Set the parameters as required
			setParams(xi, xf, dT);
		}

		/**
		* @brief Calculates the required parameters of the spline given the required boundary conditions.
		*
		* Passing a zero @p dT results in `NaN` or `Inf` outputs.
		*
		* @param xi The initial position/displacement
		* @param xf The final position/displacement
		* @param dT The time (horizontal axis when graphing the spline) between the initial and final conditions
		**/
		void setParams(double xi, double xf, double dT)
		{
			// Calculate/store the linear spline parameters
			m_xi = xi;
			m_xf = xf;
			m_tf = dT;
			m_v  = (xf - xi) / dT;
		}

		//! @brief Evaluates the spline position at the required time @p t.
		double operator()(double t) const
		{
			// Evaluate the spline position at the required time t
			return x(t);
		}

		//! @brief Evaluates the spline position at the required time @p t.
		double x(double t) const
		{
			// Evaluate the spline position at the required time t
			if(t < 0.0) return m_xi;
			else if(t > m_tf) return m_xf;
			else return m_xi + m_v*t;
		}

		//! @brief Evaluates the spline velocity at the required time @p t.
		double v(double t) const
		{
			// Evaluate the spline velocity at the required time t
			if(t < 0.0 || t > m_tf) return 0.0;
			else return m_v;
		}

		//! @brief Returns the nominal spline velocity.
		double v() const
		{
			// Return the nominal spline velocity
			return m_v;
		}

		/**
		* @brief Static method for one-time evaluation of a linear spline given the required boundary conditions.
		*
		* @param xi The initial position/displacement
		* @param xf The final position/displacement
		* @param dT The time (horizontal axis when graphing the spline) between the initial and final conditions
		* @param t The time to evaluate the spline at
		**/
		static double eval(double xi, double xf, double dT, double t)
		{
			// Evaluate the spline position at the required time t
			if(t < 0.0) return xi;
			else if(t > dT) return xf;
			else return xi + (t/dT)*(xf - xi);
		}

	private:
		// Internal variables
		double m_xi;
		double m_xf;
		double m_tf;
		double m_v;
	};

	/**
	* @class CubicSpline
	*
	* @brief Class that calculates a cubic spline to fit between two boundary conditions.
	*
	* To use this class simply instantiate an instance of it, set up the boundary conditions
	* using either the overloaded constructor or the `setParams()` function, and then dereference
	* the class via the `()` operator to evaluate the spline at a particular time. A static method
	* for one-time calculations has also been implemented. Individual position, velocity,
	* acceleration and jerk components can also be retrieved.
	*
	* Outside of the [0,dT] time range the cubic spline is simply extrapolated based on its cubic formula.
	*
	* @b Examples:
	* @code
	* // Sample time we wish to evaluate the spline at
	* myt = 0.25;
	* 
	* // Params:          xi   vi   xf    vf   dT
	* CubicSpline spline(0.0, 0.0, 1.0, -1.0, 0.5);
	* myx = spline(myt);
	* myx = spline.x(myt);
	* myv = spline.v(myt);
	* mya = spline.a(myt);
	* myj = spline.jerk();
	*
	* // OR...
	* 
	* CubicSpline spline;
	* spline.setParams(0.0, 0.0, 1.0, -1.0, 0.5);
	* myx = spline(myt); // Can also use x(), v(), a() and jerk() as before
	*
	* // OR...
	*
	* myx = CubicSpline::eval(0.0, 0.0, 1.0, -1.0, 0.5, myt);
	* @endcode
	*
	* Note that this class evaluates splines in terms of @c x vs. @c t, as opposed to
	* in the 2D plane between arbitary points.
	**/
	class CubicSpline
	{
	public:
		//! @brief Default constructor. If you use this you need to call `setParams()` manually.
		CubicSpline()
		 : m_xi(0.0)
		 , m_vi(0.0)
		 , m_ai(0.0)
		 , m_jerk(0.0)
		{}

		//! @brief Constructor overload: Sets the internal parameters as required, based on the provided boundary conditions. Refer to `setParams()` for more details.
		CubicSpline(double xi, double vi, double xf, double vf, double dT)
		{
			// Set the parameters as required
			setParams(xi, vi, xf, vf, dT);
		}

		/**
		* @brief Calculates the required parameters of the spline given the required boundary conditions.
		*
		* Passing a zero @p dT results in `NaN` or `Inf` outputs.
		*
		* @param xi The initial position/displacement
		* @param vi The initial velocity/slope
		* @param xf The final position/displacement
		* @param vf The final velocity/slope
		* @param dT The time (horizontal axis when graphing the spline) between the initial and final conditions
		**/
		void setParams(double xi, double vi, double xf, double vf, double dT)
		{
			// Calculate constants C and D
			double C = 6.0*(xf - xi - vi*dT)/(dT*dT);
			double D = 2.0*(vf - vi)/dT;

			// Remember the initial conditions
			m_xi = xi;
			m_vi = vi;

			// Calculate the required initial and final accelerations, and the corresponding jerk (slope of the acceleration ramp)
			m_ai = C - D;
			double af = 2.0*D - C;
			m_jerk = (af - m_ai) / dT;
		}

		//! @brief Evaluates the spline position at the required time @p t.
		double operator()(double t) const
		{
			// Evaluate the spline position at the required time t
			return x(t);
		}

		//! @brief Evaluates the spline position at the required time @p t.
		double x(double t) const
		{
			// Evaluate the spline position at the required time t
			return m_xi + t*(m_vi + t*(m_ai + t*m_jerk/3.0)/2.0);
		}

		//! @brief Evaluates the spline velocity at the required time @p t.
		double v(double t) const
		{
			// Evaluate the spline velocity at the required time t
			return m_vi + t*(m_ai + t*m_jerk/2.0);
		}

		//! @brief Evaluates the spline acceleration at the required time @p t.
		double a(double t) const
		{
			// Evaluate the spline acceleration at the required time t
			return m_ai + t*m_jerk;
		}

		//! @brief Returns the spline jerk.
		double jerk() const
		{
			// Return the spline jerk
			return m_jerk;
		}

		/**
		* @brief Static method for one-time evaluation of a cubic spline given the required boundary conditions.
		*
		* @param xi The initial position/displacement
		* @param vi The initial velocity/slope
		* @param xf The final position/displacement
		* @param vf The final velocity/slope
		* @param dT The time (horizontal axis when graphing the spline) between the initial and final conditions
		* @param t The time to evaluate the spline at
		**/
		static double eval(double xi, double vi, double xf, double vf, double dT, double t)
		{
			// Calculate constants C and D
			double C = 6.0*(xf - xi - vi*dT)/(dT*dT);
			double D = 2.0*(vf - vi)/dT;

			// Calculate the required initial and final accelerations, and the corresponding jerk (slope of the acceleration ramp)
			double ai = C - D;
			double af = 2.0*D - C;
			double jerk = (af - ai) / dT;
			
			// Evaluate the spline at the required time t
			return xi + t*(vi + t*(ai + t*jerk/3.0)/2.0);
		}

	private:
		// Internal variables
		double m_xi;
		double m_vi;
		double m_ai;
		double m_jerk;
	};

	/**
	* @class TrapVelSpline
	*
	* @brief Class that calculates a trapezoidal velocity spline to fit between two boundary conditions.
	*
	* To use this class simply instantiate an instance of it, set up the boundary conditions
	* using either the overloaded constructor or the `setParams()` function, and then dereference
	* the class via the `()` operator to evaluate the spline at a particular time. A static method
	* for one-time calculations has also been implemented. Individual position, velocity and
	* acceleration components can also be retrieved.
	*
	* The calculated spline uses up to three phases, each with a constant acceleration of either `a`,
	* `-a` or `0`, to satisfy the boundary conditions, while respecting the maximum allowed velocity
	* `vm` as much as possible. This generally leads to a trapezoidal-shaped velocity profile. If the
	* velocity limitation is not required to be reached during the spline, then a two phase wedge-shaped
	* velocity profile results instead. If no trapezoidal and/or wedge solution is found (should never
	* happen) then a constant velocity solution is used as a backup. This in general cannot respect
	* the initial and final velocities however.
	*
	* Outside of the spline's time domain the spline is simply extrapolated based on constant velocity
	* motion defined by the given initial and final velocities.
	*
	* @b Examples:
	* @code
	* // Sample time we wish to evaluate the spline at
	* myt = 0.25;
	*
	* // Params:            xi   vi   xf    vf   vm   a
	* TrapVelSpline spline(1.0, 0.0, 2.0, -1.0, 0.5, 2.0);
	* myx = spline(myt);
	* myx = spline.x(myt);
	* myv = spline.v(myt);
	* mya = spline.a(myt);
	*
	* // OR...
	*
	* TrapVelSpline spline;
	* spline.setParams(1.0, 0.0, 2.0, -1.0, 0.5, 2.0);
	* myx = spline(myt); // Can also use x(), v() and a() as before
	*
	* // OR...
	*
	* myx = TrapVelSpline::eval(1.0, 0.0, 2.0, -1.0, 0.5, 2.0, myt);
	* @endcode
	*
	* Note that this class evaluates splines in terms of @c x vs. @c t, as opposed to
	* in the 2D plane between arbitary points.
	**/
	class TrapVelSpline
	{
	public:
		//! @brief Default constructor. If you use this you need to call `setParams()` manually.
		TrapVelSpline()
		 : m_xi(0.0)
		 , m_vi(0.0)
		 , m_xf(0.0)
		 , m_vf(0.0)
		 , m_dx(0.0)
		 , m_soln_exists(false)
		 , m_soln_is_trap(false)
		 , m_soln_is_trumps(false)
		 , m_tf(0.0)
		 , m_trap_ati(0.0)
		 , m_trap_atf(0.0)
		 , m_trap_vms(0.0)
		 , m_trap_t1(0.0)
		 , m_trap_t2(0.0)
		 , m_trap_x1(0.0)
		 , m_wedge_as(0.0)
		 , m_wedge_t3(0.0)
		 , m_const_vt(0.0)
		{}

		//! @brief Constructor overload: Sets the internal parameters as required, based on the provided boundary conditions. Refer to `setParams()` for more details.
		TrapVelSpline(double xi, double vi, double xf, double vf, double vm, double a)
		{
			// Set the parameters as required
			setParams(xi, vi, xf, vf, vm, a);
		}

		/**
		* @brief Calculates the parameters of the spline given the required boundary conditions.
		*
		* Passing zero @p vm or @p a results in invalid `NaN` and/or `Inf` outputs.
		*
		* @param xi The initial position/displacement
		* @param vi The initial velocity/slope
		* @param xf The final position/displacement
		* @param vf The final velocity/slope
		* @param vm The maximum absolute velocity to use in reaching the final position/velocity (must be positive)
		* @param a  The constant acceleration to use in reaching the final position/velocity (must be positive)
		**/
		void setParams(double xi, double vi, double xf, double vf, double vm, double a)
		{
			// Ensure the acceleration and maximum velocity specifications are positive
			vm = fabs(vm);
			a = fabs(a);

			// Store the spline parameters
			m_xi = xi;
			m_vi = vi;
			m_xf = xf;
			m_vf = vf;

			// Calculate further spline parameters
			m_dx = m_xf - m_xi;

			// Initialise the spline solution
			m_soln_exists = false;
			m_soln_is_trap = false;
			m_soln_is_trumps = false;

			// Check the trapezoidal velocity solutions
			calcTrapSoln( vm, a); // Saturation at vm solution
			calcTrapSoln(-vm, a); // Saturation at -vm solution

			// Check the wedge velocity solutions
			calcWedgeSoln( a, true);  // Positive valley solution
			calcWedgeSoln( a, false); // Negative valley solution
			calcWedgeSoln(-a, true);  // Positive hill solution
			calcWedgeSoln(-a, false); // Negative hill solution

			// Calculate a constant velocity fallback solution (should never actually be required)
			calcConstSoln(vm);
		}

		//! @brief Retrieves the initial spline position.
		double xi() const { return m_xi; }

		//! @brief Retrieves the final spline position.
		double xf() const { return m_xf; }

		//! @brief Retrieves the initial spline velocity.
		double vi() const { return m_vi; }

		//! @brief Retrieves the final spline velocity.
		double vf() const { return m_vf; }

		//! @brief Retrieves the time duration of the calculated spline.
		double T() const
		{
			// Return the calculated spline duration
			return m_tf;
		}

		//! @brief Evaluates the spline position at the required time @p t.
		double operator()(double t) const
		{
			// Evaluate the spline position at the required time t
			return x(t);
		}

		//! @brief Evaluates the spline position at the required time @p t.
		double x(double t) const
		{
			// Evaluate the spline position at the required time t
			if(t < 0.0) return m_xi + m_vi*t;
			else if(t > m_tf) return m_xf + m_vf*(t - m_tf);
			else if(!m_soln_exists) return m_xi + m_const_vt*t;
			else if(m_soln_is_trap)
			{
				if(t <= m_trap_t1) return m_xi + t*(m_vi + 0.5*m_trap_ati*t);
				else if(t <= m_trap_t2) return m_trap_x1 + m_trap_vms*(t - m_trap_t1);
				else
				{
					double tb = m_tf - t;
					return m_xf - tb*(m_vf - 0.5*m_trap_atf*tb);
				}
			}
			else
			{
				if(t <= m_wedge_t3) return m_xi + t*(m_vi - 0.5*m_wedge_as*t);
				else
				{
					double tb = m_tf - t;
					return m_xf - tb*(m_vf - 0.5*m_wedge_as*tb);
				}
			}
		}

		//! @brief Evaluates the spline velocity at the required time @p t.
		double v(double t) const
		{
			// Evaluate the spline velocity at the required time t
			if(t < 0.0) return m_vi;
			else if(t > m_tf) return m_vf;
			else if(!m_soln_exists) return m_const_vt;
			else if(m_soln_is_trap)
			{
				if(t <= m_trap_t1) return m_vi + m_trap_ati*t;
				else if(t <= m_trap_t2) return m_trap_vms;
				else return m_vf - m_trap_atf*(m_tf - t);
			}
			else
			{
				if(t <= m_wedge_t3) return m_vi - m_wedge_as*t;
				else return m_vf - m_wedge_as*(m_tf - t);
			}
		}

		//! @brief Evaluates the spline acceleration at the required time @p t.
		double a(double t) const
		{
			// Evaluate the spline acceleration at the required time t
			if(t < 0.0 || t > m_tf || !m_soln_exists) return 0.0;
			else if(m_soln_is_trap)
			{
				if(t <= m_trap_t1) return m_trap_ati;
				else if(t <= m_trap_t2) return 0.0;
				else return m_trap_atf;
			}
			else
			{
				if(t <= m_wedge_t3) return -m_wedge_as;
				else return m_wedge_as;
			}
		}

		/**
		* @brief Static method for one-time evaluation of a trapezoidal spline given the required boundary conditions.
		*
		* @param xi The initial position/displacement
		* @param vi The initial velocity/slope
		* @param xf The final position/displacement
		* @param vf The final velocity/slope
		* @param vm The maximum absolute velocity to use in reaching the final position/velocity (must be positive)
		* @param a  The constant acceleration to use in reaching the final position/velocity (must be positive)
		* @param t  The time to evaluate the spline position at
		**/
		static double eval(double xi, double vi, double xf, double vf, double vm, double a, double t)
		{
			// Perform the calculation using a static local spline instance
			static TrapVelSpline spline;
			spline.setParams(xi, vi, xf, vf, vm, a);
			return spline.x(t);
		}

	private:
		// Trapezoidal velocity solution calculation
		void calcTrapSoln(double vms, double a) // vms should be +-vm, a should be +ve
		{
			// Calculate the required timing keypoints
			double ti1 = fabs(vms - m_vi) / a;
			double t2f = fabs(vms - m_vf) / a;
			double dxi1 = 0.5 * ti1 * (vms + m_vi);
			double dx2f = 0.5 * t2f * (vms + m_vf);
			double t12 = (m_dx - dxi1 - dx2f) / vms;

			// Return if the solution is invalid
			if(t12 < 0.0) return;

			// Calculate additional parameters
			double ti2 = ti1 + t12;
			double tif = ti2 + t2f;

			// Update the currently chosen solution if this solution is better
			if(!m_soln_exists || !m_soln_is_trumps || (tif <= m_tf))
			{
				m_trap_ati = a * sign0(vms - m_vi);
				m_trap_atf = a * sign0(m_vf - vms);
				m_trap_vms = vms;
				m_trap_t1 = ti1;
				m_trap_t2 = ti2;
				m_trap_x1 = m_xi + dxi1;
				m_tf = tif;
				m_soln_exists = true;
				m_soln_is_trap = true;
				m_soln_is_trumps = true;
			}
		}

		// Wedge velocity solution calculation
		void calcWedgeSoln(double as, bool soln) // as should be +-a, soln is a flag for whether to calculate the positive or negative solution
		{
			// Calculate the wedge solution discriminant
			double discr = 0.5*(m_vi*m_vi + m_vf*m_vf) - as*m_dx;

			// Return if there is no solution
			if(discr < 0.0) return;

			// Calculate the corner time and return if it is invalid
			double t3 = (soln ? m_vi + sqrt(discr) : m_vi - sqrt(discr)) / as;
			if(t3 < 0.0) return;

			// Calculate the total time and return if it is invalid
			double tf = 2.0*t3 + (m_vf - m_vi) / as;
			if(tf < t3) return;

			// Calculate whether the solution is trumps
			double v3 = m_vi - as*t3;
			bool is_trumps = (fabs(v3) <= fabs(m_vi)) && (v3 * m_vi >= 0.0);

			// Update the currently chosen solution if this solution is better
			bool soln_is_shorter = (tf <= m_tf);
			if(!m_soln_exists || (m_soln_is_trumps ? is_trumps && soln_is_shorter : is_trumps || soln_is_shorter))
			{
				m_wedge_as = as;
				m_wedge_t3 = t3;
				m_tf = tf;
				m_soln_exists = true;
				m_soln_is_trap = false;
				m_soln_is_trumps = is_trumps;
			}
		}

		// Constant velocity solution calculation
		void calcConstSoln(double vm)
		{
			// Update the currently chosen solution if this solution is better
			if(!m_soln_exists)
			{
				m_tf = fabs(m_dx) / vm;
				m_const_vt = sign0(m_dx) * vm;
			}
		}
		
		// Spline parameters
		double m_xi;
		double m_vi;
		double m_xf;
		double m_vf;
		double m_dx;

		// Spline solution flags
		bool m_soln_exists;
		bool m_soln_is_trap;
		bool m_soln_is_trumps;

		// Common parameters
		double m_tf;

		// Trapezoidal velocity spline parameters
		double m_trap_ati;
		double m_trap_atf;
		double m_trap_vms;
		double m_trap_t1;
		double m_trap_t2;
		double m_trap_x1;

		// Wedge velocity spline parameters
		double m_wedge_as;
		double m_wedge_t3;

		// Constant velocity spline parameters
		double m_const_vt;
	};
}

#endif /* MATH_SPLINE_H */
// EOF