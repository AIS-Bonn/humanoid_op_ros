// Implements classes that add sharp/smooth ellipsoidal deadband to a signal
// File: smooth_deadband_ell.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef SMOOTH_DEADBAND_ELL_H
#define SMOOTH_DEADBAND_ELL_H

// Includes
#include <rc_utils/math_vec_mat.h>
#include <Eigen/Core>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class SharpEllDeadband
	* 
	* @brief Add sharp ellipsoidal deadband to an N-dimensional signal.
	* 
	* In contrast to the SharpDeadband class, negative semiaxes (i.e. radius) values are treated
	* the same as their absolute value.
	**/
	template<int N> class SharpEllDeadband
	{
	public:
		// Class dimension
		static_assert(N >= 1, "The dimension N of SharpEllDeadband must be a positive integer!");
		static const int ND;

		// Typedefs
		typedef Eigen::Matrix<double, N, 1> Vec;

		// Constructor
		SharpEllDeadband() { reset(); }
		explicit SharpEllDeadband(const Vec& semiaxes, const Vec& centre = Vec::Zero()) : m_semiaxes(semiaxes), m_centre(centre) {}

		// Set/reset functions
		void reset() { m_semiaxes = m_centre = Vec::Zero(); }
		void set(const Vec& semiaxes, const Vec& centre = Vec::Zero()) { m_semiaxes = semiaxes; m_centre = centre; }

		// Get functions
		Vec semiaxes() const { return m_semiaxes; }
		Vec centre() const { return m_centre; }

		// Evaluate functions
		Vec eval(const Vec& x) const { return eval(x, m_semiaxes, m_centre); }
		static Vec eval(const Vec& x, const Vec& semiaxes, const Vec& centre) { return eval(x - centre, semiaxes); }
		static Vec eval(const Vec& x, const Vec& semiaxes)
		{
			double radius = ellipsoidRadius(semiaxes, x);
			double xnorm = x.norm();
			if(xnorm <= 0.0 || xnorm <= radius)
				return Vec::Zero();
			else
				return x * ((xnorm - radius) / xnorm);
		}

	private:
		// Data members
		Vec m_semiaxes;
		Vec m_centre;
	};

	// SharpEllDeadband constants
	template<int N> const int SharpEllDeadband<N>::ND = N;

	// Typedefs
	typedef SharpEllDeadband<1> SharpEllDeadband1D;
	typedef SharpEllDeadband<2> SharpEllDeadband2D;
	typedef SharpEllDeadband<3> SharpEllDeadband3D;
	typedef SharpEllDeadband<4> SharpEllDeadband4D;

	/**
	* @class SmoothEllDeadband
	* 
	* @brief Add smooth ellipsoidal deadband to an N-dimensional signal.
	* 
	* In contrast to the SharpDeadband class, negative semiaxes (i.e. radius) values are treated
	* the same as their absolute value.
	**/
	template<int N> class SmoothEllDeadband
	{
	public:
		// Class dimension
		static_assert(N >= 1, "The dimension N of SmoothEllDeadband must be a positive integer!");
		static const int ND;

		// Typedefs
		typedef Eigen::Matrix<double, N, 1> Vec;

		// Constructor
		SmoothEllDeadband() { reset(); }
		explicit SmoothEllDeadband(const Vec& semiaxes, const Vec& centre = Vec::Zero()) : m_semiaxes(semiaxes), m_centre(centre) {}

		// Set/reset functions
		void reset() { m_semiaxes = m_centre = Vec::Zero(); }
		void set(const Vec& semiaxes, const Vec& centre = Vec::Zero()) { m_semiaxes = semiaxes; m_centre = centre; }

		// Get functions
		Vec semiaxes() const { return m_semiaxes; }
		Vec centre() const { return m_centre; }

		// Evaluate functions
		Vec eval(const Vec& x) const { return eval(x, m_semiaxes, m_centre); }
		static Vec eval(const Vec& x, const Vec& semiaxes, const Vec& centre) { return eval(x - centre, semiaxes); }
		static Vec eval(const Vec& x, const Vec& semiaxes)
		{
			double radius = ellipsoidRadius(semiaxes, x);
			double xnorm = x.norm();
			if(xnorm <= 0.0)
				return Vec::Zero();
			else if(xnorm >= 2.0*radius)
				return x * ((xnorm - radius) / xnorm);
			else
				return x * (xnorm / (4.0 * radius));
		}

	private:
		// Data members
		Vec m_semiaxes;
		Vec m_centre;
	};

	// SmoothEllDeadband constants
	template<int N> const int SmoothEllDeadband<N>::ND = N;

	// Typedefs
	typedef SmoothEllDeadband<1> SmoothEllDeadband1D;
	typedef SmoothEllDeadband<2> SmoothEllDeadband2D;
	typedef SmoothEllDeadband<3> SmoothEllDeadband3D;
	typedef SmoothEllDeadband<4> SmoothEllDeadband4D;

	/**
	* @class DeadSmoothEllDeadband
	* 
	* @brief Add smooth ellipsoidal deadband with a zero zone to an N-dimensional signal.
	* 
	* In contrast to the SharpDeadband class, negative semiaxes (i.e. radius) values are treated
	* the same as their absolute value.
	**/
	template<int N> class DeadSmoothEllDeadband
	{
	public:
		// Class dimension
		static_assert(N >= 1, "The dimension N of DeadSmoothEllDeadband must be a positive integer!");
		static const int ND;

		// Typedefs
		typedef Eigen::Matrix<double, N, 1> Vec;

		// Constructor
		DeadSmoothEllDeadband() { reset(); }
		explicit DeadSmoothEllDeadband(const Vec& semiaxesZ, const Vec& semiaxesD, const Vec& centre = Vec::Zero()) : m_semiaxesZ(semiaxesZ), m_semiaxesD(semiaxesD), m_centre(centre) {}

		// Set/reset functions
		void reset() { m_semiaxesZ = m_semiaxesD = m_centre = Vec::Zero(); }
		void set(const Vec& semiaxesZ, const Vec& semiaxesD, const Vec& centre = Vec::Zero()) { m_semiaxesZ = semiaxesZ; m_semiaxesD = semiaxesD; m_centre = centre; }

		// Get functions
		Vec semiaxesZ() const { return m_semiaxesZ; }
		Vec semiaxesD() const { return m_semiaxesD; }
		Vec centre() const { return m_centre; }

		// Evaluate functions
		Vec eval(const Vec& x) const { return eval(x, m_semiaxesZ, m_semiaxesD, m_centre); }
		static Vec eval(const Vec& x, const Vec& semiaxesZ, const Vec& semiaxesD, const Vec& centre) { return eval(x - centre, semiaxesZ, semiaxesD); }
		static Vec eval(const Vec& x, const Vec& semiaxesZ, const Vec& semiaxesD)
		{
			Vec absSAZ = semiaxesZ.cwiseAbs();
			Vec absSAD = semiaxesD.cwiseAbs();
			double Rz = ellipsoidRadius<double, N>(absSAZ, x);
			double Rd = ellipsoidRadius<double, N>(absSAZ + absSAD, x) - Rz;
			double xnorm = x.norm();
			double xnormd = xnorm - Rz;
			if(xnorm <= 0.0 || xnormd <= 0.0)
				return Vec::Zero();
			else if(xnormd >= 2.0*Rd)
				return x * ((xnormd - Rd) / xnorm);
			else
				return x * (xnormd * xnormd / (4.0 * Rd * xnorm));
		}

	private:
		// Data members
		Vec m_semiaxesZ;
		Vec m_semiaxesD;
		Vec m_centre;
	};

	// DeadSmoothEllDeadband constants
	template<int N> const int DeadSmoothEllDeadband<N>::ND = N;

	// Typedefs
	typedef DeadSmoothEllDeadband<1> DeadSmoothEllDeadband1D;
	typedef DeadSmoothEllDeadband<2> DeadSmoothEllDeadband2D;
	typedef DeadSmoothEllDeadband<3> DeadSmoothEllDeadband3D;
	typedef DeadSmoothEllDeadband<4> DeadSmoothEllDeadband4D;
}

#endif
// EOF