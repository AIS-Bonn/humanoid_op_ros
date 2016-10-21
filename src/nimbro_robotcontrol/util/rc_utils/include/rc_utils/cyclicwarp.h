// Implements a monotonically increasing piecewise cyclic linear transform class.
// File: cyclicwarp.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CYCLICWARP_H
#define CYCLICWARP_H

// Includes
#include <vector>
#include <cmath>

// Robotcontrol utilities namespace
namespace rc_utils
{
	/**
	* @class CyclicWarp
	* 
	* @brief Applies a monotonically increasing piecewise linear transform to a cyclic value.
	**/
	class CyclicWarp
	{
	public:
		// Constructor
		explicit CyclicWarp(double modulus = RADIANS) { reset(modulus); }
		
		// Constants
		static const double UNIT;
		static const double RADIANS;
		static const double DEGREES;
		
		// Reset functions
		void reset(double modulus = RADIANS);
		void clear();
		
		// Get/set functions
		double getModulus() const { return m_M; }
		void setModulus(double modulus) { m_M = fabs(modulus); }
		void getRefValues(std::vector<double>& raw, std::vector<double>& warped);
		bool setRefValues(const std::vector<double>& raw, const std::vector<double>& warped);
		
		// Transformation functions
		double wrap(double value) const { return value - m_M * floor(value / m_M); } // Wraps a value to [0,M)
		double warp(double rawValue) const { return transform(m_raw, m_warped, rawValue); }
		double unwarp(double warpedValue) const { return transform(m_warped, m_raw, warpedValue); }
		
	private:
		// Helper functions
		double transform(const std::vector<double>& in, const std::vector<double>& out, double inValue) const;
		
		// Internal members
		double m_M;                   // Modulus of the cyclic range [0,M)
		std::size_t m_N;              // Number of reference values
		std::vector<double> m_raw;    // Raw reference values (must be cyclically monotonically increasing in [0,M))
		std::vector<double> m_warped; // Corresponding desired warped reference values (must be cyclically monotonically increasing in [0,M))
	};
}

#endif
// EOF