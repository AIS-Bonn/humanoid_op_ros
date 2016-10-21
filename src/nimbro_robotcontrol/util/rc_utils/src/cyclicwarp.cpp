// Implements a monotonically increasing piecewise cyclic linear transform class.
// File: cyclicwarp.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rc_utils/cyclicwarp.h>
#include <math.h>

// Namespaces
using namespace rc_utils;

//
// CyclicWarp class
//

// Constants
const double CyclicWarp::UNIT = 1.0;
const double CyclicWarp::RADIANS = 2.0*M_PI;
const double CyclicWarp::DEGREES = 360.0;

// Reset functions
void CyclicWarp::reset(double modulus)
{
	// Reset variables
	m_M = modulus;
	clear();
}
void CyclicWarp::clear()
{
	// Set warping to identity transformation
	m_N = 1;
	m_raw.assign(1, 0.0);
	m_warped.assign(1, 0.0);
}

// Get function for the value vectors
void CyclicWarp::getRefValues(std::vector<double>& raw, std::vector<double>& warped)
{
	// Copy out the reference values
	raw = m_raw;
	warped = m_warped;
}

// Set function for the value vectors
bool CyclicWarp::setRefValues(const std::vector<double>& raw, const std::vector<double>& warped)
{
	// Get the number of reference values
	std::size_t N = raw.size();
	if(N < 1 || N != warped.size())
		return false;
	
	// Error checking for jumps over M
	std::size_t rstart = N, wstart = N;
	for(std::size_t i = 0; i < N; i++)
	{
		if(raw[i] < 0.0 || raw[i] >= m_M) return false; // Value out of range...
		if(warped[i] < 0.0 || warped[i] >= m_M) return false; // Value out of range...
		std::size_t j = (i + 1) % N;
		if(raw[i] >= raw[j])
		{
			if(rstart != N) return false; // More than one jump over M...
			rstart = j;
		}
		if(warped[i] >= warped[j])
		{
			if(wstart != N) return false; // More than one jump over M...
			wstart = j;
		}
	}
	
	// Set the parameters
	m_N = N;
	m_raw = raw;
	m_warped = warped;
	
	// Return success
	return true;
}

// Tranformation worker function
double CyclicWarp::transform(const std::vector<double>& in, const std::vector<double>& out, double inValue) const
{
	// Ensure the input value is in range
	inValue = wrap(inValue);
	
	// Initialise the output value
	double outValue = inValue;
	
	// Perform the required warp lookup
	for(std::size_t i = 0; i < m_N; i++)
	{
		// Get the current and next reference values
		std::size_t j = (i + 1) % m_N;
		double idi = in[i];
		double idj = in[j];
		double odi = out[i];
		double odj = out[j];
		
		// If the output reference values jump over M then increase the second output value by the modulus for correct output space interpolation
		if(odi >= odj)
			odj += m_M;
		
		// If the input reference values do not jump over M and contain the input value then interpolate
		if(idi <= inValue && inValue <= idj && idi != idj)
		{
			outValue = odi + ((inValue - idi) / (idj - idi)) * (odj - odi);
			break;
		}
		
		// If the input reference values jump over M then adjust the input reference and interpolate
		if(idi >= idj)
		{
			if(inValue >= idi)
			{
				idj += m_M;
				outValue = odi + ((inValue - idi) / (idj - idi)) * (odj - odi);
				break;
			}
			if(inValue <= idj)
			{
				idi -= m_M;
				outValue = odi + ((inValue - idi) / (idj - idi)) * (odj - odi);
				break;
			}
		}
	}
	
	// Ensure the output is in range
	outValue = wrap(outValue);
	
	// Return the output value
	return outValue;
}
// EOF