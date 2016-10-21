// Generic finite impulse response (FIR) filter
// File: firfilter.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef FIRFILTER_H
#define FIRFILTER_H

// Robotcontrol utilities namespace
namespace rc_utils
{
	// Encapsulated enumeration
	class FIRFilterType
	{
	public:
		// Filter type enumeration
		enum Enum
		{
			FT_PASSTHROUGH,
			FT_AVERAGE
		};
	};
	
	/**
	* @class FIRFilter
	*
	* @brief Filter that smooths data using custom FIR filtering coefficients.
	*
	* The filter difference equation is:
	* \f[
	* y[n] = b_0x[n] + b_1x[n-1] + \dots + b_Nx[n-N] = \sum_{i=0}^N b_i x[n-i]
	* \f]
	* Where \f$ \mbox{Size} = N + 1 \f$ is the number of terms in the sum. In this implementation the \f$ b_i \f$
	* parameters are forced to sum to unity (implying unit filter DC gain).
	**/
	template<int Size = 5>
	class FIRFilter
	{
	public:
		//! Filter size
		static const int size = Size;

		//! Default constructor (sets up the filter as pass through, i.e. it simply copies the input to the output)
		FIRFilter()
		{
			// Reset the filter object
			resetAll();
			m_modified = true;
		}

		//! Constructor with FIR filter coefficients (up to 5 coefficients are set, with remaining coefficients defaulting to zero, the coefficient for the newest data is \f$ b_0 \f$)
		explicit FIRFilter(double b0, double b1 = 0.0, double b2 = 0.0, double b3 = 0.0, double b4 = 0.0)
		{
			// Set the filter coefficients
			double b[5] = {b0, b1, b2, b3, b4};
			setCoeff(b, 5);
			setBuf();
			m_modified = true;
		}

		//! Construct a FIR filter with a particular filter type
		explicit FIRFilter(FIRFilterType::Enum type)
		{
			// Initialise to a particular known filter type
			if(type == FIRFilterType::FT_PASSTHROUGH)
				setToPassThrough();
			else if(type == FIRFilterType::FT_AVERAGE)
				setToAverage();
			else
				resetAll();
			setBuf();
			m_modified = true;
		}

		//! Reset the filter object completely as if just default-constructed (this resets the filter coefficients!)
		void resetAll()
		{
			// Default filter
			setToPassThrough();
			setBuf();
		}

		//! Clear the internal buffer and fill it with zeros
		void reset()
		{
			// Clear the internal buffer
			setBuf();
		}

		//! Set the filter coefficients (up to 5 coefficients are set, with remaining coefficients defaulting to zero, the coefficient for the newest data is \f$ b_0 \f$)
		void setCoeff(double b0, double b1 = 0.0, double b2 = 0.0, double b3 = 0.0, double b4 = 0.0)
		{
			// Set the filter coefficients
			double b[5] = {b0, b1, b2, b3, b4};
			setCoeff(b, 5);
		}

		//! Set the filter coefficients (@p n coefficients are retrieved from @p b and set, with remaining coefficients defaulting to zero, and with the coefficient for the newest data coming first in the array)
		void setCoeff(const double b[], int n)
		{
			// Copy across the required coefficients
			for(int i = 0; i < n && i < Size; i++)
				m_coeff[i] = b[i];
			for(int i = n; i < Size; i++)
				m_coeff[i] = 0.0;

			// Normalise the coefficients
			normaliseCoeffs();
		}

		//! Retrieve the current filter coefficients in use (@p n coefficients are retrieved and stored in @p b, with the coefficient for the newest data coming first in the array)
		void getCoeff(double b[], int n) const
		{
			// Copy across the required coefficients
			for(int i = 0; i < n && i < Size; i++)
				b[i] = m_coeff[i];
			for(int i = Size; i < n; i++)
				b[i] = 0.0;
		}

		//! Set all the values in the buffer to a particular value and slope (increment per data value), value, or just zero (default)
		void setBuf(double value = 0.0, double slope = 0.0)
		{
			// Fill/clear the buffer
			for(int i = 0; i < Size; i++)
				m_buf[i] = value - (Size - i - 1) * slope;
			m_head = 0;
			m_modified = true;
		}

		//! Manually set the values in the buffer (@p x is assumed to be an array of @c Size doubles, oldest data should come first in @p x)
		void setBuf(const double x[])
		{
			// Populate the buffer with the required values
			for(int i = 0; i < Size; i++)
				m_buf[i] = x[i];
			m_head = 0;
			m_modified = true;
		}

		//! Retrieve the contents of the buffer (@p x is assumed to be an array of doubles of size at least @c Size, older data has a lower index in @p x)
		void getBuf(double x[]) const
		{
			// Copy out the buffer
			int j = 0;
			for(int i = m_head; i < Size; i++)
				x[j++] = m_buf[i];
			for(int i = 0; i < m_head; i++)
				x[j++] = m_buf[i];
		}

		//! Put a new input data point into the internal buffer
		void put(double x)
		{
			// Add the data point
			m_buf[m_head++] = x;
			if(m_head == Size) m_head = 0;
			m_modified = true;
		}

		//! Retrieve the current output value of the filter (calculates it every time, and can therefore be a `const` function)
		double get() const
		{
			// Evaluate the filter equation
			int j = Size - 1;
			double sum = 0.0;
			for(int i = m_head; i < Size; i++)
				sum += m_coeff[j--] * m_buf[i];
			for(int i = 0; i < m_head; i++)
				sum += m_coeff[j--] * m_buf[i];
			return sum;
		}

		//! Retrieve the current output value of the filter (caches a calculated value, thereby avoiding unnecessary calculations, but is not a `const` function)
		double value()
		{
			// Calculate the filtered output value if required
			if(m_modified)
			{
				m_value = get();
				m_modified = false;
			}

			// Return the calculated output value
			return m_value;
		}

	private:
		// Normalise the coefficients
		void normaliseCoeffs()
		{
			// Sum up the coefficients
			double sum = 0.0;
			for(int i = 0; i < Size; i++)
				sum += m_coeff[i];

			// Normalise through by the sum
			if(sum <= 0.0)
				setToPassThrough();
			else
			{
				for(int i = 0; i < Size; i++)
					m_coeff[i] /= sum;
			}

			// Invalidate buffered output value
			m_modified = true;
		}

		// Set the filter to a pass-through implementation
		void setToPassThrough()
		{
			// Set the coefficients so that the output is identical to the input
			m_coeff[0] = 1.0;
			for(int i = 1; i < Size; i++)
				m_coeff[i] = 0.0;

			// Invalidate buffered output value
			m_modified = true;
		}

		// Set the filter to an averaging implementation
		void setToAverage()
		{
			// Set the coefficients so that the output is a running average of the input
			double commonCoeff = 1.0 / Size;
			for(int i = 0; i < Size; i++)
				m_coeff[i] = commonCoeff;

			// Invalidate buffered output value
			m_modified = true;
		}

		// Coefficients vector
		double m_coeff[Size]; // Coeff of newest data b0 --> Coeff of oldest data bN

		// Data buffer
		double m_buf[Size]; // Circular buffer (oldest to newest with a head pointer)
		int m_head; // This is the location of the next place to write a data value, as well as the location of the oldest data value currently in the buffer

		// Output buffering
		double m_value;
		bool m_modified;
	};
}

#endif /* FIRFILTER_H */
// EOF