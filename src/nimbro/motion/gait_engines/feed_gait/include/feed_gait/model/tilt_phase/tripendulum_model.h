// Tripendulum model
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef TRIPENDULUM_MODEL_H
#define TRIPENDULUM_MODEL_H

// Tripendulum namespace
namespace tripendulum
{
	// Crossing energies struct
	struct CrossingEnergy
	{
		// Constructors
		CrossingEnergy() : B(0.0), F(0.0) {}
		CrossingEnergy(double B, double F) : B(B), F(F) {}

		// Operators
		CrossingEnergy operator+(const CrossingEnergy& other) const { return CrossingEnergy(B + other.B, F + other.F); }
		CrossingEnergy operator-(const CrossingEnergy& other) const { return CrossingEnergy(B - other.B, F - other.F); }

		// Data members
		double B;  //!< @brief Backwards crossing energy
		double F;  //!< @brief Forwards crossing energy
	};

	/**
	* @class TriPendModel
	* 
	* @brief Tripendulum model class.
	**/
	class TriPendModel
	{
	public:
		// Constructor and reset
		TriPendModel() { resetParam(); }
		TriPendModel(double thb, double thm, double thf, double Cbsq, double Cmsq, double Cfsq) { setParam(thb, thm, thf, Cbsq, Cmsq, Cfsq); }

		// Set/reset model parameters
		void setParam(double thb, double thm, double thf, double Cbsq, double Cmsq, double Cfsq);
		void resetParam();

		// Recalculate the dependent model parameters (automatically happens when using setParam())
		void recalc();

		// Model properties based on state
		double accel(double th) const;
		double localEnergy(double th, double thdot) const;
		double localEnergyB(double th, double thdot) const;
		double localEnergyM(double th, double thdot) const;
		double localEnergyF(double th, double thdot) const;
		CrossingEnergy crossingEnergy(double th, double thdot) const;
		CrossingEnergy crossingEnergyPE(double th) const;
		CrossingEnergy crossingEnergyKE(double thdot) const;

		// Model parameters
		double thb;   //!< @brief Centre angle of the unstable b pendulum (more negative)
		double thm;   //!< @brief Centre angle of the stable m pendulum (near zero)
		double thf;   //!< @brief Centre angle of the unstable f pendulum (more positive)
		double Cbsq;  //!< @brief Pendulum constant for the b pendulum (backwards)
		double Cmsq;  //!< @brief Pendulum constant for the m pendulum (middle)
		double Cfsq;  //!< @brief Pendulum constant for the f pendulum (forwards)

		// Calculated: Crossover angles
		double tht;  //!< @brief Crossover angle between the b and m pendulums (more negative)
		double ths;  //!< @brief Crossover angle between the m and f pendulums (more positive)

	private:
		// Calculated: Crossing energy constants
		double Cbsqcostb;
		double Cmsqcostm;
		double Cmsqcossm;
		double Cfsqcossf;
		double constb1;
		double constf1;
		double constb2;
		double constf2;
	};
}

#endif
// EOF