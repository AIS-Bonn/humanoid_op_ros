// Tripendulum model
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <feed_gait/model/tilt_phase/tripendulum_model.h>
#include <cmath>

// Namespaces
using namespace tripendulum;

//
// TriPendModel class
//

// Set model parameters
void TriPendModel::setParam(double thb, double thm, double thf, double Cbsq, double Cmsq, double Cfsq)
{
	// Transcribe the required parameters
	this->thb = thb;
	this->thm = thm;
	this->thf = thf;
	this->Cbsq = fabs(Cbsq);
	this->Cmsq = fabs(Cmsq);
	this->Cfsq = fabs(Cfsq);

	// Recalculate the dependent model parameters
	recalc();
}

// Reset model parameters
void TriPendModel::resetParam()
{
	// Set default model parameters
	setParam(0.0, 0.0, 0.0, 9.81, 9.81, 9.81);
}

// Recalculate the dependent model parameters
void TriPendModel::recalc()
{
	// Recalculate the crossover angles
	double thbmbar = 0.5*(thb + thm);
	double thfmbar = 0.5*(thf + thm);
	double delthbm = 0.5*(thb - thm);
	double delthfm = 0.5*(thf - thm);
	tht = thbmbar + atan2((Cbsq - Cmsq)*sin(delthbm), (Cbsq + Cmsq)*cos(delthbm));
	ths = thfmbar + atan2((Cfsq - Cmsq)*sin(delthfm), (Cfsq + Cmsq)*cos(delthfm));

	// Recalculate the crossing energy constants
	Cbsqcostb = Cbsq*cos(tht - thb);
	Cmsqcostm = Cmsq*cos(tht - thm);
	Cmsqcossm = Cmsq*cos(ths - thm);
	Cfsqcossf = Cfsq*cos(ths - thf);
	double constA = Cmsqcostm + Cbsqcostb;
	double constB = Cmsqcossm + Cfsqcossf;
	constb1 = Cbsq - constA;
	constf1 = Cfsq - constB;
	constb2 = constb1 + constB;
	constf2 = constf1 + constA;
}

// Calculate the acceleration at a state
double TriPendModel::accel(double th) const
{
	// Calculate the required acceleration
	if(th >= ths)
		return Cfsq*sin(th - thf);
	else if(th <= tht)
		return Cbsq*sin(th - thb);
	else
		return -Cmsq*sin(th - thm);
}

// Calculate the relevant local energy at a state
double TriPendModel::localEnergy(double th, double thdot) const
{
	// Calculate the required local energy
	if(th >= ths)
		return localEnergyF(th, thdot);
	else if(th <= tht)
		return localEnergyB(th, thdot);
	else
		return localEnergyM(th, thdot);
}

// Calculate the local b pendulum energy at a state
double TriPendModel::localEnergyB(double th, double thdot) const
{
	// Calculate the required local energy
	return thdot*thdot - 2.0*Cbsq*(1.0 - cos(th - thb));
}

// Calculate the local m pendulum energy at a state
double TriPendModel::localEnergyM(double th, double thdot) const
{
	// Calculate the required local energy
	return thdot*thdot + 2.0*Cmsq*(1.0 - cos(th - thm));
}

// Calculate the local f pendulum energy at a state
double TriPendModel::localEnergyF(double th, double thdot) const
{
	// Calculate the required local energy
	return thdot*thdot - 2.0*Cfsq*(1.0 - cos(th - thf));
}

// Calculate the crossing energy at a state
CrossingEnergy TriPendModel::crossingEnergy(double th, double thdot) const
{
	// Calculate the required crossing energy
	CrossingEnergy PE = crossingEnergyPE(th);
	CrossingEnergy KE = crossingEnergyKE(thdot);
	return PE + KE;
}

// Calculate the potential crossing energy at a state
CrossingEnergy TriPendModel::crossingEnergyPE(double th) const
{
	// Calculate the required potential crossing energy
	double hPEb, hPEf;
	if(th >= ths)
	{
		double Cfsqcoshf = Cfsq*cos(th - thf);
		hPEb = constb2 - Cfsqcoshf;
		hPEf = (th > thf ? Cfsqcoshf - Cfsq : Cfsq - Cfsqcoshf);
	}
	else if(th <= tht)
	{
		double Cbsqcoshb = Cbsq*cos(th - thb);
		hPEb = (th < thb ? Cbsqcoshb - Cbsq : Cbsq - Cbsqcoshb);
		hPEf = constf2 - Cbsqcoshb;
	}
	else
	{
		double Cmsqcoshm = Cmsq*cos(th - thm);
		hPEb = constb1 + Cmsqcoshm;
		hPEf = constf1 + Cmsqcoshm;
	}
	return CrossingEnergy(-2.0*hPEb, -2.0*hPEf);
}

// Calculate the kinetic crossing energy at a state
CrossingEnergy TriPendModel::crossingEnergyKE(double thdot) const
{
	// Calculate the required kinetic crossing energy
	double thdotsq = thdot*thdot;
	return (thdot >= 0.0 ? CrossingEnergy(-thdotsq, thdotsq) : CrossingEnergy(thdotsq, -thdotsq));
}
// EOF