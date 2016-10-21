// Walk and kick: Behaviour utilities
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <walk_and_kick/wak_utils.h>
#include <rc_utils/math_vec_mat.h>

// Namespaces
using namespace walk_and_kick;
using namespace rc_utils;

//
// WAKUtils class
//

// Calculate the tangents from a point to a circle of given centre and radius
void WAKUtils::calcCircleTangent(TangentInfo& TI, const Vec2f& point, const Vec2f& centre, float radius)
{
	// Calculate the vector, distance and angle from the point to the centre of the circle
	Vec2f vec = centre - point;
	float dist = vec.norm();
	float centreAngle = eigenAngleOf(vec);

	// Calculate the required tangents
	if(dist <= radius)
	{
		TI.posAngle = centreAngle + M_PI_2;
		TI.negAngle = centreAngle - M_PI_2;
		TI.maxAdjust = 0.0f;
		TI.length = 0.0f;
	}
	else
	{
		float alpha = asin(radius / dist);
		TI.posAngle = centreAngle + alpha;
		TI.negAngle = centreAngle - alpha;
		TI.maxAdjust = coerce<float>(M_PI - 2.0f*alpha, 0.0f, M_PI);
		TI.length = sqrt(dist*dist - radius*radius);
	}

	// Wrap the returned tangent angles to (-pi,pi]
	TI.posAngle = picut(TI.posAngle);
	TI.negAngle = picut(TI.negAngle);
}

// Calculate a path from one position to another, subject to a halo (circle) of given centre and radius
float WAKUtils::calculateHaloPath(Vec2f& fromNormal, Vec2fArray& path, const Vec2f& fromPos, const Vec2f& toPos, const Vec2f& centre, float radius, bool smoothFromNormal)
{
	// Sanity check inputs
	if(radius < 0.0f)
		radius = -radius;

	// Assign just the start position to the path array
	path.assign(1, fromPos);

	// Calculate the from and to radii
	Vec2f fromVec = fromPos - centre;
	float fromRadius = fromVec.norm();
	Vec2f toVec = toPos - centre;
	float toRadius = toVec.norm();

	// Handle case explicitly where from or to is at the centre of the circle, or the circle is just a point
	if(fromRadius <= 0.0f || toRadius <= 0.0f || radius <= 0.0f)
	{
		fromNormal = toPos - fromPos;
		float pathLen = fromNormal.norm();
		if(pathLen > 0.0f)
			fromNormal /= pathLen;
		else
			fromNormal << 1.0f, 0.0f;
		path.push_back(toPos);
		return pathLen;
	}

	// Calculate the from unit vectors fx and fy
	Vec2f fxhat = fromVec / fromRadius;
	Vec2f fyhat = eigenRotatedCCW90(fxhat);

	// Calculate the to unit vector tx
	Vec2f txhat = toVec / toRadius;

	// Calculate the from and to projected circle points
	Vec2f projFrom = centre + radius*fxhat;
	Vec2f projTo = centre + radius*txhat;

	// Calculate the from circle point angles
	float deltaFp = 0.0f, deltaFn = 0.0f;
	if(fromRadius > radius)
	{
		deltaFp = acos(radius / fromRadius);
		deltaFn = -deltaFp;
	}

	// Calculate the to angle
	float deltaT = atan2(toVec.dot(fyhat), toVec.dot(fxhat));

	// Calculate the to circle point angles
	float deltaTDev = 0.0f;
	if(toRadius > radius)
		deltaTDev = acos(radius / toRadius);
	float deltaTp = deltaT + deltaTDev;
	float deltaTn = deltaT - deltaTDev;

	// See which of the two path endpoints are inside the circle
	bool insideF = (fromRadius < radius);
	bool insideT = (toRadius < radius);

	// Construct the required solution path
	if(deltaTp >= deltaFn && deltaFp >= deltaTn) // If this is true then no circular arc component is required for the path...
	{
		// Construct the required straight line path
		if(insideF == insideT)
		{
			// Path: Straight line
			fromNormal = toPos - fromPos;
			float pathLen = fromNormal.norm();
			if(pathLen > 0.0f)
				fromNormal /= pathLen;
			else
				fromNormal << 1.0f, 0.0f;
			path.push_back(toPos);
			return pathLen;
		}
		else if(insideF)
		{
			// Path: Radial from line followed by straight line
			Vec2f segment1 = projFrom - fromPos;
			Vec2f segment2 = toPos - projFrom;
			float norm1 = segment1.norm();
			float norm2 = segment2.norm();
			if(smoothFromNormal && norm2 > 0.0f)
				fromNormal = segment2 / norm2;
			else
				fromNormal = fxhat;
			path.push_back(projFrom);
			path.push_back(toPos);
			return norm1 + norm2;
		}
		else
		{
			// Path: Straight line followed by radial to line
			Vec2f segment1 = projTo - fromPos;
			Vec2f segment2 = toPos - projTo;
			float norm1 = segment1.norm();
			float norm2 = segment2.norm();
			if(norm1 > 0.0f)
				fromNormal = segment1 / norm1;
			else
				fromNormal = -txhat;
			path.push_back(projTo);
			path.push_back(toPos);
			return norm1 + norm2;
		}
	}
	else // A circular arc component is required for the path...
	{
		// Constants
		const int numArcPoints = 30;

		// Calculate the required circle point angles on the path
		float deltaFu = deltaFp;
		float deltaTu = deltaTn;
		if(deltaT < 0.0f)
		{
			deltaFu = deltaFn;
			deltaTu = deltaTp;
		}

		// Precompute trigonometric values
		float cosDeltaFu = cos(deltaFu);
		float sinDeltaFu = sin(deltaFu);
		float cosDeltaTu = cos(deltaTu);
		float sinDeltaTu = sin(deltaTu);

		// Calculate the from normal path vector
		if(insideF && !smoothFromNormal)
			fromNormal = fxhat;
		else
		{
			fromNormal = cosDeltaFu*fyhat - sinDeltaFu*fxhat;
			if(deltaT < 0.0f)
				fromNormal = -fromNormal;
			eigenNormalize(fromNormal); // Note: Just for the last few epsilon, as fromNormal should theoretically be a unit vector here already
		}

		// Calculate the path length
		Vec2f midFrom = centre + radius*(cosDeltaFu*fxhat + sinDeltaFu*fyhat);
		Vec2f midTo = centre + radius*(cosDeltaTu*fxhat + sinDeltaTu*fyhat);
		float pathLen = (midFrom - fromPos).norm() + radius*fabs(deltaTu - deltaFu) + (toPos - midTo).norm();

		// Populate the path points array
		path.push_back(midFrom);
		float dd = (deltaTu - deltaFu) / (numArcPoints + 1);
		for(int i = 1; i < numArcPoints + 1; i++)
		{
			float delta = deltaFu + i*dd;
			path.push_back(centre + radius*(cos(delta)*fxhat + sin(delta)*fyhat));
		}
		path.push_back(midTo);
		path.push_back(toPos);

		// Return the path length
		return pathLen;
	}
}

// Calculate the wedge from a particular angle, to another particular angle, possibly forcing the sign of the output in the process
float WAKUtils::calcWedge(float fromAngle, float toAngle, int forceSign)
{
	// Calculate and return the required wedge in the range (-pi,pi], or in the range (-2*pi,2*pi) if a sign is forced
	float wedge = picut(toAngle - fromAngle);
	if(wedge < 0.0f && forceSign > 0)
		wedge += M_2PI;
	else if(wedge > 0.0f && forceSign < 0)
		wedge -= M_2PI;
	return wedge;
}
// EOF