#include "nimbroStyleTools.h"
#include "globaldefinitions.h"
#include "camera_parameters.h"


float NimbroStyleTools::signum(float a)
{
	if( a <= 0 ) 
		return -1;
	return 1;
}


Vec2f NimbroStyleTools::lotPunktAufGerade( const Vec2f& p, const Vec2f& g1, const Vec2f& g2 )
{
	Vec2f dir = g2-g1;
	float norm2 = dir.norm2();
	return g1 - ((g1-p) * dir)/norm2 * dir;
}

float NimbroStyleTools::abstandPunktGerade( const Vec2f& p, const Vec2f& g1, const Vec2f& g2 )
{
	return (lotPunktAufGerade( p, g1, g2 ) - p).norm();
}

float NimbroStyleTools::undistCamPixelX_to_undistCamPlaneX(float px)
{
	return (px-CamParam::cx)/CamParam::fx;
}

float NimbroStyleTools::undistCamPixelY_to_undistCamPlaneY(float py)
{
	return (py-CamParam::cy)/CamParam::fy;
}

float NimbroStyleTools::undistCamPlaneX_to_undistCamPixelX(float px)
{
	return px*CamParam::fx - CamParam::cx;
}

float NimbroStyleTools::undistCamPlaneY_to_undistCamPixelY(float py)
{
	return py*CamParam::fy - CamParam::cy;
}



