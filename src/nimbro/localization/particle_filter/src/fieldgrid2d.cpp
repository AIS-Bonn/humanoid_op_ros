#include "stdafx.h"
#include "fieldgrid2d.h"
#include "imgarraytools.h"

namespace FieldGrid2D {

pair < float, float >
DistToWhiteGrid::getMinMax( Vec2f mypos )
{
	float hi = -1E6, lo = 1E6;

	for ( int y = 0; y < mYdim; y++ ) {
		for ( int x = 0; x < mXdim; x++ ) {
			float f = get( Vec2i( x, y ) );
			float d2 = ( mypos - getGridToMetric( x, y ) ).norm2( );
			f = RcMath::gaussian( f, 0, getSigma( d2 ) );
			if ( f > hi ) {
				hi = f;
			}
			if ( f < lo ) {
				lo = f;
			}
		}
	}
	return pair < float, float > ( lo, hi );
}

}
