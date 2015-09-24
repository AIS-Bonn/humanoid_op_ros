#ifndef NIMBROSTYLETOOLS_H
#define NIMBROSTYLETOOLS_H

#include "Vec2f.h"
#include "Vec2i.h"
#include <math.h>
#include <algorithm>
#include <boost/concept_check.hpp>
#include <algorithm>
#include <fstream>

#define MAX_OBJECT_DIST 700

namespace NimbroStyleTools
{	
	float signum( float a);
	
	inline int round(float d)
	{
		return d >= 0.f ? int(d + 0.5f) : int(d - int(d-1.f) + 0.5f) + int(d-1.f);
	};
	
	inline float vangle(const Vec2f& v1, const Vec2f& v2 )
	{
		float norm = v2.norm()*v1.norm();
		if(norm<0.00001f) return 0.f;
		float innen = v1*v2 / norm;
		if( fabsf(innen) >= 1.f)
			return 0.f;
		float vorz = signum( v1.cross( v2) );
		return ( vorz * (float)acos( /*0.99999f* */ innen ));
	}
	
	inline float deg2rad(float d)
	{
		return d*0.017453292512f; // 180/PI
	}
	
	inline float rad2deg(float r)
	{
		return r*57.29577951f; // 180/PI
	}
	
	template<class T>
	inline T piCut(T x)
	{
		return  x < 0 ? fmod(x-M_PI, 2.0*M_PI)+M_PI : fmod(x+M_PI, 2*M_PI)-M_PI;
	}
	
	Vec2f lotPunktAufGerade( const Vec2f& p, const Vec2f& g1, const Vec2f& g2 );
	float abstandPunktGerade( const Vec2f& p, const Vec2f& g1, const Vec2f& g2 );
	
	float undistCamPixelX_to_undistCamPlaneX(float px);
	float undistCamPixelY_to_undistCamPlaneY(float py);
	
	float undistCamPlaneX_to_undistCamPixelX(float px);
	float undistCamPlaneY_to_undistCamPixelY(float py);
	

	template < class T, int W, int H >
	void
	writePGM( const T ( & arr )[ H ][ W ], const char* filename)
	{
		double maxval = *std::max_element((T*)arr,(T*)arr+H*W);
		maxval = std::max((double)maxval,0.00001);
		std::ofstream os(filename);
		os << "P2 "<<W << " " << H<<" 255"<<std::endl;
		for ( int y = 0; y < H; y++ ) {
			for ( int x = 0; x < W; x++ ) {
				//os.width ( 3 );
				os<<(int)(255 * arr[H-1-y][x] / maxval)<<" ";
			}
			os<<std::endl;
		}
		os.close();
	}

		// taken from http://www.wol.net.pk/mtshome/cppComputerGraphics.html#Line
	// and modified for callback
	// Idea: Track along line (x1,y1)-(x2,y2) and call the CallBack object with the coordinates
	//   stop if callBack returns false
// 	template < class T >
// 	bool
// 	bresenham( T & callBack, const Vec2f a, const Vec2f b, int xres, int yres, float xoff, float yoff, float gxoff, float gyoff )
// 	{
// 		return bresenham( callBack, ( int ) ( ( a.x + xoff ) / xres + gxoff ), ( int ) ( ( a.y + yoff ) / yres + gyoff ), ( int ) ( ( b.x + xoff ) / xres + gxoff ), ( int ) ( ( b.y + yoff ) / yres + gyoff ) );
// 	}
// 
// 	template < class T >
// 	bool
// 	bresenham( T & callBack, const Vec2i a, const Vec2i b )
// 	{
// 		return bresenham( callBack, a.x, a.y, b.x, b.y );
// 	}

	template < class T >
	bool
	bresenham( T & callBack, const int x_1, const int y_1, const int x_2, const int y_2 )
	{
		int x1 = x_1;
		int y1 = y_1;
		int x2 = x_2;
		int y2 = y_2;

		if ( x_1 > x_2 ) {
			x1 = x_2;
			y1 = y_2;
			x2 = x_1;
			y2 = y_1;
		}
		int dx = abs( x2 - x1 );
		int dy = abs( y2 - y1 );
		int inc_dec = ( ( y2 >= y1 ) ? 1 : -1 );

		if ( dx > dy ) {
			int two_dy = ( 2 * dy );
			int two_dy_dx = ( 2 * ( dy - dx ) );
			int p = ( ( 2 * dy ) - dx );
			int x = x1;
			int y = y1;

			if ( !callBack( x, y ) ) {
				return false;
			}
			while ( x < x2 ) {
				x++;
				if ( p < 0 ) {
					p += two_dy;
				}
				else {
					y += inc_dec;
					p += two_dy_dx;
				}
				if ( !callBack( x, y ) ) {
					return false;
				}
			}
		}
		else {
			int two_dx = ( 2 * dx );
			int two_dx_dy = ( 2 * ( dx - dy ) );
			int p = ( ( 2 * dx ) - dy );

			int x = x1;
			int y = y1;

			if ( !callBack( x, y ) ) {
				return false;
			}
			while ( y != y2 ) {
				y += inc_dec;
				if ( p < 0 ) {
					p += two_dx;
				}
				else {
					x++;
					p += two_dx_dy;
				}
				if ( !callBack( x, y ) ) {
					return false;
				}
			}
		}
		return true;
	}

	template < class T >
	bool
	bresenham_circle( T & callBack, const float h, const float k, const float r )
	{
		return bresenham_circle( callBack, ( int ) h, ( int ) k, ( int ) r );
	}

	template < class T >
	bool
	bresenham_circle( T & callBack, const int h, const int k, const int r )
	{
		int x = 0;
		int y = r;
		int p = ( 3 - ( 2 * r ) );

		do {
			//if(!callBack((h+x),(k+y))) return false;
			//if(!callBack((h+y),(k+x))) return false;
			//if(!callBack((h+y),(k-x))) return false;
			//if(!callBack((h+x),(k-y))) return false;
			//if(!callBack((h-x),(k-y))) return false;
			//if(!callBack((h-y),(k-x))) return false;
			//if(!callBack((h-y),(k+x))) return false;
			//if(!callBack((h-x),(k+y))) return false;

			if ( !callBack( ( h + x ), ( k + y ) ) ) {
				return false;
			}
			if ( !callBack( ( h + y ), ( k + x ) ) ) {
				return false;
			}
			if ( !callBack( ( h + y ), ( k - x ) ) ) {
				return false;
			}
			if ( !callBack( ( h + x ), ( k - y ) ) ) {
				return false;
			}
			if ( !callBack( ( h - x ), ( k - y ) ) ) {
				return false;
			}
			if ( !callBack( ( h - y ), ( k - x ) ) ) {
				return false;
			}
			if ( !callBack( ( h - y ), ( k + x ) ) ) {
				return false;
			}
			if ( !callBack( ( h - x ), ( k + y ) ) ) {
				return false;
			}
			x++;

			if ( p < 0 ) {
				p += ( ( 4 * x ) + 6 );
			}
			else {
				y--;
				p += ( ( 4 * ( x - y ) ) + 10 );
			}
		} while ( x <= y );
		return true;
	}
	
	template < class T, int W, int H >
	class MinColorChecker
	{
	private:
		T( &ivArr )[ H ][ W ];
		int ivMin;
	public:
		MinColorChecker( T ( & a )[ H ][ W ], int m ) : ivArr( a )
			, ivMin( m ) { }

		inline bool operator () ( int x, int y ) const
		{
			return ivMin <= ivArr[ y ][ x ];
		}

	};

	template < class T, int W, int H, int R >
	class MinColorCheckerRadius
	{
	private:
		T( &ivArr )[ H ][ W ];
		int ivMin;
	public:
		MinColorCheckerRadius( T ( & a )[ H ][ W ], int m ) : ivArr( a )
			, ivMin( m ) { }

		inline bool operator () ( int x, int y ) const
		{
			for(int i=x-R; i<=x+R; ++i)
				for(int j=y-R;j<=y+R; ++j)
					if(ivMin <= ivArr[ i ][ j ])
						return true;
			return false;
		}
	};
	
	template < class T, int W, int H >
	bool
	checkIfColoredOnWholeLine( T ( & arr )[ H ][ W ], int x0, int y0, int x1, int y1, int minC = 0, int radius=0 )
	{
		assert( std::min( x0, x1 ) >= radius );
		assert( std::min( y0, y1 ) >= radius );
		assert( std::max( x0, x1 ) < W-radius );
		assert( std::max( y0, y1 ) < H-radius );
		switch(radius){
		case 0:
		{
			MinColorChecker<T, W, H> blub(arr, minC);
			return bresenham(blub, x0, y0, x1, y1 );
		}
		case 1:
		{
			MinColorCheckerRadius< T, W, H, 1> blub(arr, minC);
			return bresenham( blub, x0, y0, x1, y1 );
		}
		default: 
			assert(false);
			return false;
		}
	}
	

	////////////////////////////////////////////////////////////////////////
	// sum over array window (by point and offsets)
	// - count only pixels with at least v>=minval
	// image boundaries are automatically checked for
	////////////////////////////////////////////////////////////////////////
	template < class T, int W, int H >
	bool
	sumIn2DArrayRelative( const T ( & arr )[ H ][ W ], int & sum, int & num, const Vec2i pos, const int radiusx, const int radiusy, const int minval = 0 )
	{
		return sumIn2DArray( arr, sum, num, pos.x - radiusx, pos.x + radiusx + 1, pos.y - radiusy, pos.y + radiusy + 1, minval );
	}

	template < class T, int W, int H >
	bool
	sumIn2DArrayRelative( const T ( & arr )[ H ][ W ], int & sum, int & num, const int xpos, const int ypos, const int radiusx, const int radiusy, const int minval = 0 )
	{
		return sumIn2DArray( arr, sum, num, xpos - radiusx, xpos + radiusx + 1, ypos - radiusy, ypos + radiusy + 1, minval );
	}

	template < class T, int W, int H >
	bool
	sumIn2DArray( const T ( & arr )[ H ][ W ], int & sum, int & num, int minx, int maxx, int miny, int maxy, const int minval = 0 )
	{
		if ( 0 > minx ) {
			minx = 0;
		}
		if ( 0 > miny ) {
			miny = 0;
		}
		if ( 0 > maxx ) {
			maxx = 0;
		}
		if ( 0 > maxy ) {
			maxy = 0;
		}
		if ( W < minx ) {
			minx = W;
		}
		if ( H < miny ) {
			miny = H;
		}
		if ( W < maxx ) {
			maxx = W;
		}
		if ( H < maxy ) {
			maxy = H;
		}

	#ifdef _DEBUG

		assert( maxx >= minx ); //!! LiuW: Bug here if without the following two FAST_SWAPs!!!
		assert( maxy >= miny );

	#else

		if ( minx > maxx || miny > maxy  ) {

			sum = 1;
			num = 1;

			return false;

		}

	#endif

		int lsum = 0;                         // do not access sum as a reference, slow!
		num = ( maxx - minx ) * ( maxy - miny );
		for ( int y = miny; y < maxy; y++ ) {
			for ( int x = minx; x < maxx; x++ ) {
				int val = arr[ y ][ x ];
				if ( val > minval ) {
					lsum += val;
				}
			}
		}
		sum = lsum;
		return num > 0;
	}

	////////////////////////////////////////////////////////////////////////
	//  same as above, only we sum over the maximum in two values
	////////////////////////////////////////////////////////////////////////
	template < class T, int W, int H >
	bool
	sumIn2ArraysRelative( T ( & arr )[ H ][ W ], T ( & arr2 )[ H ][ W ], int & sum, int & num, Vec2i ref, int ox, int oy, int minval = 0 )
	{
		return sumIn2Arrays( arr, arr2, sum, num, ref.x - ox, ref.x + ox, ref.y - oy, ref.y + oy, minval );
	}

	template < class T, int W, int H >
	bool
	sumIn2Arrays( T ( & arr )[ H ][ W ], T ( & arr2 )[ H ][ W ], int & sum, int & num, int minx, int maxx, int miny, int maxy, int minval = 0 )
	{

	#ifdef _DEBUG

		assert( maxx >= minx ); // LiuW: It never had problem here. But this will be safer.
		assert( maxy >= miny );

	#endif

		if ( 0 > minx ) {
			minx = 0;
		}
		if ( 0 > miny ) {
			miny = 0;
		}
		if ( W < maxx ) {
			maxx = W;
		}
		if ( H < maxy ) {
			maxy = H;
		}
		sum = 0;
		num = ( maxx - minx ) * ( maxy - miny );
		for ( int y = miny; y < maxy; y++ ) {
			for ( int x = minx; x < maxx; x++ ) {
				int val = arr[ y ][ x ] + arr2[ y ][ x ];
				if ( val > minval ) {
					sum += val;
				}
			}
		}
		return true;
	}
	
	/*
	* counts color-transitions in a circle
	*/
	template < class T, int H, int W >
	struct TransitionCounter {
		Vec2i * mCircle;
		int mCircleSize8; //<- 8tel
		int mCnt;
		int mLastFrom;
		int mLastTo;
		T( & mFrom )[ H ][ W ];
		T( & mTo )[ H ][ W ];
		enum state {NOTHING,IN_FROM,IN_TO};
		int mFromToCnt;
		int mToFromCnt;
		int mode;
		static const int mMaxTransitions = 10;
		int mTransitionIdx[ mMaxTransitions ];
		int mTransitionIdxNum;
		//CM
		//this doesn't seem to have any purpose in the code anymore
// 		list < Vec2i > & mDebug;
// 		list < Vec2i > & mDebug2;
		TransitionCounter( int radius, T ( & from )[ H ][ W ], T ( & to )[ H ][ W ]/*, list < Vec2i > & debug, list < Vec2i > & debug2 */)
			: mCnt( 0 )
			, mLastFrom( 0 )
			, mLastTo( 0 )
			, mFrom( from )
			, mTo( to )
			, mFromToCnt( 0 )
			, mToFromCnt( 0 )
// 			, mDebug( debug )
// 			, mDebug2( debug2 )
			, mTransitionIdxNum( 0 )
		{
			int circle_size = 24;
			mode = NOTHING;

			switch ( radius ) {
			case 8:
				circle_size = 48;
				break;

			case 7:
				circle_size = 40;
				break;

			case 6:
				circle_size = 40;
				break;

			case 5:
				circle_size = 32;
				break;

			case 4:
				circle_size = 24;
				break;

			default:
				assert( false );
			}
			mCircle = new Vec2i[ circle_size ];
			mCircleSize8 = circle_size / 8;
		}

		~TransitionCounter( )
		{
			delete[] mCircle;
		}

		inline bool operator () ( int x, int y )
		{
			int base = mCnt / 8; //<- Integer division!
			int octant = mCnt % 8;

			if ( octant % 2 == 1 ) {base = mCircleSize8 - base - 1;}
			mCircle[ base + octant * mCircleSize8 ] = Vec2i( x, y );
			mCnt++;
			if ( ( x < 0 ) || ( y < 0 ) ) {return false;}
			if ( ( x >= W ) || ( y >= H ) ) {return false;}
			return true;
		}

		inline void
		getTXing( Vec2i & a, Vec2i & b, Vec2i & c )
		{
			assert( mTransitionIdxNum == 3 );
			a = mCircle[ mTransitionIdx[ 0 ] ];
			b = mCircle[ mTransitionIdx[ 1 ] ];
			c = mCircle[ mTransitionIdx[ 2 ] ];
		}

		inline void
		getLXing( Vec2i & a, Vec2i & b )
		{
			assert( mTransitionIdxNum == 2 );
			a = mCircle[ mTransitionIdx[ 0 ] ];
			b = mCircle[ mTransitionIdx[ 1 ] ];
		}

		inline void
		run( )
		{
			int i, f, t;

			for ( i = 0; i < mCircleSize8 * 8; i++ ) {
				//mDebug2.push_back(mCircle[i]);
				int x = mCircle[ i ].x;
				int y = mCircle[ i ].y;
				f = mFrom[ y ][ x ];
				t = mTo[   y ][ x ];
				if ( 0 );
				else if ( mode == NOTHING ){
					if(t>f)
						mode = IN_TO;
					else if(f>t)
						mode = IN_FROM; // else: stay in NOTHING until you find sth _at_all_
				}
				else if ( mode == IN_FROM  && t > 1.2f * mLastTo && f <= mLastFrom ) {
					mFromToCnt++;
					mTransitionIdx[ mTransitionIdxNum++ ] = i;
					mode = 2;
				}
				else if ( mode == IN_TO && f > 1.2f * mLastFrom && t <= mLastTo ) {
					mToFromCnt++;
					mode = 1;
				}
				mLastTo = t;
				mLastFrom = f;
			}
			for ( i = 0; i < mTransitionIdxNum; i++ ) {
				int best_val = -1;
				int best_val_idx = 0;
				for ( int p = std::max( 0, mTransitionIdx[ i ] - 2 );
					p <= std::min( 8 * mCircleSize8 - 1, mTransitionIdx[ i ] + 2 );
					p++ ) {
						t = mTo[ mCircle[ p ].y ][ mCircle[ p ].x ];
						if ( t > best_val ) {
							best_val = t;
							best_val_idx = p;
						}
				}
				mTransitionIdx[ i ] = best_val_idx;
				//mDebug.push_back(mCircle[ mTransitionIdx[i] ]);
			}
		}

	};

};

enum WorldObjectType {
	Circle,
	PositiveMarker,
	NegativeMarker,	
	Marker,
	LineWO,
	FieldLineWO,
	FL_PositiveGoal,
	FL_NegativeGoal,
	FL_CenterLine,
// #if PF_ASSOC_FIELDLINES_PER_PARTICLE // dont associate the mean pose with side lines
// 	FL_SidePositive,
// 	FL_SideNegative,
// #endif
	FL_LongPenaltyPositive,
	FL_LongPenaltyNegative,

	// * Crossings. First the general (unmatched) ones:
	LineXingT,
	LineXingX,
	LineXingL,

	// * Now the specific corners:
	LX_LeftNegativeField,
	LX_RightNegativeField,
	LX_LeftNegativePenalty,
	LX_RightNegativePenalty,
	LX_LeftPositiveField,
	LX_RightPositiveField,
	LX_LeftPositivePenalty,
	LX_RightPositivePenalty,

	// * As seen when looking at yellow goal
	TX_CentralRight,
	TX_CentralLeft,
	TX_LeftNegativePenalty,
	TX_RightNegativePenalty,
	TX_LeftPositivePenalty,
	TX_RightPositivePenalty,

	// * Now the Xings, named after their poles
	XX_PositiveSide,
	XX_NegativeSide,
	NumWorldObjects
};

class EgoObject
{
protected:
	Vec2f ivPos;
	float ivEgoOrientation;
	Vec2i ivImgPos;
	float ivDist;
	float ivAng;                          // in rad
	float ivConf;                         // [0..1]
	WorldObjectType ivWorldObjectType;
public:
	EgoObject(){
		ivConf=0.f;
	}
	// * Initialization * //
	inline void
	reset( ) {
		initCartesian( 0, 0, 0 );
	}

	inline void
	initCartesian( int ax, int ay, float c = -1 )
	{initCartesian( ( float ) ax, ( float ) ay, c );}

	inline void
	initCartesian( Vec2f p, float c = -1 )
	{
		initCartesian( p.x, p.y, c );
	}

	inline void
	initCartesian( float ax, float ay, float c = -1 )
	{
		if ( c >= 0 ) {
			ivConf = c;
		}
		ivPos.x = ax;
		ivPos.y = ay;
		ivAng = atan2( ax, ay ); //<- Axis of coordinate system are swapped: (x=-1, y=0) eq. angle=-pi/2; (x=0, y=1) eq. angle=0; (x=1, y=0) eq. angle=pi/2;
		ivDist = ivPos.norm( );
		if ( ivDist > MAX_OBJECT_DIST ) {
			initPolar( ivAng, MAX_OBJECT_DIST );
		}
	}

	inline void
	initPolar( float a, float d, float c = -1 )
	{
		if ( c >= 0 ) {
			ivConf = c;
		}
		ivAng = a;
		ivDist = d;
		if ( ivDist > MAX_OBJECT_DIST ) {
			ivDist = MAX_OBJECT_DIST;
		}
		ivPos.x = sin( ivAng ) * ivDist;
		ivPos.y = cos( ivAng ) * ivDist; //<- Swapped axis, see above
	}

	inline void
	setEgoOrientation( float ang ) {ivEgoOrientation = ang;}

	inline float
	getEgoOrientation( ) const {return ivEgoOrientation;}

	inline float &
	conf( ) {return ivConf;}

	inline const Vec2f &
	pos( ) const {return ivPos;}

	inline const Vec2i &
	img_pos( ) const {return ivImgPos;}

	inline Vec2i &
	img_pos( ) {return ivImgPos;}

	inline int &
	img_x( ) {return ivImgPos.x;}

	inline const int &
	img_x( )   const {return ivImgPos.x;}

	inline int &
	img_y( ) {return ivImgPos.y;}

	inline const int &
	img_y( )  const {return ivImgPos.y;}

	inline float
	x( )       const {return ivPos.x;}

	inline float
	y( )       const {return ivPos.y;}

	inline float
	dist( )  const {return ivDist;}

	inline float
	ang( )   const {return ivAng;}

	inline float
	conf( )  const {return ivConf;}

	inline WorldObjectType
	wotype( ) const {return ivWorldObjectType;}

	inline WorldObjectType &
	wotype( ) {return ivWorldObjectType;}

	// * Conversion * //
	operator Vec2f ( ) const {return ivPos;}
};



#endif