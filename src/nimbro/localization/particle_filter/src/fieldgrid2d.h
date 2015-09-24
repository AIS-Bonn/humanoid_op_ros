

#ifndef __FIELDGRID2D_H__
#define __FIELDGRID2D_H__

#include "pf_math.h"
#include <field_model/field_model.h>

	namespace FieldGrid2D {

/***************************************************************************
 * This class represents a grid of the field. Each grid cell has value_type
 * It should be derived to draw something on it.
 ***************************************************************************/
	template < class value_type >
	class FieldGrid2D
	{
	public:
		const int mRes;      // scale of grid (4: 1 gridcell == 4 cm)
		const float mResInv; // 1 / mRes
		const float mXoff;   // safety margin of grid in X-dir
		const float mYoff;   // safety margin of grid in Y-dir
		const int mXdim;     // size of grid (field + 2*margin)
		const int mYdim;     // size of grid (field + 2*margin)
		const int mGridXoff; // half a field
		const int mGridYoff; // half a field
		value_type * mGrid;  // the actual grid

	public:
		/** returns the resolution of the grid in cm per cell */
		inline int
		getGridResolution( ) const {return mRes;}

		/** returns the grid coordinate of a point in centimetric coordinates */
		inline Vec2i getCentimetricToGrid( const Vec2f & v ) const;

		/** returns the grid coordinate of a point in centimetric coordinates */
		inline Vec2i getCentimetricToGrid( float wx, float wy ) const;

		/** returns the grid coordinate of a point in metric coordinates */
		inline Vec2i getMetricToGrid( const Vec2f & v ) const;

		/** returns the grid coordinate of a point in metric coordinates */
		inline Vec2i getMetricToGrid( float wx, float wy ) const;

		/** returns the metric coordinates of a grid cell */
		inline Vec2f getGridToMetric( const Vec2i & v ) const;

		/** returns the metric coordinates of a grid cell */
		inline Vec2f getGridToMetric( int x, int y ) const;

	public:
		FieldGrid2D( int res, float margin = 50.f )
			: mRes( res )
			, mResInv( 1.f / mRes )
			, mXoff( margin )
			, mYoff( margin )
		{
			field_model::FieldModel* field = field_model::FieldModel::getInstance();

			mXdim = (2 * mXoff + field->width() * 100) / mRes;
			mYdim = (2 * mYoff + field->length() * 100) / mRes;
			mGridXoff = field->width() * 100 / mRes / 2;  // TODO: sollte das nicht noch plus mXoff sein?
			mGridYoff = field->length() * 100 / mRes / 2;
			mGrid = new value_type[ mYdim * mXdim ];
		}

		/** returns the value at grid-pos v */
		inline const value_type &
		get( Vec2i v ) const
		{
			assert( isInside( v ) );
			return mGrid[ v.y() * mXdim + v.x() ];
		}

		/** returns the value at grid-pos v */
		inline value_type &
		get( Vec2i v )
		{
			assert( isInside( v ) );
			return mGrid[ v.y() * mXdim + v.x() ];
		}

		/** returns the value at metric pos v */
		inline const value_type &
		getMetric( Vec2f v ) const {return get( getMetricToGrid( v ) );}

		/** returns the value at metric pos v */
		inline value_type &
		getMetric( Vec2f v ) {return get( getMetricToGrid( v ) );}

		/** returns the value at centimetric pos v */
		inline const value_type &
		getCentimetric( Vec2f v ) const {return get( getCentimetricToGrid( v ) );}

		/** returns the value at centimetric pos v */
		inline value_type &
		getCentimetric( Vec2f v ) {return get( getCentimetricToGrid( v ) );}

		/** sets the value at grid-pos v */
		inline void
		set( Vec2i v, const value_type & t ) {get( v ) = t;}

		/** sets the value at metric pos v */
		inline void
		setMetric( Vec2f v, const value_type & t ) {getMetric( v ) = t;}

		/** sets the value at centimetric pos v */
		inline void
		setCentimetric( Vec2f v, const value_type & t ) {getCentimetric( v ) = t;}

		/** checks, whether a gridpos is in the grid */
		inline bool
		isInsideX( int x ) const {return x >= 0 && x < mXdim;}

		/** checks, whether a gridpos is in the grid */
		inline bool
		isInsideY( int y ) const {return y >= 0 && y < mYdim;}

		/** checks, whether a gridpos is in the grid */
		inline bool
		isInside( Vec2i v ) const {return isInsideX( v.x ) && isInsideY( v.y );}

		/** checks, whether a metric pos is in the grid */
		inline bool
		isInsideMetricX( float x ) const
		{
			x *= 100;
			return fabs( x ) < ( field_model::FieldModel::getInstance()->width() * 100 / 2 + mXoff );
		}

		/** checks, whether a metric pos is in the grid */
		inline bool
		isInsideMetricY( float y ) const
		{
			y *= 100;
			return fabs( y ) < ( field_model::FieldModel::getInstance()->length() * 100 / 2 + mYoff );
		}

		/** checks, whether a metric pos is in the grid */
		inline bool
		isInsideMetric( Vec2f v ) const {return isInsideMetricX( v.x ) && isInsideMetricY( v.y );}

	};


/** ***********************************************************************
* This class represents the distance to the next white line on the
* field.
**************************************************************************/
	class DistToWhiteGrid : public FieldGrid2D < float >
	{
	public:
		DistToWhiteGrid( )
			: FieldGrid2D < float > ( 4, 50 )
		{ }

		inline float
		getSigma( float distance )
		{
			#define LN_HIGHER_ERROR_IF_FAR_AWAY 1
#if LN_HIGHER_ERROR_IF_FAR_AWAY
				return ( distance / 9.f + 0.4f );

#else
				return LN_SIGMA 0.4f;
#endif
		}

		/** returns the likelihood of a white piece if I am at px,py and
		 * line is relative to me at ox,oy */
		inline std::pair < float, float > getLineLikelihood( float px, float py, float ox, float oy );

		/** computes the line likelihood field by drawing lines on the grid */
		template < class PF >
		void precomputeLineLikelihoodField( const PF & pf );

		/** for drawing: determine minimum and maximum likelihood */
		std::pair < float, float > getMinMax( Vec2f mypos );

	};


/** *****************************************************************
* This Grid knows the closest world object for a grid cell
********************************************************************/
	class ClosestWorldObject : public FieldGrid2D < WorldObjectType >
	{
	public:
		ClosestWorldObject( )
			: FieldGrid2D < WorldObjectType > ( 4, 50 )
		{ }

		template < class PF >
		void init( std::vector < WorldObjectType > &, const PF & );

	};

	};

#include "fieldgrid2d.cxx"

#endif /*__FIELDGRID2D_H__*/
