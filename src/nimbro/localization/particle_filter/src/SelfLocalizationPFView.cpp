#include "stdafx.h"

#include "SelfLocalizationPFView.h"

#include "gsl/gsl_matrix.h"
#include "gsl/gsl_linalg.h"

#include <math.h>

#define sl m_selfLocalization
typedef Particle < Vec3f, Mat22f > ParticleT;
typedef ParticleSet < Vec3f, Mat22f > ParticleSetT;

void
gsl_err_handler( const char * reason,
                 const char * file,
                 int          line,
                 int          gsl_errno ) { }

SelfLocalizationPFView::SelfLocalizationPFView( SelfLocalizationPF * selfLocalization )
	: m_selfLocalization( selfLocalization )
	, m_clickx( 0 )
	, m_clicky( 0 )
#if TEENSIZE_FIELD
	, scale(65.f)                                                 // 1m in pixels
#else
	, scale(75.f)                                                 // 1m in pixels
#endif
{

	LOGBRUSH logBrush;

	logBrush.lbStyle = BS_HOLLOW;
	transparentBrush = CreateBrushIndirect( &logBrush );
	blueBrush = CreateSolidBrush( 0xFF7F00 );
	yellowBrush = CreateSolidBrush( 0x00FFFF );
	redBrush = CreateSolidBrush( 0x0000FF );

	greenBrush = CreateSolidBrush( 0x00FF00 );
	orangeBrush = CreateSolidBrush( 0x007FFF );
	grayBrush = CreateSolidBrush( 0x7F7F7F );

	magentaBrush = CreateSolidBrush( 0xFF00FF );
	cyanBrush = CreateSolidBrush( 0xFFFF00 );

	redPen = ::CreatePen( PS_SOLID, 3, 0x0000FF );
	greenPen = ::CreatePen( PS_SOLID, 3, 0x00FF00 );
	thinPen = ::CreatePen( PS_SOLID, 1, 0x000000 );
	dashedPen = ::CreatePen( PS_DASH, 1, 0xDFDFDF );
	grayPen = ::CreatePen( PS_SOLID, 1, 0xDFDFDF );
	thickPen = ::CreatePen( PS_SOLID, 3, 0x000000 );
	thickGrayPen = ::CreatePen( PS_SOLID, 3, 0x7F7F7F );
}

void
SelfLocalizationPFView::draw( HDC hDC )
{
	if(!sl)
		return;
	HBRUSH whiteBrush = CreateSolidBrush( 0xFFFFFF );
	SelectObject( hDC, whiteBrush );
	Rectangle( hDC, 0, 0, 80 + 3 * 200, 80 + 3 * 140 );

	// * Draw the field lines. * //
	const int dx = ( int ) scale * CCV::fieldLength / 200;
	const int dy = ( int ) scale * CCV::fieldWidth / 200;
	SelectObject( hDC, transparentBrush );
	Rectangle( hDC, centerX - dx, centerY - dy, centerX + dx, centerY + dy );
	MoveToEx( hDC, centerX, centerY - dy, NULL );
	LineTo( hDC, centerX, centerY + dy );


	// * Draw the center circle. * //
	const int circleSize = ( int ) scale * CCV::centerCircleDiameter / 200;
	Ellipse( hDC, centerX - circleSize, centerY - circleSize, centerX + circleSize, centerY + circleSize );

	// * Draw the markers. * //
	for ( int m = YellowMarker; m <= BlueMarker; m++ ) {  //<- LiuW: Actually we only have these two Markers left at RoboCup 2009,
                                                          //<-       but Jorg says it's okay to leave the For statement. I agree.
		const int mx = ( int ) ( sl->m_worldObjectPos[ m ].y * scale + centerX );
		const int my = ( int ) ( sl->m_worldObjectPos[ m ].x * scale + centerY );
		const int size = ( int ) ( scale / 10 );
		MoveToEx( hDC, mx - size, my, NULL );
		LineTo( hDC, mx + size + 1, my );
		MoveToEx( hDC, mx, my - size, NULL );
		LineTo( hDC, mx, my + size + 1 );
	}

	// * Draw the goals. * //
	for ( int g = -1; g <= 1; g++ ) {
		if ( g == 0 ) {
			continue;
		}
		int gdx = ( int ) ( g * scale * max(20,CCV::goalDepth) / 100 );
		int gdy = ( int ) ( scale * CCV::goalWidth / 200 );
		if ( g < 0 ) {
			SelectObject( hDC, yellowBrush );
		}
		else {
			SelectObject( hDC, blueBrush );
		}
		Rectangle( hDC, centerX - g * dx - gdx, centerY - gdy, centerX - g * ( dx - 1 ), centerY + gdy );

		// * Draw goal area. * //
		const int gax = ( int ) ( scale * CCV::goalAreaDepth / 100 );
		const int gay = ( int ) ( scale * CCV::goalAreaWidth / 200 );
		MoveToEx( hDC, centerX - g * dx, centerY - gay, NULL );
		LineTo( hDC, centerX - g * ( dx - gax ), centerY - gay );
		LineTo( hDC, centerX - g * ( dx - gax ), centerY + gay );
		LineTo( hDC, centerX - g * dx, centerY + gay );
	}
#ifdef DRIBBLING_CHALLENGE
		if ( 1 ) {
			int s = 15;
			CoordinateXY pos;
			SelectObject( hDC, thickPen );
			pos.x = ( int ) ( sl->m_worldObjectPos[ BlueCyanPole ].y * scale );
			pos.y = ( int ) ( sl->m_worldObjectPos[ BlueCyanPole ].x * scale );
			pos.x += centerX;
			pos.y += centerY;
			SelectObject( hDC, cyanBrush );
			Ellipse( hDC, pos.x - s, pos.y - s, pos.x + s, pos.y + s );
			pos.x = ( int ) ( sl->m_worldObjectPos[ YellowCyanPole ].y * scale );
			pos.y = ( int ) ( sl->m_worldObjectPos[ YellowCyanPole ].x * scale );
			pos.x += centerX;
			pos.y += centerY;
			Ellipse( hDC, pos.x - s, pos.y - s, pos.x + s, pos.y + s );
			SelectObject( hDC, magentaBrush );
			pos.x = ( int ) ( sl->m_worldObjectPos[ MagentaPole ].y * scale );
			pos.y = ( int ) ( sl->m_worldObjectPos[ MagentaPole ].x * scale );
			pos.x += centerX;
			pos.y += centerY;
			Ellipse( hDC, pos.x - s, pos.y - s, pos.x + s, pos.y + s );
		}
#endif


	// * Draw poles. * //
	for ( int wo = YellowPole; wo <= BluePole; wo++ ) {
		CoordinateXY pos;
		pos.x = ( int ) ( sl->m_worldObjectPos[ wo ].y * scale );
		pos.y = ( int ) ( sl->m_worldObjectPos[ wo ].x * scale );

		HBRUSH brush1 = blueBrush;
		HBRUSH brush2 = yellowBrush;
		SelectObject( hDC, thickPen );
		if ( wo == YellowPole) {
			brush1 = yellowBrush;
			brush2 = blueBrush;
		}
		int px = centerX + pos.x;
		int py = centerY + pos.y;
		const int size = 15;
		const int size2 = ( size - 1 ) * 2 / 3;
		const int size3 = ( size - 1 ) * 1 / 3;
		SelectObject( hDC, brush1 );
		Ellipse( hDC, px - size, py - size, px + size, py + size );
		SelectObject( hDC, thinPen );
		SelectObject( hDC, brush2 );
		Ellipse( hDC, px - size2, py - size2, px + size2, py + size2 );
		SelectObject( hDC, brush1 );
		Ellipse( hDC, px - size3, py - size3, px + size3, py + size3 );
	}
	float fldmin = 1E6, fldmax = -1E6;
	if ( 0 ) {
		;
	}
	else if ( 0 ) {
		Vec2f mypos( sl->m_mean.getXYVec( ) );

		// * Draw line likelihood field. * //
		pair < float, float > minmax = sl->m_distToWhiteGrid.getMinMax( mypos );
		float fldmin = minmax.first, fldmax = minmax.second;
		float range = fldmax - fldmin;
		for ( int y = 0; y < sl->m_distToWhiteGrid.mYdim; y++ ) {
			for ( int x = 0; x < sl->m_distToWhiteGrid.mXdim; x++ ) {
				float f = sl->m_distToWhiteGrid.get( Vec2i( x, y ) );
				float d2 = ( mypos - sl->m_distToWhiteGrid.getGridToMetric( x, y ) ).norm2( );
				f = RcMath::gaussian( f, 0, sl->m_distToWhiteGrid.getSigma( d2 ) );
				//if(f < 0.01) continue;
				Vec2f gpos = sl->m_distToWhiteGrid.getGridToMetric( x, y );
				int px = ( int ) ( centerX + gpos.y * scale );
				int py = ( int ) ( centerY + gpos.x * scale );
				const int size = 2;
				unsigned char c = ( unsigned char )( 256 - max( 1, min( 256, ( f - fldmin ) / range * 256 ) ) );
				HBRUSH myBrush = CreateSolidBrush( GetNearestColor( hDC, RGB( c, c, c ) ) );
				HPEN myPen = CreatePen( PS_SOLID, 1, GetNearestColor( hDC, RGB( c, c, c ) ) );
				SelectObject( hDC, myBrush );
				SelectObject( hDC, myPen );
				Rectangle( hDC, px - size, py - size, px + size, py + size );
				DeleteObject( myPen );
				DeleteObject( myBrush );
			}
		}
	}
	else if ( 0 && PARTICLE_DEBUGGING && m_clickx != 0 && m_clicky != 0 ) {
		// * Draw selected particles only. * //
		Vec2f clickpos = Vec2f( ( float ) m_clicky - centerY, ( float ) m_clickx - centerX ) / scale;

		float radius = 0.2;                                                  // m
		const ParticleSetT & particles = sl->getParticles( );
		const ParticleT * best_particle = NULL;
		float best_particle_lh = -1E6;
		for ( unsigned int p = 0; p < sl->m_currNumParticles; p++ ) {
			const Vec3f & state = particles[ p ].getState( );
			if ( ( state.getXYVec( ) - clickpos ).norm2( ) < radius * radius ) {
				float new_lh = exp( particles[ p ].getWeight( ) );
				if ( new_lh > best_particle_lh ) {
					best_particle_lh = new_lh;
					best_particle = &( particles[ p ] );
				}
			}
		}
		if ( best_particle != NULL ) {
			float x = best_particle->getState( ).x;
			float y = best_particle->getState( ).y;

			int size = ( int ) ( best_particle_lh * 10.f + 0.5f );

			CoordinateXY pos;
			pos.x = ( int ) ( y * scale );
			pos.y = ( int ) ( x * scale );

			int px = centerX + pos.x - size / 2;
			int py = centerY + pos.y - size / 2;
			Ellipse( hDC, px, py, px + size, py + size );

			float orn = best_particle->getState( ).z;
			float ori_x = 0.08f * scale * cosf( orn );
			float ori_y = -0.08f * scale * sinf( orn );

			//SelectObject( hDC, thickPen );
			px += size / 2;
			py += size / 2;
			MoveToEx( hDC, px, py, NULL );
			LineTo( hDC, ( int ) ( px + ori_x ), ( int ) ( py + ori_y ) );

#if PARTICLE_DEBUGGING
				typedef vector < pair < std::wstring, Vec3f > > DebugVecT;
				DebugVecT::const_iterator it;
				for ( it = best_particle->m_DebuggingVector.begin( ); it != best_particle->m_DebuggingVector.end( ); it++ ) {
					SelectObject( hDC, redBrush );
					size = 4;
					const std::wstring & s = it->first;
					const Vec3f & v = it->second;

					pos.x = ( int ) ( v.y * scale );
					pos.y = ( int ) ( v.x * scale );

					int px = centerX + pos.x - size / 2;
					int py = centerY + pos.y - size / 2;
					Ellipse( hDC, px - size, py - size, px + size, py + size );
				}
#endif
		}
	}
	else if ( 0 ) {
		// * Draw particle grid. * //
		const int res = 4;
		const int sizey = ( CCV::fieldLength + 100 ) / res;
		const int sizex = ( CCV::fieldWidth + 100 ) / res;
		float maxlikehood[ sizey ][ sizex ];
		float bestorientn[ sizey ][ sizex ];
		memset( &maxlikehood, 0, sizeof ( maxlikehood ) );
		const ParticleSetT & particles = sl->getParticles( );
		SelectObject( hDC, redBrush );
		SelectObject( hDC, thinPen );

		Vec3f maxLhState;
		float maxLhStateVal = -1E6;

		for ( unsigned int p = 0; p < sl->m_currNumParticles; p++ ) {
			const Vec3f & state = particles[ p ].getState( );
			Vec2i gpos = sl->m_distToWhiteGrid.getMetricToGrid( state.x, state.y );
			if ( ( gpos.y < 0 ) || ( gpos.x < 0 ) || ( gpos.y >= sizey ) || ( gpos.x >= sizex ) ) {
				continue;
			}
			float & old_lh = maxlikehood[ gpos.y ][ gpos.x ];
			float new_lh = exp( particles[ p ].getWeight( ) );
			if ( maxLhStateVal < new_lh ) {
				maxLhState = state;
				maxLhStateVal = new_lh;
			}
			if ( old_lh < new_lh ) {
				old_lh = new_lh;
				bestorientn[ gpos.y ][ gpos.x ] = state.z;
			}
		}
		for ( int y = -CCV::fieldLength / 2 - 30; y < CCV::fieldLength / 2 + 30; y += res ) {
			for ( int x = -CCV::fieldWidth / 2 - 30; x < CCV::fieldWidth / 2 + 30; x += res ) {
				Vec2i gpos = sl->m_distToWhiteGrid.getCentimetricToGrid( ( float ) x, ( float ) y );
				if ( ( gpos.y < 0 ) || ( gpos.x < 0 ) || ( gpos.y >= sl->m_distToWhiteGrid.mYdim ) || ( gpos.x >= sl->m_distToWhiteGrid.mXdim ) ) {
					continue;
				}
				float & lh = maxlikehood[ gpos.y ][ gpos.x ];
				int size = ( int ) ( lh * 10.f + 0.5f );
				//size += 1;
				if ( size < 1 ) {
					continue;
				}
				//size = min(size,7);
				CoordinateXY pos;
				pos.x = ( int ) ( y / 100.f * scale );
				pos.y = ( int ) ( x / 100.f * scale );

				int px = centerX + pos.x - size / 2;
				int py = centerY + pos.y - size / 2;
				Ellipse( hDC, px, py, px + size, py + size );

				float orn = bestorientn[ gpos.y ][ gpos.x ];
				float ori_x = 0.08f * scale * cosf( orn );
				float ori_y = -0.08f * scale * sinf( orn );

				//SelectObject( hDC, thickPen );
				px += size / 2;
				py += size / 2;
				MoveToEx( hDC, px, py, NULL );
				LineTo( hDC, ( int ) ( px + ori_x ), ( int ) ( py + ori_y ) );
			}
		}
		RECT rect;
		rect.top = ( LONG ) ( centerY + ( scale * CCV::fieldWidth + 100 ) / 200 + 20 );
		rect.left = centerX - 50;
		rect.right = rect.left + 300;
		rect.bottom = rect.top + 30;

		static const int strlen = 100;
		_TCHAR str[ strlen ];
		_stprintf_s( str, strlen, _T( "WARNING: PARTICLE GRID MODE! %i" ), sl->m_importance_time );
		SelectObject( hDC, redBrush );
		SelectObject( hDC, greenPen );
		Rectangle( hDC, rect.left, rect.top, rect.right, rect.bottom );
		DrawText( hDC, str, int ( _tcsclen( str ) ), &rect, 0 );
	}
	else {

		// * Draw particles. * //
		ParticleSetT * particles;
		SelectObject( hDC, redBrush );
		SelectObject( hDC, thinPen );
		int numParticles;
		if ( sl->m_hasResampled ) {
			particles = &sl->m_particles[ 1 - sl->m_currParticleSetIdx ];
			numParticles = sl->m_prevNumParticles;
		}
		else {
			particles = &sl->m_particles[ sl->m_currParticleSetIdx ];
			numParticles = sl->m_currNumParticles;
		}
		float maxlh = -1e10;
		for ( int p = 0; p < numParticles; p++ ) {
			float lh = ( *particles )[ p ].getWeight( );
			if ( maxlh < lh ) {
				maxlh = lh;
			}
		}
		for ( int p = 0; p < numParticles; p++ ) {
			const ParticleT & particle = ( *particles )[ p ];
			const Vec3f & state = particle.getState( );

			CoordinateXY pos;
			pos.x = ( int ) ( state.y * scale );
			pos.y = ( int ) ( state.x * scale );

			const int size = ( int ) ( exp( particle.getWeight( ) - maxlh ) * 8 + 0.5 );

			int px = centerX + pos.x - size / 2;
			int py = centerY + pos.y - size / 2;

			Ellipse( hDC, px, py, px + size + 1, py + size + 1 );

			float ori_x = ( size / 4.f ) * 0.08f * scale * cosf( state.z );
			float ori_y = -( size / 4.f ) * 0.08f * scale * sinf( state.z );

			//		SelectObject( hDC, thickPen );
			px += size / 2;
			py += size / 2;
			MoveToEx( hDC, px, py, NULL );
			LineTo( hDC, ( int ) ( px + ori_x ), ( int ) ( py + ori_y ) );

		}
	}

	// * Draw pose estimate. * //
	SelectObject( hDC, greenBrush );
	SelectObject( hDC, thickPen );
	const Vec3f & state = sl->m_mean;
	CoordinateXY pos;
	pos.x = ( int ) ( state.y * scale );
	pos.y = ( int ) ( state.x * scale );

	int px = centerX + pos.x;
	int py = centerY + pos.y;
	const int size = 3 + ( int ) ( sl->m_confidence * 7 );
	Ellipse( hDC, px - size, py - size, px + size, py + size );

	float ori_x = ( 0.1f + sl->m_confidence ) * 0.12f * scale * cosf( state.z );
	float ori_y = sl->m_confidence * -0.12f * scale * sinf( state.z );

	MoveToEx( hDC, px, py, NULL );
	LineTo( hDC, ( int ) ( px + ori_x ), ( int ) ( py + ori_y ) );

	// pose covariance
	gsl_set_error_handler( &gsl_err_handler );
	SelectObject( hDC, thickGrayPen );
	gsl_matrix * cov = gsl_matrix_alloc( 2, 2 );

	for ( unsigned int i = 0; i < 2; i++ ) {
		for ( unsigned int j = 0; j < 2; j++ ) {
			gsl_matrix_set( cov, i, j, sl->m_cov.Mij[ i ][ j ] );
		}
	}
	if ( gsl_linalg_cholesky_decomp( cov ) != GSL_EDOM ) {
		const float cov_mult = 3.f;
		float sampleAngle = 0;
		const float sampleAngleInc = 2.f * M_PI / 20.f;
		float last_ori_y = ( float ) ( cov_mult * scale * gsl_matrix_get( cov, 0, 0 ) );
		float last_ori_x = ( float ) ( cov_mult * scale * gsl_matrix_get( cov, 1, 0 ) );
		while ( sampleAngle <= 2.f * M_PI ) {
			sampleAngle += sampleAngleInc;
			ori_y = ( float ) ( cov_mult * scale * ( gsl_matrix_get( cov, 0, 0 ) * cosf( sampleAngle ) ) );
			ori_x = ( float ) ( cov_mult * scale * ( gsl_matrix_get( cov, 1, 0 ) * cosf( sampleAngle ) + gsl_matrix_get( cov, 1, 1 ) * sinf( sampleAngle ) ) );
			MoveToEx( hDC, ( int ) ( px + last_ori_x ), ( int ) ( py + last_ori_y ), NULL );
			LineTo( hDC, ( int ) ( px + ori_x ), ( int ) ( py + ori_y ) );
			last_ori_x = ori_x;
			last_ori_y = ori_y;
		}
	}
	gsl_matrix_free( cov );
	gsl_set_error_handler( NULL );

	// * Draw observations relative to mean pose. * //
	vector < LandmarkObservation > & obs = sl->m_observations;
	for ( int i = 0; i < ( int ) obs.size( ); i++ ) {
		LandmarkObservation & l = obs[ i ];
		if ( l.getType( ) == LineWO ) {
			Pose p;
			Vec2f lpos( l.getX( ), l.getY( ) );
			lpos.rotate( state.z );
			p.init( state.x + lpos.x, state.y + lpos.y, 0, 0.5 );

			CoordinateXY pos;
			pos.x = ( int ) ( p.y( ) * scale );
			pos.y = ( int ) ( p.x( ) * scale );

			int lx = centerX + pos.x;
			int ly = centerY + pos.y;

			//MoveToEx(hDC,px,py,NULL);
			//LineTo(hDC,lx,ly);
			int size = 2;
			SelectObject( hDC, greenBrush );
			Ellipse( hDC, lx - size, ly - size, lx + size, ly + size );
		}
	}
	const int strlen = 200;
	_TCHAR str[ strlen ];
	RECT rect;
	rect.top = ( LONG ) ( centerY + ( scale * CCV::fieldWidth + 100 ) / 200 + 40 );
	rect.left = centerX - 200;
	rect.right = rect.left + 400;
	rect.bottom = rect.top + 30;

	_stprintf_s( str, strlen, _T( "c(%4.2f,%4.2f,%4.2f), N=%d, Neff=%4.2f, R:%d, U:%2.2f" ),
	            sl->m_lastControlInput.x,
	            sl->m_lastControlInput.y,
	            sl->m_lastControlInput.z,
	            sl->m_currNumParticles,
	            sl->m_effectiveSampleSize,
	            ( sl->m_hasResampled ? 1 : 0 ),
				sl->m_uniformReplacementProbability);
	DrawText( hDC, str, int ( _tcsclen( str ) ), &rect, 0 );

	RECT rect2;
	rect2.top = ( LONG ) ( centerY + ( scale * CCV::fieldWidth ) / 200 + 65 );
	rect2.left = centerX - 200;
	rect2.right = rect2.left + 400;
	rect2.bottom = rect2.top + 30;

	_stprintf_s( str, strlen, _T( "p(%4.2f,%4.2f,%4.2f), obsratio=%4.2f, c=%4.2f" ),
	             sl->m_mean.x,
	             sl->m_mean.y,
	             sl->m_mean.z,
	             ( float ) ( sl->m_fastAverageLikelihood / sl->m_slowAverageLikelihood ),
	             sl->m_confidence );
	DrawText( hDC, str, int ( _tcsclen( str ) ), &rect2, 0 );
}
