
#include "SelfLocalizationPF.h"

#include <math.h>
#include <algorithm>
#include <Eigen/src/Geometry/Rotation2D.h>

namespace disabled {

double SelfLocalizationPF::landmarkLikelihoodDiscount[ NumWorldObjects ];
extern float gDeltaTFact;
extern bool   gPFmode;

// if this is defined as 1, particles are grid-like distributed in x,y,theta
// they don't move and are resetted each cycle such that instantaneous distributions
// can be visualized. Be sure to turn on ParticleGrid-Viewing in the
// SelflocalizationPFView class, such that only the best theta for each x,y is shown.
#define PARTICLE_FILTER_GRID_TEST 0


struct active_vision_comparator{
	std::map<WorldObjectType,int> levels;
	active_vision_comparator(std::map<WorldObjectType,int>& l):levels(l){}
	bool operator()(const LandmarkObservation& o, const LandmarkObservation& p){
		// returns true if o is less than p, 
		//   where "less" means 
		std::map<WorldObjectType,int>::iterator loit = levels.find(o.getType());
		std::map<WorldObjectType,int>::iterator lpit = levels.find(p.getType());
		int lo = (loit==levels.end())?100:loit->second;
		int lp = (lpit==levels.end())?101:lpit->second;
        return lo * o.getConfidence() < lp * p.getConfidence();
	}
};

UniformParticleInitializer::UniformParticleInitializer( )
{
	// init random number generator
	const gsl_rng_type * T;

	gsl_rng_env_setup( );
	T = gsl_rng_default;
	m_rng = gsl_rng_alloc( T );
	gsl_rng_set( m_rng, ( unsigned long ) time( NULL ) );
}

void
UniformParticleInitializer::initParticle( ParticleT & particle )
{
	field_model::FieldModel* field = field_model::FieldModel::getInstance();

	float fl = 100.f * field->length() + 100.f;
	float fw = 100.f * field->width()  + 100.f;
	particle.setState( Vec3f(
		0.01f*(-fw/2.f + fw * ( float ) gsl_rng_uniform( m_rng )),
		0.01f*(-fl/2.f + fl * ( float ) gsl_rng_uniform( m_rng )),
		( float ) ( -M_PI + 2.f * M_PI * ( float ) gsl_rng_uniform( m_rng ) ) ) );
}

PoseParticleInitializer::PoseParticleInitializer( const Vec3f& pose, const Vec3f& stddev )
: m_pose(pose)
, m_stddev(stddev)
{
	// init random number generator
	const gsl_rng_type * T;

	gsl_rng_env_setup( );
	T = gsl_rng_default;
	m_rng = gsl_rng_alloc( T );
	gsl_rng_set( m_rng, ( unsigned long ) time( NULL ) );
}

void
PoseParticleInitializer::initParticle( ParticleT & particle )
{
	particle.setState(Vec3f(
		m_pose.x() + gsl_ran_gaussian( m_rng, m_stddev.x() ),
		m_pose.y() + gsl_ran_gaussian( m_rng, m_stddev.y() ),
		picut(m_pose.z() + gsl_ran_gaussian( m_rng, m_stddev.z()))));
}

GridParticleInitializer::GridParticleInitializer( SelfLocalizationPF * p ) 
: m_Selflocalization( p ) 
{ 
	// init random number generator
	const gsl_rng_type * T;

	gsl_rng_env_setup( );
	T = gsl_rng_default;
	m_rng = gsl_rng_alloc( T );
	gsl_rng_set( m_rng, ( unsigned long ) time( NULL ) );
}
void
GridParticleInitializer::initParticle( ParticleT & particle )
{
	// NASTY HACK
	field_model::FieldModel* field = field_model::FieldModel::getInstance();

	const float minx = -field->width() * 100 / 200 - 0.3f;
	const float miny = -field->length() * 100 / 200 - 0.3f;
// 	const float mintheta = 0;

	const float maxx = -minx;
// 	const float maxy = -miny;
	const float maxtheta = M_PI;
	const float res = 12 / 100.f;
	const float tres = 36;
	static float x = minx;
	static float y = miny;
	static float theta = 0;

	static bool fixedParticleInitialized = false;

	if ( !fixedParticleInitialized ) {
		particle.setState( Vec3f( 0.02f, 0.78f, 3.66479179 ) );
		fixedParticleInitialized = true;
		return;
	}
	if ( theta >= 0 ) {
		theta += 2 * M_PI / tres;
		theta = -theta;
	}
	else {
		theta = -theta;
	}
	if ( theta >= maxtheta ) {
		theta = 0;
		x += res;
	}
	if ( x > maxx ) {
		x = minx;
		y += res;
	}
	particle.setState( Vec3f( x, y, theta + 0.1f*(float)gsl_rng_uniform( m_rng )));
}

SimpleMotionModel::SimpleMotionModel( )
{
	// init random number generator
	const gsl_rng_type * T;

	gsl_rng_env_setup( );
	T = gsl_rng_default;
	m_rng = gsl_rng_alloc( T );
	gsl_rng_set( m_rng, ( unsigned long ) time( NULL ) );
}

Vec3f
SimpleMotionModel::sampleState( const Vec3f & lastPose, const Vec3f & controlInput, float delta_t )
{
	const float alpha_x_x = 0.4f;
	const float alpha_x_y = 0.04f;
	const float alpha_x_theta = 0.4f / M_PI;
	const float alpha_y_x = 0.04f;
	const float alpha_y_y = 0.4f;
	const float alpha_y_theta = 0.4f / M_PI;
	const float alpha_theta_x = 0.8f;
	const float alpha_theta_y = 0.8f;
	const float alpha_theta_theta = 0.4f;

	delta_t *= gDeltaTFact; 
	Vec3f noisyControlInput(0,0,0);
	float std_x = delta_t + alpha_x_x * fabsf( controlInput.x() ) + alpha_x_y * fabsf( controlInput.y() ) + alpha_x_theta * fabsf( controlInput.z() );
	float std_y = delta_t + alpha_y_x * fabsf( controlInput.x() ) + alpha_y_y * fabsf( controlInput.y() ) + alpha_y_theta * fabsf( controlInput.z() );
	float std_theta = 1.57f * delta_t + alpha_theta_x * fabsf( controlInput.x() ) + alpha_theta_y * fabsf( controlInput.y() ) + alpha_theta_theta * fabsf( controlInput.z() );

	noisyControlInput = controlInput + Vec3f(
		( float ) gsl_ran_gaussian( m_rng, std_x ),
		( float ) gsl_ran_gaussian( m_rng, std_y ),
		( float ) gsl_ran_gaussian( m_rng, std_theta ) );

	Vec2f dPose = Vec2f( noisyControlInput.x(), noisyControlInput.y() );
	dPose = Eigen::Rotation2Df(lastPose.z()) * dPose;

	return Vec3f( lastPose.x() + dPose.x(),
	             lastPose.y() + dPose.y(),
	             picut( lastPose.z() + noisyControlInput.z() ) );
}

SimpleObservationModel::SimpleObservationModel( SelfLocalizationPF * selfLocalization ) : m_selfLocalization( selfLocalization ) { }
bool   SimpleObservationModel::isFieldLine(WorldObjectType wo){
	return FL_YellowGoal <= wo && wo <= FL_LongPenaltyBlue;
}

SimpleObservationModel::LineParam
SimpleObservationModel::getLineParam(const ParticleT& particle, WorldObjectType wo){
	Vec2f sl(1,0), el(0,1);
	switch(wo){
		case FL_BlueGoal:
			sl = m_selfLocalization->worldObjectPos(TX_LeftBluePenalty);
			el = m_selfLocalization->worldObjectPos(TX_RightBluePenalty);
			break;
		case FL_YellowGoal:
			sl = m_selfLocalization->worldObjectPos(TX_LeftYellowPenalty);
			el = m_selfLocalization->worldObjectPos(TX_RightYellowPenalty);
			break;
#if PF_ASSOC_FIELDLINES_PER_PARTICLE // dont mix up side lines when only associating w/ global pose
		case FL_SideBlueYellowBlue:
			sl = m_selfLocalization->worldObjectPos(LX_RightBlueField);
			el = m_selfLocalization->worldObjectPos(LX_LeftYellowField);
			break;
		case FL_SideYellowBlueYellow:
			sl = m_selfLocalization->worldObjectPos(LX_LeftBlueField);
			el = m_selfLocalization->worldObjectPos(LX_RightYellowField);
			break;
#endif
		case FL_LongPenaltyBlue:
			sl = m_selfLocalization->worldObjectPos(LX_LeftBluePenalty);
			el = m_selfLocalization->worldObjectPos(LX_RightBluePenalty);
			break;
		case FL_LongPenaltyYellow:
			sl = m_selfLocalization->worldObjectPos(LX_LeftYellowPenalty);
			el = m_selfLocalization->worldObjectPos(LX_RightYellowPenalty);
			break;
		case FL_CenterLine:
			sl = m_selfLocalization->worldObjectPos(TX_CentralLeft);
			el = m_selfLocalization->worldObjectPos(TX_CentralRight);
			break;
		default:
			assert(false);
	}
	LineParam lp;
	sl -= particle.getState().getXYVec();
	el -= particle.getState().getXYVec();
	Vec2f lot = RcMath::lotPunktAufGerade(Vec2f(0,0),sl,el);
	lp.dist = lot.norm();
	sl = particle.getUserData( ).transposed( ) * sl;
	//el = particle.getUserData( ).transposed( ) * el; // not necessary: lot needs to be rotated, anyway.
	lot = particle.getUserData( ).transposed( ) * lot;
	lp.ang = atan2(lot.x-sl.x,lot.y-sl.y);
	lp.dist *= RcMath::signum(lot.y);
	return lp;
}

Likelihood
SimpleObservationModel::minAndLikelihood( const ParticleT & particle, const LandmarkObservation & observation )
{
	Likelihood mal;
	Vec2f expCart;
	if(isFieldLine(observation.getType())){
		LineParam lp = getLineParam(particle,observation.getType());
		mal.min_mahal_dist = 0.0;
		mal.min_mahal_dist += .5 * (observation.m_3std_range*observation.m_3std_range/(observation.s_std_linerange*observation.s_std_linerange));
		mal.min_mahal_dist += .5 * (observation.m_3std_lineorient*observation.m_3std_lineorient/(observation.s_std_lineorient*observation.s_std_lineorient));

		double orient_innov = observation.getOrientation() - lp.ang;
		orient_innov = RcMath::piCut(orient_innov);
		if(orient_innov >  M_PI_2) orient_innov = -M_PI + orient_innov;
		if(orient_innov < -M_PI_2) orient_innov =  M_PI + orient_innov;
		double dist_innov = observation.getRange( ) - lp.dist;
		if(!observation.doNotCheckDistSign() && RcMath::signum(lp.dist*observation.getRange()) == -1.f) 
			mal.mahal_dist = INT_MAX;
		else
			mal.mahal_dist = .5* ( dist_innov*dist_innov     / (observation.m_std_range   * observation.m_std_range)
			+                       orient_innov*orient_innov / (observation.s_std_lineorient * observation.s_std_lineorient) );
	}else{
		expCart =  m_selfLocalization->worldObjectPos( observation.getType( ) );
		expCart -= particle.getState().getXYVec();
		mal.min_mahal_dist = 0;
		if ( ( m_selfLocalization->m_CV->getCameraHeading( ) == CAM_FRONT ) && ( observation.getRange( ) > 3.f ) )
			mal.min_mahal_dist += 0.0;
		else 
			mal.min_mahal_dist += .5*(observation.m_3std_range  *observation.m_3std_range   / (observation.s_std_range*observation.s_std_range));
		mal.min_mahal_dist     += .5*(observation.m_3std_bearing*observation.m_3std_bearing / (observation.s_std_bearing*observation.s_std_bearing));

		if ( observation.getType( ) == NumWorldObjects )
			mal.mahal_dist = INT_MAX;
		else {
			//expCart.rotate( -particle.getState().z - RcMath::pi_halbe );
			expCart = particle.getUserData( ).transposed( ) * expCart; // rotate
			expCart.dreheCW90( );

			// workaround for compiler bug in release mode...
			float bearing_innov = observation.getBearing( ) - atan2( expCart.y, expCart.x );
			while ( bearing_innov >   M_PI ) bearing_innov -= 2.f * M_PI;
			while ( bearing_innov <= -M_PI ) bearing_innov += 2.f * M_PI;
			mal.mahal_dist = 0;
			if ( ( m_selfLocalization->m_CV->getCameraHeading( ) == CAM_FRONT ) && ( observation.getRange( ) > 3.f ) )
				mal.mahal_dist += 0.0;
			else {
				float range_innov = observation.getRange( ) - expCart.norm( );
				mal.mahal_dist += 0.5*(range_innov*range_innov/(observation.m_std_range*observation.m_std_range));
			}
			mal.mahal_dist     += 0.5*(bearing_innov*bearing_innov/(observation.s_std_bearing*observation.s_std_bearing));
		}
	}
	return mal;
}

double
SimpleObservationModel::minLikelihood( const ParticleT & particle, const LandmarkObservation & observation )
{
	Likelihood mal;
	Vec2f expCart;

	if(isFieldLine(observation.getType())){
		LineParam lp = getLineParam(particle,observation.getType());
		mal.min_mahal_dist    = 0.0;
		mal.min_mahal_dist   += 0.5 *(observation.m_3std_range     *observation.m_3std_range  /(observation.s_std_linerange*observation.s_std_linerange));
		mal.min_mahal_dist   += 0.5 *(observation.m_3std_lineorient*observation.m_3std_lineorient/(observation.s_std_lineorient*observation.s_std_lineorient));
	}else{
		expCart =  m_selfLocalization->worldObjectPos( observation.getType( ) );
		expCart -= particle.getState().getXYVec();

		mal.min_mahal_dist = 0.0;
		if ( ( m_selfLocalization->m_CV->getCameraHeading( ) == CAM_FRONT ) && ( observation.getRange( ) > 3.f ) )
			mal.min_mahal_dist += 0.0;
		else
			mal.min_mahal_dist += 0.5 *(observation.m_3std_range*observation.m_3std_range / (observation.s_std_range*observation.s_std_range));
		mal.min_mahal_dist     += 0.5 *(observation.m_3std_bearing*observation.m_3std_bearing / (observation.s_std_bearing*observation.s_std_bearing));
	}

	return mal.min_mahal_dist;
}

double
SimpleObservationModel::likelihood( const ParticleT & particle, const LandmarkObservation & observation )
{
	Likelihood mal;
	Vec2f expCart;

	if(isFieldLine(observation.getType())){
		LineParam lp = getLineParam(particle,observation.getType());
		mal.min_mahal_dist  = 0.0;
		mal.min_mahal_dist += 0.5*(observation.m_3std_range  *observation.m_3std_range  /(observation.s_std_linerange  *observation.s_std_linerange));
		mal.min_mahal_dist += 0.5*(observation.m_3std_lineorient*observation.m_3std_lineorient/(observation.s_std_lineorient*observation.s_std_lineorient));

		double orient_innov = observation.getOrientation() - lp.ang;
		orient_innov = RcMath::piCut(orient_innov);
		if(orient_innov >  M_PI_2) orient_innov = -M_PI + orient_innov;
		if(orient_innov < -M_PI_2) orient_innov =  M_PI + orient_innov;
		double range_innov = observation.getRange( ) - lp.dist;
		mal.mahal_dist  = 0.0;
		mal.mahal_dist += 0.5*(range_innov*range_innov/(observation.m_std_range*observation.m_std_range));
		mal.mahal_dist += 0.5*(orient_innov*orient_innov/(observation.s_std_lineorient*observation.s_std_lineorient));
		if(!observation.doNotCheckDistSign() && RcMath::signum(lp.dist*observation.getRange()) == -1.f) 
			mal.mahal_dist = INT_MAX;
	}else{
			expCart =  m_selfLocalization->worldObjectPos( observation.getType( ) );
			expCart -= particle.getState().getXYVec();
			if ( observation.getType( ) == NumWorldObjects )
				mal.mahal_dist = INT_MAX;
			else {
				Vec2f expCart( m_selfLocalization->worldObjectPos( observation.getType( ) ) - particle.getState( ).getXYVec( ) );
				//expCart.rotate( -particle.getState().z - RcMath::pi_halbe );
				expCart = particle.getUserData( ).transposed( ) * expCart; // rotate
				expCart.dreheCW90( );

				// workaround for compiler bug in release mode...
				float bearing_innov = observation.getBearing( ) - atan2( expCart.y, expCart.x );
				while ( bearing_innov >   M_PI ) bearing_innov -= 2.f * M_PI;
				while ( bearing_innov <= -M_PI ) bearing_innov += 2.f * M_PI;
				mal.mahal_dist = 0.0;
				if ( ( m_selfLocalization->m_CV->getCameraHeading( ) == CAM_FRONT ) && ( observation.getRange( ) > 3.f ) )
					mal.mahal_dist    += 0.0;
				else {
					double range_innov = observation.getRange( ) - expCart.norm( );
					mal.mahal_dist    += 0.5*(range_innov*range_innov/(observation.m_std_range*observation.m_std_range));
				}
				mal.mahal_dist        += 0.5*(bearing_innov*bearing_innov/(observation.s_std_bearing*observation.s_std_bearing));
			}
	}
	return mal.mahal_dist;
}

double
SimpleObservationModel::likelihoodDiscount( const LandmarkObservation & observation )
{
	return SelfLocalizationPF::landmarkLikelihoodDiscount[ observation.getType( ) ];
}

SelfLocalizationPF::SelfLocalizationPF( )
	: ParticleFilter < Vec3f, Vec3f, LandmarkObservation, Mat22f > (
#if PARTICLE_FILTER_GRID_TEST
	    new GridParticleInitializer( this ),
	    new NoMotionModel( ),
	    new SimpleObservationModel( this ),
	    1000,
	    80000,
#else
	    new UniformParticleInitializer(),
	    new SimpleMotionModel( ),
	    new SimpleObservationModel( this ),
	    250,
	    1000,
#endif
	    0.01,
	    0.05 )
	, m_CV( NULL )
	, useDPoles( true )
{
	m_minEffectiveSampleSizeRatio = 0.5f; //0.5f;
	m_uniformReplacementProbability = 0.05f;
	m_actualUniformReplacementProbability = m_uniformReplacementProbability;
	m_usingActualReplacement = false;

	m_confidence = 0;
	m_cov.setEye( );
	m_cov *= 0.01f;

	more_is_better[YellowGoal]     =10;
	more_is_better[BlueGoal]       =10;

	more_is_better[YellowPole]     =8;
	more_is_better[BluePole]       =8;
	
	more_is_better[YellowPost]     =6;
	more_is_better[BluePost]       =6;
	more_is_better[GP_BlueLeft]    =6;
	more_is_better[GP_BlueRight]   =6;
	more_is_better[GP_YellowLeft]  =6;
	more_is_better[GP_YellowRight] =6;

	for(int i=TX_CentralRight;i<=TX_RightYellowPenalty;i++)
		more_is_better[(WorldObjectType)i]          =4;
	more_is_better[LineXingT]      =4;
	
	more_is_better[Marker]               =3;
	more_is_better[YellowMarker]         =3;
	more_is_better[BlueMarker]           =3;
	
	more_is_better[LineXingX]           = 3;
	more_is_better[XX_YellowBlueYellow] = 3;
	more_is_better[XX_BlueYellowBlue]   = 3;

	for(int i=LX_LeftBlueField;i<=LX_RightYellowPenalty;i++)
		more_is_better[(WorldObjectType)i]          = 2;
	more_is_better[LineXingL]      = 1;

	more_is_better[FieldLineWO]    =0;
	more_is_better[LineWO]         =0;
	for(int i=FL_YellowGoal;i<=FL_LongPenaltyBlue;i++)
		more_is_better[(WorldObjectType)i] = 0;


	//position of Objects (Landmarks, etc)
	m_worldObjectPos[ BlueGoal ].x = 0;
	m_worldObjectPos[ BlueGoal ].y = -CCV::fieldLength / 2 - CCV::goalDepth; // back side of goal

	m_worldObjectPos[ YellowGoal ].x = 0;
	m_worldObjectPos[ YellowGoal ].y = CCV::fieldLength / 2 + CCV::goalDepth; // back side of goal

	m_worldObjectPos[ YellowPole ].x = -CCV::fieldWidth / 2 - CCV::poleToField; // seen in image
	m_worldObjectPos[ YellowPole ].y = 0;

	m_worldObjectPos[ BluePole ].x = CCV::fieldWidth / 2 + CCV::poleToField; // seen in image
	m_worldObjectPos[ BluePole ].y = 0;

	m_worldObjectPos[ Circle ].x = 0;
	m_worldObjectPos[ Circle ].y = 0;

	// * LiuW: At RoboCup 2009, There will be only two X-Markers left.                    * //
	// *       Left the privious MiddleYellowMarker & MiddleBlueMarker and change name to * //
	// *       YellowMarker & BlueMarker.                                                 * //
	m_worldObjectPos[ YellowMarker ].x = 0;
	m_worldObjectPos[ YellowMarker ].y = CCV::fieldLength / 2 - CCV::penaltyMarkerDist;

	m_worldObjectPos[ BlueMarker ].x = 0;
	m_worldObjectPos[ BlueMarker ].y = -CCV::fieldLength / 2 + CCV::penaltyMarkerDist;

	m_worldObjectPos[ GP_YellowLeft ].x = -CCV::goalWidth / 2;
	m_worldObjectPos[ GP_YellowLeft ].y =  CCV::fieldLength / 2;

	m_worldObjectPos[ GP_YellowRight ].x = CCV::goalWidth / 2;
	m_worldObjectPos[ GP_YellowRight ].y = CCV::fieldLength / 2;

	m_worldObjectPos[ GP_BlueLeft ].x =  CCV::goalWidth / 2;
	m_worldObjectPos[ GP_BlueLeft ].y = -CCV::fieldLength / 2;

	m_worldObjectPos[ GP_BlueRight ].x = -CCV::goalWidth / 2;
	m_worldObjectPos[ GP_BlueRight ].y = -CCV::fieldLength / 2;

	// FieldLine Xings
	m_worldObjectPos[ LX_LeftBlueField ]    = Vec2f( +CCV::fieldWidth / 2, -CCV::fieldLength / 2 );
	m_worldObjectPos[ LX_RightBlueField ]   = Vec2f( -CCV::fieldWidth / 2, -CCV::fieldLength / 2 );
	m_worldObjectPos[ LX_LeftBluePenalty ]  = Vec2f( +CCV::goalAreaWidth / 2, -CCV::fieldLength / 2 + CCV::goalAreaDepth );
	m_worldObjectPos[ LX_RightBluePenalty ] = Vec2f( -CCV::goalAreaWidth / 2, -CCV::fieldLength / 2 + CCV::goalAreaDepth );

	m_worldObjectPos[ LX_LeftYellowField ]    = Vec2f( -CCV::fieldWidth / 2, +CCV::fieldLength / 2 );
	m_worldObjectPos[ LX_RightYellowField ]   = Vec2f( +CCV::fieldWidth / 2, +CCV::fieldLength / 2 );
	m_worldObjectPos[ LX_LeftYellowPenalty ]  = Vec2f( -CCV::goalAreaWidth / 2, +CCV::fieldLength / 2 - CCV::goalAreaDepth );
	m_worldObjectPos[ LX_RightYellowPenalty ] = Vec2f( +CCV::goalAreaWidth / 2, +CCV::fieldLength / 2 - CCV::goalAreaDepth );

	m_worldObjectPos[ TX_LeftBluePenalty ]    = Vec2f( +CCV::goalAreaWidth / 2, -CCV::fieldLength / 2 );
	m_worldObjectPos[ TX_RightBluePenalty ]   = Vec2f( -CCV::goalAreaWidth / 2, -CCV::fieldLength / 2 );
	m_worldObjectPos[ TX_LeftYellowPenalty ]  = Vec2f( -CCV::goalAreaWidth / 2, +CCV::fieldLength / 2 );
	m_worldObjectPos[ TX_RightYellowPenalty ] = Vec2f( +CCV::goalAreaWidth / 2, +CCV::fieldLength / 2 );
	m_worldObjectPos[ TX_CentralLeft ]        = Vec2f( -CCV::fieldWidth / 2, +0 );
	m_worldObjectPos[ TX_CentralRight ]       = Vec2f( +CCV::fieldWidth / 2, +0 );

	m_worldObjectPos[ XX_YellowBlueYellow ]   = Vec2f( -0.5f*CCV::centerCircleDiameter, 0);
	m_worldObjectPos[ XX_BlueYellowBlue ]     = Vec2f( +0.5f*CCV::centerCircleDiameter, 0);

	// Xing orientations

	m_worldObjectPos[ TX_LeftBluePenalty ].z    = 0;
	m_worldObjectPos[ TX_RightBluePenalty ].z   = 0;
	m_worldObjectPos[ TX_LeftYellowPenalty ].z  = M_PI;
	m_worldObjectPos[ TX_RightYellowPenalty ].z = M_PI;
	m_worldObjectPos[ TX_CentralLeft ].z        = -M_PI_2;
	m_worldObjectPos[ TX_CentralRight ].z       = M_PI_2;

	m_worldObjectPos[ LX_LeftBlueField ].z      = RcMath::deg2rad( 45.f );
	m_worldObjectPos[ LX_RightYellowPenalty ].z = RcMath::deg2rad( 45.f );
	m_worldObjectPos[ LX_RightBlueField ].z     = RcMath::deg2rad( -45.f );
	m_worldObjectPos[ LX_LeftYellowPenalty ].z  = RcMath::deg2rad( -45.f );
	m_worldObjectPos[ LX_LeftYellowField ].z    = RcMath::deg2rad( 45.f + 180.f );
	m_worldObjectPos[ LX_RightBluePenalty ].z   = RcMath::deg2rad( 45.f + 180.f );
	m_worldObjectPos[ LX_RightYellowField ].z   = RcMath::deg2rad( -45.f + 180.f );
	m_worldObjectPos[ LX_LeftBluePenalty ].z    = RcMath::deg2rad( -45.f + 180.f );

#ifdef DRIBBLING_CHALLENGE
	m_worldObjectPos[ MagentaPole ].x = 0;
	m_worldObjectPos[ MagentaPole ].y = 0;

	m_worldObjectPos[ BlueCyanPole ] = m_worldObjectPos[ BlueMarker ]; //<- LiuW: It was MiddleBlueMarker before RoboCup 2009.
	m_worldObjectPos[ BlueCyanPole ].y += 20;
	m_worldObjectPos[ YellowCyanPole ] = m_worldObjectPos[ YellowMarker ]; //<- LiuW: It was MiddleYellowMarker before RoboCup 2009.
	m_worldObjectPos[ YellowCyanPole ].y -= 20;
#endif

	for ( int i = 0; i < NumWorldObjects; i++ ) {
		m_worldObjectPos[ i ].x /= 100.f;
		m_worldObjectPos[ i ].y /= 100.f;
	}

	for ( unsigned int i = 0; i < NumWorldObjects; i++ ) {
		landmarkLikelihoodDiscount[ i ] = 0.4;
	}
	double markerDiscount = 0.9;
	double standardDiscount = 0.95;

	// * LiuW: At RoboCup 2009, There will be only two X-Markers left.                    * //
	// *       Left the privious MiddleYellowMarker & MiddleBlueMarker and change name to * //
	// *       YellowMarker & BlueMarker.                                                 * //
	landmarkLikelihoodDiscount[ YellowMarker ] *= markerDiscount;
	landmarkLikelihoodDiscount[ BlueMarker ]   *= markerDiscount;

/*
	landmarkLikelihoodDiscount[ LeftYellowMarker ]   *= markerDiscount;
	landmarkLikelihoodDiscount[ MiddleYellowMarker ] *= markerDiscount;
	landmarkLikelihoodDiscount[ RightYellowMarker ]  *= markerDiscount;
	landmarkLikelihoodDiscount[ LeftBlueMarker ]     *= markerDiscount;
	landmarkLikelihoodDiscount[ MiddleBlueMarker ]   *= markerDiscount;
	landmarkLikelihoodDiscount[ RightBlueMarker ]    *= markerDiscount;
*/

	float poleDiscount = 0.6f;
	landmarkLikelihoodDiscount[ YellowPole ]  *= standardDiscount;
	landmarkLikelihoodDiscount[ BluePole ]    *= standardDiscount;

	float goalDiscount = 0.6f;
	landmarkLikelihoodDiscount[ BlueGoal ]         *= standardDiscount;
	landmarkLikelihoodDiscount[ YellowGoal ]       *= standardDiscount;
	landmarkLikelihoodDiscount[ GP_YellowLeft ]    *= goalDiscount;
	landmarkLikelihoodDiscount[ GP_YellowRight ]   *= goalDiscount;
	landmarkLikelihoodDiscount[ GP_BlueLeft ]      *= goalDiscount;
	landmarkLikelihoodDiscount[ GP_BlueRight ]     *= goalDiscount;

	// L crossings are discounted more
	for(int i=LX_LeftBlueField; i<=LX_RightYellowPenalty;i++){
		landmarkLikelihoodDiscount[i] *= 0.7;
	}
	// T-crossings slightly less than T-xings
	for(int i=TX_CentralLeft; i<=TX_RightYellowPenalty;i++){
		landmarkLikelihoodDiscount[i] *= 0.8;
	}
	// X-crossings are most reliable
	for(int i=XX_BlueYellowBlue; i<=XX_YellowBlueYellow;i++){
		landmarkLikelihoodDiscount[i] *= 0.9;
	}

	//landmarkLikelihoodDiscount[ LineWO ] *= standardDiscount;

	m_distToWhiteGrid.precomputeLineLikelihoodField( *this );

	std::vector < WorldObjectType > wotarr;

	wotarr.clear( );
	wotarr.push_back( LX_LeftBlueField );
	wotarr.push_back( LX_RightYellowPenalty );
	m_closestL_045.init( wotarr, *this );

	wotarr.clear( );
	wotarr.push_back( LX_RightYellowField );
	wotarr.push_back( LX_LeftBluePenalty );
	m_closestL_135.init( wotarr, *this );

	wotarr.clear( );
	wotarr.push_back( LX_LeftYellowField );
	wotarr.push_back( LX_RightBluePenalty );
	m_closestL_225.init( wotarr, *this );

	wotarr.clear( );
	wotarr.push_back( LX_RightBlueField );
	wotarr.push_back( LX_LeftYellowPenalty );
	m_closestL_315.init( wotarr, *this );

	wotarr.clear();
	wotarr.push_back( XX_BlueYellowBlue );
	wotarr.push_back( XX_YellowBlueYellow );
	m_closestX.init(wotarr, *this);


	wotarr.clear( );
	wotarr.push_back( TX_CentralLeft );
	wotarr.push_back( TX_CentralRight );
	wotarr.push_back( TX_LeftBluePenalty );
	wotarr.push_back( TX_LeftYellowPenalty );
	wotarr.push_back( TX_RightBluePenalty );
	wotarr.push_back( TX_RightYellowPenalty );
	m_closestTCornerGrid.init( wotarr, *this );
} // CONSTRUCTOR

void
SelfLocalizationPF::update( float delta_t, Vec3f controlInput, WideAngle_CV * cv, float twistedHeadingAngle )
{

	/*twistedHeadingAngle=0;*/

	m_CV = cv;
	m_twistedHeadingAngle = twistedHeadingAngle;

	m_lastControlInput = controlInput;
	m_lastDelta_t = delta_t;

	//for( unsigned int p = 0; p < m_particles[m_currParticleSetIdx].size(); p++ ) {
	//	m_particleInitializer->initParticle( m_particles[m_currParticleSetIdx][p] );
	//}
	const float uniformReplacementTreshold = 0.02f; // scale between 0 and 1 in interval [UniformReplacementTreshold;1]
	float newUniformReplacementProbability = min( 1.f, max( 0.f, 1.f - ( float ) ( m_fastAverageLikelihood / m_slowAverageLikelihood ) - uniformReplacementTreshold) );
	newUniformReplacementProbability *= 1.f / (1.f-uniformReplacementTreshold);

	const float urp_alpha = 0.99;
	m_actualUniformReplacementProbability = urp_alpha * m_actualUniformReplacementProbability + (1.f-urp_alpha)*newUniformReplacementProbability;
	if(gPFmode==1 && (!commBuffer.data[ 0 ].halt_PPCGUI || commBuffer.cmd[0].Command!=CMD_PLAY)){
		m_uniformReplacementProbability = 0.f;
		m_usingActualReplacement = false;
	}else{
		if(!m_usingActualReplacement)
			// reset to a high value when switching to "stop"
			m_uniformReplacementProbability = m_actualUniformReplacementProbability = 0.2f;
		else {
#ifdef COPEDO
			m_uniformReplacementProbability = max(0.0f, m_actualUniformReplacementProbability);
#else
			m_uniformReplacementProbability = max(0.05f, m_actualUniformReplacementProbability);
#endif
		}
#if DRIBBLE_AROUND_OBSTACLE_COMPETITION
		setPose(Vec3f(0,-0.2f,0), Vec3f(0.2,0.2,M_PI/6.));
#endif
		m_usingActualReplacement = true;
	}

	// 1. sample
	sample( controlInput, delta_t );

	//for(vector<ParticleT>::iterator it = m_particles->begin(); it!= m_particles->end(); it++){
	//	assert(fabs(it->getState().x) < 20);
	//	assert(fabs(it->getState().y) < 20);
	//	assert(fabs(it->getState().z) < 7);
	//}

	// 2. init observations
	m_observations.clear( );

#if 1 // non-line-related observations
	for( int i=0; i<2;i++ ){
		const EgoObject& goal = cv->ivGoal[ i ];
		float conf = goal.conf( );
		if ( conf == 0.f )
			continue;
		WorldObjectType wo = i==0?BlueGoal:YellowGoal;
		m_observations.push_back( LandmarkObservation( 0.01f * goal.dist( ), - goal.ang( ) - twistedHeadingAngle, conf, wo ) );
	}
	for ( int i = 0; i < cv->ivGoalPostNum; i++ ) {
		EgoObject & o = cv->ivGoalPost[ i ];
		if ( o.conf( ) > 0 ) {
			m_observations.push_back( LandmarkObservation( 0.01f * o.dist( ), - o.ang( ) - twistedHeadingAngle, o.conf( ), o.wotype( ) ) );
		}
	}
	for ( unsigned int i = 0; i < 2; i++ ) {
		EgoObject & o = cv->ivPole[ i ];
		if ( o.conf( ) > 0 ) {
			m_observations.push_back( LandmarkObservation( 0.01f * o.dist( ), - o.ang( ) - twistedHeadingAngle, o.conf( ), o.wotype( ) ) );
		}
	}
#ifdef DRIBBLING_CHALLENGE
		if ( useDPoles ) {
			for ( int i = 0; i < 2; i++ ) {
				EgoObject & o = cv->ivDPole[ i ];
				if ( o.conf( ) > 0 ) {
					m_observations.push_back( LandmarkObservation( 0.01f * o.dist( ), -( o.ang( ) + twistedHeadingAngle ), o.conf( ), o.wotype( ) ) );
				}
			}
		}
#endif

	for ( unsigned int i = 0; i < MAX_MARKERS_PER_CAMERA; i++ ) {
		EgoObject & o = cv->ivMarker[ i ];
		if ( o.conf( ) > 0 ) {
			m_observations.push_back( LandmarkObservation( 0.01f * o.dist( ), - o.ang( ) - twistedHeadingAngle, o.conf( ), Marker ) );
		}
	}
#endif // * other observations * //

	if ( CCV::USE_LINES ) {
#if 1  // real line-related observations (in #else, you can add fake observations for debugging)
		// * Fieldline-corners * //
		for ( int i = 0; i < cv->ln_field_line_intersections_num; i++ ) {
			EgoObject & o = cv->ln_field_line_intersections[ i ];
			if ( o.conf( ) < 0.1f ) 
			{	// not confident enough
				continue;
			}
			if( (o.wotype() == LineXingT  && o.dist() > 160) )
			{	// otherwise, we get problems with lookup table for 
				// data association, which does not consider angle.
				continue;
			}
			if ( (o.wotype( ) != LineXingX  && o.wotype() != LineXingT) ) { 
				// cannot use L-crossings as long as 
				// their orientation is not correctly determined!
				//continue;
			}
			m_observations.push_back( LandmarkObservation( 0.01f * o.dist( ), - o.ang( ) - twistedHeadingAngle , o.conf( ), o.wotype( ) ) );
			m_observations.back( ).setOrientation( o.getEgoOrientation( ) - twistedHeadingAngle ); // TODO: need twistedHeadingAngle!?
		}
		// * All found lines * //
		int usedLines = 0;
		for ( int i = 0; i < cv->ln_field_lines_num && usedLines < 3; i++ ) {
			ObjectRecognition::FieldLine2 & l = cv->ln_field_lines[ i ];
			if ( l.conf > 0.1f ) {
				float ly          = (float)l.lot_world.x;
				float lx          = (float)l.lot_world.y;
				float dist        = l.lot_world.norm();
				float ang_to_lot  = atan2(ly,lx);
				float ang_of_line = RcMath::piCut(atan2((float)(l.e_world.x - l.s_world.x), (float)(l.e_world.y - l.s_world.y)));
				dist *= RcMath::signum(lx);
				m_observations.push_back( LandmarkObservation( 0.01f * dist, -ang_to_lot-twistedHeadingAngle, l.conf, l.wo ) );
				m_observations.back().setOrientation(ang_of_line+twistedHeadingAngle);
				usedLines++;
			}
		}
#elif 0
		// * Fieldline-corners fakes * //
		//m_observations.push_back( LandmarkObservation( 1.0f, -RcMath::deg2rad( - 45 )-twistedHeadingAngle, 1.0f, LineXingX ) );
		//m_observations.back( ).setOrientation( RcMath::deg2rad( 90 ) );
		//m_observations.push_back( LandmarkObservation( 1.0f, -RcMath::deg2rad( + 45 )-twistedHeadingAngle, 1.0f, LineXingT ) );


		twistedHeadingAngle = 0.39;
		ObjectRecognition::FieldLine2 l;
		l.conf        = 1.0f;
		l.wo          = FL_LongPenaltyBlue;
		l.lot_world.x = 8;
		l.lot_world.y = 41;
		l.s_world     = cv->color2WorldCoord(Vec2i(8,41));
		l.e_world     = cv->color2WorldCoord(Vec2i(33,3));
		float ly          = l.lot_world.x;
		float lx          = l.lot_world.y;
		float dist        = l.lot_world.norm();
		float ang_to_lot  = atan2(ly,lx);
		float ang_of_line = RcMath::piCut(atan2((float)(l.e_world.x - l.s_world.x), (float)(l.e_world.y - l.s_world.y)) + twistedHeadingAngle);
		dist *= RcMath::signum(lx);
		m_observations.push_back( LandmarkObservation( 0.01f * l.lot_world.norm(), -ang_to_lot-twistedHeadingAngle, l.conf, l.wo ) );
		m_observations.back().setOrientation(ang_of_line-twistedHeadingAngle);
#endif
	}

	if(m_observations.size() >2)
	{	// we have many observations, throw away the unconfident ones.
		vector<LandmarkObservation>::iterator it=m_observations.begin();
		while(it!=m_observations.end() && m_observations.size()>2){
			if(it->getConfidence() < 0.4 || it->getType() == LineXingL) 
				it = m_observations.erase(it);
			else
				it++;
		}
	}

	// precalculate some variables for all observations
	for_each( m_observations.begin( ), m_observations.end( ), mem_fun_ref( &LandmarkObservation::precalc ) );

	if ( m_observations.size( ) > 0 ) {

#if PARTICLE_FILTER_GRID_TEST
		// reset weights
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			m_particles[ m_currParticleSetIdx ][ p ].setWeight( 0.f );
		}
#endif

		// 3. importance
		long int l = GetTickCount( );
		importance( m_observations );
		m_importance_time = GetTickCount( ) - l;

		// hs: hack: do not consider positions outside the field in corners.
		//for ( unsigned int _p = 0; _p < m_currNumParticles; _p++ ) {
		//	ParticleT& p = m_particles[ m_currParticleSetIdx ][ _p ];
		//	bool isOutside = fabs(p.getState().x) > 0.01f*CCV::fieldWidth/2.f
		//		||           fabs(p.getState().y) > 0.01f*CCV::fieldLength/2.f;
		//	bool isInCorner = isOutside 
		//		&&  fabs(p.getState().x) > 2.f 
		//		&& fabs(p.getState().y) > 1.5f;
		//	if( isInCorner )
		//		p.setWeight(p.getWeight()/0.1f);
		//}


#if PARTICLE_FILTER_GRID_TEST
			float maxlh = -1e10;
			for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
				float lh = m_particles[ m_currParticleSetIdx ][ p ].getWeight( );
				if ( maxlh < lh ) {
					maxlh = lh;
				}
			}
			for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
				m_particles[ m_currParticleSetIdx ][ p ].setWeight(
				    m_particles[ m_currParticleSetIdx ][ p ].getWeight( ) - maxlh );
			}
#else
			// 4. resample
			resample( );
#endif

	} // END of IF ( m_observations.size( ) > 0 ) 


	// determine a good landmark to look at for "active vision"
	//if(!m_observations.empty()){
	//	active_vision_comparator avc(more_is_better);
	//	LandmarkObservation o = *std::max_element(m_observations.begin(), m_observations.end(), avc);
	//	m_landmark_to_track.initPolar(o.getBearing(), o.getRange(), o.getConfidence());
	//	if(more_is_better[o.getType()]<2) m_landmark_to_track.conf()=0.f;
	//}else{
		m_landmark_to_track.conf()=0.f;
	//}

	determinePositionEstimate( );
}

void
SelfLocalizationPF::determinePositionEstimate( )
{

	ParticleT * bestParticle = NULL;
	float maxweight = -1000.f;
	double weightSum = 0;
	ParticleSetT & particles = m_particles[ m_currParticleSetIdx ];

	for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
		ParticleT & particle = particles[ p ];
		if ( ( bestParticle == NULL ) || ( particle.getWeight( ) > maxweight ) ) {
			maxweight = particle.getWeight( );
			bestParticle = &particle;
		}
		weightSum += exp( ( double ) particle.getWeight( ) );
	}
	//m_mean = bestParticle->getState();
	//m_cov.setEye();
	//m_confidence = 1;

	const float windowCartDist2 = 0.5f * 0.5f;
	const float windowAngDist = 0.3f;

	Vec3f lastMean;
	double lastMeanWeightFraction = 0;
	meanShift( m_mean.getXYVec( ), m_mean.z, windowCartDist2, windowAngDist, lastMean, lastMeanWeightFraction );
	float lastMeanConfidence = 0;
	if ( weightSum != 0 ) {
		lastMeanConfidence = ( float ) max( 0.f, min( 1.f, lastMeanWeightFraction / weightSum ) );
	}

	m_mean = lastMean;
	m_confidence = lastMeanConfidence;

	if ( weightSum != 0 ) {
		Vec3f bestParticleMean;
		double bestParticleWeightFraction = 0;
		meanShift( bestParticle->getState( ).getXYVec( ), bestParticle->getState( ).z, windowCartDist2, windowAngDist, bestParticleMean, bestParticleWeightFraction );
		float bestParticleConfidence = ( float ) max( 0.f, min( 1.f, bestParticleWeightFraction / weightSum ) );
		if ( bestParticleConfidence > 4.5f * lastMeanConfidence ) {
			m_mean       = bestParticleMean;
			m_confidence = bestParticleConfidence;
		}
	}
	// * Compute weighted sample covariance * //
	m_cov.setZero( );
	int numParticles = 0;
	weightSum = 0;
	Vec2f m_mean2( m_mean.getXYVec( ) );
	for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
		ParticleT & particle = particles[ p ];
		const Vec3f & state = particles[ p ].getState( );
		Vec2f cartPos = state.getXYVec( );
		if ( ( ( cartPos - m_mean2 ).norm2( ) <= windowCartDist2 ) && ( fabsf( RcMath::piCut( state.z - m_mean.z ) ) <= windowAngDist ) ) {
			double weight = exp( ( double ) particles[ p ].getWeight( ) ) * ( 1.0 - ( cartPos - m_mean2 ).norm2( ) / windowCartDist2 );
			Vec3f diff( cartPos.x - m_mean2.x, cartPos.y - m_mean2.y, RcMath::piCut( state.z - m_mean.z ) );
			m_cov.Mij[ 0 ][ 0 ] += ( float ) ( weight * diff.x * diff.x );
			m_cov.Mij[ 0 ][ 1 ] += ( float ) ( weight * diff.x * diff.y );
			m_cov.Mij[ 0 ][ 2 ] += ( float ) ( weight * diff.x * diff.z );
			m_cov.Mij[ 1 ][ 0 ] += ( float ) ( weight * diff.y * diff.x );
			m_cov.Mij[ 1 ][ 1 ] += ( float ) ( weight * diff.y * diff.y );
			m_cov.Mij[ 1 ][ 2 ] += ( float ) ( weight * diff.y * diff.z );
			m_cov.Mij[ 2 ][ 0 ] += ( float ) ( weight * diff.z * diff.x );
			m_cov.Mij[ 2 ][ 1 ] += ( float ) ( weight * diff.z * diff.y );
			m_cov.Mij[ 2 ][ 2 ] += ( float ) ( weight * diff.z * diff.z );
			weightSum += weight;
			numParticles++;
		}
	}
	if ( ( weightSum == 0 ) || ( numParticles < 10 ) ) {
		m_cov.setEye( );
		m_cov *= 0.01f;
	}
	else {
		m_cov /= ( float ) weightSum;
	}
	m_confidence *= min( 1.f, max( 0, ( float ) ( m_fastAverageLikelihood / m_slowAverageLikelihood ) ) );

#if _DEBUG
	// TODO: REMOVE THIS!!!!
	//m_confidence = 1;
	//m_mean.x     = .01f * 0.01f ; 
	//m_mean.y     = .01f * (-CCV::fieldLength/2+0);
	//m_mean.z     = 0;
	//m_cov.setEye( );
	//m_cov *= 0.01f;
#endif
		
	m_mean.y = max(-0.01f*CCV::fieldLength/2-100.f, min(0.01f*((float)CCV::fieldLength/2+100.f),m_mean.y));
	m_mean.x = max(-0.01f*CCV::fieldWidth/2-100.f, min(0.01f*((float)CCV::fieldWidth/2+100.f),m_mean.x));

}

void
SelfLocalizationPF::meanShift( Vec2f startCartPos, float startOri, float maxCartDist2, float maxAngDist, Vec3f & mean, double & weightSumFraction )
{

	Vec2f lastCartMean( startCartPos );
	float lastOriMean( startOri );
	double lastWeightSumFraction = 0;
	int i = 0;
	const int maxNumIterations = 20;

	while ( i++ < maxNumIterations ) {
		Vec2f windowCartMean( 0, 0 );
		Vec2f windowDirMean( 0, 0 );
		float windowOriMean = 0;
		double weightSum = 0;
		ParticleSetT & particles = m_particles[ m_currParticleSetIdx ];
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			const Vec3f & state = particles[ p ].getState( );
			Vec2f cartPos = state.getXYVec( );
			if ( ( ( cartPos - lastCartMean ).norm2( ) <= maxCartDist2 ) && ( fabsf( RcMath::piCut( state.z - lastOriMean ) ) <= maxAngDist ) ) {
				double weight = exp( ( double ) particles[ p ].getWeight( ) );
				windowCartMean += ( float ) weight * cartPos;
				windowDirMean += ( float ) weight * Vec2f( cosf( state.z ), sinf( state.z ) );
				weightSum += weight;
			}
		}
		if ( weightSum == 0 ) {
			windowCartMean = lastCartMean;
			windowOriMean = lastOriMean;
		}
		else {
			windowCartMean /= ( float ) weightSum;
			windowOriMean = atan2( windowDirMean.y, windowDirMean.x );
		}
		if ( ( ( windowCartMean - lastCartMean ).norm2( ) < 0.001f ) || ( fabsf( RcMath::piCut( windowOriMean - lastOriMean ) ) < 0.001f ) ) {
			lastCartMean = windowCartMean;
			lastOriMean = windowOriMean;
			lastWeightSumFraction = weightSum;
			break;
		}
		lastCartMean = windowCartMean;
		lastOriMean = windowOriMean;
	}
	mean = Vec3f( lastCartMean.x, lastCartMean.y, lastOriMean );
	weightSumFraction = lastWeightSumFraction;
}

void 
SelfLocalizationPF::establishDataAssociation( ObjectRecognition::FieldLine2 & l, float twistedHeadingAngle, double& lik, WorldObjectType& wot )
{
	ParticleT part;
	part.setState(m_mean);

	float ly          = (float)l.lot_world.x;
	float lx          = (float)l.lot_world.y;
	float dist        = l.lot_world.norm();
	float ang_to_lot  = atan2(ly,lx);
	float ang_of_line = RcMath::piCut(atan2((float)(l.e_world.x - l.s_world.x), (float)(l.e_world.y - l.s_world.y)) + twistedHeadingAngle);
	dist *= RcMath::signum(lx);
	LandmarkObservation obs( 0.01f * dist, -ang_to_lot+twistedHeadingAngle, l.conf, l.wo );
	obs.setDoNotCheckDistSign(true);
	obs.setOrientation(ang_of_line+twistedHeadingAngle);
	obs.precalc();
	part.getUserData( ).setRot( part.getState( ).z );

	lik = exp(-m_observationModel->minAndLikelihood(part,obs).mahal_dist);
	wot = obs.getType();
}
void 
SelfLocalizationPF::establishDataAssociation( EgoObject & eo, float twistedHeadingAngle, double& lik, WorldObjectType& wot )
{
	ParticleT part;
	part.setState(m_mean);
	LandmarkObservation obs(0.01f*eo.dist(),-eo.ang()+twistedHeadingAngle,eo.conf(),eo.wotype());
	vector<LandmarkObservation> ov; ov.push_back(obs);
	ov.back().precalc();
	establishDataAssociation(part,ov);
	lik = exp(-ov.back().getMinAndLikelihood().mahal_dist);
	wot = ov.back().getType();
}

void SelfLocalizationPF::setPose(const Vec3f& pose, const Vec3f stddev){
	if(commBuffer.data[ 0 ].halt_PPCGUI){
		PoseParticleInitializer poseParticleInitializer(pose,stddev);
		for( unsigned int p = 0; p < m_particles[m_currParticleSetIdx].size(); p++ ) {
			poseParticleInitializer.initParticle( m_particles[m_currParticleSetIdx][p] );
		}
	}
}

void
SelfLocalizationPF::establishDataAssociation( ParticleT & particle, std::vector < LandmarkObservation > & observations )
{
	vector < LandmarkObservation >::iterator blueGoal = observations.end( ), yellowGoal = observations.end( );
	vector < LandmarkObservation >::iterator o = observations.begin( );

	particle.getUserData( ).setRot( particle.getState( ).z );

	for (; o != observations.end( ); o++ ) {
		LandmarkObservation & obs = *o;

		if ( 0 ) {
			;
		}
		else if ( obs.getType( ) == YellowGoal ) {
			yellowGoal = o;
		}
		else if ( obs.getType( ) == BlueGoal ) {
			blueGoal = o;
		}
		if ( 0 ) {
			;
		}
#ifdef DRIBBLING_CHALLENGE
		else if ( obs.getType( ) == CyanPole || obs.getType( ) == BlueCyanPole || obs.getType( ) == YellowCyanPole ) {
			std::vector < DataAssociationPair > matches;
			obs.setType( BlueCyanPole );
			double minLikelihood = m_observationModel->minLikelihood( particle, obs );

			obs.setType( BlueCyanPole );
			matches.push_back( DataAssociationPair( m_observationModel->likelihood( particle, obs ), BlueCyanPole ) );
			obs.setType( YellowCyanPole );
			matches.push_back( DataAssociationPair( m_observationModel->likelihood( particle, obs ), YellowCyanPole ) );

			obs.setType( determineMatch( matches, minLikelihood ) );
		}
#endif
		else if ( obs.getType( ) >= YellowMarker && obs.getType( ) <= Marker ) {

			std::vector < DataAssociationPair > matches;

			obs.setType( YellowMarker );
			double minLikelihood = m_observationModel->minLikelihood( particle, obs );
			for ( unsigned int wo = YellowMarker; wo <= BlueMarker; wo++ ) { 				                                                             
				obs.setType( ( WorldObjectType ) wo );
				matches.push_back( DataAssociationPair( m_observationModel->likelihood( particle, obs ), ( WorldObjectType ) wo ) );
			}
			obs.setType( determineMatch( matches, minLikelihood, obs ) );
		}
#if PF_ASSOC_FIELDLINES_PER_PARTICLE
		else if ( ( obs.getType( ) >= FL_YellowGoal && obs.getType( ) <= FL_LongPenaltyBlue ) ) {
			std::vector < DataAssociationPair > matches;
			obs.setType(FL_YellowGoal);
			double minLikelihood = m_observationModel->minLikelihood( particle, obs );
			for(int wo=FL_YellowGoal;wo<=FL_LongPenaltyBlue;wo++){
				obs.setType((WorldObjectType)wo);
				matches.push_back( DataAssociationPair( m_observationModel->likelihood( particle, obs ), ( WorldObjectType ) wo ) );
			}
			obs.setType( determineMatch( matches, minLikelihood, obs ) );
		}
#endif

#if USE_NAO_GOALS
		else if (obs.getType() >= BluePost && obs.getType() <= GP_YellowRight){
			std::vector < DataAssociationPair > matches;
			double minLikelihood=0;
			if(obs.getType() == BluePost || obs.getType() == GP_BlueLeft || obs.getType() == GP_BlueRight){

				WorldObjectType wo = GP_BlueLeft;
				obs.setType(wo);
				minLikelihood = m_observationModel->minLikelihood( particle, obs );
				DataAssociationPair p1( m_observationModel->likelihood( particle, obs ), ( WorldObjectType ) wo );
				matches.push_back( p1 );

				wo = GP_BlueRight;
				obs.setType(wo);
				DataAssociationPair p2( m_observationModel->likelihood( particle, obs ), ( WorldObjectType ) wo );
				matches.push_back( p2 );
			}
			else if(obs.getType() == YellowPost || obs.getType() == GP_YellowLeft || obs.getType() == GP_YellowRight){
				WorldObjectType wo = GP_YellowLeft;
				obs.setType(wo);
				minLikelihood = m_observationModel->minLikelihood( particle, obs );
				DataAssociationPair p1( m_observationModel->likelihood( particle, obs ), ( WorldObjectType ) wo );

				wo = GP_YellowRight;
				obs.setType(wo);
				DataAssociationPair p2( m_observationModel->likelihood( particle, obs ), ( WorldObjectType ) wo );

				matches.push_back( p1 );
				matches.push_back( p2 );
			}
			obs.setType( determineMatch( matches, minLikelihood, obs ) );
		}
#endif
		else if ( obs.getType( ) == LineXingL || ( obs.getType( ) >= LX_LeftBlueField && obs.getType( ) <= LX_RightYellowPenalty ) ) {

			//std::vector< DataAssociationPair > matches;
			//obs.setType(LX_LeftBlueField);
			//double minLikelihood = m_observationModel->minLikelihood(particle,obs);
			//for( unsigned int wo = LX_LeftBlueField; wo <= LX_RightYellowPenalty; wo++ ) {
			//	obs.setType( (WorldObjectType)wo );
			//	matches.push_back( DataAssociationPair( m_observationModel->likelihood( particle, obs ), (WorldObjectType)wo ) );
			//}
			//obs.setType( determineMatch( matches, minLikelihood ) );
			//continue;
			Vec2f rotobs = particle.getUserData( ).transposed( ) * Vec2f( obs.getX( ), obs.getY( ) );
			Vec2f mpos = particle.getState( ).getXYVec( ) + rotobs;
			Vec2i gpos = m_closestL_045.getMetricToGrid( mpos );
			if ( !m_closestL_045.isInside( gpos ) ) {
				gpos.x = max( gpos.x, 0 );
				gpos.y = max( gpos.y, 0 );
				gpos.x = min( gpos.x, m_closestL_045.mXdim - 1 );
				gpos.y = min( gpos.y, m_closestL_045.mYdim - 1 );
			}
			float absorient = RcMath::piCut( +obs.getOrientation( ) - particle.getState( ).z );
			if ( 0 ) {
				;
			}
			else if ( absorient <= -M_PI_2 ) {
				obs.setType( m_closestL_135.get( gpos ) );
			}
			else if ( -M_PI_2 < absorient && absorient <= 0 ) {
				obs.setType( m_closestL_045.get( gpos ) );
			}
			else if ( 0 < absorient && absorient <= M_PI_2 ) {
				obs.setType( m_closestL_315.get( gpos ) );
			}
			else if ( M_PI_2 < absorient ) {
				obs.setType( m_closestL_225.get( gpos ) );
			}
#if PARTICLE_DEBUGGING
			typedef pair < std::wstring, Vec3f > DebugPair;
			particle.m_DebuggingVector.push_back( DebugPair( L"", worldObjectPos( obs.getType( ) ) ) );
#endif

			assert( obs.getType( ) >= 0 && obs.getType( ) <= NumWorldObjects );
		}
		else if ( obs.getType( ) == LineXingT || ( obs.getType( ) >= TX_CentralRight && obs.getType( ) <= TX_RightYellowPenalty ) ) {
			Vec2f rotobs = particle.getUserData( ).transposed( ) * Vec2f( obs.getX( ), obs.getY( ) );
			Vec2f mpos = particle.getState( ).getXYVec( ) + rotobs;
			Vec2i gpos = m_closestTCornerGrid.getMetricToGrid( mpos );
			if ( !m_closestTCornerGrid.isInside( gpos ) ) {
				gpos.x = max( gpos.x, 0 );
				gpos.y = max( gpos.y, 0 );
				gpos.x = min( gpos.x, m_closestTCornerGrid.mXdim - 1 );
				gpos.y = min( gpos.y, m_closestTCornerGrid.mYdim - 1 );
			}
			WorldObjectType wo_closest = m_closestTCornerGrid.get( gpos );
			obs.setType( wo_closest );
			// ***********************************************************
			// the object closest in x/y coordinates is not good enough:
			// if the (circular) distributions around the corners overlap
			// the wrong association in the overlapping area results in
			// particles there dying.
			// We therefore throw dice nevertheless, but do not consider 
			// objects far away from the closest one.
			// ***********************************************************
			std::vector< DataAssociationPair > matches;
			obs.setType(TX_LeftBluePenalty);
			double minLikelihood = m_observationModel->minLikelihood(particle,obs);
			for( unsigned int wo = TX_CentralRight; wo <= TX_RightYellowPenalty; wo++ ) 
			{
				if(    (wo_closest == TX_LeftBluePenalty   || wo_closest == TX_RightBluePenalty)
					&& (wo         == TX_LeftYellowPenalty || wo         == TX_RightYellowPenalty))
					continue;
				if(    (wo         == TX_LeftBluePenalty   || wo         == TX_RightBluePenalty)
					&& (wo_closest == TX_LeftYellowPenalty || wo_closest == TX_RightYellowPenalty))
					continue;
				//if(    (wo_closest == TX_CentralLeft       || wo_closest == TX_CentralRight)
				//	&& (wo         != TX_CentralLeft       && wo         != TX_CentralRight))
				//	continue;
				obs.setType( (WorldObjectType)wo );
				matches.push_back( DataAssociationPair( m_observationModel->likelihood( particle, obs ), (WorldObjectType)wo ) );
			}
			obs.setType( determineMatch( matches, minLikelihood,obs ) );
		}
		else if( obs.getType() == LineXingX || obs.getType()==XX_YellowBlueYellow || obs.getType()==XX_BlueYellowBlue){
			std::vector< DataAssociationPair > matches;
			obs.setType(XX_YellowBlueYellow);
			double minLikelihood = m_observationModel->minLikelihood(particle,obs);

			obs.setType( XX_YellowBlueYellow );
			matches.push_back( DataAssociationPair( m_observationModel->likelihood( particle, obs ), obs.getType() ) );
			obs.setType( XX_BlueYellowBlue );
			matches.push_back( DataAssociationPair( m_observationModel->likelihood( particle, obs ), obs.getType()) );

			obs.setType( determineMatch( matches, minLikelihood ,obs) );
		}
	}
}

WorldObjectType
SelfLocalizationPF::determineMaxLikelihoodMatch( std::vector < SelfLocalizationPF::DataAssociationPair > & matches, double maxMahalDist, LandmarkObservation& obs )
{
	double minMahalDist = maxMahalDist;
	WorldObjectType wo0 = matches[0].type;

	obs.setMahal(minMahalDist,maxMahalDist);
	std::vector < DataAssociationPair >::iterator it = matches.begin( );
	while ( it != matches.end( ) ) {
		if ( it->mahal_dist  <= minMahalDist) {
			minMahalDist      = it->mahal_dist;
			wo0               = it->type;
		}
		++it;
	}
	obs.setMahal(minMahalDist, maxMahalDist);
	return wo0;
}

/*
 * this is not needed, since we do not do SLAM and bad decisions now do not have a bad influence on later stages
WorldObjectType
SelfLocalizationPF::determineLikelihoodSamplingMatch( std::vector < DataAssociationPair > & matches, double minLikelihood, LandmarkObservation& obs )
{

	WorldObjectType wo0 = matches[ 0 ].type;

	double sumLikelihoods = 0;

	std::vector < DataAssociationPair >::iterator it = matches.begin( );
	while ( it != matches.end( ) ) {
		if ( it->likelihood < minLikelihood ) {
			it = matches.erase( it );
		}
		else {
			sumLikelihoods += it->likelihood;
			++it;
		}
	}
	obs.setLik(minLikelihood, minLikelihood);
	if ( sumLikelihoods > 0 ) {
		double n = gsl_rng_uniform( m_rng );
		double m_sum = 0;
		double inv_sumLikelihoods = 1.0 / sumLikelihoods;
		for ( unsigned int i = 0; i < matches.size( ); i++ ) {
			DataAssociationPair& dp = matches[ i ];
			m_sum += dp.likelihood * inv_sumLikelihoods;
			if ( m_sum > n ) {
				obs.setLik(dp.likelihood,minLikelihood);
				return dp.type;
			}
		}
	}
	// it does not matter which match is taken, the obs will have min likelihood
	return wo0;
}
*/

const double LandmarkObservation::m_max_2range = 2.0 * 10.0;
const double LandmarkObservation::m_max_2range_inv = 1.0 / LandmarkObservation::m_max_2range;
const double LandmarkObservation::m_pi2_inv = 0.5 / M_PI;
float LandmarkObservation::s_std_bearing = 0.2f;
float LandmarkObservation::s_std_lineorient = 0.45f;
float LandmarkObservation::s_std_range = 0.5f;
float LandmarkObservation::s_std_linerange = 0.4f;
float LandmarkObservation::s_alpha_uniform_base = 0.2f;


}