#pragma once

#include "ParticleFilter.h"


#include "fieldgrid2d.h"

#include <map>
#include <math.h>

#include <gsl/gsl_rng.h>
#include "gsl/gsl_randist.h"

namespace disabled
{

using namespace PF;

class WideAngle_CV;
class SelfLocalizationPF;

#define USE_NEGATIVE_EVIDENCE 0

typedef Particle < Vec3f, Mat22f > ParticleT;
typedef ParticleSet < Vec3f, Mat22f > ParticleSetT;

/// @cond
class UniformParticleInitializer : public AbstractParticleInitializer < Vec3f, Mat22f >
{
public:
	UniformParticleInitializer( );
	~UniformParticleInitializer( ) {gsl_rng_free( m_rng );}

	void initParticle( ParticleT & particle );

protected:
	gsl_rng * m_rng;
};
/// @endcond

class PoseParticleInitializer : public AbstractParticleInitializer < Vec3f, Mat22f >
{
public:
	PoseParticleInitializer(const Vec3f& pose, const Vec3f& stddev );
	~PoseParticleInitializer( ) { gsl_rng_free( m_rng ); }

	void initParticle( ParticleT & particle );

protected:
	Vec3f m_pose;
	Vec3f m_stddev;
	gsl_rng * m_rng;
};

class GridParticleInitializer : public AbstractParticleInitializer < Vec3f, Mat22f >
{
public:
	GridParticleInitializer( SelfLocalizationPF * p );

	~GridParticleInitializer( ) { }

	void initParticle( ParticleT & particle );

protected:
	SelfLocalizationPF * m_Selflocalization;
	gsl_rng * m_rng;

};

class SimpleMotionModel : public AbstractMotionModel < Vec3f, Vec3f >
{
public:
	SimpleMotionModel( );
	~SimpleMotionModel( ) {gsl_rng_free( m_rng );}

	Vec3f sampleState( const Vec3f & lastPose, const Vec3f & controlInput, float delta_t );

protected:
	gsl_rng * m_rng;
};

class NoMotionModel : public AbstractMotionModel < Vec3f, Vec3f >
{
public:
	NoMotionModel( ) { }

	~NoMotionModel( ) { }

	inline Vec3f
	sampleState( const Vec3f & lastPose, const Vec3f & controlInput, float delta_t )
	{
		return lastPose;
	}

};


class LandmarkObservation
{
public:
	LandmarkObservation( float range, float bearing, float confidence, WorldObjectType type )
		: m_range( range )
		, m_bearing( bearing )
		, m_confidence( confidence )
		, m_type( type )
		,m_mahal_was_set(false)
		,m_do_not_check_dist_sign(false)
	{
		m_x = range * sin( bearing );
		m_y = range * cos( bearing );
	}

	LandmarkObservation( float x, float y, float confidence, WorldObjectType type, bool ) // hs: initialize x, y only. added for lines
		: m_confidence( confidence )
		, m_x( x )
		, m_y( y )
		, m_type( type )
		, m_mahal_was_set(false)
	    , m_do_not_check_dist_sign(false)
	{ }

	~LandmarkObservation( ) { }

	inline float
	getRange( ) const {return m_range;}

	inline void
	setRange( float range ) {m_range = range;}

	inline float
	getOrientation( ) const {return m_orientation;}

	inline void
	setOrientation( float orn ) {m_orientation = orn;}

	inline float
	getBearing( ) const {return m_bearing;}

	inline void
	setBearing( float bearing ) {m_bearing = bearing;}

	inline float
	getX( ) const {return m_x;}

	inline float
	getY( ) const {return m_y;}

	inline float
	getConfidence( ) const {return m_confidence;}

	inline bool doNotCheckDistSign()const{return m_do_not_check_dist_sign;}
	inline void setDoNotCheckDistSign(bool b){m_do_not_check_dist_sign = b;}

	inline void
	setConfidence( float confidence ) {m_confidence = confidence;}

	inline WorldObjectType
	getType( ) const {return m_type;}

	inline void
	setType( WorldObjectType type ) {m_type = type;}

	inline void
	setMahal( const double& mahal, const double& maxMahal){
		m_mahal_was_set = true;
		m_mahal         = mahal;
		m_maxmahal      = maxMahal;
	}
	inline bool isMahalSet(){return m_mahal_was_set;}
	inline Likelihood 
	getMinAndLikelihood(){
		assert(m_mahal_was_set);
		Likelihood mal;
		mal.mahal_dist     = m_mahal;
		mal.min_mahal_dist = m_maxmahal;
		return mal;
	}

#if USE_NEGATIVE_EVIDENCE
		inline bool
		isVirtual( ) const {return m_virtual;}

		inline void
		setVirtual( ) {m_virtual = true;}

		inline bool
		isExpectedButUnseen( ) const {return m_expectedButUnseen;}

		inline void
		setExpectedButUnseen( bool b ) {m_expectedButUnseen = b;}
#endif


		inline void precalc();

	inline void
	mahal_dists_to_likelihoods(Likelihood& lik){
		lik.minLikelihood = this->m_alphauniform_X_max2rangeinv_X_pi2inv
			+ this->m_alpha_exp
			* exp(-lik.min_mahal_dist);
		if(lik.mahal_dist > INT_MAX/2)
			lik.likelihood = lik.minLikelihood;
		else
			lik.likelihood = this->m_alphauniform_X_max2rangeinv_X_pi2inv
			+ this->m_alpha_exp
			* exp(-lik.mahal_dist);
	}

protected:
	float m_range, m_bearing, m_confidence, m_orientation;
	float m_x, m_y;
	WorldObjectType m_type;
#if USE_NEGATIVE_EVIDENCE
		bool m_virtual;
		bool m_expectedButUnseen;
#endif
public:
	static float s_std_lineorient;
	static float s_std_linerange;
	static float s_std_bearing;
	static float s_std_range;
	static float s_alpha_uniform_base;
	static const double m_max_2range;
	static const double m_max_2range_inv;
	static const double m_pi2_inv;

	// the following are precalculated by the obs.model once for each obs
	double m_alpha_uniform;
	double m_alpha_exp;
	double m_std_range;
	double m_3std_range;
	double m_3std_bearing;
	double m_3std_lineorient;
	double m_alphauniform_X_max2rangeinv_X_pi2inv;

	// the following are precalculated for observations 
	// for which we need to throw dice.
	bool   m_mahal_was_set;
	double m_mahal;
	double m_maxmahal;

	// do not check sign of distance for association with global pose
	bool   m_do_not_check_dist_sign;
};

class SimpleObservationModel : public AbstractObservationModel < Vec3f, LandmarkObservation, Mat22f >
{
public:
	SimpleObservationModel( SelfLocalizationPF * selfLocalization );
	~SimpleObservationModel( ) { }

	struct LineParam{
		float dist;
		float ang;
	};

//	inline void setSelfLocalizationPointer( SelfLocalizationPF* selfLocalization ) { m_selfLocalization = selfLocalization; }

	Likelihood minAndLikelihood( const ParticleT & particle, const LandmarkObservation & observation );
	double likelihood( const ParticleT & particle, const LandmarkObservation & observation );

	double minLikelihood( const ParticleT & particle, const LandmarkObservation & observation );

	double likelihoodDiscount( const LandmarkObservation & observation );

	LineParam  getLineParam(const ParticleT& particle, WorldObjectType wo);
	static bool       isFieldLine(WorldObjectType wo);

protected:
	SelfLocalizationPF * m_selfLocalization;

	double m_dummy;
};

class SelfLocalizationPF : public ParticleFilter < Vec3f, Vec3f, LandmarkObservation, Mat22f >
{

friend class SelfLocalizationPFView;
friend class SimpleObservationModel;

public:
	SelfLocalizationPF( );
	~SelfLocalizationPF( ) { }

	void setPose(const Vec3f& pose, const Vec3f stddev);

	void anticipateObservation( WideAngle_CV * cv, float twistedHeadingAngle );
	void update( float delta_t, Vec3f controlInput, WideAngle_CV * cv, float twistedHeadingAngle );

	inline const Vec2f
	worldObjectPos( unsigned int wo ) const {return m_worldObjectPos[ wo ].head<2>( );}

	inline const float
	worldObjectOrn( unsigned int wo ) const {return m_worldObjectPos[ wo ].z();}

	static double landmarkLikelihoodDiscount[ NumWorldObjects ];

	inline const Vec3f &
	getPoseMean( ) const {return m_mean;}

	inline const Pose
	getPose( ) const {return Pose( 100 * m_mean.x(), 100 * m_mean.y(), -m_mean.z(), m_confidence );}

	inline const Mat33f &
	getPoseCov( ) const {return m_cov;}

	inline const float &
	getPoseConf( ) const {return m_confidence;}

// 	inline const EgoObject&
// 	getLandmarkToTrack() const{return m_landmark_to_track;}

	bool useDPoles;

protected:

	class DataAssociationPair
	{
	public:
		DataAssociationPair( double l, WorldObjectType t ) : mahal_dist( l )
			, type( t ) { }

		~DataAssociationPair( ) { }

		double mahal_dist;
		WorldObjectType type;
	};
	void establishDataAssociation( ParticleT & particle, std::vector < LandmarkObservation > & observations );
public:
// 	void establishDataAssociation( EgoObject & obs, float twistedHeadingAngle, double& lik, WorldObjectType& wot );
// 	void establishDataAssociation( ObjectRecognition::FieldLine2 & obs, float twistedHeadingAngle, double& lik, WorldObjectType& wot );
protected:

	WorldObjectType
	determineMatch( std::vector < DataAssociationPair > & matches, double minLikelihood, LandmarkObservation& obs )
	{
		//return determineLikelihoodSamplingMatch( matches, minLikelihood, obs );
		return determineMaxLikelihoodMatch( matches, minLikelihood, obs );
	}

	WorldObjectType determineMaxLikelihoodMatch( std::vector < DataAssociationPair > & matches, double minLikelihood, LandmarkObservation& obs );
	WorldObjectType determineLikelihoodSamplingMatch( std::vector < DataAssociationPair > & matches, double minLikelihood, LandmarkObservation& obs );


	void determinePositionEstimate( );
	void meanShift( Vec2f startCartPos, float startOri, float maxCartDist, float maxAngDist, Vec3f & mean, double & weightSumFraction );

	float m_lastDelta_t;
	Vec3f m_lastControlInput;

	Vec3f m_mean;
	Mat33f m_cov;
	float m_confidence;

	Vec3f m_worldObjectPos[ NumWorldObjects ];

	std::vector < LandmarkObservation > m_observations;
// 	EgoObject m_landmark_to_track;
	std::map<WorldObjectType,int> more_is_better;

	WideAngle_CV * m_CV;
	float m_twistedHeadingAngle;

	float m_actualUniformReplacementProbability; // not necessarily /effective/ URP
	bool  m_usingActualReplacement;

	long int m_importance_time;

	FieldGrid2D::DistToWhiteGrid m_distToWhiteGrid;
	FieldGrid2D::ClosestWorldObject m_closestL_045;                         //< plus  45 grad
	FieldGrid2D::ClosestWorldObject m_closestL_135;                         //< plus  45 grad
	FieldGrid2D::ClosestWorldObject m_closestL_225;                         //< plus  45 grad
	FieldGrid2D::ClosestWorldObject m_closestL_315;                         //< minus 45 grad
	FieldGrid2D::ClosestWorldObject m_closestX;

	FieldGrid2D::ClosestWorldObject m_closestTCornerGrid;
};

void
LandmarkObservation::precalc( )
{
	double alpha_uniform = s_alpha_uniform_base + ( 1.f - s_alpha_uniform_base ) * ( 1.f - this->getConfidence( ) );

	m_alpha_exp = 1.0 - alpha_uniform;

	if(SimpleObservationModel::isFieldLine(this->m_type))
		m_std_range = s_std_linerange + fabs(0.25f * this->getRange( ));
	else
		m_std_range = s_std_range + 0.25f * this->getRange( );

	m_alpha_uniform = s_alpha_uniform_base + ( 1.f - s_alpha_uniform_base ) * ( 1.f - this->getConfidence( ) );
	m_alpha_exp = 1.0 - alpha_uniform;
	m_alphauniform_X_max2rangeinv_X_pi2inv = m_alpha_uniform * m_max_2range_inv * m_pi2_inv;

	m_3std_range = 3.0 * m_std_range;
	m_3std_bearing = 3.0 * s_std_bearing;
	m_3std_lineorient = 3.0 * s_std_lineorient;
}

}