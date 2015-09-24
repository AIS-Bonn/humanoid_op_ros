// JS: Particle filter implementation, 17.11.2007
// Adapted to ROS system by Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <iostream>
#include <vector>

#include "pf_math.h"

#include <gsl/gsl_rng.h>

#include <time.h>

#include <cfloat>

// * The entire ParticleFilter is within PF namespace * //
namespace PF {

struct Likelihood {
	double likelihood;
	double minLikelihood;
	double mahal_dist;      // mahalanobis dist for likelihood
	double min_mahal_dist;  // mahal dist for minLikelihood
};

// ************************************************************************************ //

template < class T_state, class T_userdata = void * >
class Particle
{
public:
	Particle( ) { }

	~Particle( ) { }

	inline void
	setState( const T_state & state ) {m_state = state;}

	inline const T_state &
	getState( ) const {return m_state;}

	inline void
	setWeight( float weight ) {m_weight = weight;}

	inline float
	getWeight( ) const {return m_weight;}

	inline void
	increaseWeight( float inc ) {m_weight += inc;}

	inline void
	setUserData( const T_userdata & d ) {m_userdata = d;}

	inline T_userdata &
	getUserData( ) {return m_userdata;}

	inline const T_userdata &
	getUserData( ) const {return m_userdata;}

#define PARTICLE_DEBUGGING 0
#if PARTICLE_DEBUGGING
	std::vector < pair < std::wstring, Vec3f > > m_DebuggingVector;
#endif

protected:
	T_state m_state;
	float m_weight;
	T_userdata m_userdata;
};

// ************************************************************************************ //

template < class T_state, class T_userdata = void * >
class AbstractParticleInitializer
{
public:
	typedef Particle<T_state, T_userdata> ParticleT;

	AbstractParticleInitializer( ) { }

	virtual ~AbstractParticleInitializer( ) { }

	virtual void initParticle( ParticleT & particle ) = 0;

};

// ************************************************************************************ //

template < class T_state, class T_userdata = void * >
class ParticleSet : public std::vector < Particle < T_state, T_userdata > >
{
public:
	ParticleSet( ) { }

	~ParticleSet( ) { }

};

// ************************************************************************************ //

template < class T_state, class T_control >
class AbstractMotionModel
{
public:
	AbstractMotionModel( ) { }

	virtual ~AbstractMotionModel( ) { }

	virtual T_state sampleState( const T_state & lastPose, const T_control & controlInput, float delta_t ) = 0;

};

// ************************************************************************************ //

template < class T_state, class T_obs, class T_userdata = void * >
class AbstractObservationModel
{
public:
	typedef Particle<T_state, T_userdata> ParticleT;

	AbstractObservationModel( ) { }

	virtual ~AbstractObservationModel( ) { }

	virtual double likelihood( const ParticleT& particle, const T_obs & observation ) = 0;

	float
	logLikelihood( const ParticleT& particle, const T_obs & observation ) {return ( float ) log( likelihood( particle, observation ) );}

	// p_0, or the lowest likelihood when associating
	virtual double minLikelihood( const ParticleT& particle, const T_obs & observation ) = 0;
	virtual Likelihood minAndLikelihood( const ParticleT& particle, const T_obs & observation ) = 0;

	virtual double likelihoodDiscount( const T_obs & observation ) = 0;

};

// ************************************************************************************ //

// * MAIN CLASS * //
template < class T_state, class T_control, class T_obs, class T_userdata = void * >
class ParticleFilter
{
public:
	typedef Particle<T_state, T_userdata> ParticleT;
	typedef ParticleSet<T_state, T_userdata> ParticleSetT;
	typedef AbstractParticleInitializer<T_state, T_userdata> InitializerT;
	typedef AbstractMotionModel<T_state, T_control> MotionModelT;
	typedef AbstractObservationModel<T_state, T_obs, T_userdata> ObservationModelT;

	ParticleFilter( AbstractParticleInitializer < T_state, T_userdata > * particleInitializer,
	                AbstractMotionModel < T_state, T_control > * motionModel,
	                AbstractObservationModel < T_state, T_obs, T_userdata > * observationModel,
	                unsigned int minNumParticles,
	                unsigned int maxNumParticles,
	                double slowAvgLikelihoodLearningRate,
	                double fastAvgLikelihoodLearningRate )
		: m_particleInitializer( particleInitializer )
		, m_motionModel( motionModel )
		, m_observationModel( observationModel )
		, m_minNumParticles( minNumParticles )
		, m_maxNumParticles( maxNumParticles )
		, m_currNumParticles( maxNumParticles )
		, m_prevNumParticles( 0 )
		, m_hasResampled( false )
		, m_minEffectiveSampleSizeRatio( 1.f )
		, m_slowAvgLikelihoodLearningRate( slowAvgLikelihoodLearningRate )
		, m_fastAvgLikelihoodLearningRate( fastAvgLikelihoodLearningRate )
	{

		m_effectiveSampleSize = ( float ) maxNumParticles;

		// * Init random number generator. * //
		const gsl_rng_type * T;
		gsl_rng_env_setup( );
		T = gsl_rng_default;
		m_rng = gsl_rng_alloc( T );
		gsl_rng_set( m_rng, 12345 );

		// * Init particles * //
		for ( unsigned int i = 0; i < 2; i++ ) {
			m_particles[ i ].clear( );
			m_particles[ i ].resize( maxNumParticles );
			for ( unsigned int p = 0; p < maxNumParticles; p++ ) {
				m_particleInitializer->initParticle( m_particles[ i ][ p ] );
				m_particles[ i ][ p ].setWeight( 0 );
			}
		}
		m_currParticleSetIdx = 0;

		m_slowAverageLikelihood = 1;
		m_fastAverageLikelihood = 0;

		fprintf(stderr, "after PF cons\n");
	}

	~ParticleFilter( )
	{
		gsl_rng_free( m_rng );
		delete m_particleInitializer;
		delete m_motionModel;
		delete m_observationModel;
	}

	// per-particle data association
	// save association information within observations
	virtual void
	establishDataAssociation( Particle < T_state, T_userdata > & particle, std::vector < T_obs > & observations ) { }

	virtual void
	establishDataAssociation( Particle < T_state, T_userdata > & particle, std::vector < T_obs * > & observations ) { }

	virtual void
	sample( const T_control & controlInput, float delta_t )
	{
		ParticleSet < T_state, T_userdata > & particles = m_particles[ m_currParticleSetIdx ];
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			Particle < T_state, T_userdata > & particle = particles[ p ];
			particle.setState( m_motionModel->sampleState( particle.getState( ), controlInput, delta_t ) );
		}
	}

	void
	importance( std::vector < T_obs * > & observations )
	{
		double avgLikelihood = 0;

		ParticleSet < T_state, T_userdata > & particles = m_particles[ m_currParticleSetIdx ];
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			Particle < T_state, T_userdata > & particle = particles[ p ];
			establishDataAssociation( particle, observations );
			for ( unsigned int o = 0; o < observations.size( ); o++ ) {
				T_obs * observation = observations[ o ];
				Likelihood mal;
				if(observation->isLikSet())
					mal = observation->getMinAndLikelihood();
				else
					mal = m_observationModel->minAndLikelihood( particle, *observation );
				observation->mahal_dists_to_likelihoods(mal);
				double obsLikelihood = std::max( mal.likelihood, mal.minLikelihood );
				avgLikelihood += obsLikelihood;
				particle.increaseWeight( ( float ) ( m_observationModel->likelihoodDiscount( *observation ) * log( obsLikelihood ) ) );
			}
		}
		if ( observations.size( ) > 0 ) {
			avgLikelihood /= ( ( double ) ( observations.size( ) * m_currNumParticles ) );
			m_slowAverageLikelihood += ( float ) ( m_slowAvgLikelihoodLearningRate * ( avgLikelihood - m_slowAverageLikelihood ) );
			m_fastAverageLikelihood += ( float ) ( m_fastAvgLikelihoodLearningRate * ( avgLikelihood - m_fastAverageLikelihood ) );
		}
		float maxlogweight = -FLT_MAX;
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			Particle < T_state, T_userdata > & particle = particles[ p ];
			if ( particle.getWeight( ) > maxlogweight ) {
				maxlogweight = particle.getWeight( );
			}
		}
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			Particle < T_state, T_userdata > & particle = particles[ p ];
			particle.setWeight( particle.getWeight( ) - maxlogweight );
		}
	}

	void
	importance( std::vector < T_obs > & observations )
	{
		double avgLikelihood = 0;

		ParticleSet < T_state, T_userdata > & particles = m_particles[ m_currParticleSetIdx ];
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			Particle < T_state, T_userdata > & particle = particles[ p ];
			establishDataAssociation( particle, observations );
			for ( unsigned int o = 0; o < observations.size( ); o++ ) {
				T_obs & observation = observations[ o ];
				Likelihood mal;
				if(observation.isMahalSet())
					mal = observation.getMinAndLikelihood();
				else
					mal = m_observationModel->minAndLikelihood( particle, observation );
				observation.mahal_dists_to_likelihoods(mal);
				double obsLikelihood = std::max( mal.likelihood, mal.minLikelihood );
				avgLikelihood += obsLikelihood;
				particle.increaseWeight( ( float ) ( m_observationModel->likelihoodDiscount( observation ) * log( obsLikelihood ) ) );
				assert( particle.getWeight( ) == particle.getWeight( ) );
			}
		}
		if ( observations.size( ) > 0 ) {
			avgLikelihood /= ( ( double ) ( observations.size( ) * m_currNumParticles ) );
			assert( avgLikelihood == avgLikelihood );
			m_slowAverageLikelihood += ( float ) ( m_slowAvgLikelihoodLearningRate * ( avgLikelihood - m_slowAverageLikelihood ) );
			m_fastAverageLikelihood += ( float ) ( m_fastAvgLikelihoodLearningRate * ( avgLikelihood - m_fastAverageLikelihood ) );
		}
		float maxlogweight = -FLT_MAX;
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			Particle < T_state, T_userdata > & particle = particles[ p ];
			if ( particle.getWeight( ) > maxlogweight ) {
				maxlogweight = particle.getWeight( );
			}
		}
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			Particle < T_state, T_userdata > & particle = particles[ p ];
			particle.setWeight( particle.getWeight( ) - maxlogweight );
		}
	}

	void
	resample( )
	{
		// low-variance-sampler
		// sample with replacement
		ParticleSet < T_state, T_userdata > & particles = m_particles[ m_currParticleSetIdx ];
		ParticleSet < T_state, T_userdata > & tmp_particles = m_particles[ 1 - m_currParticleSetIdx ];


		// rescale weights for improved numerical stability
		float maxLogWeight = 0;
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			float logweight = particles[ p ].getWeight( );
			if ( logweight > maxLogWeight ) {
				maxLogWeight = logweight;
			}
		}
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			particles[ p ].setWeight( particles[ p ].getWeight( ) - maxLogWeight );
		}
		float sumWeight = 0;
		float minLogWeight = 0;
		maxLogWeight = 0;
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			float logweight = particles[ p ].getWeight( );
			sumWeight += ( float ) exp( ( double ) logweight );
			if ( logweight > maxLogWeight ) {
				maxLogWeight = logweight;
			}
			if ( logweight < minLogWeight ) {
				minLogWeight = logweight;
			}
		}
		float avgObsLikelihoodRatio = std::min( 1.f, std::max( 0.f, ( float ) ( m_fastAverageLikelihood / m_slowAverageLikelihood ) ) );

		// resample in any case if weights get numerically instable
		// also resample for low avg obs likelihood to induce uniform particles
		if ( ( avgObsLikelihoodRatio > 0.8f ) && ( minLogWeight > -200.f ) && ( maxLogWeight < 200.f ) ) {

			// compute effective sample size, if necessary
			if ( m_minEffectiveSampleSizeRatio < 1.f ) {
				double sumSqrWeights = 0;
				for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
					sumSqrWeights += exp( 2.0 * ( double ) particles[ p ].getWeight( ) );
				}
				if ( ( sumSqrWeights == 0 ) || ( sumWeight == 0 ) ) {
					m_effectiveSampleSize = ( float ) m_currNumParticles;
				}
				else {
					m_effectiveSampleSize = ( float ) ( ( sumWeight * sumWeight ) / sumSqrWeights );
				}
				if ( m_effectiveSampleSize >= ( m_minEffectiveSampleSizeRatio * ( ( float ) m_currNumParticles ) ) ) {
					m_hasResampled = false;
					return;
				}
			}
		}
		// 1. normalize weights of particles to resampling probabilities
		if ( sumWeight == 0 ) {
			const float psize_inv = 1.f / ( ( float ) m_currNumParticles );
			for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
				particles[ p ].setWeight( psize_inv );
			}
		}
		else {
			for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
				particles[ p ].setWeight( ( float ) ( exp( particles[ p ].getWeight( ) ) / sumWeight ) );
			}
		}
		unsigned int newParticleSetSize = m_minNumParticles + ( unsigned int )( ( 1.f - std::max( 0.f, std::min( 1.f, ( avgObsLikelihoodRatio - 0.5f ) / 0.5f ) ) ) * ( m_maxNumParticles - m_minNumParticles ) );
		float n_inc = 1.f / ( ( float ) newParticleSetSize );

		// 2. resample
		float n = ( ( float ) gsl_rng_uniform( m_rng ) ) * n_inc;
		float p_sum = 0;
		unsigned int tmp_idx = 0;
		for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
			p_sum += particles[ p ].getWeight( );
			while ( p_sum > n ) {
				tmp_particles[ tmp_idx ] = particles[ p ];
				tmp_particles[ tmp_idx ].setWeight( 0 );
				n += n_inc;
				tmp_idx++;
				if ( tmp_idx >= newParticleSetSize ) {
					break;
				}
			}
			if ( tmp_idx >= newParticleSetSize ) {
				break;
			}
		}

		// 5. replace fraction at random
		for ( unsigned int p = 0; p < newParticleSetSize; p++ ) {
			if ( gsl_rng_uniform( m_rng ) < m_uniformReplacementProbability ) {
				m_particleInitializer->initParticle( tmp_particles[ p ] );
				//tmp_particles[p].setWeight( 0 );
			}
			tmp_particles[ p ].setWeight( 0 );
		}
		// swap
		m_currParticleSetIdx = 1 - m_currParticleSetIdx;
		m_prevNumParticles = m_currNumParticles;
		m_currNumParticles = newParticleSetSize;
		m_hasResampled = true;
	}

	const ParticleSet < T_state, T_userdata > &
	getParticles( ) const {return m_particles[ m_currParticleSetIdx ];}

protected:

	// double buffered for improved performance
	ParticleSet < T_state, T_userdata > m_particles[ 2 ];
	unsigned int m_currParticleSetIdx;

	AbstractParticleInitializer < T_state, T_userdata > * m_particleInitializer;
	AbstractMotionModel < T_state, T_control > * m_motionModel;
	AbstractObservationModel < T_state, T_obs, T_userdata > * m_observationModel;

	float m_slowAverageLikelihood;
	float m_fastAverageLikelihood;

	unsigned int m_minNumParticles;
	unsigned int m_maxNumParticles;
	unsigned int m_currNumParticles, m_prevNumParticles;
	bool m_hasResampled;

	gsl_rng * m_rng;

	float m_minEffectiveSampleSizeRatio;
	float m_effectiveSampleSize;

	float m_uniformReplacementProbability;

	double m_slowAvgLikelihoodLearningRate;
	double m_fastAvgLikelihoodLearningRate;
};

} // END of Namespace PF

#endif
