#ifndef SIMULATOR_H
#define SIMULATOR_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particle.h"

// class containing objects to be simulated
class Simulator {
public:
    Simulator();
        
    void simulate();
    
    int getNumParticles();
    
    Particle* getParticle(int);
    
    double getTimeStep();
    
    void reset();
	double getLambda(int particleNum);
	Eigen::Vector3d getConstraintForce(int particleNum, double lambda);
	Eigen::Vector3d getLegalAcceleration(int particleNum, Eigen::Vector3d constraintForce);
	void particleSim();
private:
    double mTimeStep;       // time step
    std::vector<Particle> mParticles;
};

#endif  // SIMULATOR_H
