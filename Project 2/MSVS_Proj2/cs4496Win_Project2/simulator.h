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
	double analyticalStep();
	double explicitEulerStep();
private:
    double mTimeStep;       // time step
    double mElapsedTime;    // time pased since beginning of simulation
	double startingHeight;   //This is the startingHeight for all the particles
	double startingVelocity; // A var to hold the initial velocity in the event I get extra credit
    std::vector<Particle> mParticles;
};

#endif  // SIMULATOR_H
